// pupper_controller.c
// SDL2 Game Hat controller for Mini-Pupper with live camera feed
// Sends HTTPS movement commands and displays camera stream on HUD
//
// Compile:
//   gcc -o pupper_controller pupper_controller.c -lSDL2 -lcurl
//
// Usage:
//   ./pupper_controller [robot-ip]

#include <SDL2/SDL.h>
#include <SDL2/SDL_image.h>
#include <curl/curl.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

// Configuration
#define WINDOW_W  800
#define WINDOW_H  600
#define DEFAULT_ROBOT_IP "10.170.8.136"
#define ROBOT_PORT 8444
#define COMMAND_INTERVAL_MS 100
#define CAMERA_FETCH_INTERVAL_MS 150  // ~6-7 FPS camera fetch

// Direction enum
typedef enum {
    DIR_NONE,
    DIR_FORWARD,
    DIR_BACKWARD,
    DIR_LEFT,
    DIR_RIGHT
} Direction;

static const char* dir_to_string(Direction d) {
    switch (d) {
        case DIR_FORWARD:  return "forward";
        case DIR_BACKWARD: return "backward";
        case DIR_LEFT:     return "left";
        case DIR_RIGHT:    return "right";
        default:           return "stop";
    }
}

static const char* dir_to_display(Direction d) {
    switch (d) {
        case DIR_FORWARD:  return "FORWARD";
        case DIR_BACKWARD: return "BACKWARD";
        case DIR_LEFT:     return "LEFT";
        case DIR_RIGHT:    return "RIGHT";
        default:           return "STOPPED";
    }
}

// JPEG buffer for camera frames
typedef struct {
    unsigned char* data;
    size_t size;
    size_t capacity;
} JpegBuffer;

static void jpeg_buf_init(JpegBuffer* buf) {
    buf->capacity = 256 * 1024;  // 256KB initial
    buf->data = (unsigned char*)malloc(buf->capacity);
    buf->size = 0;
}

static void jpeg_buf_free(JpegBuffer* buf) {
    free(buf->data);
    buf->data = NULL;
    buf->size = 0;
    buf->capacity = 0;
}

static void jpeg_buf_reset(JpegBuffer* buf) {
    buf->size = 0;
}

static size_t jpeg_write_callback(void* contents, size_t size, size_t nmemb, void* userp) {
    JpegBuffer* buf = (JpegBuffer*)userp;
    size_t total = size * nmemb;

    if (buf->size + total > buf->capacity) {
        size_t newcap = buf->capacity;
        while (newcap < buf->size + total) newcap *= 2;
        unsigned char* newdata = (unsigned char*)realloc(buf->data, newcap);
        if (!newdata) return 0;
        buf->data = newdata;
        buf->capacity = newcap;
    }

    memcpy(buf->data + buf->size, contents, total);
    buf->size += total;
    return total;
}

// Curl state
static CURL* g_curl_move = NULL;
static CURL* g_curl_cam = NULL;
static char g_move_url[256];
static char g_cam_url[256];
static bool g_connected = false;
static bool g_camera_ok = false;
static char g_status_msg[128] = "Connecting...";
static Uint32 g_last_move_send = 0;
static Uint32 g_last_cam_fetch = 0;
static JpegBuffer g_jpeg_buf;

// Suppress curl output for move commands
static size_t write_devnull(void* p, size_t s, size_t n, void* u) {
    (void)p; (void)u;
    return s * n;
}

static int curl_init(const char* robot_ip) {
    if (curl_global_init(CURL_GLOBAL_DEFAULT) != 0) return 0;

    // Move command curl handle
    g_curl_move = curl_easy_init();
    if (!g_curl_move) return 0;

    snprintf(g_move_url, sizeof(g_move_url), "https://%s:%d/move", robot_ip, ROBOT_PORT);
    curl_easy_setopt(g_curl_move, CURLOPT_POST, 1L);
    curl_easy_setopt(g_curl_move, CURLOPT_SSL_VERIFYPEER, 0L);
    curl_easy_setopt(g_curl_move, CURLOPT_SSL_VERIFYHOST, 0L);
    curl_easy_setopt(g_curl_move, CURLOPT_TIMEOUT, 1L);
    curl_easy_setopt(g_curl_move, CURLOPT_CONNECTTIMEOUT, 1L);
    curl_easy_setopt(g_curl_move, CURLOPT_WRITEFUNCTION, write_devnull);

    // Camera fetch curl handle
    g_curl_cam = curl_easy_init();
    if (!g_curl_cam) return 0;

    snprintf(g_cam_url, sizeof(g_cam_url), "https://%s:%d/camera", robot_ip, ROBOT_PORT);
    curl_easy_setopt(g_curl_cam, CURLOPT_HTTPGET, 1L);
    curl_easy_setopt(g_curl_cam, CURLOPT_SSL_VERIFYPEER, 0L);
    curl_easy_setopt(g_curl_cam, CURLOPT_SSL_VERIFYHOST, 0L);
    curl_easy_setopt(g_curl_cam, CURLOPT_TIMEOUT, 2L);
    curl_easy_setopt(g_curl_cam, CURLOPT_CONNECTTIMEOUT, 1L);
    curl_easy_setopt(g_curl_cam, CURLOPT_URL, g_cam_url);
    curl_easy_setopt(g_curl_cam, CURLOPT_WRITEFUNCTION, jpeg_write_callback);
    curl_easy_setopt(g_curl_cam, CURLOPT_WRITEDATA, &g_jpeg_buf);

    jpeg_buf_init(&g_jpeg_buf);

    printf("[CTRL] Move URL: %s\n", g_move_url);
    printf("[CTRL] Camera URL: %s\n", g_cam_url);
    return 1;
}

static void curl_shutdown(void) {
    if (g_curl_move) curl_easy_cleanup(g_curl_move);
    if (g_curl_cam) curl_easy_cleanup(g_curl_cam);
    jpeg_buf_free(&g_jpeg_buf);
    curl_global_cleanup();
}

static void send_command(Direction dir) {
    if (!g_curl_move) return;

    Uint32 now = SDL_GetTicks();
    if (now - g_last_move_send < COMMAND_INTERVAL_MS) return;
    g_last_move_send = now;

    char json[128];
    snprintf(json, sizeof(json), "{\"move_dir\":\"%s\"}", dir_to_string(dir));

    struct curl_slist* headers = NULL;
    headers = curl_slist_append(headers, "Content-Type: application/json");

    curl_easy_setopt(g_curl_move, CURLOPT_URL, g_move_url);
    curl_easy_setopt(g_curl_move, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(g_curl_move, CURLOPT_POSTFIELDS, json);

    CURLcode res = curl_easy_perform(g_curl_move);

    if (res == CURLE_OK) {
        long http_code = 0;
        curl_easy_getinfo(g_curl_move, CURLINFO_RESPONSE_CODE, &http_code);
        g_connected = true;
        snprintf(g_status_msg, sizeof(g_status_msg), "Connected (HTTP %ld)", http_code);
    } else {
        g_connected = false;
        snprintf(g_status_msg, sizeof(g_status_msg), "Error: %s", curl_easy_strerror(res));
    }

    curl_slist_free_all(headers);
}

// Fetch camera frame and update SDL texture
static SDL_Texture* fetch_camera_frame(SDL_Renderer* renderer, SDL_Texture* current_tex) {
    if (!g_curl_cam) return current_tex;

    Uint32 now = SDL_GetTicks();
    if (now - g_last_cam_fetch < CAMERA_FETCH_INTERVAL_MS) return current_tex;
    g_last_cam_fetch = now;

    jpeg_buf_reset(&g_jpeg_buf);

    CURLcode res = curl_easy_perform(g_curl_cam);

    if (res != CURLE_OK || g_jpeg_buf.size == 0) {
        g_camera_ok = false;
        return current_tex;
    }

    long http_code = 0;
    curl_easy_getinfo(g_curl_cam, CURLINFO_RESPONSE_CODE, &http_code);
    if (http_code != 200) {
        g_camera_ok = false;
        return current_tex;
    }

    // Load JPEG from memory using SDL2_image
    SDL_RWops* rw = SDL_RWFromMem(g_jpeg_buf.data, (int)g_jpeg_buf.size);
    if (!rw) {
        g_camera_ok = false;
        return current_tex;
    }

    SDL_Surface* surface = IMG_Load_RW(rw, 1);
    if (!surface) {
        g_camera_ok = false;
        return current_tex;
    }

    if (current_tex) SDL_DestroyTexture(current_tex);
    current_tex = SDL_CreateTextureFromSurface(renderer, surface);
    SDL_FreeSurface(surface);
    g_camera_ok = true;

    return current_tex;
}

// Rendering
static void draw_rect(SDL_Renderer* r, int x, int y, int w, int h,
                      Uint8 cr, Uint8 cg, Uint8 cb) {
    SDL_Rect rect = {x, y, w, h};
    SDL_SetRenderDrawColor(r, cr, cg, cb, 255);
    SDL_RenderFillRect(r, &rect);
}

static void draw_rect_outline(SDL_Renderer* r, int x, int y, int w, int h,
                              Uint8 cr, Uint8 cg, Uint8 cb) {
    SDL_Rect rect = {x, y, w, h};
    SDL_SetRenderDrawColor(r, cr, cg, cb, 255);
    SDL_RenderDrawRect(r, &rect);
}

static void draw_arrow_up(SDL_Renderer* r, int cx, int cy, int size, bool active) {
    Uint8 cr = active ? 0 : 60, cg = active ? 245 : 80, cb = active ? 255 : 90;
    SDL_SetRenderDrawColor(r, cr, cg, cb, 255);
    for (int row = 0; row < size; row++) {
        int half = (row * size) / size;
        SDL_RenderDrawLine(r, cx - half, cy + row, cx + half, cy + row);
    }
}

static void draw_arrow_down(SDL_Renderer* r, int cx, int cy, int size, bool active) {
    Uint8 cr = active ? 0 : 60, cg = active ? 245 : 80, cb = active ? 255 : 90;
    SDL_SetRenderDrawColor(r, cr, cg, cb, 255);
    for (int row = 0; row < size; row++) {
        int half = ((size - row) * size) / size;
        SDL_RenderDrawLine(r, cx - half, cy + row, cx + half, cy + row);
    }
}

static void draw_arrow_left(SDL_Renderer* r, int cx, int cy, int size, bool active) {
    Uint8 cr = active ? 0 : 60, cg = active ? 245 : 80, cb = active ? 255 : 90;
    SDL_SetRenderDrawColor(r, cr, cg, cb, 255);
    for (int col = 0; col < size; col++) {
        int half = (col * size) / size;
        SDL_RenderDrawLine(r, cx + col, cy - half, cx + col, cy + half);
    }
}

static void draw_arrow_right(SDL_Renderer* r, int cx, int cy, int size, bool active) {
    Uint8 cr = active ? 0 : 60, cg = active ? 245 : 80, cb = active ? 255 : 90;
    SDL_SetRenderDrawColor(r, cr, cg, cb, 255);
    for (int col = 0; col < size; col++) {
        int half = ((size - col) * size) / size;
        SDL_RenderDrawLine(r, cx + col, cy - half, cx + col, cy + half);
    }
}

static void draw_hud(SDL_Renderer* r, Direction dir, SDL_Texture* cam_tex) {
    // Dark background
    SDL_SetRenderDrawColor(r, 3, 10, 15, 255);
    SDL_RenderClear(r);

    // Camera feed area (left side, main area)
    int cam_x = 10;
    int cam_y = 10;
    int cam_w = 540;
    int cam_h = 405;

    // Camera border
    draw_rect_outline(r, cam_x - 1, cam_y - 1, cam_w + 2, cam_h + 2, 0, 180, 200);

    if (cam_tex) {
        SDL_Rect cam_rect = {cam_x, cam_y, cam_w, cam_h};
        SDL_RenderCopy(r, cam_tex, NULL, &cam_rect);
    } else {
        // No camera feed - draw placeholder
        draw_rect(r, cam_x, cam_y, cam_w, cam_h, 10, 15, 20);
        // Cross pattern to indicate no signal
        SDL_SetRenderDrawColor(r, 30, 50, 60, 255);
        SDL_RenderDrawLine(r, cam_x, cam_y, cam_x + cam_w, cam_y + cam_h);
        SDL_RenderDrawLine(r, cam_x + cam_w, cam_y, cam_x, cam_y + cam_h);
    }

    // Camera status indicator
    if (g_camera_ok) {
        draw_rect(r, cam_x + 5, cam_y + 5, 8, 8, 57, 255, 20);  // green
    } else {
        draw_rect(r, cam_x + 5, cam_y + 5, 8, 8, 255, 45, 85);   // red
    }

    // D-pad area (right side)
    int dpad_cx = 680;
    int dpad_cy = 200;
    int arrow_size = 30;
    int spacing = 60;

    // D-pad background
    draw_rect(r, 580, 100, 210, 230, 7, 20, 32);
    draw_rect_outline(r, 580, 100, 210, 230, 0, 100, 120);

    // Arrows
    draw_arrow_up(r, dpad_cx, dpad_cy - spacing, arrow_size, dir == DIR_FORWARD);
    draw_arrow_down(r, dpad_cx, dpad_cy + spacing - arrow_size, arrow_size, dir == DIR_BACKWARD);
    draw_arrow_left(r, dpad_cx - spacing, dpad_cy, arrow_size, dir == DIR_LEFT);
    draw_arrow_right(r, dpad_cx + spacing - arrow_size, dpad_cy, arrow_size, dir == DIR_RIGHT);

    // Center dot
    if (dir == DIR_NONE) {
        draw_rect(r, dpad_cx - 6, dpad_cy - 6, 12, 12, 255, 190, 0);
    } else {
        draw_rect(r, dpad_cx - 4, dpad_cy - 4, 8, 8, 50, 50, 60);
    }

    // Connection status area (right side, below d-pad)
    draw_rect(r, 580, 350, 210, 65, 7, 20, 32);
    draw_rect_outline(r, 580, 350, 210, 65, 0, 100, 120);

    // Connection dot
    if (g_connected) {
        draw_rect(r, 595, 365, 12, 12, 57, 255, 20);
    } else {
        draw_rect(r, 595, 365, 12, 12, 255, 45, 85);
    }

    // Direction indicator bar (bottom)
    int bar_y = 430;
    draw_rect(r, 10, bar_y, 780, 50, 7, 20, 32);
    draw_rect(r, 10, bar_y, 780, 2, 0, 180, 200);

    // Active direction highlight
    if (dir != DIR_NONE) {
        int bw = 0;
        int bx = 0;
        switch (dir) {
            case DIR_FORWARD:  bx = 10;  bw = 195; break;
            case DIR_BACKWARD: bx = 205; bw = 195; break;
            case DIR_LEFT:     bx = 400; bw = 195; break;
            case DIR_RIGHT:    bx = 595; bw = 195; break;
            default: break;
        }
        draw_rect(r, bx, bar_y + 10, bw, 30, 0, 245, 255);
    }

    // Bottom info bar
    draw_rect(r, 0, WINDOW_H - 80, WINDOW_W, 80, 5, 12, 18);
    draw_rect(r, 0, WINDOW_H - 80, WINDOW_W, 2, 0, 120, 140);

    // Status indicators in bottom bar
    // Robot connection
    if (g_connected) {
        draw_rect(r, 20, WINDOW_H - 60, 10, 10, 57, 255, 20);
    } else {
        draw_rect(r, 20, WINDOW_H - 60, 10, 10, 255, 45, 85);
    }

    // Camera status
    if (g_camera_ok) {
        draw_rect(r, 20, WINDOW_H - 40, 10, 10, 57, 255, 20);
    } else {
        draw_rect(r, 20, WINDOW_H - 40, 10, 10, 255, 45, 85);
    }

    SDL_RenderPresent(r);
}

// Main
int main(int argc, char* argv[]) {
    const char* robot_ip = DEFAULT_ROBOT_IP;
    if (argc > 1) {
        robot_ip = argv[1];
    } else {
        const char* env_ip = getenv("PUPPER_IP");
        if (env_ip && *env_ip) robot_ip = env_ip;
    }

    printf("[CTRL] Mini-Pupper Controller + Camera\n");
    printf("[CTRL] Robot IP: %s\n", robot_ip);
    printf("[CTRL] Controls: Arrow keys / WASD / D-pad\n");
    printf("[CTRL] Exit: ESC or Q (left trigger)\n");

    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        fprintf(stderr, "SDL_Init failed: %s\n", SDL_GetError());
        return 1;
    }

    if (!curl_init(robot_ip)) {
        fprintf(stderr, "Failed to initialize curl\n");
        SDL_Quit();
        return 1;
    }

    SDL_Window* win = SDL_CreateWindow(
        "Mini-Pupper Controller",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        WINDOW_W, WINDOW_H, SDL_WINDOW_SHOWN);

    SDL_Renderer* renderer = SDL_CreateRenderer(win, -1,
        SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);

    bool running = true;
    Direction current_dir = DIR_NONE;
    Direction prev_dir = DIR_NONE;
    SDL_Texture* cam_texture = NULL;

    while (running) {
        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_QUIT) {
                running = false;
            }
            if (e.type == SDL_KEYDOWN) {
                SDL_Keycode k = e.key.keysym.sym;
                if (k == SDLK_ESCAPE || k == SDLK_q) {
                    running = false;
                }
            }
        }

        // Read current key state
        const Uint8* keys = SDL_GetKeyboardState(NULL);

        Direction new_dir = DIR_NONE;
        if (keys[SDL_SCANCODE_UP] || keys[SDL_SCANCODE_W])
            new_dir = DIR_FORWARD;
        else if (keys[SDL_SCANCODE_DOWN] || keys[SDL_SCANCODE_S])
            new_dir = DIR_BACKWARD;
        else if (keys[SDL_SCANCODE_LEFT] || keys[SDL_SCANCODE_A])
            new_dir = DIR_LEFT;
        else if (keys[SDL_SCANCODE_RIGHT] || keys[SDL_SCANCODE_D])
            new_dir = DIR_RIGHT;

        current_dir = new_dir;

        // Send movement commands
        if (current_dir != DIR_NONE) {
            send_command(current_dir);
        } else if (prev_dir != DIR_NONE) {
            send_command(DIR_NONE);
        }
        prev_dir = current_dir;

        // Fetch camera frame
        cam_texture = fetch_camera_frame(renderer, cam_texture);

        // Draw HUD
        draw_hud(renderer, current_dir, cam_texture);

        SDL_Delay(16);
    }

    // Stop robot before exiting
    g_last_move_send = 0;
    send_command(DIR_NONE);

    if (cam_texture) SDL_DestroyTexture(cam_texture);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(win);
    curl_shutdown();
    SDL_Quit();

    printf("[CTRL] Controller shut down. Robot stopped.\n");
    return 0;
}
