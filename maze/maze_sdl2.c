// maze_sdl2.c
// SDL2 maze + HTTPS telemetry client using libcurl.
// Sends per-move telemetry + final mission summary.
// DEV ONLY: disables TLS certificate + hostname verification.

#include <SDL2/SDL.h>
#include <stdbool.h>
#include <stdint.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <curl/curl.h>

#define MAZE_W 21
#define MAZE_H 15
#define CELL   32
#define PAD    16

/* ================= Telemetry config ================= */

#define TELEMETRY_URL_1 "https://10.170.8.101:8444/move"
#define TELEMETRY_URL_2 "https://10.170.8.109:8444/move"
#define MISSION_SUMMARY_URL "https://10.170.8.109:8444/mission_summary"

#define ROBOT_ID "mini-pupper-01"
#define MISSION_TYPE "patrol"

/* ================= Maze ================= */

enum { WALL_N = 1, WALL_E = 2, WALL_S = 4, WALL_W = 8 };

typedef struct {
  uint8_t walls;
  bool visited;
} Cell;

static Cell g[MAZE_H][MAZE_W];

static inline bool in_bounds(int x, int y) {
  return (x >= 0 && x < MAZE_W && y >= 0 && y < MAZE_H);
}

static void knock_down(int x, int y, int nx, int ny) {
  if (nx == x && ny == y - 1) {
    g[y][x].walls &= ~WALL_N;
    g[ny][nx].walls &= ~WALL_S;
  } else if (nx == x + 1 && ny == y) {
    g[y][x].walls &= ~WALL_E;
    g[ny][nx].walls &= ~WALL_W;
  } else if (nx == x && ny == y + 1) {
    g[y][x].walls &= ~WALL_S;
    g[ny][nx].walls &= ~WALL_N;
  } else if (nx == x - 1 && ny == y) {
    g[y][x].walls &= ~WALL_W;
    g[ny][nx].walls &= ~WALL_E;
  }
}

static void maze_init(void) {
  for (int y = 0; y < MAZE_H; y++)
    for (int x = 0; x < MAZE_W; x++) {
      g[y][x].walls = WALL_N | WALL_E | WALL_S | WALL_W;
      g[y][x].visited = false;
    }
}

static void maze_generate(int sx, int sy) {
  typedef struct { int x, y; } P;
  P stack[MAZE_W * MAZE_H];
  int top = 0;

  g[sy][sx].visited = true;
  stack[top++] = (P){sx, sy};

  while (top > 0) {
    P c = stack[top - 1];
    int x = c.x, y = c.y;

    P n[4];
    int nc = 0;
    const int dx[4] = {0,1,0,-1};
    const int dy[4] = {-1,0,1,0};

    for (int i = 0; i < 4; i++) {
      int nx = x + dx[i], ny = y + dy[i];
      if (in_bounds(nx, ny) && !g[ny][nx].visited)
        n[nc++] = (P){nx, ny};
    }

    if (!nc) { top--; continue; }

    int p = rand() % nc;
    knock_down(x, y, n[p].x, n[p].y);
    g[n[p].y][n[p].x].visited = true;
    stack[top++] = n[p];
  }

  for (int y = 0; y < MAZE_H; y++)
    for (int x = 0; x < MAZE_W; x++)
      g[y][x].visited = false;
}

/* ================= Telemetry ================= */

static CURL* g_curl = NULL;

static int telemetry_init(void) {
  if (curl_global_init(CURL_GLOBAL_DEFAULT) != 0) return 0;
  g_curl = curl_easy_init();
  if (!g_curl) return 0;

  curl_easy_setopt(g_curl, CURLOPT_POST, 1L);
  curl_easy_setopt(g_curl, CURLOPT_TIMEOUT_MS, 2000L);
  curl_easy_setopt(g_curl, CURLOPT_SSL_VERIFYPEER, 0L);
  curl_easy_setopt(g_curl, CURLOPT_SSL_VERIFYHOST, 0L);

  return 1;
}

static void telemetry_shutdown(void) {
  if (g_curl) curl_easy_cleanup(g_curl);
  curl_global_cleanup();
}

static void post_json(const char* url, const char* json) {
  struct curl_slist* h = NULL;
  h = curl_slist_append(h, "Content-Type: application/json");

  curl_easy_setopt(g_curl, CURLOPT_URL, url);
  curl_easy_setopt(g_curl, CURLOPT_HTTPHEADER, h);
  curl_easy_setopt(g_curl, CURLOPT_POSTFIELDS, json);
  curl_easy_setopt(g_curl, CURLOPT_POSTFIELDSIZE, (long)strlen(json));

  curl_easy_perform(g_curl);
  curl_slist_free_all(h);
}

/* per-move telemetry */
static void send_move(int px, int py, bool won) {
  char json[256];
  snprintf(json, sizeof(json),
    "{\"player\":{\"x\":%d,\"y\":%d},\"won\":%s}",
    px, py, won ? "true" : "false");

  post_json(TELEMETRY_URL_1, json);
  post_json(TELEMETRY_URL_2, json);
}

/* final mission summary */
static void send_mission_summary(
  uint64_t start_ms,
  uint64_t end_ms,
  int left, int right, int straight, int reverse,
  bool success
) {
  uint64_t duration = (end_ms - start_ms) / 1000;

  char json[512];
  snprintf(json, sizeof(json),
    "{"
    "\"robot_id\":\"%s\","
    "\"mission_type\":\"%s\","
    "\"moves_left_turn\":%d,"
    "\"moves_right_turn\":%d,"
    "\"moves_straight\":%d,"
    "\"moves_reverse\":%d,"
    "\"moves_total\":%d,"
    "\"distance_traveled\":%d,"
    "\"duration_seconds\":%" PRIu64 ","
    "\"mission_result\":\"%s\","
    "\"abort_reason\":\"\""
    "}",
    ROBOT_ID,
    MISSION_TYPE,
    left, right, straight, reverse,
    left + right + straight + reverse,
    left + right + straight + reverse,
    duration,
    success ? "success" : "abort"
  );

  post_json(MISSION_SUMMARY_URL, json);
}

/* ================= main ================= */

int main(void) {
  srand((unsigned)time(NULL));
  SDL_Init(SDL_INIT_VIDEO);
  telemetry_init();

  int px = 0, py = 0;
  int left = 0, right = 0, straight = 0, reverse = 0;

  uint64_t mission_start = SDL_GetTicks64();

  maze_init();
  maze_generate(0, 0);

  SDL_Window* w = SDL_CreateWindow(
    "Maze",
    SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
    PAD*2 + MAZE_W*CELL,
    PAD*2 + MAZE_H*CELL,
    SDL_WINDOW_SHOWN
  );

  SDL_Renderer* r = SDL_CreateRenderer(w, -1, SDL_RENDERER_ACCELERATED);

  bool running = true, won = false;

  while (running) {
    SDL_Event e;
    while (SDL_PollEvent(&e)) {
      if (e.type == SDL_QUIT) running = false;

      if (e.type == SDL_KEYDOWN && !won) {
        int ox = px, oy = py;

        if (e.key.keysym.sym == SDLK_UP)    py--, straight++;
        if (e.key.keysym.sym == SDLK_DOWN)  py++, reverse++;
        if (e.key.keysym.sym == SDLK_LEFT)  px--, left++;
        if (e.key.keysym.sym == SDLK_RIGHT) px++, right++;

        if (px != ox || py != oy)
          send_move(px, py, false);

        if (px == MAZE_W - 1 && py == MAZE_H - 1) {
          won = true;
          uint64_t mission_end = SDL_GetTicks64();
          send_move(px, py, true);
          send_mission_summary(
            mission_start, mission_end,
            left, right, straight, reverse,
            true
          );
        }
      }
    }

    SDL_SetRenderDrawColor(r, 20, 20, 20, 255);
    SDL_RenderClear(r);
    SDL_RenderPresent(r);
  }

  SDL_DestroyRenderer(r);
  SDL_DestroyWindow(w);
  telemetry_shutdown();
  SDL_Quit();
  return 0;
}
