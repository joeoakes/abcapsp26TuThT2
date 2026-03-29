// maze_sdl2.c
// SDL2 maze + HTTPS telemetry + full mission JSON reporting + robot control
// DEV ONLY: disables TLS certificate + hostname verification.
// Press P to toggle Mini-Pupper robot control on/off.

#include <SDL2/SDL.h>
#include <stdbool.h>
#include <stdint.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include <curl/curl.h>

#define MAZE_W 21
#define MAZE_H 15
#define CELL   32
#define PAD    16

#define TELEMETRY_URL "https://10.170.8.130:8444/move"
#define AI_MISSION_URL "https://10.170.8.109:8444/mission_end"
#define LOCAL_MISSION_URL "http://localhost:8080/mission_end"
#define DEVICE_ID "mini-pupper-01"

/* Robot control config */
#define DEFAULT_ROBOT_IP "10.170.8.136"
#define ROBOT_PORT 8444
#define ROBOT_CELL_SIZE_M 0.1016        /* 4 inches = 0.1016 meters per cell */
#define ROBOT_MOVE_TIMEOUT_MS 8000      /* Max time for one forward step */
#define ROBOT_ROTATE_TIMEOUT_MS 8000    /* Max time to wait for rotation */
#define ROBOT_HEADING_TOLERANCE 15.0    /* Degrees - close enough to target */
#define ROBOT_HEADING_POLL_MS 100       /* How often to check heading/position */
#define ROBOT_DISTANCE_TOLERANCE 0.02   /* Meters - close enough to target distance */

static char g_robot_url[256] = "";

/* for converting time to a more user-friendly format */
static void format_time(time_t t, char *buf, size_t size) {
    struct tm tm_info;
    localtime_r(&t, &tm_info);
    strftime(buf, size, "%Y-%m-%d %H:%M:%S", &tm_info);
}

/* ================= Mission Stats ================= */

static time_t mission_start_time = 0;
static int moves_left = 0;
static int moves_right = 0;
static int moves_straight = 0;
static int moves_reverse = 0;
static int moves_total = 0;
static double distance_traveled = 0.0;
static bool mission_active = false;

/* ================= Robot Control State ================= */

static bool robot_enabled = false;       /* Toggle with P key */
static int robot_heading = 0;            /* 0=north, 90=east, 180=south, 270=west */
static bool robot_moving = false;        /* Input locked while true */
static bool robot_connected = false;     /* Last connection status */

/* Heading constants */
#define HEADING_NORTH 0
#define HEADING_EAST  90
#define HEADING_SOUTH 180
#define HEADING_WEST  270

/* ================= Maze Data ================= */

enum { WALL_N = 1, WALL_E = 2, WALL_S = 4, WALL_W = 8 };

typedef struct {
  uint8_t walls;
  bool visited;
} Cell;

static Cell g[MAZE_H][MAZE_W];

/* ================= Maze Logic ================= */

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
    P cur = stack[top - 1];
    int x = cur.x, y = cur.y;

    P neigh[4];
    int ncount = 0;

    const int dx[4] = {0,1,0,-1};
    const int dy[4] = {-1,0,1,0};

    for (int i = 0; i < 4; i++) {
      int nx = x + dx[i], ny = y + dy[i];
      if (in_bounds(nx, ny) && !g[ny][nx].visited)
        neigh[ncount++] = (P){nx, ny};
    }

    if (ncount == 0) { top--; continue; }

    int pick = rand() % ncount;
    int nx = neigh[pick].x, ny = neigh[pick].y;

    knock_down(x, y, nx, ny);
    g[ny][nx].visited = true;
    stack[top++] = (P){nx, ny};
  }

  for (int y = 0; y < MAZE_H; y++)
    for (int x = 0; x < MAZE_W; x++)
      g[y][x].visited = false;
}

/* ================= Rendering ================= */

static void draw_maze(SDL_Renderer* r) {
  SDL_SetRenderDrawColor(r, 15,15,18,255);
  SDL_RenderClear(r);

  SDL_SetRenderDrawColor(r, 230,230,230,255);

  int ox = PAD, oy = PAD;

  for (int y=0;y<MAZE_H;y++)
    for (int x=0;x<MAZE_W;x++) {
      int x0 = ox + x*CELL;
      int y0 = oy + y*CELL;
      int x1 = x0 + CELL;
      int y1 = y0 + CELL;

      uint8_t w = g[y][x].walls;

      if (w & WALL_N) SDL_RenderDrawLine(r,x0,y0,x1,y0);
      if (w & WALL_E) SDL_RenderDrawLine(r,x1,y0,x1,y1);
      if (w & WALL_S) SDL_RenderDrawLine(r,x0,y1,x1,y1);
      if (w & WALL_W) SDL_RenderDrawLine(r,x0,y0,x0,y1);
    }
}

static void draw_player_goal(SDL_Renderer* r,int px,int py) {
  int ox=PAD, oy=PAD;

  SDL_Rect goal={ox+(MAZE_W-1)*CELL+6, oy+(MAZE_H-1)*CELL+6, CELL-12, CELL-12};
  SDL_SetRenderDrawColor(r,40,160,70,255);
  SDL_RenderFillRect(r,&goal);

  SDL_Rect p={ox+px*CELL+8, oy+py*CELL+8, CELL-16, CELL-16};
  SDL_SetRenderDrawColor(r,255,255,0,255);
  SDL_RenderFillRect(r,&p);
}

/* Draw robot status indicator */
static void draw_robot_status(SDL_Renderer* r, int win_w, int win_h) {
  if (!robot_enabled) return;

  /* Background bar at bottom */
  SDL_Rect bar = {0, win_h - 24, win_w, 24};
  SDL_SetRenderDrawColor(r, 10, 10, 30, 255);
  SDL_RenderFillRect(r, &bar);

  /* Robot enabled indicator - green dot */
  SDL_Rect dot = {6, win_h - 18, 12, 12};
  if (robot_connected) {
    SDL_SetRenderDrawColor(r, 57, 255, 20, 255);  /* green */
  } else {
    SDL_SetRenderDrawColor(r, 255, 45, 85, 255);  /* red */
  }
  SDL_RenderFillRect(r, &dot);

  /* Heading indicator arrow */
  int arrow_cx = win_w / 2;
  int arrow_cy = win_h - 12;
  int arrow_len = 8;

  SDL_SetRenderDrawColor(r, 0, 245, 255, 255);  /* cyan */

  int dx = 0, dy = 0;
  switch (robot_heading) {
    case HEADING_NORTH: dy = -arrow_len; break;
    case HEADING_EAST:  dx = arrow_len; break;
    case HEADING_SOUTH: dy = arrow_len; break;
    case HEADING_WEST:  dx = -arrow_len; break;
  }
  SDL_RenderDrawLine(r, arrow_cx, arrow_cy, arrow_cx + dx, arrow_cy + dy);

  /* If robot is moving, show a yellow bar */
  if (robot_moving) {
    SDL_Rect moving_bar = {win_w - 60, win_h - 18, 50, 12};
    SDL_SetRenderDrawColor(r, 255, 190, 0, 255);  /* amber */
    SDL_RenderFillRect(r, &moving_bar);
  }
}

/* ================= Movement ================= */

static bool try_move(int* px,int* py,int dx,int dy){
  int x=*px,y=*py;
  int nx=x+dx,ny=y+dy;
  if(!in_bounds(nx,ny)) return false;

  uint8_t w=g[y][x].walls;

  if(dx==0&&dy==-1&&(w&WALL_N))return false;
  if(dx==1&&dy==0&&(w&WALL_E))return false;
  if(dx==0&&dy==1&&(w&WALL_S))return false;
  if(dx==-1&&dy==0&&(w&WALL_W))return false;

  *px=nx; *py=ny;
  return true;
}

static void regenerate(int* px,int* py,SDL_Window* win){
  maze_init();
  maze_generate(0,0);
  *px=0; *py=0;
  SDL_SetWindowTitle(win,"SDL2 Maze - Reach the green goal (R to regenerate)");

  mission_start_time=time(NULL);
  mission_active=true;
  moves_left=moves_right=moves_straight=moves_reverse=0;
  moves_total=0;
  distance_traveled=0.0;

  /* Reset robot heading on new maze */
  robot_heading = HEADING_NORTH;
  robot_moving = false;
}

/* ================= Curl ================= */

static CURL* g_curl=NULL;

/* Suppress curl response body */
static size_t write_devnull(void* p, size_t s, size_t n, void* u) {
  (void)p; (void)u;
  return s * n;
}

static int telemetry_init(void){
  if(curl_global_init(CURL_GLOBAL_DEFAULT)!=0) return 0;
  g_curl=curl_easy_init();
  if(!g_curl) return 0;

  curl_easy_setopt(g_curl,CURLOPT_POST,1L);
  curl_easy_setopt(g_curl,CURLOPT_SSL_VERIFYPEER,0L);
  curl_easy_setopt(g_curl,CURLOPT_SSL_VERIFYHOST,0L);
  curl_easy_setopt(g_curl,CURLOPT_TIMEOUT,2L);
  curl_easy_setopt(g_curl,CURLOPT_CONNECTTIMEOUT,1L);
  curl_easy_setopt(g_curl,CURLOPT_WRITEFUNCTION,write_devnull);
  return 1;
}

static void telemetry_shutdown(void){
  if(g_curl) curl_easy_cleanup(g_curl);
  curl_global_cleanup();
}

/* Telemetry */
static void telemetry_send(int px,int py,bool won){
  if(!g_curl) return;

  uint32_t ts=SDL_GetTicks();
  char json[256];

  snprintf(json,sizeof(json),
    "{\"device_id\":\"%s\",\"ts_ms\":%" PRIu32
    ",\"player\":{\"x\":%d,\"y\":%d},\"won\":%s}",
    DEVICE_ID,ts,px,py,won?"true":"false");

  struct curl_slist* headers=NULL;
  headers=curl_slist_append(headers,"Content-Type: application/json");

  curl_easy_setopt(g_curl,CURLOPT_URL,TELEMETRY_URL);
  curl_easy_setopt(g_curl,CURLOPT_HTTPHEADER,headers);
  curl_easy_setopt(g_curl,CURLOPT_POSTFIELDS,json);

  curl_easy_perform(g_curl);
  curl_slist_free_all(headers);
}

/* Mission JSON */
static void ai_mission_send(const char* result,const char* abort_reason){
  if(!g_curl) return;

  time_t end_time=time(NULL);
  int duration=(int)difftime(end_time,mission_start_time);

  char start_str[64];
  char end_str[64];
  
  format_time(mission_start_time, start_str, sizeof(start_str));
  format_time(end_time, end_str, sizeof(end_str));

  char json[1024];

  snprintf(json,sizeof(json),
    "{"
    "\"team\":\"team2tt\","
    "\"device_id\":\"%s\","
    "\"start_time\":\"%s\","
    "\"end_time\":\"%s\","
    "\"moves_left_turn\":%d,"
    "\"moves_right_turn\":%d,"
    "\"moves_straight\":%d,"
    "\"moves_reverse\":%d,"
    "\"moves_total\":%d,"
    "\"distance_traveled\":%.2f,"
    "\"duration_seconds\":%d,"
    "\"mission_result\":\"%s\","
    "\"abort_reason\":\"%s\""
    "}",
    DEVICE_ID,
    start_str,end_str,
    moves_left,moves_right,moves_straight,moves_reverse,
    moves_total,distance_traveled,duration,
    result,abort_reason);

  struct curl_slist* headers=NULL;
  headers=curl_slist_append(headers,"Content-Type: application/json");

  curl_easy_setopt(g_curl,CURLOPT_URL,AI_MISSION_URL);
  curl_easy_setopt(g_curl,CURLOPT_HTTPHEADER,headers);
  curl_easy_setopt(g_curl,CURLOPT_POSTFIELDS,json);

  curl_easy_perform(g_curl);
  curl_slist_free_all(headers);

  /* Also send to local dashboard server */
  headers = curl_slist_append(NULL, "Content-Type: application/json");
  curl_easy_setopt(g_curl, CURLOPT_URL, LOCAL_MISSION_URL);
  curl_easy_setopt(g_curl, CURLOPT_HTTPHEADER, headers);
  curl_easy_setopt(g_curl, CURLOPT_POSTFIELDS, json);
  curl_easy_perform(g_curl);
  curl_slist_free_all(headers);
}

/* ================= Robot Control ================= */

/* Send a single move command to the robot */
static void robot_send_cmd(const char* move_dir) {
  if (!g_curl || !robot_enabled) return;

  char json[128];
  snprintf(json, sizeof(json), "{\"move_dir\":\"%s\"}", move_dir);

  struct curl_slist* headers = NULL;
  headers = curl_slist_append(headers, "Content-Type: application/json");

  curl_easy_setopt(g_curl, CURLOPT_URL, g_robot_url);
  curl_easy_setopt(g_curl, CURLOPT_HTTPHEADER, headers);
  curl_easy_setopt(g_curl, CURLOPT_POSTFIELDS, json);

  CURLcode res = curl_easy_perform(g_curl);
  robot_connected = (res == CURLE_OK);

  curl_slist_free_all(headers);
}

/* Calculate shortest rotation direction between two headings */
/* Returns positive for clockwise, negative for counterclockwise */
static double heading_diff_d(double from, double to) {
  double diff = to - from;
  while (diff > 180.0) diff -= 360.0;
  while (diff < -180.0) diff += 360.0;
  return diff;
}

/* Buffer for overhead camera response */
typedef struct {
  char data[1024];
  size_t size;
} ResponseBuf;

static size_t response_write(void* contents, size_t size, size_t nmemb, void* userp) {
  ResponseBuf* buf = (ResponseBuf*)userp;
  size_t total = size * nmemb;
  if (buf->size + total < sizeof(buf->data) - 1) {
    memcpy(buf->data + buf->size, contents, total);
    buf->size += total;
    buf->data[buf->size] = '\0';
  }
  return total;
}

/* Fetch current robot odom (heading + position).
   Returns true if successful. */
static bool robot_get_odom(double* out_yaw, double* out_x, double* out_y) {
  if (!g_curl) return false;

  /* Build odom URL from robot URL: same host, /odom instead of /move */
  char odom_url[256];
  strncpy(odom_url, g_robot_url, sizeof(odom_url));
  char* last_slash = strrchr(odom_url, '/');
  if (last_slash) {
    strcpy(last_slash, "/odom");
  } else {
    return false;
  }

  ResponseBuf resp = { .size = 0 };
  resp.data[0] = '\0';

  CURL* odom_curl = curl_easy_init();
  if (!odom_curl) return false;

  curl_easy_setopt(odom_curl, CURLOPT_URL, odom_url);
  curl_easy_setopt(odom_curl, CURLOPT_HTTPGET, 1L);
  curl_easy_setopt(odom_curl, CURLOPT_TIMEOUT, 1L);
  curl_easy_setopt(odom_curl, CURLOPT_CONNECTTIMEOUT, 1L);
  curl_easy_setopt(odom_curl, CURLOPT_SSL_VERIFYPEER, 0L);
  curl_easy_setopt(odom_curl, CURLOPT_SSL_VERIFYHOST, 0L);
  curl_easy_setopt(odom_curl, CURLOPT_WRITEFUNCTION, response_write);
  curl_easy_setopt(odom_curl, CURLOPT_WRITEDATA, &resp);

  CURLcode res = curl_easy_perform(odom_curl);
  curl_easy_cleanup(odom_curl);

  if (res != CURLE_OK) return false;

  /* Check valid */
  char* vis = strstr(resp.data, "\"valid\"");
  if (!vis) return false;
  if (!strstr(vis, "true")) return false;

  /* Parse yaw */
  char* yaw_str = strstr(resp.data, "\"yaw\"");
  if (!yaw_str) return false;
  yaw_str = strchr(yaw_str, ':');
  if (!yaw_str) return false;
  if (out_yaw) *out_yaw = strtod(yaw_str + 1, NULL);

  /* Parse x */
  char* x_str = strstr(resp.data, "\"x\"");
  if (!x_str) return false;
  x_str = strchr(x_str, ':');
  if (!x_str) return false;
  if (out_x) *out_x = strtod(x_str + 1, NULL);

  /* Parse y */
  char* y_str = strstr(resp.data, "\"y\"");
  if (!y_str) return false;
  y_str = strchr(y_str, ':');
  if (!y_str) return false;
  if (out_y) *out_y = strtod(y_str + 1, NULL);

  return true;
}

/* Rotate robot to target heading using odom feedback.
   Falls back to fixed-time rotation if odom is unavailable. */
static void robot_rotate_to(int target_heading) {
  double current_yaw, dummy_x, dummy_y;

  /* Try odom-guided rotation */
  if (robot_get_odom(&current_yaw, &dummy_x, &dummy_y)) {
    Uint32 start_time = SDL_GetTicks();

    while (SDL_GetTicks() - start_time < ROBOT_ROTATE_TIMEOUT_MS) {
      double diff = heading_diff_d(current_yaw, (double)target_heading);

      /* Close enough? */
      if (fabs(diff) < ROBOT_HEADING_TOLERANCE) {
        robot_send_cmd("stop");
        printf("[ROBOT] Reached heading %d (actual: %.1f)\n", target_heading, current_yaw);
        return;
      }

      /* Send rotation command */
      if (diff > 0) {
        robot_send_cmd("right");
      } else {
        robot_send_cmd("left");
      }

      SDL_Delay(ROBOT_HEADING_POLL_MS);

      /* Re-check heading */
      if (!robot_get_odom(&current_yaw, &dummy_x, &dummy_y)) {
        robot_send_cmd("stop");
        printf("[ROBOT] Lost odom during rotation\n");
        SDL_Delay(200);
        return;
      }
    }

    /* Timeout */
    robot_send_cmd("stop");
    printf("[ROBOT] Rotation timeout (target: %d, actual: %.1f)\n", target_heading, current_yaw);
    return;
  }

  /* Fallback: fixed-time rotation (no odom available) */
  printf("[ROBOT] No odom available - using fixed-time rotation\n");
  int diff = (int)heading_diff_d((double)robot_heading, (double)target_heading);

  if (diff == 0) return;

  int abs_diff = abs(diff);
  int rotate_ms = (abs_diff * 3000) / 90;

  if (diff > 0) {
    robot_send_cmd("right");
  } else {
    robot_send_cmd("left");
  }

  SDL_Delay(rotate_ms);
  robot_send_cmd("stop");
  SDL_Delay(200);
}

/* Walk forward one cell using odom distance feedback.
   Falls back to time-based if odom unavailable. */
static void robot_walk_one_cell(void) {
  double start_yaw, start_x, start_y;

  /* Try distance-based movement */
  if (robot_get_odom(&start_yaw, &start_x, &start_y)) {
    Uint32 start_time = SDL_GetTicks();

    robot_send_cmd("forward");

    while (SDL_GetTicks() - start_time < ROBOT_MOVE_TIMEOUT_MS) {
      SDL_Delay(ROBOT_HEADING_POLL_MS);

      double cur_yaw, cur_x, cur_y;
      if (!robot_get_odom(&cur_yaw, &cur_x, &cur_y)) {
        robot_send_cmd("stop");
        printf("[ROBOT] Lost odom during walk\n");
        return;
      }

      double dx = cur_x - start_x;
      double dy = cur_y - start_y;
      double dist = sqrt(dx * dx + dy * dy);

      if (dist >= ROBOT_CELL_SIZE_M - ROBOT_DISTANCE_TOLERANCE) {
        robot_send_cmd("stop");
        printf("[ROBOT] Walked %.3fm (target: %.3fm)\n", dist, ROBOT_CELL_SIZE_M);
        return;
      }
    }

    /* Timeout */
    robot_send_cmd("stop");
    printf("[ROBOT] Walk timeout\n");
    return;
  }

  /* Fallback: time-based (no odom) */
  printf("[ROBOT] No odom - using time-based walk\n");
  robot_send_cmd("forward");
  SDL_Delay(1500);
  robot_send_cmd("stop");
}

/* Execute a maze move: rotate to target heading then walk one cell.
   Blocks until complete. */
static void robot_execute_move(int target_heading) {
  if (!robot_enabled) return;

  robot_moving = true;

  /* Rotate to face the correct direction */
  if (robot_heading != target_heading) {
    robot_rotate_to(target_heading);
  }

  /* Walk forward one cell */
  robot_walk_one_cell();

  /* Update heading */
  robot_heading = target_heading;

  robot_moving = false;
}

/* Robot update is now a no-op since movement blocks until complete */
static void robot_update(void) {
  /* Movement is handled synchronously in robot_execute_move */
}

/* ================= main ================= */

int main(int argc, char* argv[]){
  srand((unsigned)time(NULL));

  /* Get robot IP from argument or environment or default */
  const char* robot_ip = DEFAULT_ROBOT_IP;
  if (argc > 1) {
    robot_ip = argv[1];
  } else {
    const char* env_ip = getenv("PUPPER_IP");
    if (env_ip && *env_ip) robot_ip = env_ip;
  }
  snprintf(g_robot_url, sizeof(g_robot_url), "https://%s:%d/move", robot_ip, ROBOT_PORT);

  printf("[MAZE] Robot URL: %s\n", g_robot_url);
  printf("[MAZE] Robot heading via odom: https://%s:%d/odom\n", robot_ip, ROBOT_PORT);
  printf("[MAZE] Press P to toggle robot control (currently OFF)\n");

  if(SDL_Init(SDL_INIT_VIDEO)!=0) return 1;
  telemetry_init();

  int win_w=PAD*2+MAZE_W*CELL;
  int win_h=PAD*2+MAZE_H*CELL;

  SDL_Window* win=SDL_CreateWindow(
    "SDL2 Maze - Reach the green goal (R to regenerate, P for robot)",
    SDL_WINDOWPOS_CENTERED,SDL_WINDOWPOS_CENTERED,
    win_w,win_h,SDL_WINDOW_SHOWN);

  SDL_Renderer* r=SDL_CreateRenderer(win,-1,
    SDL_RENDERER_ACCELERATED|SDL_RENDERER_PRESENTVSYNC);

  int px=0,py=0;
  regenerate(&px,&py,win);

  bool running=true,won=false;

  while(running){
    /* Update robot state (check if movement is done) */
    robot_update();

    SDL_Event e;
    while(SDL_PollEvent(&e)){
      if(e.type==SDL_QUIT){
        if(mission_active)
          ai_mission_send("aborted","window closed");
        if(robot_enabled) robot_send_cmd("stop");
        running=false;
      }

      if(e.type==SDL_KEYDOWN){
        SDL_Keycode k=e.key.keysym.sym;

        if(k==SDLK_ESCAPE || k==SDLK_q){
          if(mission_active)
            ai_mission_send("aborted","user exited");
          if(robot_enabled) robot_send_cmd("stop");
          running=false;
        }

        /* Toggle robot control */
        if(k==SDLK_p){
          robot_enabled = !robot_enabled;
          if(robot_enabled){
            printf("[MAZE] Robot control ENABLED\n");
            robot_heading = HEADING_NORTH;
          } else {
            printf("[MAZE] Robot control DISABLED\n");
            robot_send_cmd("stop");
          }
        }

        if(k==SDLK_r){
          if(robot_enabled) robot_send_cmd("stop");
          regenerate(&px,&py,win);
          won=false;
        }

        /* Only allow movement if not won and robot isn't mid-move */
        if(!won && !robot_moving){
          bool moved=false;
          int target_heading = -1;

          if(k==SDLK_UP||k==SDLK_w){
            moved=try_move(&px,&py,0,-1);
            if(moved){ moves_straight++; target_heading=HEADING_NORTH; }
          }
          if(k==SDLK_RIGHT||k==SDLK_d){
            moved=try_move(&px,&py,1,0);
            if(moved){ moves_right++; target_heading=HEADING_EAST; }
          }
          if(k==SDLK_DOWN||k==SDLK_s){
            moved=try_move(&px,&py,0,1);
            if(moved){ moves_reverse++; target_heading=HEADING_SOUTH; }
          }
          if(k==SDLK_LEFT||k==SDLK_a){
            moved=try_move(&px,&py,-1,0);
            if(moved){ moves_left++; target_heading=HEADING_WEST; }
          }

          if(moved){
            moves_total++;
            distance_traveled+=1.0;
            telemetry_send(px,py,won);

            /* Send movement to robot */
            if(robot_enabled && target_heading >= 0){
              robot_execute_move(target_heading);
            }
          }

          if(px==MAZE_W-1&&py==MAZE_H-1){
            won=true;
            if(mission_active){
              ai_mission_send("success","none");
              mission_active=false;
            }
            if(robot_enabled) robot_send_cmd("stop");
            SDL_SetWindowTitle(win,"You win! Press R to regenerate");
          }
        }
      }
    }

    draw_maze(r);
    draw_player_goal(r,px,py);
    draw_robot_status(r, win_w, win_h);
    SDL_RenderPresent(r);
  }

  /* Stop robot before cleanup */
  if(robot_enabled) robot_send_cmd("stop");

  SDL_DestroyRenderer(r);
  SDL_DestroyWindow(win);
  telemetry_shutdown();
  SDL_Quit();
  system("xinit /usr/bin/chromium-browser --kiosk --no-sandbox --disable-infobars http://localhost:8080/ -- :0");
  return 0;
}
