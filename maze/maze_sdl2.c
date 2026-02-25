// maze_sdl2.c
// SDL2 maze + HTTPS telemetry + full mission JSON reporting
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

#define TELEMETRY_URL "https://10.170.8.101:8444/move"
#define AI_MISSION_URL "https://10.170.8.109:8444/mission_end"
#define DEVICE_ID "mini-pupper-01"

/* ================= Mission Stats ================= */

static time_t mission_start_time = 0;
static int moves_left = 0;
static int moves_right = 0;
static int moves_straight = 0;
static int moves_reverse = 0;
static int moves_total = 0;
static double distance_traveled = 0.0;
static bool mission_active = false;

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
}

/* ================= Curl ================= */

static CURL* g_curl=NULL;

static int telemetry_init(void){
  if(curl_global_init(CURL_GLOBAL_DEFAULT)!=0) return 0;
  g_curl=curl_easy_init();
  if(!g_curl) return 0;

  curl_easy_setopt(g_curl,CURLOPT_POST,1L);
  curl_easy_setopt(g_curl,CURLOPT_SSL_VERIFYPEER,0L);
  curl_easy_setopt(g_curl,CURLOPT_SSL_VERIFYHOST,0L);
  return 1;
}

static void telemetry_shutdown(void){
  if(g_curl) curl_easy_cleanup(g_curl);
  curl_global_cleanup();
}

/* Telemetry */
static void telemetry_send(int px,int py,bool won){
  if(!g_curl) return;

  uint64_t ts=SDL_GetTicks64();
  char json[256];

  snprintf(json,sizeof(json),
    "{\"device_id\":\"%s\",\"ts_ms\":%" PRIu64
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

  char json[1024];

  snprintf(json,sizeof(json),
    "{"
    "\"team\":\"team2tt\","
    "\"device_id\":\"%s\","
    "\"start_time\":%ld,"
    "\"end_time\":%ld,"
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
    mission_start_time,end_time,
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
}

/* ================= main ================= */

int main(void){
  srand((unsigned)time(NULL));

  if(SDL_Init(SDL_INIT_VIDEO)!=0) return 1;
  telemetry_init();

  int win_w=PAD*2+MAZE_W*CELL;
  int win_h=PAD*2+MAZE_H*CELL;

  SDL_Window* win=SDL_CreateWindow(
    "SDL2 Maze - Reach the green goal (R to regenerate)",
    SDL_WINDOWPOS_CENTERED,SDL_WINDOWPOS_CENTERED,
    win_w,win_h,SDL_WINDOW_SHOWN);

  SDL_Renderer* r=SDL_CreateRenderer(win,-1,
    SDL_RENDERER_ACCELERATED|SDL_RENDERER_PRESENTVSYNC);

  int px=0,py=0;
  regenerate(&px,&py,win);

  bool running=true,won=false;

  while(running){
    SDL_Event e;
    while(SDL_PollEvent(&e)){
      if(e.type==SDL_QUIT){
        if(mission_active)
          ai_mission_send("aborted","window closed");
        running=false;
      }

      if(e.type==SDL_KEYDOWN){
        SDL_Keycode k=e.key.keysym.sym;

        if(k==SDLK_ESCAPE){
          if(mission_active)
            ai_mission_send("aborted","user exited");
          running=false;
        }

        if(k==SDLK_r){
          regenerate(&px,&py,win);
          won=false;
        }

        if(!won){
          bool moved=false;

          if(k==SDLK_UP||k==SDLK_w){moved|=try_move(&px,&py,0,-1); if(moved)moves_straight++;}
          if(k==SDLK_RIGHT||k==SDLK_d){moved|=try_move(&px,&py,1,0); if(moved)moves_right++;}
          if(k==SDLK_DOWN||k==SDLK_s){moved|=try_move(&px,&py,0,1); if(moved)moves_reverse++;}
          if(k==SDLK_LEFT||k==SDLK_a){moved|=try_move(&px,&py,-1,0); if(moved)moves_left++;}

          if(moved){
            moves_total++;
            distance_traveled+=1.0;
            telemetry_send(px,py,won);
          }

          if(px==MAZE_W-1&&py==MAZE_H-1){
            won=true;
            if(mission_active)
              ai_mission_send("success","none");
            SDL_SetWindowTitle(win,"You win! Press R to regenerate");
          }
        }
      }
    }

    draw_maze(r);
    draw_player_goal(r,px,py);
    SDL_RenderPresent(r);
  }

  SDL_DestroyRenderer(r);
  SDL_DestroyWindow(win);
  telemetry_shutdown();
  SDL_Quit();
  return 0;
}
