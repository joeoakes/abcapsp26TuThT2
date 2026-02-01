// missions/mission_dashboard.c
// Mission Dashboard (terminal) for Mini-Pupper / Maze missions.
//
// Purpose:
//   - Launched by the Maze SDL2 app when Left Trigger / L button is pressed.
//   - Reads mission summary from Redis hash: mission:{mission_id}:summary
//   - Prints a clean "mission report" to the terminal.
//
// Build (with Redis support):
//   sudo apt-get install -y gcc make libhiredis-dev
//   make
//
// Build (without Redis; prints placeholders):
//   make NO_REDIS=1
//
// Run:
//   ./mission_dashboard <mission_id> [redis_host] [redis_port]
//
// Example:
//   ./mission_dashboard 2f1c0b5d-9d2a-4d8b-b5ad-2d7c6a0fd6b3 127.0.0.1 6379
//
// Notes:
//   - Keep this in a subfolder: ./missions/mission_dashboard
//   - Maze should launch via execl("./missions/mission_dashboard", "mission_dashboard", mission_id, NULL);

#define _POSIX_C_SOURCE 200809L

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifndef NO_REDIS
  #include <hiredis/hiredis.h>
#endif

static void print_usage(const char* argv0) {
  fprintf(stderr,
    "Usage: %s <mission_id> [redis_host] [redis_port]\n"
    "  mission_id   UUID for the mission\n"
    "  redis_host   default: 127.0.0.1\n"
    "  redis_port   default: 6379\n"
    "\n"
    "Reads Redis hash: mission:{mission_id}:summary\n",
    argv0
  );
}

static const char* safe_s(const char* s) { return (s && *s) ? s : "(none)"; }

static void print_header(void) {
  printf("============================================================\n");
  printf("                  MINI-PUPPER MISSION REPORT\n");
  printf("============================================================\n");
}

static void print_kv(const char* k, const char* v) {
  printf("%-20s : %s\n", k, safe_s(v));
}

static void print_footer(void) {
  printf("============================================================\n");
}

#ifndef NO_REDIS
// Fetch a single field from a hash. Returns malloc'd string or NULL.
static char* hget(redisContext* c, const char* key, const char* field) {
  redisReply* r = (redisReply*)redisCommand(c, "HGET %s %s", key, field);
  if (!r) return NULL;
  char* out = NULL;
  if (r->type == REDIS_REPLY_STRING) {
    out = strdup(r->str);
  }
  freeReplyObject(r);
  return out;
}
#endif

int main(int argc, char** argv) {
  if (argc < 2) {
    print_usage(argv[0]);
    return 2;
  }

  const char* mission_id = argv[1];
  const char* host = (argc >= 3) ? argv[2] : "127.0.0.1";
  int port = (argc >= 4) ? atoi(argv[3]) : 6379;

  char key[256];
  snprintf(key, sizeof(key), "mission:%s:summary", mission_id);

  print_header();
  print_kv("mission_id", mission_id);

#ifdef NO_REDIS
  (void)host; (void)port; (void)key;
  print_kv("robot_id", "(redis disabled)");
  print_kv("mission_type", "(redis disabled)");
  print_kv("start_time", "(redis disabled)");
  print_kv("end_time", "(redis disabled)");
  print_kv("moves_left_turn", "(redis disabled)");
  print_kv("moves_right_turn", "(redis disabled)");
  print_kv("moves_straight", "(redis disabled)");
  print_kv("moves_reverse", "(redis disabled)");
  print_kv("moves_total", "(redis disabled)");
  print_kv("distance_traveled", "(redis disabled)");
  print_kv("duration_seconds", "(redis disabled)");
  print_kv("mission_result", "(redis disabled)");
  print_kv("abort_reason", "(redis disabled)");
  print_footer();
  return 0;
#else
  struct timeval tv;
  tv.tv_sec = 2;
  tv.tv_usec = 0;

  redisContext* c = redisConnectWithTimeout(host, port, tv);
  if (!c || c->err) {
    fprintf(stderr, "ERROR: Redis connection failed to %s:%d\n", host, port);
    if (c && c->errstr) fprintf(stderr, "  %s\n", c->errstr);
    if (c) redisFree(c);

    printf("\nNOTE: If Redis isn't running, start it:\n");
    printf("  sudo service redis-server start\n");
    printf("or run:\n");
    printf("  redis-server\n");
    print_footer();
    return 1;
  }

  // Pull fields (matches the schema you described).
  char* robot_id          = hget(c, key, "robot_id");
  char* mission_type      = hget(c, key, "mission_type");
  char* start_time        = hget(c, key, "start_time");
  char* end_time          = hget(c, key, "end_time");
  char* m_left            = hget(c, key, "moves_left_turn");
  char* m_right           = hget(c, key, "moves_right_turn");
  char* m_straight        = hget(c, key, "moves_straight");
  char* m_reverse         = hget(c, key, "moves_reverse");
  char* m_total           = hget(c, key, "moves_total");
  char* distance          = hget(c, key, "distance_traveled");
  char* duration_seconds  = hget(c, key, "duration_seconds");
  char* mission_result    = hget(c, key, "mission_result");
  char* abort_reason      = hget(c, key, "abort_reason");

  // If the key doesn't exist, Redis will return nil for fields. Give a helpful message.
  if (!robot_id && !mission_type && !start_time && !mission_result) {
    printf("\nWARNING: No mission data found in Redis for:\n");
    printf("  %s\n\n", key);
    printf("Expected fields include: robot_id, mission_type, start_time, ...\n");
    printf("If you haven't logged the mission yet, add it from the Maze app.\n\n");
  }

  print_kv("robot_id", robot_id);
  print_kv("mission_type", mission_type);
  print_kv("start_time", start_time);
  print_kv("end_time", end_time);
  print_kv("moves_left_turn", m_left);
  print_kv("moves_right_turn", m_right);
  print_kv("moves_straight", m_straight);
  print_kv("moves_reverse", m_reverse);
  print_kv("moves_total", m_total);
  print_kv("distance_traveled", distance);
  print_kv("duration_seconds", duration_seconds);
  print_kv("mission_result", mission_result);
  print_kv("abort_reason", abort_reason);

  print_footer();

  free(robot_id);
  free(mission_type);
  free(start_time);
  free(end_time);
  free(m_left);
  free(m_right);
  free(m_straight);
  free(m_reverse);
  free(m_total);
  free(distance);
  free(duration_seconds);
  free(mission_result);
  free(abort_reason);

  redisFree(c);
  return 0;
#endif
}
