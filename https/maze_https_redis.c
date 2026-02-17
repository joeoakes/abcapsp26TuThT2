// maze_https_redis.c
// HTTPS server that writes mission data to Redis.
// NO JSON parsing. NO telemetry ingestion.

#include <microhttpd.h>
#include <hiredis/hiredis.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <signal.h>
#include <time.h>
#include <unistd.h>

#ifndef MHD_HTTP_OK
#define MHD_HTTP_OK 200
#endif

static volatile sig_atomic_t g_stop = 0;

static void on_sigint(int signo) {
  (void)signo;
  g_stop = 1;
}

/* ================= Responses ================= */

static enum MHD_Result respond_text(
  struct MHD_Connection* c,
  unsigned int code,
  const char* text
) {
  struct MHD_Response* r =
    MHD_create_response_from_buffer(strlen(text), (void*)text, MHD_RESPMEM_MUST_COPY);
  enum MHD_Result ret = MHD_queue_response(c, code, r);
  MHD_destroy_response(r);
  return ret;
}

/* ================= Redis context ================= */

typedef struct {
  redisContext* redis;
} RedisCtx;

/* ================= Request handler ================= */

static enum MHD_Result handler(
  void* cls,
  struct MHD_Connection* c,
  const char* url,
  const char* method,
  const char* version,
  const char* upload_data,
  size_t* upload_data_size,
  void** con_cls
) {
  (void)version;
  (void)upload_data;
  (void)upload_data_size;
  (void)con_cls;

  RedisCtx* rctx = (RedisCtx*)cls;

  if (strcmp(method, "POST") != 0)
    return respond_text(c, MHD_HTTP_METHOD_NOT_ALLOWED, "Use POST\n");

  /* Example endpoint: POST /mission_end */
  if (strcmp(url, "/mission_end") == 0) {

    time_t now = time(NULL);

    redisCommand(
      rctx->redis,
      "HSET team2ttmission:TEST_MISSION "
      "mission_id %s "
      "robot_id %s "
      "mission_result %s "
      "abort_reason %s "
      "end_time %ld",
      "TEST_MISSION",
      "TEST_ROBOT",
      "success",
      "user exited",
      (long)now
    );

    return respond_text(c, MHD_HTTP_OK, "Mission recorded\n");
  }

  return respond_text(c, MHD_HTTP_NOT_FOUND, "Not found\n");
}

/* ================= main ================= */

int main(void) {
  signal(SIGINT, on_sigint);
  signal(SIGTERM, on_sigint);

  /* Redis is local-only (correct) */
  redisContext* redis = redisConnect("10.170.8.109", 6379);
  if (!redis || redis->err) {
    fprintf(stderr, "Redis connection failed\n");
    return 1;
  }

  RedisCtx rctx = { redis };

  struct MHD_Daemon* d = MHD_start_daemon(
    MHD_USE_INTERNAL_POLLING_THREAD | MHD_USE_TLS,
    8444,
    NULL, NULL,
    &handler, &rctx,
    MHD_OPTION_HTTPS_MEM_KEY,  /* TLS key PEM */,
    MHD_OPTION_HTTPS_MEM_CERT, /* TLS cert PEM */,
    MHD_OPTION_END
  );

  if (!d) {
    fprintf(stderr, "Failed to start HTTPS server\n");
    redisFree(redis);
    return 1;
  }

  printf("HTTPS Redis mission server running on port 8444\n");

  while (!g_stop)
    sleep(1);

  MHD_stop_daemon(d);
  redisFree(redis);
  return 0;
}
