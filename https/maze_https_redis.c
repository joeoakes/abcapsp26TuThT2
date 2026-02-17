// maze_https_redis.c
// HTTPS telemetry server: POST JSON to /move, store in Redis (localhost).
// Requires libmicrohttpd with TLS + hiredis.

#include <microhttpd.h>
#include <hiredis/hiredis.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <signal.h>
#include <time.h>

#include <cjson/cJSON.h>

#ifndef MHD_HTTP_OK
#define MHD_HTTP_OK 200
#endif

static volatile sig_atomic_t g_stop = 0;

static void on_sigint(int signo) {
  (void)signo;
  g_stop = 1;
}

static enum MHD_Result respond_text(struct MHD_Connection* c, unsigned int code, const char* text) {
  struct MHD_Response* r =
    MHD_create_response_from_buffer(strlen(text), (void*)text, MHD_RESPMEM_MUST_COPY);
  MHD_add_response_header(r, "Content-Type", "text/plain");
  enum MHD_Result ret = MHD_queue_response(c, code, r);
  MHD_destroy_response(r);
  return ret;
}

static enum MHD_Result respond_json(struct MHD_Connection* c, unsigned int code, const char* json) {
  struct MHD_Response* r =
    MHD_create_response_from_buffer(strlen(json), (void*)json, MHD_RESPMEM_MUST_COPY);
  MHD_add_response_header(r, "Content-Type", "application/json");
  enum MHD_Result ret = MHD_queue_response(c, code, r);
  MHD_destroy_response(r);
  return ret;
}

/* ================= Request body buffer ================= */

typedef struct {
  char* data;
  size_t size;
} BodyBuf;

/* ================= Redis context ================= */

typedef struct {
  redisContext* redis;
} RedisCtx;

/* ================= Handle POST /move ================= */

static enum MHD_Result handle_post_move(struct MHD_Connection* c, RedisCtx* rctx, const char* body) {
  if (!body || !*body)
    return respond_text(c, MHD_HTTP_BAD_REQUEST, "Empty body\n");

  cJSON* json = cJSON_Parse(body);
  if (!json)
    return respond_text(c, MHD_HTTP_BAD_REQUEST, "Invalid JSON\n");

  const cJSON* device = cJSON_GetObjectItem(json, "device_id");
  const cJSON* player = cJSON_GetObjectItem(json, "player");
  const cJSON* won    = cJSON_GetObjectItem(json, "won");

  if (!device || !player) {
    cJSON_Delete(json);
    return respond_text(c, MHD_HTTP_BAD_REQUEST, "Missing fields\n");
  }

  int x = cJSON_GetObjectItem(player, "x")->valueint;
  int y = cJSON_GetObjectItem(player, "y")->valueint;

  time_t now = time(NULL);

  /* Write to Redis */
  redisCommand(
    rctx->redis,
    "HSET team2ttmission:LIVE "
    "device_id %s "
    "player_x %d "
    "player_y %d "
    "won %s "
    "last_update %ld",
    device->valuestring,
    x,
    y,
    (won && cJSON_IsTrue(won)) ? "true" : "false",
    (long)now
  );

  cJSON_Delete(json);
  return respond_json(c, MHD_HTTP_OK, "{\"ok\":true}\n");
}

/* ================= Main request handler ================= */

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
  RedisCtx* rctx = (RedisCtx*)cls;

  if (strcmp(url, "/move") != 0)
    return respond_text(c, MHD_HTTP_NOT_FOUND, "Not found\n");

  if (strcmp(method, "GET") == 0)
    return respond_text(c, MHD_HTTP_OK, "POST JSON to /move\n");

  if (strcmp(method, "POST") != 0)
    return respond_text(c, MHD_HTTP_METHOD_NOT_ALLOWED, "Use POST\n");

  if (*con_cls == NULL) {
    BodyBuf* b = calloc(1, sizeof(BodyBuf));
    *con_cls = b;
    return MHD_YES;
  }

  BodyBuf* b = (BodyBuf*)(*con_cls);

  if (*upload_data_size != 0) {
    b->data = realloc(b->data, b->size + *upload_data_size + 1);
    memcpy(b->data + b->size, upload_data, *upload_data_size);
    b->size += *upload_data_size;
    b->data[b->size] = 0;
    *upload_data_size = 0;
    return MHD_YES;
  }

  enum MHD_Result ret = handle_post_move(c, rctx, b->data);
  free(b->data);
  free(b);
  *con_cls = NULL;
  return ret;
}

/* ================= main ================= */

int main(void) {
  signal(SIGINT, on_sigint);
  signal(SIGTERM, on_sigint);

  redisContext* redis = redisConnect("127.0.0.1", 6379);
  if (!redis || redis->err) {
    fprintf(stderr, "Redis error\n");
    return 1;
  }

  RedisCtx rctx = { redis };

  struct MHD_Daemon* d = MHD_start_daemon(
    MHD_USE_INTERNAL_POLLING_THREAD | MHD_USE_TLS,
    8444,
    NULL, NULL,
    &handler, &rctx,
    MHD_OPTION_HTTPS_MEM_KEY,  /* your key */,
    MHD_OPTION_HTTPS_MEM_CERT, /* your cert */,
    MHD_OPTION_END
  );

  if (!d) {
    fprintf(stderr, "Failed to start HTTPS server\n");
    redisFree(redis);
    return 1;
  }

  printf("HTTPS Redis bridge listening on 8444\n");

  while (!g_stop) sleep(1);

  MHD_stop_daemon(d);
  redisFree(redis);
  return 0;
}
