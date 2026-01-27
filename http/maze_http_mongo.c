/*
 * maze_http_mongo.c
 *
 * Minimal HTTP JSON receiver in C that writes documents into MongoDB.
 *
 * - HTTP server: libmicrohttpd
 * - MongoDB: mongo-c-driver (libmongoc + libbson)
 *
 * Endpoint:
 *   POST /move   (Content-Type: application/json)
 *   Body: JSON document (your telemetry payload)
 *
 * Env vars (optional):
 *   MONGO_URI= mongodb://localhost:27017
 *   MONGO_DB=  maze
 *   MONGO_COL= moves
 *   LISTEN_PORT=8080
 *
 * Build (Linux/Raspberry Pi OS/Ubuntu):
 *   sudo apt-get install -y libmicrohttpd-dev libmongoc-dev libbson-dev pkg-config
 *   gcc -O2 -Wall -Wextra -std=c11 maze_http_mongo.c -o maze_http_mongo \
 *       $(pkg-config --cflags --libs libmicrohttpd libmongoc-1.0)
 *
 * Run:
 *   ./maze_http_mongo
 *
 * Test:
 *   curl -sS -X POST http://localhost:8080/move \
 *     -H "Content-Type: application/json" \
 *     -d '{"event_type":"player_move","input":{"device":"joystick","move_sequence":1},"player":{"position":{"x":1,"y":2}},"goal_reached":false,"timestamp":"2026-01-25T11:42:18Z"}'
 */

#include <microhttpd.h>
#include <mongoc/mongoc.h>
#include <bson/bson.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <signal.h>
#include <time.h>

#ifndef MHD_HTTP_OK
#define MHD_HTTP_OK 200
#endif

static volatile sig_atomic_t g_stop = 0;

static void on_sigint(int signo) {
  (void)signo;
  g_stop = 1;
}

static const char* getenv_or(const char* k, const char* defv) {
  const char* v = getenv(k);
  return (v && *v) ? v : defv;
}

static int respond_text(struct MHD_Connection* connection, unsigned int status, const char* text) {
  struct MHD_Response* response = MHD_create_response_from_buffer(
      strlen(text), (void*)text, MHD_RESPMEM_MUST_COPY);
  if (!response) return MHD_NO;
  MHD_add_response_header(response, "Content-Type", "text/plain; charset=utf-8");
  int ret = MHD_queue_response(connection, status, response);
  MHD_destroy_response(response);
  return ret;
}

static int respond_json(struct MHD_Connection* connection, unsigned int status, const char* json) {
  struct MHD_Response* response = MHD_create_response_from_buffer(
      strlen(json), (void*)json, MHD_RESPMEM_MUST_COPY);
  if (!response) return MHD_NO;
  MHD_add_response_header(response, "Content-Type", "application/json; charset=utf-8");
  int ret = MHD_queue_response(connection, status, response);
  MHD_destroy_response(response);
  return ret;
}

typedef struct {
  char* data;
  size_t size;
  size_t cap;
} BodyBuf;

static void bodybuf_free(BodyBuf* b) {
  if (!b) return;
  free(b->data);
  b->data = NULL;
  b->size = b->cap = 0;
}

static bool bodybuf_append(BodyBuf* b, const char* chunk, size_t chunk_size) {
  if (chunk_size == 0) return true;
  if (b->size + chunk_size + 1 > b->cap) {
    size_t newcap = b->cap ? b->cap : 1024;
    while (newcap < b->size + chunk_size + 1) newcap *= 2;
    char* nd = (char*)realloc(b->data, newcap);
    if (!nd) return false;
    b->data = nd;
    b->cap = newcap;
  }
  memcpy(b->data + b->size, chunk, chunk_size);
  b->size += chunk_size;
  b->data[b->size] = '\0';
  return true;
}

typedef struct {
  mongoc_client_t* client;
  mongoc_collection_t* col;
} MongoCtx;

static bson_t* json_to_bson_with_received_at(const char* json, bson_error_t* err) {
  // Parse incoming JSON into BSON
  bson_t* doc = bson_new_from_json((const uint8_t*)json, -1, err);
  if (!doc) return NULL;

  // Add received_at as an ISO8601 string in UTC (simple + human-friendly)
  time_t now = time(NULL);
  struct tm tmbuf;
#if defined(_WIN32)
  gmtime_s(&tmbuf, &now);
#else
  gmtime_r(&now, &tmbuf);
#endif
  char ts[32];
  strftime(ts, sizeof(ts), "%Y-%m-%dT%H:%M:%SZ", &tmbuf);

  BSON_APPEND_UTF8(doc, "received_at", ts);
  return doc;
}

static int handle_post_move(struct MHD_Connection* connection, MongoCtx* mctx, const char* body) {
  if (!body || !*body) {
    return respond_text(connection, MHD_HTTP_BAD_REQUEST, "Empty request body\n");
  }

  bson_error_t err;
  bson_t* doc = json_to_bson_with_received_at(body, &err);
  if (!doc) {
    char msg[512];
    snprintf(msg, sizeof(msg), "Invalid JSON: %s\n", err.message);
    return respond_text(connection, MHD_HTTP_BAD_REQUEST, msg);
  }

  bson_t reply;
  bson_init(&reply);

  bool ok = mongoc_collection_insert_one(mctx->col, doc, NULL, &reply, &err);
  bson_destroy(doc);

  if (!ok) {
    bson_destroy(&reply);
    char msg[512];
    snprintf(msg, sizeof(msg), "MongoDB insert failed: %s\n", err.message);
    return respond_text(connection, MHD_HTTP_INTERNAL_SERVER_ERROR, msg);
  }

  // Return the insertedId if present (MongoDB returns it in the reply)
  const bson_value_t* insertedId = NULL;
  bson_iter_t it;
  if (bson_iter_init_find(&it, &reply, "insertedId")) {
    insertedId = bson_iter_value(&it);
  }

  char out[512];
  if (insertedId && insertedId->value_type == BSON_TYPE_OID) {
    char oidstr[25];
    bson_oid_to_string(&insertedId->value.v_oid, oidstr);
    snprintf(out, sizeof(out), "{\"ok\":true,\"inserted_id\":\"%s\"}\n", oidstr);
  } else {
    snprintf(out, sizeof(out), "{\"ok\":true}\n");
  }

  bson_destroy(&reply);
  return respond_json(connection, MHD_HTTP_OK, out);
}

static int request_handler(void* cls,
                           struct MHD_Connection* connection,
                           const char* url,
                           const char* method,
                           const char* version,
                           const char* upload_data,
                           size_t* upload_data_size,
                           void** con_cls) {
  (void)version;
  MongoCtx* mctx = (MongoCtx*)cls;

  // Only one endpoint for simplicity
  const bool is_move = (0 == strcmp(url, "/move"));

  if (0 == strcmp(method, "GET")) {
    if (!is_move) return respond_text(connection, MHD_HTTP_NOT_FOUND, "Not found\n");
    return respond_text(connection, MHD_HTTP_OK, "POST JSON to /move\n");
  }

  if (0 != strcmp(method, "POST")) {
    return respond_text(connection, MHD_HTTP_METHOD_NOT_ALLOWED, "Use POST\n");
  }

  if (!is_move) {
    return respond_text(connection, MHD_HTTP_NOT_FOUND, "Not found\n");
  }

  // First call: create per-connection buffer
  if (*con_cls == NULL) {
    BodyBuf* b = (BodyBuf*)calloc(1, sizeof(BodyBuf));
    if (!b) return MHD_NO;
    *con_cls = (void*)b;
    return MHD_YES;
  }

  BodyBuf* b = (BodyBuf*)(*con_cls);

  // Collect upload data in chunks
  if (*upload_data_size != 0) {
    if (!bodybuf_append(b, upload_data, *upload_data_size)) {
      bodybuf_free(b);
      free(b);
      *con_cls = NULL;
      return respond_text(connection, MHD_HTTP_INTERNAL_SERVER_ERROR, "Out of memory\n");
    }
    *upload_data_size = 0; // tell MHD we consumed this chunk
    return MHD_YES;
  }

  // upload_data_size == 0 => request finished, process body
  int ret = handle_post_move(connection, mctx, b->data ? b->data : "");

  bodybuf_free(b);
  free(b);
  *con_cls = NULL;

  return ret;
}

int main(void) {
  signal(SIGINT, on_sigint);
  signal(SIGTERM, on_sigint);

  const char* mongo_uri = getenv_or("MONGO_URI", "mongodb://localhost:27017");
  const char* dbname = getenv_or("MONGO_DB", "maze");
  const char* colname = getenv_or("MONGO_COL", "moves");
  const char* port_s = getenv_or("LISTEN_PORT", "8080");
  int port = atoi(port_s);
  if (port <= 0 || port > 65535) port = 8080;

  mongoc_init();

  bson_error_t err;
  mongoc_client_t* client = mongoc_client_new(mongo_uri);
  if (!client) {
    fprintf(stderr, "Failed to create MongoDB client from URI: %s\n", mongo_uri);
    mongoc_cleanup();
    return 1;
  }

  // Basic ping to surface connectivity issues early
  bson_t ping_cmd;
  bson_init(&ping_cmd);
  BSON_APPEND_INT32(&ping_cmd, "ping", 1);

  bson_t ping_reply;
  bool ping_ok = mongoc_client_command_simple(
      client, "admin", &ping_cmd, NULL, &ping_reply, &err);
  bson_destroy(&ping_cmd);

  if (!ping_ok) {
    fprintf(stderr, "MongoDB ping failed: %s\n", err.message);
    mongoc_client_destroy(client);
    mongoc_cleanup();
    return 1;
  }
  bson_destroy(&ping_reply);

  mongoc_collection_t* col = mongoc_client_get_collection(client, dbname, colname);

  MongoCtx mctx = { client, col };

  struct MHD_Daemon* d = MHD_start_daemon(
      MHD_USE_INTERNAL_POLLING_THREAD,
      (uint16_t)port,
      NULL, NULL,
      &request_handler, &mctx,
      MHD_OPTION_CONNECTION_TIMEOUT, (unsigned int)10,
      MHD_OPTION_END);

  if (!d) {
    fprintf(stderr, "Failed to start HTTP server on port %d\n", port);
    mongoc_collection_destroy(col);
    mongoc_client_destroy(client);
    mongoc_cleanup();
    return 1;
  }

  printf("Listening on http://0.0.0.0:%d\n", port);
  printf("MongoDB: %s  DB=%s  Collection=%s\n", mongo_uri, dbname, colname);
  printf("POST JSON to /move\n");
  fflush(stdout);

  while (!g_stop) {
    struct timespec ts = {0, 200000000L}; // 200ms
    nanosleep(&ts, NULL);
  }

  printf("\nShutting down...\n");
  MHD_stop_daemon(d);

  mongoc_collection_destroy(col);
  mongoc_client_destroy(client);
  mongoc_cleanup();
  return 0;
}
