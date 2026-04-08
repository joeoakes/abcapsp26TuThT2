// maze_https_mongo.c
// HTTPS telemetry server: POST JSON to /move, insert into MongoDB.
// Requires libmicrohttpd built with TLS support (often via GnuTLS).

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

static enum MHD_Result respond_text(struct MHD_Connection* connection, unsigned int status, const char* text) {
  struct MHD_Response* response = MHD_create_response_from_buffer(
      strlen(text), (void*)text, MHD_RESPMEM_MUST_COPY);
  if (!response) return MHD_NO;
  MHD_add_response_header(response, "Content-Type", "text/plain; charset=utf-8");
  enum MHD_Result ret = MHD_queue_response(connection, status, response);
  MHD_destroy_response(response);
  return ret;
}

static enum MHD_Result respond_json(struct MHD_Connection* connection, unsigned int status, const char* json) {
  struct MHD_Response* response = MHD_create_response_from_buffer(
      strlen(json), (void*)json, MHD_RESPMEM_MUST_COPY);
  if (!response) return MHD_NO;
  MHD_add_response_header(response, "Content-Type", "application/json; charset=utf-8");
  enum MHD_Result ret = MHD_queue_response(connection, status, response);
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
  bson_t* doc = bson_new_from_json((const uint8_t*)json, -1, err);
  if (!doc) return NULL;

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

static enum MHD_Result handle_post_move(struct MHD_Connection* connection, MongoCtx* mctx, const char* body) {
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

static enum MHD_Result request_handler(void* cls,
                                      struct MHD_Connection* connection,
                                      const char* url,
                                      const char* method,
                                      const char* version,
                                      const char* upload_data,
                                      size_t* upload_data_size,
                                      void** con_cls) {
  (void)version;
  MongoCtx* mctx = (MongoCtx*)cls;

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

  if (*con_cls == NULL) {
    BodyBuf* b = (BodyBuf*)calloc(1, sizeof(BodyBuf));
    if (!b) return MHD_NO;
    *con_cls = (void*)b;
    return MHD_YES;
  }

  BodyBuf* b = (BodyBuf*)(*con_cls);

  if (*upload_data_size != 0) {
    if (!bodybuf_append(b, upload_data, *upload_data_size)) {
      bodybuf_free(b);
      free(b);
      *con_cls = NULL;
      return respond_text(connection, MHD_HTTP_INTERNAL_SERVER_ERROR, "Out of memory\n");
    }
    *upload_data_size = 0;
    return MHD_YES;
  }

  enum MHD_Result ret = handle_post_move(connection, mctx, b->data ? b->data : "");

  bodybuf_free(b);
  free(b);
  *con_cls = NULL;

  return ret;
}

static char* read_file(const char* path) {
  FILE* f = fopen(path, "rb");
  if (!f) return NULL;
  if (fseek(f, 0, SEEK_END) != 0) { fclose(f); return NULL; }
  long n = ftell(f);
  if (n < 0) { fclose(f); return NULL; }
  rewind(f);

  char* buf = (char*)malloc((size_t)n + 1);
  if (!buf) { fclose(f); return NULL; }

  size_t got = fread(buf, 1, (size_t)n, f);
  fclose(f);
  if (got != (size_t)n) { free(buf); return NULL; }
  buf[n] = '\0';
  return buf;
}

int main(void) {
  signal(SIGINT, on_sigint);
  signal(SIGTERM, on_sigint);

  const char* mongo_uri = getenv_or("MONGO_URI", "mongodb://localhost:27017");
  const char* dbname    = getenv_or("MONGO_DB",  "maze");
  const char* colname   = getenv_or("MONGO_COL", "team2fmoves");

  // HTTPS default port
  const char* port_s = getenv_or("LISTEN_PORT", "8444");
  int port = atoi(port_s);
  if (port <= 0 || port > 65535) port = 8444;

  // TLS files — paths are relative to the repo https/certs/ directory.
  // Override with env vars if running from a different working directory.
  const char* ca_path  = getenv_or("TLS_CA",   "certs/ca.crt");
  const char* crt_path = getenv_or("TLS_CERT", "certs/logger-server.crt");
  const char* key_path = getenv_or("TLS_KEY",  "certs/logger-server.key");

  char* ca_pem         = read_file(ca_path);
  char* server_crt_pem = read_file(crt_path);
  char* server_key_pem = read_file(key_path);
  if (!ca_pem || !server_crt_pem || !server_key_pem) {
    fprintf(stderr, "Failed to read TLS files.\n");
    fprintf(stderr, "  TLS_CA   = %s%s\n", ca_path,  ca_pem  ? " (ok)" : " *** NOT FOUND ***");
    fprintf(stderr, "  TLS_CERT = %s%s\n", crt_path, server_crt_pem ? " (ok)" : " *** NOT FOUND ***");
    fprintf(stderr, "  TLS_KEY  = %s%s\n", key_path, server_key_pem ? " (ok)" : " *** NOT FOUND ***");
    fprintf(stderr, "Run generate_certs.sh then copy ca.crt + logger-server.{crt,key} here.\n");
    free(ca_pem);
    free(server_crt_pem);
    free(server_key_pem);
    return 1;
  }

  mongoc_init();

  bson_error_t err;
  mongoc_client_t* client = mongoc_client_new(mongo_uri);
  if (!client) {
    fprintf(stderr, "Failed to create MongoDB client from URI: %s\n", mongo_uri);
    free(server_crt_pem);
    free(server_key_pem);
    mongoc_cleanup();
    return 1;
  }

  // Basic ping to surface connectivity issues early
  bson_t ping_cmd;
  bson_init(&ping_cmd);
  BSON_APPEND_INT32(&ping_cmd, "ping", 1);

  bson_t ping_reply;
  bool ping_ok = mongoc_client_command_simple(client, "admin", &ping_cmd, NULL, &ping_reply, &err);
  bson_destroy(&ping_cmd);

  if (!ping_ok) {
    fprintf(stderr, "MongoDB ping failed: %s\n", err.message);
    mongoc_client_destroy(client);
    free(server_crt_pem);
    free(server_key_pem);
    mongoc_cleanup();
    return 1;
  }
  bson_destroy(&ping_reply);

  mongoc_collection_t* col = mongoc_client_get_collection(client, dbname, colname);
  MongoCtx mctx = { client, col };

  struct MHD_Daemon* d = MHD_start_daemon(
      MHD_USE_INTERNAL_POLLING_THREAD | MHD_USE_TLS,
      (uint16_t)port,
      NULL, NULL,
      &request_handler, &mctx,
      MHD_OPTION_HTTPS_MEM_KEY,        server_key_pem,
      MHD_OPTION_HTTPS_MEM_CERT,       server_crt_pem,
      /* mTLS — require clients to present a cert signed by our CA */
      MHD_OPTION_HTTPS_MEM_TRUST_CERT, ca_pem,
      MHD_OPTION_CONNECTION_TIMEOUT, (unsigned int)10,
      MHD_OPTION_END);

  if (!d) {
    fprintf(stderr, "Failed to start HTTPS server on port %d (is libmicrohttpd built with TLS?)\n", port);
    mongoc_collection_destroy(col);
    mongoc_client_destroy(client);
    free(server_crt_pem);
    free(server_key_pem);
    mongoc_cleanup();
    return 1;
  }

  printf("Listening on https://0.0.0.0:%d\n", port);
  printf("mTLS: client certificates REQUIRED (CA: %s)\n", ca_path);
  printf("Database backend: MongoDB\n");
  printf("MongoDB URI: %s\n", mongo_uri);
  printf("Database name: %s\n", dbname);
  printf("Collection: %s\n", colname);
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
  free(ca_pem);
  free(server_crt_pem);
  free(server_key_pem);
  mongoc_cleanup();
  return 0;
}
