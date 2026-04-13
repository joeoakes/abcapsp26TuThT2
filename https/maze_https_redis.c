// maze_https_redis.c
// HTTPS server that stores maze static state and runtime updates in Redis
// All posts go to /mission_end and are distinguished by "msg_type"
//
// mTLS: uses ai-server.crt + ai-server.key as server identity.
// Clients (Game HAT) must present a cert signed by ca.crt to connect.

#include <errno.h>
#include <microhttpd.h>
#include <hiredis/hiredis.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <inttypes.h>
#include <cjson/cJSON.h>

#define DEFAULT_PORT 8444

static const char *cert_file = "certs/ai-server.crt";
static const char *key_file  = "certs/ai-server.key";
static const char *ca_file   = "certs/ca.crt";

static redisContext *redis;

/* ================= Connection Structure ================= */

struct connection_info {
    char *data;
    size_t size;
};

/* ================= File Reader ================= */

static char *read_file(const char *path) {
    FILE *f = fopen(path, "rb");
    if (!f) return NULL;

    fseek(f, 0, SEEK_END);
    long n = ftell(f);
    fseek(f, 0, SEEK_SET);

    char *buf = malloc(n + 1);
    if (!buf) {
        fclose(f);
        return NULL;
    }

    if (fread(buf, 1, n, f) != (size_t)n) {
        fclose(f);
        free(buf);
        return NULL;
    }

    buf[n] = '\0';
    fclose(f);
    return buf;
}

/* ================= Hash Helper ================= */

static uint64_t fnv1a_64(const char *s) {
    uint64_t hash = 14695981039346656037ULL;
    while (*s) {
        hash ^= (unsigned char)*s++;
        hash *= 1099511628211ULL;
    }
    return hash;
}

/* ================= Redis Helpers ================= */

static void free_reply(redisReply *reply) {
    if (reply) freeReplyObject(reply);
}

static int append_history_entry(const char *sid, cJSON *entry) {
    char key[256];
    redisReply *reply = NULL;
    cJSON *history_arr = NULL;
    char *updated_json = NULL;
    int ok = 0;

    snprintf(key, sizeof(key), "team2ttmaze:%s:history", sid);
    reply = redisCommand(redis, "GET %s", key);

    if (reply && reply->type == REDIS_REPLY_STRING && reply->str) {
        history_arr = cJSON_Parse(reply->str);
    } else {
        history_arr = cJSON_CreateArray();
    }
    free_reply(reply);
    reply = NULL;

    if (!history_arr || !cJSON_IsArray(history_arr)) {
        if (history_arr) cJSON_Delete(history_arr);
        history_arr = cJSON_CreateArray();
    }

    if (!cJSON_AddItemToArray(history_arr, entry)) {
        cJSON_Delete(entry);
        cJSON_Delete(history_arr);
        return 0;
    }

    updated_json = cJSON_PrintUnformatted(history_arr);
    cJSON_Delete(history_arr);

    if (!updated_json) {
        return 0;
    }

    reply = redisCommand(redis, "SET %s %s", key, updated_json);
    if (reply) ok = 1;
    free_reply(reply);
    free(updated_json);

    return ok;
}

static int init_runtime_keys(const char *sid, int start_x, int start_y) {
    char key[256];
    char start_pos[64];
    redisReply *reply = NULL;

    snprintf(start_pos, sizeof(start_pos), "%d,%d", start_x, start_y);

    snprintf(key, sizeof(key), "team2ttmaze:%s:visited", sid);
    reply = redisCommand(redis, "DEL %s", key);
    free_reply(reply);
    reply = redisCommand(redis, "SADD %s %s", key, start_pos);
    free_reply(reply);

    snprintf(key, sizeof(key), "team2ttmaze:%s:history", sid);
    reply = redisCommand(redis, "SET %s %s", key, "[]");
    free_reply(reply);

    snprintf(key, sizeof(key), "team2ttmaze:%s:plan", sid);
    reply = redisCommand(redis, "SET %s %s", key, "[]");
    free_reply(reply);

    snprintf(key, sizeof(key), "team2ttmaze:%s:plan_index", sid);
    reply = redisCommand(redis, "SET %s %d", key, 0);
    free_reply(reply);

    cJSON *start_entry = cJSON_CreateObject();
    if (!start_entry) return 0;
    cJSON_AddStringToObject(start_entry, "event", "start");
    cJSON_AddNumberToObject(start_entry, "x", start_x);
    cJSON_AddNumberToObject(start_entry, "y", start_y);

    return append_history_entry(sid, start_entry);
}

/* ================= Message Handlers ================= */

static int handle_maze_init(cJSON *root) {
    cJSON *session = cJSON_GetObjectItem(root, "session_id");
    cJSON *width   = cJSON_GetObjectItem(root, "width");
    cJSON *height  = cJSON_GetObjectItem(root, "height");
    cJSON *cells   = cJSON_GetObjectItem(root, "cells");
    cJSON *start   = cJSON_GetObjectItem(root, "start");
    cJSON *goal    = cJSON_GetObjectItem(root, "goal");

    if (!(cJSON_IsString(session) &&
          cJSON_IsNumber(width) &&
          cJSON_IsNumber(height) &&
          cJSON_IsArray(cells) &&
          cJSON_IsObject(start) &&
          cJSON_IsObject(goal))) {
        fprintf(stderr, "Invalid maze_init JSON\n");
        return 0;
    }

    const char *sid = session->valuestring;
    char key[256];
    redisReply *reply = NULL;

    cJSON *sx = cJSON_GetObjectItem(start, "x");
    cJSON *sy = cJSON_GetObjectItem(start, "y");
    cJSON *gx = cJSON_GetObjectItem(goal, "x");
    cJSON *gy = cJSON_GetObjectItem(goal, "y");

    if (!(cJSON_IsNumber(sx) && cJSON_IsNumber(sy) &&
          cJSON_IsNumber(gx) && cJSON_IsNumber(gy))) {
        fprintf(stderr, "Invalid start/goal fields in maze_init\n");
        return 0;
    }

    reply = redisCommand(redis, "SET team2ttmaze:latest_session_id %s", sid);
    free_reply(reply);

    snprintf(key, sizeof(key), "team2ttmaze:%s:width", sid);
    reply = redisCommand(redis, "SET %s %d", key, width->valueint);
    free_reply(reply);

    snprintf(key, sizeof(key), "team2ttmaze:%s:height", sid);
    reply = redisCommand(redis, "SET %s %d", key, height->valueint);
    free_reply(reply);

    char *cells_str = cJSON_PrintUnformatted(cells);
    if (!cells_str) return 0;

    snprintf(key, sizeof(key), "team2ttmaze:%s:cells", sid);
    reply = redisCommand(redis, "SET %s %s", key, cells_str);
    free_reply(reply);

    snprintf(key, sizeof(key), "team2ttmaze:%s:start_x", sid);
    reply = redisCommand(redis, "SET %s %d", key, sx->valueint);
    free_reply(reply);

    snprintf(key, sizeof(key), "team2ttmaze:%s:start_y", sid);
    reply = redisCommand(redis, "SET %s %d", key, sy->valueint);
    free_reply(reply);

    snprintf(key, sizeof(key), "team2ttmaze:%s:goal_x", sid);
    reply = redisCommand(redis, "SET %s %d", key, gx->valueint);
    free_reply(reply);

    snprintf(key, sizeof(key), "team2ttmaze:%s:goal_y", sid);
    reply = redisCommand(redis, "SET %s %d", key, gy->valueint);
    free_reply(reply);

    /* Hash the static maze content only */
    cJSON *sig_root = cJSON_CreateObject();
    if (sig_root) {
        cJSON_AddNumberToObject(sig_root, "width", width->valueint);
        cJSON_AddNumberToObject(sig_root, "height", height->valueint);
        cJSON_AddItemReferenceToObject(sig_root, "cells", cells);
        cJSON_AddItemReferenceToObject(sig_root, "start", start);
        cJSON_AddItemReferenceToObject(sig_root, "goal", goal);

        char *normalized_json = cJSON_PrintUnformatted(sig_root);
        if (normalized_json) {
            uint64_t sig = fnv1a_64(normalized_json);
            char sig_hex[32];
            snprintf(sig_hex, sizeof(sig_hex), "%016" PRIx64, sig);

            snprintf(key, sizeof(key), "team2ttmaze:%s:maze_sig", sid);
            reply = redisCommand(redis, "SET %s %s", key, sig_hex);
            free_reply(reply);

            free(normalized_json);
        }
        cJSON_Delete(sig_root);
    }

    free(cells_str);

    if (!init_runtime_keys(sid, sx->valueint, sy->valueint)) {
        fprintf(stderr, "Failed to initialize runtime keys for session %s\n", sid);
        return 0;
    }

    printf("Stored maze_init for session: %s\n", sid);
    return 1;
}

static int handle_runtime_update(cJSON *root) {
    cJSON *session = cJSON_GetObjectItem(root, "session_id");
    cJSON *x = cJSON_GetObjectItem(root, "x");
    cJSON *y = cJSON_GetObjectItem(root, "y");
    cJSON *event = cJSON_GetObjectItem(root, "event");
    cJSON *move = cJSON_GetObjectItem(root, "move");

    if (!(cJSON_IsString(session) &&
          cJSON_IsNumber(x) &&
          cJSON_IsNumber(y) &&
          cJSON_IsString(event))) {
        fprintf(stderr, "Invalid runtime_update JSON\n");
        return 0;
    }

    const char *sid = session->valuestring;
    char key[256];
    char pos[64];
    redisReply *reply = NULL;

    snprintf(pos, sizeof(pos), "%d,%d", x->valueint, y->valueint);

    snprintf(key, sizeof(key), "team2ttmaze:%s:visited", sid);
    reply = redisCommand(redis, "SADD %s %s", key, pos);
    free_reply(reply);

    cJSON *entry = cJSON_CreateObject();
    if (!entry) return 0;

    cJSON_AddStringToObject(entry, "event", event->valuestring);
    cJSON_AddNumberToObject(entry, "x", x->valueint);
    cJSON_AddNumberToObject(entry, "y", y->valueint);

    if (cJSON_IsString(move) && move->valuestring && move->valuestring[0] != '\0') {
        cJSON_AddStringToObject(entry, "move", move->valuestring);
    }

    if (!append_history_entry(sid, entry)) {
        fprintf(stderr, "Failed to append history for session %s\n", sid);
        return 0;
    }

    printf("Stored runtime_update for session: %s (%s @ %d,%d)\n",
           sid, event->valuestring, x->valueint, y->valueint);
    return 1;
}

/* ================= Request Handler ================= */

static enum MHD_Result handle_post(
    void *cls,
    struct MHD_Connection *connection,
    const char *url,
    const char *method,
    const char *version,
    const char *upload_data,
    size_t *upload_data_size,
    void **con_cls)
{
    (void)version;
    (void)cls;

    if (strcmp(method, "POST") != 0 || strcmp(url, "/mission_end") != 0)
        return MHD_NO;

    if (*con_cls == NULL) {
        struct connection_info *ci = calloc(1, sizeof(*ci));
        if (!ci) return MHD_NO;
        *con_cls = ci;
        return MHD_YES;
    }

    struct connection_info *ci = *con_cls;

    if (*upload_data_size != 0) {
        char *newbuf = realloc(ci->data, ci->size + *upload_data_size + 1);
        if (!newbuf) {
            fprintf(stderr, "Out of memory while receiving POST body\n");
            free(ci->data);
            free(ci);
            *con_cls = NULL;
            return MHD_NO;
        }

        ci->data = newbuf;
        memcpy(ci->data + ci->size, upload_data, *upload_data_size);
        ci->size += *upload_data_size;
        ci->data[ci->size] = '\0';

        *upload_data_size = 0;
        return MHD_YES;
    }

    int ok = 0;
    cJSON *root = cJSON_Parse(ci->data);

    if (!root) {
        fprintf(stderr, "Invalid JSON\n");
    } else {
        cJSON *msg_type = cJSON_GetObjectItem(root, "msg_type");

        if (!cJSON_IsString(msg_type) || !msg_type->valuestring) {
            fprintf(stderr, "Missing msg_type in JSON\n");
        } else if (strcmp(msg_type->valuestring, "maze_init") == 0) {
            ok = handle_maze_init(root);
        } else if (strcmp(msg_type->valuestring, "runtime_update") == 0) {
            ok = handle_runtime_update(root);
        } else {
            fprintf(stderr, "Unknown msg_type: %s\n", msg_type->valuestring);
        }

        cJSON_Delete(root);
    }

    const char *response = ok ? "OK" : "ERROR";
    unsigned int status = ok ? MHD_HTTP_OK : MHD_HTTP_BAD_REQUEST;

    struct MHD_Response *resp =
        MHD_create_response_from_buffer(
            strlen(response),
            (void *)response,
            MHD_RESPMEM_PERSISTENT
        );

    enum MHD_Result ret = MHD_queue_response(connection, status, resp);
    MHD_destroy_response(resp);

    free(ci->data);
    free(ci);
    *con_cls = NULL;

    return ret;
}

/* ================= Main ================= */

int main(void) {
    redis = redisConnect("localhost", 6379);
    if (redis == NULL || redis->err) {
        fprintf(stderr, "Failed to connect to Redis\n");
        if (redis) redisFree(redis);
        return 1;
    }

    char *cert_pem = read_file(cert_file);
    char *key_pem  = read_file(key_file);
    char *ca_pem   = read_file(ca_file);

    if (!cert_pem || !key_pem || !ca_pem) {
        fprintf(stderr, "Failed to read TLS files.\n");
        fprintf(stderr, "  cert: %s\n  key: %s\n  ca: %s\n", cert_file, key_file, ca_file);
        fprintf(stderr, "Run generate_certs.sh and SCP certs to this device.\n");
        free(cert_pem);
        free(key_pem);
        free(ca_pem);
        redisFree(redis);
        return 1;
    }

    // mTLS: server presents ai-server.crt, requires clients to present
    // a cert signed by ca.crt (MHD_OPTION_HTTPS_MEM_TRUST).
    struct MHD_Daemon *daemon = MHD_start_daemon(
        MHD_USE_THREAD_PER_CONNECTION | MHD_USE_TLS,
        DEFAULT_PORT,
        NULL, NULL,
        &handle_post, NULL,
        MHD_OPTION_HTTPS_MEM_CERT,       cert_pem,
        MHD_OPTION_HTTPS_MEM_KEY,        key_pem,
        MHD_OPTION_HTTPS_MEM_TRUST, ca_pem,
        MHD_OPTION_END
    );

    if (!daemon) {
        fprintf(stderr, "Failed to start HTTPS server\n");
        free(cert_pem);
        free(key_pem);
        free(ca_pem);
        redisFree(redis);
        return 1;
    }

    printf("========================================\n");
    printf("Database backend: Redis\n");
    printf("Redis host: localhost\n");
    printf("Redis port: 6379\n");
    printf("Namespace: team2ttmaze\n");
    printf("Endpoint: /mission_end\n");
    printf("mTLS enabled — clients must present a cert signed by %s\n", ca_file);
    printf("msg_type values:\n");
    printf("  maze_init\n");
    printf("  runtime_update\n");
    printf("========================================\n\n");
    printf("HTTPS Redis server running on port %d\n", DEFAULT_PORT);

    getchar();

    MHD_stop_daemon(daemon);
    free(cert_pem);
    free(key_pem);
    free(ca_pem);
    redisFree(redis);

    return 0;
}
