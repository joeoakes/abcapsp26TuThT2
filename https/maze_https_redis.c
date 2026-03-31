// maze_https_redis.c
// HTTPS server that pushes mission JSON into Redis

#include <errno.h>
#include <microhttpd.h>
#include <hiredis/hiredis.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <cjson/cJSON.h>

#define DEFAULT_PORT 8444
#define POSTBUFFERSIZE 4096

static const char *cert_file = "certs/server.crt";
static const char *key_file  = "certs/server.key";

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
    if (!buf) { fclose(f); return NULL; }

    if (fread(buf, 1, n, f) != (size_t)n) {
        fclose(f);
        free(buf);
        return NULL;
    }

    buf[n] = '\0';
    fclose(f);
    return buf;
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

    /* First call: allocate connection struct */
    if (*con_cls == NULL) {
        struct connection_info *ci = calloc(1, sizeof(*ci));
        *con_cls = ci;
        return MHD_YES;
    }

    struct connection_info *ci = *con_cls;

    /* Collect POST body */
    if (*upload_data_size != 0) {
        ci->data = realloc(ci->data, ci->size + *upload_data_size + 1);
        memcpy(ci->data + ci->size, upload_data, *upload_data_size);
        ci->size += *upload_data_size;
        ci->data[ci->size] = '\0';

        *upload_data_size = 0;
        return MHD_YES;
    }

    /* ================= Redis Insert ================= */

    cJSON* root = cJSON_Parse(ci->data);

    if (!root) {
        fprintf(stderr, "Invalid JSON\n");
    }
    else {
        cJSON* session = cJSON_GetObjectItem(root, "session_id");
        cJSON* width = cJSON_GetObjectItem(root, "width");
        cJSON* height = cJSON_GetObjectItem(root, "height");
        cJSON* cells = cJSON_GetObjectItem(root, "cells");
        cJSON* start = cJSON_GetObjectItem(root, "start");
        cJSON* goal = cJSON_GetObjectItem(root, "goal");

        if (session && width && height && cells && start && goal) {
            const char* sid = session->valuestring;
            char key[256];

            /* width */
            snprintf(key, sizeof(key), "team2ffmaze:%s:width", sid);
            redisCommand(redis, "SET %s %d", key, width->valueint);

            /* height */
            snprintf(key, sizeof(key), "team2ffmaze:%s:height", sid);
            redisCommand(redis, "SET %s %d", key, height->valueint);

            /* cells */
            char* cells_str = cJSON_PrintUnformatted(cells);
            snprintf(key, sizeof(key), "team2ffmaze:%s:cells", sid);
            redisCommand(redis, "SET %s %s", key, cells_str);
            free(cells_str);

            /* start */
            cJSON* sx = cJSON_GetObjectItem(start, "x");
            cJSON* sy = cJSON_GetObjectItem(start, "y");

            snprintf(key, sizeof(key), "team2ffmaze:%s:start_x", sid);
            redisCommand(redis, "SET %s %d", key, sx->valueint);

            snprintf(key, sizeof(key), "team2ffmaze:%s:start_y", sid);
            redisCommand(redis, "SET %s %d", key, sy->valueint);

            /* goal */
            cJSON* gx = cJSON_GetObjectItem(goal, "x");
            cJSON* gy = cJSON_GetObjectItem(goal, "y");

            snprintf(key, sizeof(key), "team2ffmaze:%s:goal_x", sid);
            redisCommand(redis, "SET %s %d", key, gx->valueint);

            snprintf(key, sizeof(key), "team2ffmaze:%s:goal_y", sid);
            redisCommand(redis, "SET %s %d", key, gy->valueint);

            printf("team2ffmaze stored for session: %s\n", sid);
        }

        cJSON_Delete(root);
    }

    /* ================= HTTP Response ================= */

    const char *response = "Mission recorded";
    struct MHD_Response *resp =
        MHD_create_response_from_buffer(
            strlen(response),
            (void *)response,
            MHD_RESPMEM_PERSISTENT
        );

    int ret = MHD_queue_response(connection, MHD_HTTP_OK, resp);
    MHD_destroy_response(resp);

    free(ci->data);
    free(ci);
    *con_cls = NULL;

    return ret;
}

/* ================= Main ================= */

int main(void) {

    /* Connect to Redis */
    redis = redisConnect("localhost", 6379);
    if (redis == NULL || redis->err) {
        fprintf(stderr, "Failed to connect to Redis\n");
        return 1;
    }

    /* Load TLS certificate and key */
    char *cert_pem = read_file(cert_file);
    char *key_pem  = read_file(key_file);

    if (!cert_pem || !key_pem) {
        fprintf(stderr, "Failed to read cert/key files\n");
        return 1;
    }

    struct MHD_Daemon *daemon = MHD_start_daemon(
        MHD_USE_THREAD_PER_CONNECTION | MHD_USE_TLS,
        DEFAULT_PORT,
        NULL, NULL,
        &handle_post, NULL,
        MHD_OPTION_HTTPS_MEM_CERT, cert_pem,
        MHD_OPTION_HTTPS_MEM_KEY,  key_pem,
        MHD_OPTION_END
    );

    if (!daemon) {
        fprintf(stderr, "Failed to start HTTPS server\n");
        return 1;
    }

    printf("========================================\n");
    printf("Database backend: Redis\n");
    printf("Redis host: localhost\n");
    printf("Redis port: 6379\n");
    printf("Key namespace example: team2tt_mission\n");
    printf("========================================\n\n");

    printf("HTTPS Redis mission server running on port %d\n",
           DEFAULT_PORT);

    getchar(); 

    MHD_stop_daemon(daemon);
    free(cert_pem);
    free(key_pem);
    redisFree(redis);

    return 0;
}
