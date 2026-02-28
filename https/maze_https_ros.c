#include <microhttpd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define DEFAULT_PORT 8444

const char *uri_str;
const char *db_name;
const char *col_name;

/* --- Helper to load file content into memory (REQUIRED for MHD SSL) --- */
char *load_file(const char *filename) {
    FILE *f = fopen(filename, "rb");
    if (!f) {
        perror(filename);
        return NULL;
    }
    fseek(f, 0, SEEK_END);
    long length = ftell(f);
    fseek(f, 0, SEEK_SET);

    char *buffer = malloc(length + 1);
    if (buffer) {
        fread(buffer, 1, length, f);
        buffer[length] = '\0';
    }
    fclose(f);
    return buffer;
}

struct connection_info {
    char *data;
    size_t size;
};

static void get_utc_iso8601(char *buf, size_t len) {
    time_t now = time(NULL);
    struct tm tm;
    gmtime_r(&now, &tm);
    strftime(buf, len, "%Y-%m-%dT%H:%M:%SZ", &tm);
}

#include <ctype.h>  // for isspace

// --- Example movement functions ---
void move_forward()
{ 
    system("ros2 topic pub -r 5 -t 5 /cmd_vel geometry_msgs/msg/Twist \"{linear: {x: 0.15}, angular: {z: 0.0}}\""); 
}
void move_backward() 
{
    system("ros2 topic pub -r 5 -t 5 /cmd_vel geometry_msgs/msg/Twist \"{linear: {x: -0.15}, angular: {z: 0.0}}\""); 
}
void move_left()
{ 
    // Rotate left in place
    system("ros2 topic pub -r 5 -t 16 /cmd_vel geometry_msgs/msg/Twist \"{linear: {x: 0.0}, angular: {z: 0.5}}\""); 
    move_forward();
}
void move_right()
{ 
    // Rotate right in place
    system("ros2 topic pub -r 5 -t 16 /cmd_vel geometry_msgs/msg/Twist \"{linear: {x: 0.0}, angular: {z: -0.5}}\""); 
    move_forward();
}

// --- Helper to trim whitespace ---
char* trim_whitespace(char* str) {
    while(isspace((unsigned char)*str)) str++;
    if(*str == 0) return str;
    char *end = str + strlen(str) - 1;
    while(end > str && isspace((unsigned char)*end)) end--;
    end[1] = '\0';
    return str;
}

// --- Handle POST request ---
static int handle_post(void *cls,
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

    if (strcmp(method, "POST") != 0 || strcmp(url, "/move") != 0)
        return MHD_NO;

    if (*con_cls == NULL) {
        struct connection_info *ci = calloc(1, sizeof(*ci));
        *con_cls = ci;
        return MHD_YES;
    }

    struct connection_info *ci = *con_cls;

    if (*upload_data_size != 0) {
        ci->data = realloc(ci->data, ci->size + *upload_data_size + 1);
        memcpy(ci->data + ci->size, upload_data, *upload_data_size);
        ci->size += *upload_data_size;
        ci->data[ci->size] = '\0';
        *upload_data_size = 0;
        return MHD_YES;
    }

    // --- Parse move_dir from JSON body (very basic) ---
    char *move_dir = NULL;
    char *found = strstr(ci->data, "\"move_dir\"");
    if (found) {
        // Skip to the ':' and then the value
        char *colon = strchr(found, ':');
        if (colon) {
            colon++; // move past ':'
            while(isspace((unsigned char)*colon) || *colon=='\"') colon++;
            char *end = colon;
            while(*end && *end != '\"' && *end != ',' && *end != '}') end++;
            *end = '\0';
            move_dir = trim_whitespace(colon);
        }
    }

    // --- Call movement function based on move_dir ---
    if (move_dir) {
        if (strcmp(move_dir, "forward") == 0) move_forward();
        else if (strcmp(move_dir, "backward") == 0) move_backward();
        else if (strcmp(move_dir, "left") == 0) move_left();
        else if (strcmp(move_dir, "right") == 0) move_right();
        else printf("Unknown move_dir: %s\n", move_dir);
    } else {
        printf("move_dir not provided in request\n");
    }

    const char *response = "{\"status\":\"ok\"}";
    struct MHD_Response *resp =
        MHD_create_response_from_buffer(strlen(response),
                                         (void *)response,
                                         MHD_RESPMEM_PERSISTENT);

    int ret = MHD_queue_response(connection, MHD_HTTP_OK, resp);
    MHD_destroy_response(resp);

    free(ci->data);
    free(ci);
    *con_cls = NULL;

    return ret;
}

int main(void) {
    const char *cert_path = "certs/server.crt";
    const char *key_path  = "certs/server.key";

    /* 1. LOAD CERT KEY INTO MEMORY BUFFERS */
    char *key_pem = load_file(key_path);
    char *cert_pem = load_file(cert_path);

    if (!key_pem || !cert_pem) {
        fprintf(stderr, "Error: Could not read certs.\n");
        fprintf(stderr, "Please run: mkdir -p certs && openssl req -x509 -newkey rsa:4096 -keyout certs/server.key -out certs/server.crt -days 365 -nodes -subj '/CN=localhost'\n");
        return 1;
    }


    /* 2. Pass MEMORY BUFFERS (cert_pem, key_pem) to the daemon */
    struct MHD_Daemon *daemon = MHD_start_daemon(
        MHD_USE_THREAD_PER_CONNECTION | MHD_USE_TLS | MHD_USE_INTERNAL_POLLING_THREAD,
        DEFAULT_PORT,
        NULL, NULL,
        &handle_post, NULL,
        MHD_OPTION_HTTPS_MEM_CERT, cert_pem,
        MHD_OPTION_HTTPS_MEM_KEY, key_pem,
        MHD_OPTION_END);

    if (!daemon) {
        fprintf(stderr, "Failed to start HTTPS server. Check port %d\n", DEFAULT_PORT);
        return 1;
    }

    printf("----------------------------------------------------------------------\n");
    printf("HTTPS server listening on https://0.0.0.0:%d\n", DEFAULT_PORT);
    printf("Post JSON to /move\n");
    printf("----------------------------------------------------------------------\n");
    getchar(); 

    MHD_stop_daemon(daemon);
    
    free(key_pem);
    free(cert_pem);

    return 0;
}
