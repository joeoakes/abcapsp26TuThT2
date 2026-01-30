# maze_https_mongo
**C HTTPS JSON → MongoDB Server**

This project is a secure (HTTPS/TLS) version of the original `maze_http_mongo` server.

It is a small **C** program that:

- Listens for **HTTPS** requests (TLS)
- Receives a **JSON document**
- Appends a server timestamp
- Inserts the document into **MongoDB**

---

## Endpoint

- **POST** `/move`
- **Content-Type:** `application/json`
- **Protocol:** HTTPS

The server automatically appends:

```json
"received_at": "YYYY-MM-DDTHH:MM:SSZ"
```

---

## Requirements

### Libraries
- `libmicrohttpd` (with TLS / GnuTLS support)
- MongoDB C Driver:
  - `libmongoc`
  - `libbson`
- `pkg-config`
- `gcc` or `clang`

### Debian / Ubuntu / Raspberry Pi OS

```bash
sudo apt update
sudo apt install -y   build-essential   pkg-config   libmicrohttpd-dev   libgnutls28-dev   libmongoc-dev   libbson-dev
```

---

## TLS Certificates (Required)

For development and lab use, generate a **self-signed certificate**:

```bash
mkdir certs
cd certs

openssl req -x509 -newkey rsa:2048   -keyout server.key   -out server.crt   -days 365   -nodes   -subj "/CN=localhost"
```

Expected files:

```
certs/server.crt
certs/server.key
```

---

## Build

```bash
gcc -O2 -Wall -Wextra -std=c11 maze_https_mongo.c -o maze_https_mongo   $(pkg-config --cflags --libs libmicrohttpd libmongoc-1.0 gnutls)
```

---

## Run

### Default configuration

```bash
./maze_https_mongo
```

Defaults:
- **Port:** `8443`
- **Mongo URI:** `mongodb://localhost:27017`
- **Database:** `maze`
- **Collection:** `moves`
- **TLS Cert:** `certs/server.crt`
- **TLS Key:** `certs/server.key`

### Override using environment variables

```bash
LISTEN_PORT=9443 CERT_FILE=certs/server.crt KEY_FILE=certs/server.key MONGO_URI="mongodb://localhost:27017" MONGO_DB="maze" MONGO_COL="moves" ./maze_https_mongo
```

---

## Test with curl

Because a self-signed certificate is used, include `-k`:

```bash
curl -k -X POST https://localhost:8443/move   -H "Content-Type: application/json"   -d '{
    "event_type": "player_move",
    "input": {
      "device": "joystick",
      "move_sequence": 1
    },
    "player": {
      "position": { "x": 1, "y": 2 }
    },
    "goal_reached": false,
    "timestamp": "2026-01-25T11:42:18Z"
  }'
```

Expected response:

```json
{"status":"ok"}
```

---

## SDL / Game Client Notes

From an SDL or C-based client, HTTPS requests are commonly sent using **libcurl**.

For development (self-signed certificates only):

```c
curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 0L);
curl_easy_setopt(curl, CURLOPT_SSL_VERIFYHOST, 0L);
```

⚠️ Do **not** disable certificate verification in production.

---

## Production Notes

For production deployments:

- Use a **CA-signed certificate** (Let’s Encrypt or internal CA)
- Enable TLS verification on clients
- Consider:
  - Mutual TLS (client certificates)
  - JWT or API key authentication
  - Running behind a reverse proxy (nginx)

---

## Summary

✔ Encrypted HTTPS transport  
✔ Same JSON payload as HTTP version  
✔ MongoDB ingestion unchanged  
✔ Ideal for labs, SDL games, and telemetry pipelines  
