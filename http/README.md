# maze_http_mongo (C HTTP JSON -> MongoDB)

This is a small **C** program that:
- listens for HTTP requests
- receives a JSON document
- inserts it into MongoDB

## Endpoint
- `POST /move` with `Content-Type: application/json`
- Body: your telemetry JSON

The server also appends:
- `"received_at": "<UTC ISO8601>"`

## Requirements
- libmicrohttpd
- MongoDB C driver (libmongoc + libbson)
- pkg-config
- gcc/clang

### Debian/Ubuntu/Raspberry Pi OS
```bash
sudo apt update
sudo apt install -y build-essential pkg-config libmicrohttpd-dev libmongoc-dev libbson-dev
```

## Build
```bash
gcc -O2 -Wall -Wextra -std=c11 maze_http_mongo.c -o maze_http_mongo \
  $(pkg-config --cflags --libs libmicrohttpd libmongoc-1.0)
```

## Run
Defaults:
- `LISTEN_PORT=8080`
- `MONGO_URI=mongodb://localhost:27017`
- `MONGO_DB=maze`
- `MONGO_COL=moves`

```bash
./maze_http_mongo
```

Or:
```bash
LISTEN_PORT=9000 MONGO_URI="mongodb://localhost:27017" MONGO_DB="maze" MONGO_COL="moves" ./maze_http_mongo
```

## Test with curl
```bash
curl -sS -X POST http://localhost:8080/move \
  -H "Content-Type: application/json" \
  -d '{"event_type":"player_move","input":{"device":"joystick","move_sequence":1},"player":{"position":{"x":1,"y":2}},"goal_reached":false,"timestamp":"2026-01-25T11:42:18Z"}'
```

## Notes for SDL integration
In your SDL maze app, you would `POST` that JSON to this service (localhost or another machine).
A common way in C is `libcurl`:
- queue events in your game loop
- send them from a worker thread (so the renderer never stalls)
