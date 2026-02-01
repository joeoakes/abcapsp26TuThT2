# Mission Dashboard (C) – subfolder tool for Maze / Mini‑Pupper

This is a small **terminal "mission report" app** intended to be launched from the Maze SDL2 program when the **Left Trigger / L button** is pressed.

It reads mission summary fields from a Redis hash:

- `mission:{mission_id}:summary`

and prints a clean report to stdout.

---

## Folder layout (recommended)

Place this in a subfolder inside your Maze project:

```
maze/
├── maze                    (your main executable)
├── maze_sdl2.c             (your main source)
└── missions/
    ├── mission_dashboard.c
    ├── Makefile
    └── mission_dashboard   (built output)
```

Your Maze program should launch it using:

```c
execl("./missions/mission_dashboard", "mission_dashboard", mission_id, NULL);
```

---

## Build

### With Redis support (recommended)

Install dependencies:

```bash
sudo apt-get update
sudo apt-get install -y gcc make libhiredis-dev
```

Build:

```bash
cd missions
make
```

### Without Redis (prints placeholders)

```bash
cd missions
make NO_REDIS=1
```

---

## Run

```bash
./mission_dashboard <mission_id> [redis_host] [redis_port]
```

Example:

```bash
./mission_dashboard 2f1c0b5d-9d2a-4d8b-b5ad-2d7c6a0fd6b3 127.0.0.1 6379
```

---

## Expected Redis fields (Hash)

This tool attempts to read the following fields (missing fields are shown as `(none)`):

- `robot_id` (e.g. `mini-pupper-01`)
- `mission_type` (`patrol|follow|explore|delivery|search`)
- `start_time`
- `end_time`
- `moves_left_turn`
- `moves_right_turn`
- `moves_straight`
- `moves_reverse`
- `moves_total`
- `distance_traveled`
- `duration_seconds`
- `mission_result` (`success|failed|aborted`)
- `abort_reason`

---

## Quick Redis sanity test

If you want to create a fake mission record:

```bash
redis-cli HSET mission:TEST:summary \
  robot_id mini-pupper-01 \
  mission_type patrol \
  start_time "2026-02-01T10:00:00-05:00" \
  end_time "2026-02-01T10:03:12-05:00" \
  moves_left_turn 10 \
  moves_right_turn 8 \
  moves_straight 42 \
  moves_reverse 2 \
  moves_total 62 \
  distance_traveled 62 \
  duration_seconds 192 \
  mission_result success \
  abort_reason ""
```

Then run:

```bash
./mission_dashboard TEST
```

---

## Next step (if you want)

If you upload your current Maze source again, I can wire the exact trigger handling and pass the `mission_id` automatically.
