# Redis Installation & Verification Guide
## WSL (Ubuntu) and macOS

This guide walks you through **installing Redis**, **starting the service**, and **verifying it works** on:

- Windows Subsystem for Linux (WSL2 – Ubuntu)
- macOS (Intel or Apple Silicon)

---

## Part 1: Install Redis on WSL (Ubuntu)

### 1. Open WSL
From Windows Terminal, open your Ubuntu distribution.

Verify:
```bash
lsb_release -a
```

---

### 2. Update packages
```bash
sudo apt update && sudo apt upgrade -y
```

---

### 3. Install Redis
```bash
sudo apt install redis-server -y
```

Verify installation:
```bash
redis-server --version
```

---

### 4. Start Redis

Most WSL systems:
```bash
sudo service redis-server start
```

Check status:
```bash
sudo service redis-server status
```

If systemd is not enabled:
```bash
redis-server
```
(Leave the terminal open.)

---

### 5. Test Redis (WSL)

```bash
redis-cli
```

Inside Redis:
```text
ping
```
Expected output:
```text
PONG
```

Test data:
```text
set test "hello redis"
get test
```

---

## Part 2: Install Redis on macOS

### 1. Install Homebrew (if needed)
```bash
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
```

Verify:
```bash
brew --version
```

---

### 2. Install Redis
```bash
brew update
brew install redis
```

Verify:
```bash
redis-server --version
```

---

### 3. Start Redis

Start as a background service:
```bash
brew services start redis
```

Or start manually:
```bash
redis-server
```

---

### 4. Test Redis (macOS)

```bash
redis-cli ping
```

Expected output:
```text
PONG
```

Test data:
```bash
redis-cli
set test "hello mac redis"
get test
```

---

## Part 3: Cross‑Platform Verification

### Check Redis is listening on port 6379

**WSL / Linux:**
```bash
ss -lntp | grep 6379
```

**macOS:**
```bash
lsof -i :6379
```

---

## Part 4: Common Troubleshooting

### Redis not responding?
- Make sure the server is running
- Confirm port `6379`
- Use `127.0.0.1` instead of `localhost` if needed

### Reset Redis
```bash
redis-cli FLUSHALL
```

---

## Part 5: Quick Python Test (Optional)

```python
import redis

r = redis.Redis(host="localhost", port=6379)
print(r.ping())
```

Expected:
```text
True
```

---

## Summary

Redis is now installed, running, and verified on your system.

You can safely use it for:
- Robotics mission logging
- AI agents
- C / Python / ROS2 projects

Happy hacking!
