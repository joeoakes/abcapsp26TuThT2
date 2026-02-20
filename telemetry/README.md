## Telemetry

### Overview
All systems communicate using HTTPS POST requests with JSON payloads.
Telemetry messages are sent to:
- Mini-Pupper HTTPS Server
- Logging Server HTTPS Server
- AI HTTPS Server (Mission messages)

### Endpoint
- **Method:** POST
- **Path:** /move
- **URL Example:** https://10.170.8.101:8444/move
- **Header:** Content-Type: application/json

---

### Example Telemetry Payload
```json
{
  "event_type": "session_end",
  "level": 1,
  "input": { "device": "keyboard", "move_sequence": 1 },
  "player": { "position": { "x": 1, "y": 0 } },
  "goal_reached": false,
  "timestamp": "2026-02-19T03:30:08Z",
  "received_at": "2026-02-19T03:30:08Z"
}
```

---

### Test with curl (Run in WSL with Global Protect VPN running)
```bash
curl -k -X POST https://10.170.8.101:8444/move \
  -H "Content-Type: application/json" \
  -d '{
    "event_type": "session_end",
    "level": 1,
    "input": { "device": "keyboard", "move_sequence": 1 },
    "player": { "position": { "x": 1, "y": 0 } },
    "goal_reached": false,
    "timestamp": "2026-02-19T03:30:08Z",
    "received_at": "2026-02-19T03:30:08Z"
  }'
```
