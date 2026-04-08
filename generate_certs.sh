#!/bin/bash
# generate_certs.sh
# Generates a private CA and per-device X.509 certificates for mTLS.
#
# Run this ONCE on a trusted machine (your laptop or the Logger Server).
# Then copy each device's cert+key only to that device — never share private keys.
#
# Output layout (place under your repo's https/certs/):
#
#   https/certs/
#   ├── ca.crt              ← CA cert  (safe to share / commit)
#   ├── ca.key              ← CA key   (KEEP PRIVATE — do NOT commit to git)
#   ├── logger-server.crt   ← Logger Server identity
#   ├── logger-server.key
#   ├── ai-server.crt       ← AI Server identity
#   ├── ai-server.key
#   ├── gamehat.crt         ← Game HAT client identity (used when POSTing)
#   ├── gamehat.key
#   ├── pupper-1.crt        ← Mini-Pupper #1 identity
#   ├── pupper-1.key
#   └── pupper-2.crt        ← Mini-Pupper #2 (add more as needed)
#       pupper-2.key
#
# NETWORK COMPATIBILITY (home vs PSU):
#   Server certs (logger-server, ai-server) include their fixed PSU IPs as SANs —
#   those servers are only reachable on PSU's network anyway.
#
#   Robot certs (pupper-1, pupper-2) use DNS name SANs only (no IP) so they
#   work on both the PSU network and your home network where the IP changes.
#   Connect to the robot by hostname instead of IP:
#
#     1. On your home router, give each Pupper a static DHCP reservation.
#     2. Add entries to /etc/hosts on the Game HAT AND your laptop:
#
#          PSU network  (10.170.x.x):
#            10.170.8.136  pupper-1
#
#          Home network  (192.168.x.x — update when IP changes):
#            192.168.1.50  pupper-1
#
#     3. Run the maze with the hostname instead of IP:
#          ./maze_sdl2 pupper-1
#
# Usage:
#   chmod +x generate_certs.sh
#   ./generate_certs.sh
#
# Requirements: openssl (installed on all project devices)

set -euo pipefail

CERT_DIR="https/certs"
CA_DAYS=3650      # CA valid 10 years
CERT_DAYS=825     # Device certs valid ~2 years (Apple/browser limit)
KEY_BITS=4096

mkdir -p "$CERT_DIR"
cd "$CERT_DIR"

echo "=============================================="
echo " Team2TT mTLS Certificate Generator"
echo "=============================================="
echo ""

# ==============================================================
# STEP 1 — Create the private Certificate Authority (CA)
# ==============================================================
echo "[1/6] Generating CA key and self-signed certificate..."

openssl genrsa -out ca.key $KEY_BITS 2>/dev/null

openssl req -x509 -new -nodes \
  -key ca.key \
  -sha256 \
  -days $CA_DAYS \
  -out ca.crt \
  -subj "/C=US/ST=Pennsylvania/L=Abington/O=PennStateAbington/OU=Team2TT/CN=Team2TT-CA"

echo "    ca.key + ca.crt created"

# ==============================================================
# Helper: sign_cert <name> <cn> <san_ip>
#   name   = filename prefix (e.g. "logger-server")
#   cn     = Common Name shown in cert
#   san_ip = fixed IP to add as SAN, or "none" for DNS-only
#            (use "none" for robots that change IP on home network)
# ==============================================================
sign_cert() {
  local NAME="$1"
  local CN="$2"
  local SAN_IP="$3"

  # Always include the DNS name so hostname-based connections work.
  # Always include localhost/127.0.0.1 for local testing.
  local SAN="DNS:${CN},DNS:localhost,IP:127.0.0.1"
  if [ "$SAN_IP" != "none" ]; then
    SAN="${SAN},IP:${SAN_IP}"
  fi

  # Key
  openssl genrsa -out "${NAME}.key" $KEY_BITS 2>/dev/null

  # CSR
  openssl req -new -nodes \
    -key "${NAME}.key" \
    -out "${NAME}.csr" \
    -subj "/C=US/ST=Pennsylvania/L=Abington/O=PennStateAbington/OU=Team2TT/CN=${CN}"

  # Sign with CA — include SAN so modern TLS stacks accept it
  openssl x509 -req \
    -in "${NAME}.csr" \
    -CA ca.crt \
    -CAkey ca.key \
    -CAcreateserial \
    -out "${NAME}.crt" \
    -days $CERT_DAYS \
    -sha256 \
    -extfile <(printf "subjectAltName=%s\nbasicConstraints=CA:FALSE\nkeyUsage=digitalSignature,keyEncipherment\nextendedKeyUsage=serverAuth,clientAuth" "$SAN")

  rm -f "${NAME}.csr"
  echo "    ${NAME}.crt + ${NAME}.key created  (CN=${CN}, SAN=${SAN})"
}

# ==============================================================
# STEP 2 — Logger Server (fixed PSU IP — include as SAN)
# ==============================================================
echo "[2/6] Logger Server cert (10.170.8.130)..."
sign_cert "logger-server" "logger-server" "10.170.8.130"

# ==============================================================
# STEP 3 — AI Server (fixed PSU IP — include as SAN)
# ==============================================================
echo "[3/6] AI Server cert (10.170.8.109)..."
sign_cert "ai-server" "ai-server" "10.170.8.109"

# ==============================================================
# STEP 4 — Mini-Pupper #1
# DNS SAN only — no IP — so cert works on both PSU and home network.
# Connect by hostname "pupper-1" and maintain /etc/hosts on each device.
# ==============================================================
echo "[4/6] Mini-Pupper #1 cert (DNS only — works on any network)..."
sign_cert "pupper-1" "pupper-1" "none"

# ==============================================================
# STEP 5 — Mini-Pupper #2
# Same DNS-only approach. Add more robots by copying this block.
# ==============================================================
echo "[5/6] Mini-Pupper #2 cert (DNS only — works on any network)..."
sign_cert "pupper-2" "pupper-2" "none"

# ==============================================================
# STEP 6 — Game HAT (client cert — presented when POSTing to servers)
# DNS SAN only — Game HAT IP also changes between networks.
# ==============================================================
echo "[6/6] Game HAT client cert..."
sign_cert "gamehat" "gamehat" "none"

# ==============================================================
# Permissions — private keys readable only by owner
# ==============================================================
chmod 600 ca.key *.key
chmod 644 ca.crt *.crt

echo ""
echo "=============================================="
echo " Done. Files in: $CERT_DIR"
echo "=============================================="
echo ""
echo " DEPLOYMENT — copy only what each device needs:"
echo ""
echo "  Logger Server (10.170.8.130):"
echo "    scp https/certs/ca.crt https/certs/logger-server.crt https/certs/logger-server.key \\"
echo "        user@10.170.8.130:~/abcapsp26TuThT2/https/certs/"
echo ""
echo "  AI Server (10.170.8.109):"
echo "    scp https/certs/ca.crt https/certs/ai-server.crt https/certs/ai-server.key \\"
echo "        user@10.170.8.109:~/abcapsp26TuThT2/https/certs/"
echo ""
echo "  Mini-Pupper #1:"
echo "    scp https/certs/ca.crt https/certs/pupper-1.crt https/certs/pupper-1.key \\"
echo "        ubuntu@pupper-1:~/abcapsp26TuThT2/https/certs/"
echo ""
echo "  Mini-Pupper #2:"
echo "    scp https/certs/ca.crt https/certs/pupper-2.crt https/certs/pupper-2.key \\"
echo "        ubuntu@pupper-2:~/abcapsp26TuThT2/https/certs/"
echo ""
echo "  Game HAT (client cert for outbound curl calls):"
echo "    scp https/certs/ca.crt https/certs/gamehat.crt https/certs/gamehat.key \\"
echo "        pi@gamehat:~/abcapsp26TuThT2/https/certs/"
echo ""
echo " /etc/hosts setup (on Game HAT and laptop):"
echo "   PSU network:"
echo "     10.170.8.136  pupper-1"
echo "   Home network (update when IP changes):"
echo "     192.168.x.x   pupper-1"
echo ""
echo " Connect to robot by hostname:"
echo "   ./maze_sdl2 pupper-1"
echo ""
echo " IMPORTANT — add to .gitignore:"
echo "   echo 'https/certs/*.key' >> .gitignore"
echo "   echo 'https/certs/ca.key' >> .gitignore"
echo ""
