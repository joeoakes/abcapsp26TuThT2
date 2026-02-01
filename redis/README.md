# abcapsp26FriT1

# 🐾 Mini-Pupper Swarm Exploration with Secure AI-RAG Diagnostics

**Senior Capstone – Spring 2026 Friday Team 1**  
**Penn State Abington – CMPSC & IT**

---

## 📌 Project Overview

This project implements a **secure, autonomous swarm of Mini-Pupper quadruped robots** capable of **parallel maze exploration** using **reinforcement learning**, **ROS2**, and **AI-assisted reasoning via RAG (Retrieval-Augmented Generation)**.

Robots operate locally with real-time autonomy while securely logging mission telemetry to a centralized system. A high-performance AI station performs **operator reasoning, diagnostics, and shared swarm intelligence** using vector databases.

---

## 🎯 Objectives

- Autonomous **multi-robot exploration** of an augmented-reality maze  
- Secure **robot-to-cloud telemetry and logging**
- **AI-assisted diagnostics** and mission reasoning using RAG
- Full **operational security** using certificates and mTLS
- Production-quality **testing, documentation, and DevOps workflow**
- Real-time **GUI mission dashboard** and teleoperation

---

## 🐕 Mini-Pupper Robot Platform

**Hardware**
- Raspberry Pi 4 Model B
- Quad-core ARM Cortex-A72 @ 1.5 GHz
- Camera + LiDAR
- 12 × Micro Servo Motors
- Dual power rails:
  - 5V → Raspberry Pi
  - 6V → Servos
- Wi-Fi (2.4 / 5 GHz)

**Software**
- Ubuntu Linux
- ROS2 (Foxy / Humble)
- Python
- AprilTags for identification & telemetry
- SSH for secure remote access
- X.509 digital certificates for identity

**Capabilities**
- Local autonomy
- Sensor fusion
- Secure communications
- Swarm participation

---

## 🧠 AI & Compute Infrastructure

### Quantum X Computer I9 (Telemetry & Logging)
- NVIDIA RTX 4090 GPU
- Mission + telemetry logging service
- MongoDB backend
- VPN-secured SSH access
- mTLS (mutual authentication)
- Certificate-based identity

### AI Station Spark (RAG & Reasoning)
- Spark DGX Supercomputer
- GB10 Grace Blackwell Superchip
- ~1 Petaflop performance
- 128 GB Unified LPDDR5X memory
- Secure VPN connectivity

**AI Services**
- RAG-based operator reasoning
- Diagnostics and anomaly detection
- Redis vector database for shared swarm knowledge

---

## 🎮 Teleoperation & GUI Dashboard

**Features**
- Raspberry Pi Game HAT controller
- Heads-Up Mission Dashboard
- Real-time robot health monitoring
- Mission activity visualization
- Log inspection and replay

**Tech Stack**
- Python Plotly Dash **or** React + ECharts
- FastAPI WebSockets for real-time updates
- Secure backend APIs
- MongoDB log integration

---

## 🔐 Security Architecture

- SSH for remote administration
- VPN for network isolation
- mTLS (mutual TLS):
  - Server proves identity
  - Robot/client proves identity
- X.509 certificates for every robot
- Zero-trust communication model

---

## 🧪 Testing & Quality Assurance

The project follows **industry-grade testing standards**:

- Unit Testing
- Integration Testing
- System Testing
- Regression Testing

**Tooling**
- Python testing frameworks
- CI-ready structure
- Full code coverage targets

---

## 📦 Project Management & DevOps

- All code hosted on **GitHub**
- Python dependency management via **Poetry**
- Full **PyDoc documentation**
- SCRUM methodology
  - Stand-ups twice per week
- Issue tracking and milestones
- Versioned releases

---

## 📚 Key Technologies

- Robotics: Quadrupeds, ROS2
- AI/ML: Reinforcement Learning, RAG
- Databases: MongoDB, Redis (Vector DB)
- Security: VPN, SSH, mTLS, Certificates
- Web: FastAPI, WebSockets
- Visualization: Plotly Dash / React + ECharts

---

## 🚀 Expected Outcomes

- Demonstration of **parallel swarm exploration**
- Secure, real-world robotics deployment
- AI-assisted diagnostics using modern RAG pipelines
- Fully documented, production-ready system
- Scalable foundation for future research

---

## 👨‍🏫 Academic Context

This project serves as a **Senior Capstone** for students in:

- Computer Science (CMPSC)
- Information Technology (IT)

Emphasis is placed on:
- Systems engineering
- Secure distributed computing
- Robotics + AI integration
- Professional software practices

---

## 📜 License

This project is developed for academic and research purposes.  
Licensing will be determined prior to public release.

---

## ✨ Acknowledgments

- Penn State Abington
- CMPSC & IT Programs
- Open-source robotics and AI communities

---
