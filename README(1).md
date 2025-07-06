# ROS-Filter-Vergleich (KF, EKF, PF)

Dieses Projekt vergleicht Kalman Filter (KF), Extended Kalman Filter (EKF) und Particle Filter (PF) mit einem TurtleBot3 in einer simulierten ROS-Umgebung. Alle Filter wurden individuell implementiert und getestet.

## 🔧 Installation

```bash
git clone <REPO-URL> ./start_ros_container.sh
cd start_ros_container.sh
```

> Stelle sicher, dass Docker und NVIDIA Container Toolkit korrekt eingerichtet sind.

## ▶️ Projekt starten

```bash
./start_ros_container.sh
```

Im Container:

```bash
roslaunch problab all_filter.launch
```

Dies lädt alle drei Filter (KF, EKF, PF) gleichzeitig zur Ausführung und zum Vergleich.

## 🔀 Einzelne Filter starten

Jeder Filter kann auch separat gestartet werden:

- **Kalman Filter (KF):**
  ```bash
  roslaunch problab kf.launch
  ```

- **Extended Kalman Filter (EKF):**
  ```bash
  roslaunch problab ekf.launch
  ```

- **Particle Filter (PF):**
  ```bash
  roslaunch problab pf.launch
  ```

## 📁 Projektstruktur

```
start_ros_container.sh/
├── docker/                  # Dockerfiles und Setup-Skripte
├── ros_ws_prolab/           # ROS-Workspace mit Source-Code und Launch-Files
│   ├── src/
│   └── launch/
│       ├── all_filter.launch
│       ├── kf.launch
│       ├── ekf.launch
│       └── pf.launch
└── start_ros_container.sh   # Startskript für Docker-Container
```

## 📌 Voraussetzungen

- Ubuntu 20.04
- Docker & NVIDIA Container Toolkit
- ROS Noetic
- TurtleBot3 Simulation
