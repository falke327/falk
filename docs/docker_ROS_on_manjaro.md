# ROS 2 Jazzy Desktop unter Manjaro via Docker

Diese Anleitung beschreibt die Installation und Nutzung von ROS 2 Jazzy Desktop unter Manjaro Linux mithilfe von Docker. Es wird ein eigenes Docker-Image gebaut, das die Desktop-Komponenten von ROS 2 enthält. Ziel ist es, mit einem Talker/Listener-Test die Kommunikation mit einem Raspberry Pi (ROS im Container) zu prüfen.

---

## Voraussetzungen

* Manjaro Linux (x86\_64)
* Docker installiert
* Benutzer ist Mitglied der Docker-Gruppe
* X11-basierte Desktopumgebung

---

## Schritt 1: Docker installieren (falls nicht vorhanden)

```bash
sudo pacman -S docker
sudo systemctl enable --now docker
sudo usermod -aG docker $USER
newgrp docker
```

---

## Schritt 2: X11-Freigabe für GUI-Anwendungen (rqt, rviz2)

```bash
xhost +local:docker
```

Das erlaubt Containern den Zugriff auf den X-Server.

---

## Schritt 3: Dockerfile für ROS 2 Jazzy Desktop erstellen

Erstelle eine Datei namens `Dockerfile.jazzy-desktop`:

```Dockerfile
FROM osrf/ros:jazzy-desktop

RUN apt update && apt install -y \
    ros-jazzy-desktop \
    && apt clean

CMD ["bash"]
```

Dieses Image erweitert das ros-base-Image um grafische Tools wie `rviz2`, `rqt`, etc.

---

## Schritt 4: Docker-Image bauen

```bash
docker build -f Dockerfile.jazzy-desktop -t ros2:jazzy-desktop .
```

Der Build dauert je nach Internetgeschwindigkeit und Rechenleistung einige Minuten.

---

## Schritt 5: Container starten (mit GUI-Unterstützung)

```bash
docker run -it \
  --net=host \
  --env="DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --name ros2_jazzy_desktop \
  ros2:jazzy-desktop
```

**Parameter erklärt:**

* `--net=host`: Notwendig für DDS-Kommunikation (ROS 2 Discovery)
* `DISPLAY` + Volume: Erlaubt GUI-Ausgabe auf Host-Desktop
* `--name`: Benennung des Containers

---

## Schritt 6: Kommunikation testen (Talker / Listener)

### Terminal A: Auf Manjaro (Desktop Container)

```bash
source /opt/ros/jazzy/setup.bash
ros2 run demo_nodes_cpp talker
```

### Terminal B: Auf Raspberry Pi (ROS-Container)

```bash
source /opt/ros/jazzy/setup.bash
ros2 run demo_nodes_cpp listener
```

Beide Container müssen `--net=host` verwenden, damit ROS 2 Discovery funktioniert.

---

## Fehlerbehebung

### 🔹 Netzwerkcheck

```bash
ping <raspberry_pi_ip>
```

### 🔹 Firewall deaktivieren (bei Problemen)

```bash
sudo ufw disable
# oder
sudo systemctl stop firewalld
```

### 🔹 ROS 2 Ports prüfen (UDP 7400 etc.)

```bash
ss -tulpn | grep 7400
```

---

## Weiterführende Schritte

* Integration von `docker-compose` zur dauerhaften Verwaltung
* Mount von ROS 2 Workspaces (`-v ~/ros2_ws:/root/ros2_ws`)
* Nutzung von `rviz2`, `rqt`, `ros2 topic echo`, etc.
* Eigene Nodes (Python/C++) entwickeln

---

## Sicherheitshinweis

X11 und `--net=host` + Docker bedeutet wenig Isolierung. Verwende Images nur aus vertrauenswürdigen Quellen oder baue selbst, wie oben gezeigt.

---

## Statusüberblick

| Komponente       | Status        |
| ---------------- | ------------- |
| ROS 2 Docker Pi  | ✅             |
| ROS 2 Desktop PC | ✅             |
| Talker/Listener  | 🔄 Testbereit |
| rviz2/rqt        | 🔲 Optional   |


