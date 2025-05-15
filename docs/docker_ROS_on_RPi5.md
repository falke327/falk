# ROS 2 Jazzy in Docker auf Raspberry Pi OS (Bookworm)

Diese Anleitung beschreibt die komplette Einrichtung von Docker auf einem Raspberry Pi (mit Raspberry Pi OS Bookworm), den Bau eines eigenen ROS 2 Jazzy Docker-Images (ros-base), sowie den Start eines Containers mit passender Konfiguration.

## Voraussetzungen

* Raspberry Pi mit Raspberry Pi OS Bookworm (Lite oder Desktop)
* Netzwerkzugriff (SSH oder lokal)
* ARM64-Architektur (empfohlen)

---

## Schritt 1: Docker auf dem Raspberry Pi installieren

```bash
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
```

Nach der Installation den aktuellen Benutzer zur Docker-Gruppe hinzufügen:

```bash
sudo usermod -aG docker $USER
```

**Hinweis:** Danach abmelden und erneut anmelden (oder `newgrp docker`), um Docker ohne `sudo` verwenden zu können.

Test:

```bash
docker run hello-world
```

---

## Schritt 2: Dockerfile für ROS 2 Jazzy erstellen

Erstelle eine Datei namens `Dockerfile.jazzy` mit folgendem Inhalt:

```Dockerfile
FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt update && apt install -y \
    locales curl gnupg2 lsb-release \
    && locale-gen en_US.UTF-8 \
    && update-locale LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8 \
    && apt clean

ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | \
    gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=arm64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/ros2.list

RUN apt update && apt install -y \
    ros-jazzy-ros-base \
    python3-argcomplete \
    && apt clean

RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

CMD ["bash"]
```

---

## Schritt 3: Docker-Image bauen

```bash
docker build -f Dockerfile.jazzy -t ros2:jazzy .
```

**Hinweis:** Der Build-Vorgang kann je nach Pi-Modell 10–60 Minuten dauern.

Optional kann das Image auch auf einem schnelleren Rechner (ARM64-kompatibel) gebaut und per `docker save` / `docker load` übertragen werden.

---

## Schritt 4: Container starten

### Interaktiver Entwicklungscontainer:

```bash
docker run -it --net=host --privileged --name=ros2_jazzy ros2:jazzy
```

**Erklärung:**

* `-it`: Interaktive Shell
* `--net=host`: ROS 2 benötigt Host-Netzwerkzugriff (z. B. für DDS)
* `--privileged`: Zugriff auf Geräte wie Kamera, GPIO, I2C
* `--name=ros2_jazzy`: Name des Containers

### Persistenter Container mit Workspace:

```bash
docker run -it --net=host --privileged \
  -v ~/ros2_ws:/root/ros2_ws \
  --name=ros2_jazzy_dev \
  ros2:jazzy
```

Damit bleibt der ROS-Workspace erhalten.

---

## Schritt 5: ROS 2 im Container nutzen

Im Container musst du zunächst die ROS-Umgebung aktivieren:

```bash
source /opt/ros/jazzy/setup.bash
```

Beispiel:

```bash
ros2 run demo_nodes_cpp talker
```

---

## Weitere Hinweise

* Volumes können genutzt werden für Workspaces, Konfigdateien, Logs etc.
* Zusätzliche Devices (z. B. `/dev/video0`) können mit `--device=/dev/video0` eingebunden werden.
* Für GPIO/I2C/SPI ist `--privileged` oder gezieltes Mounten von `/dev` nötig.

---

## Sicherheitshinweis

Container mit `--privileged` sind **nicht isoliert**. Wenn du fremde Images nutzt, besteht das Risiko vollständiger Host-Kompromittierung.
Daher: Immer vertrauenswürdige Dockerfiles verwenden und möglichst selbst bauen.

---

## Nächste Schritte

* Docker-Compose Setup?
* Erweiterung des Docker-Images für Kamera, GPIO, rclpy?
* Automatischer Start beim Booten?

