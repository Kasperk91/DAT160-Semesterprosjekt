# DAT160 – Search and Rescue

Semesterprosjekt i DAT160 Intelligent Robots, HVL, høsten 2026.

## Om prosjektet

To autonome TurtleBot3-roboter utforsker en skadet bygning,
oppdager ArUco-markører og rapporterer posisjonene deres
til poengsystemet.

Løsningen benytter:

- A* for ruteplanlegging
- Kartbasert navigasjon
- Synlighetsbasert utforsking
- Automatisk kameraskanning
- ArUco-markørgjenkjenning
- TF2-koordinattransformasjoner
- Kommunikasjon og samarbeid gjennom ROS2

Ved oppdagelse av stor brann samles begge robotene ved
brannen før de fortsetter den individuelle utforskingen.

Markørposisjoner og kjøreruter er ikke forhåndsprogrammert.

## Systemkrav

- Ubuntu 20.04
- ROS2 Foxy
- Gazebo
- Python 3
- OpenCV med ArUco-støtte
- NumPy
- transforms3d
- tf_transformations

Testede Python-versjoner:

- opencv-python: 4.12.0.88
- numpy: 1.17.4
- transforms3d: 0.4.2

## Installasjon

Kjør følgende fra prosjektets rotmappe:

    source /opt/ros/foxy/setup.bash
    rosdep install --from-paths src --ignore-src -r -y
    colcon build --symlink-install

## Oppstart

Åpne to terminaler i prosjektets rotmappe.

Kjør følgende i begge terminalene:

    source /opt/ros/foxy/setup.bash
    source install/setup.bash
    export ROS_DOMAIN_ID=24
    export ROS_LOCALHOST_ONLY=1
    export GAZEBO_MODEL_PATH="$PWD/src/multi_robot_challenge_23/models:$GAZEBO_MODEL_PATH"

Terminal 1 – Simulering og robotstyring:

    ros2 launch rescue_navigation rescue_full.launch.py scenario:=w1

Terminal 2 – Poengsystem:

    ros2 run scoring scoring

Dersom poengsystemet allerede kjører, trenger man ikke
starte det på nytt.

## Kartvalg

Prosjektet støtter fem kartvarianter:

- w1
- w2
- w3
- w4
- w5

Eksempel på oppstart med kart 3:

    ros2 launch rescue_navigation rescue_full.launch.py scenario:=w3

Kart, startposisjoner og arbeidsfordeling tilpasses
automatisk til valgt variant.

## Testresultater

Kart 1: 700 poeng, fire godkjente markører og vellykket
samarbeid ved stor brann.

Kart 2: Autonom navigasjon, kameraskanning og oppstart
av poengsystemet er verifisert.

Kart 3–5 er konfigurert, men ikke individuelt testet.

## Prosjektstruktur

- rescue_navigation: Navigasjon, utforsking og koordinering
- rescue_interfaces: Egne ROS2-meldinger
- multi_robot_challenge_23: Simulering, kart og modeller
- multi_robot_scoring: Poengsystem
- ros2_aruco: Markørgjenkjenning
- turtlebot3_manipulation: Robotmodeller og støttepakker

