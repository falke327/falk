# FALK Roadmap

## Meilensteine

1. Fahrbereites Chassis mit Motorsteuerung
2. Integration einfacher Sensorik (Ultraschall, IR, etc.)
3. Anbindung einer Kamera und erste Bildverarbeitung
4. Bau und Steuerung eines Greifarms
5. Sprachsteuerung
6. Folgefunktion (dem Nutzer folgen)
7. Autonome Navigation und Erkundung

## Prototypphase

[ ] Erster Prototyp - Ich möchte mit einem Raspberri Pi 5 und einem L298N zwei Motoren einer RP6 Kettenwanne ansteuern.
Dabei soll der Raspberry mit einer USB Tastatur Steuerbefehle bekommen. Umsetzung erfolgt in Python. Der Pi wird über 
eine Akkubank versorgt, der L298N mit den Batterien des RP6.

[ ] Zweiter Prototyp - Überführung des ersten Prototyps in ein ROS2 Projekt. Die Anforderungen bleiben die gleichen.

[ ] Verbindungstest des Pi mit einem Pico über I2C. 

## Chassis

[ ] Aufbau des Grundchassis aus einer Kunststoffbox und PVC Rohr. Zunächst mit 6 Motoren für die Räder. Eine Lenkung ist 
noch nicht vorgesehen. Das wird zu Begin nur durch Rotation durch unterschiedliche Fahrtrichtung rechts und links möglich sein.

[ ] Aufbau einfacher Energieversurgung mit einem LiPo Akkupack. Zunächst abgesichert durch einen Summer gegen Tiefenentladung.
Einbau der Sicherung und des Hauptschalters und einer Status LED.

[ ] Aufbau der Hauptschaltung Pi, Pico, Stepdown Module und Motortreiber. Versorgung des Gesamtsystems durch den LiPo Akku.

[ ] Erste Fahrt von FALK. Anforderungen entsprechen denen von Prototyp 1. Kurzer Test der Gelände-Fähigkeiten.

[ ] Erarbeiten eines Steuerungs Konzeptes. Sollen 4 oder alle 6 Räder gelenkt werden? Wie werden die Motoren neu aufgehängt? Wie werden die notwendigen Servos angesprochen? Kann Pico 1 das auch noch leisten oder ist bereits ein zweiter notwendig?

[ ] Erarbeiten eines Lenkungsschemas. Welche Radstellungen sollen zu welcher Fortbewegung führen? Evtl. Fernsteuerung von Tastatur auf Game-Controller umstellen.

## Sensorik

[ ] Ergänzung einfacher Odometrie mit Scheibenencodern.

[ ] Ergänzung um einen Ultraschallsensor zur Objekterkennung

[ ] Die Energieversorgung soll nun durch den Pico abgesichert werden anstelle des Summers.

## Bildverarbeitung

## Greifarm

## Sprachsteuerung

## Personenfolger

## Autonome Navigation