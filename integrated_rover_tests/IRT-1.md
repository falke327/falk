## IRT-1 Testprotokoll – Integrierter Rover Test 1

### Datum
Juni 2025 

### Zielsetzung
- Überprüfung der Stromversorgung des Systems mit LiPo-Akku und 5V Buck Converter für den Raspberry Pi
- Überprüfung der grundlegenden Fahrfunktionen: Vorwärts, Rückwärts, Rotation links/rechts
- Erste Integrationstest des Systems mit Chassis, Verkabelung und Steuerungseinheit

### Testaufbau
- Stromversorgung:
  - LiPo 3S Akku
  - Step-down auf 6V für Antrieb über XL4016
  - Buck-Converter auf 5V (USB-Ausgang) für den Raspberry Pi
- Steuerung:
  - Raspberry Pi über SSH erreichbar
  - Programm startet manuell und steuert Motoren
- Motorsteuerung:
  - L298N H-Brücke, je eine Seite für drei Motoren
- Chassis:
  - Rocker-Bogie-System aus PVC-Rohren
  - Achsen: Schrauben durch PVC-Rohre

### Testablauf & Beobachtungen
1. **Spannungskontrolle vor Start**:
   - Netzgerät auf 11.5 V gestellt, alle Spannungswandler überprüft → OK

2. **Leerlauftest**:
   - Versorgung über Netzgerät
   - Motoren laufen im Leerlauf → OK
   - Gesamtstromaufnahme ca. 1,5 A → akzeptabel

3. **Praxisbetrieb mit Akku**:
   - Umstellen auf LiPo-Akku
   - System startet sauber, SSH erreichbar, Programm läuft
   - **Vorwärtsfahrt:** stabil
   - **Rückwärtsfahrt:** stabil
   - **Rotation rechts:** Blockade – Chassis verdreht sich durch Spiel in Gelenken

### Probleme & Beobachtungen
- Gelenke des Rocker-Bogie verdrehen sich bei Rotation
- Zwei Räder blockieren bei Rotation durch Verkanten
- Fahrwerk rutscht bei längerer Geradeausfahrt leicht auseinander
- PVC-Struktur ist unter Systemgewicht instabil

### Fazit
- **Teilerfolg:** Basisfunktionen laufen, Versorgung stabil, Steuerung funktioniert
- **Kritisch:** Mechanik aus PVC unzureichend, insbesondere bei Drehungen
- **Konsequenz:** Umstieg auf stabileres System (z. B. MakerBeam) notwendig

---

## IRT-1 Test Report – Integrated Rover Test 1

### Date
June 2025

### Objectives
- Verify power delivery using LiPo battery and 5V buck converter for the Raspberry Pi
- Verify basic movement capabilities: forward, backward, rotate left/right
- First integration test of chassis, wiring, and control unit

### Test Setup
- Power:
  - LiPo 3S battery
  - Step-down to 6V for drive system (XL4016)
  - 5V USB buck converter for Raspberry Pi
- Control:
  - Raspberry Pi accessible via SSH
  - Control program launches and runs motors
- Motor control:
  - L298N H-bridge, one side driving three motors each
- Chassis:
  - Rocker-bogie system built from PVC tubing
  - Axles made from screws through PVC tubes

### Test Procedure & Observations
1. **Voltage Check:**
   - Power supply set to 11.5 V, all converters verified → OK

2. **Idle Test:**
   - Supplied by bench power supply
   - Motors spin in idle → OK
   - Overall current ~1.5 A → acceptable

3. **Field Test with Battery:**
   - Switched to LiPo
   - System boots cleanly, SSH reachable, program starts
   - **Forward movement:** stable
   - **Reverse movement:** stable
   - **Right rotation:** blocked – chassis distorts due to joint play

### Issues & Observations
- Rocker-bogie joints twist under torque
- Two wheels jam during rotation due to mechanical misalignment
- Chassis drifts apart during prolonged straight movement
- PVC framework mechanically unstable under system load

### Conclusion
- **Partial success:** power and control functional
- **Critical:** mechanical system (PVC) inadequate for expected forces
- **Next step:** switch to more robust frame (e.g. MakerBeam)

