## System Architecture – Task Distribution

```mermaid
graph TD
    subgraph RP6 Main Board [RP6 Main Board (ATmega32)]
        M1[Motor Control<br/>(PWM, Encoders)]
        M2[Line Sensors]
        M3[Bumper Sensors]
        M4[Battery Monitoring]
        M5[Basic Diagnostics]
    end

    subgraph RP6 Extension Board [RP6 Extension Board (ATmega32)]
        E1[IR / US / Environmental Sensors]
        E2[LCD Display]
        E3[Button Input]
        E4[Sensor Fusion<br/>Data Preprocessing]
        E5[I²C/UART to Raspberry Pi]
    end

    subgraph RaspberryPi [Raspberry Pi (Python/C)]
        R1[High-Level Logic]
        R2[Pathfinding / AI]
        R3[Computer Vision]
        R4[Voice Command Processing]
        R5[Autonomous Exploration]
    end

    %% Connections
    E5 --> RaspberryPi
    M1 --> RP6 Main Board
    M2 --> RP6 Main Board
    M3 --> RP6 Main Board
    M4 --> RP6 Main Board
    M5 --> RP6 Main Board

    E1 --> RP6 Extension Board
    E2 --> RP6 Extension Board
    E3 --> RP6 Extension Board
    E4 --> RP6 Extension Board

    RP6 Extension Board --> RP6 Main Board
