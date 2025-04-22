## System Architecture – Task Distribution

```mermaid
graph TD
    subgraph RP6_Main_Board [RP6 Main Board : ATmega32]
        M1[Motor Control - PWM, Encoders]
        M2[Line Sensors]
        M3[Bumper Sensors]
        M4[Battery Monitoring]
        M5[Basic Diagnostics]
    end

    subgraph RP6_Extension_Board [RP6 Extension Board : ATmega32]
        E1[IR / US / Environmental Sensors]
        E2[LCD Display]
        E3[Button Input]
        E4[Sensor Fusion and Preprocessing]
        E5[I2C or UART to Raspberry Pi]
    end

    subgraph Raspberry_Pi [Raspberry Pi]
        R1[High-Level Logic]
        R2[Pathfinding / AI]
        R3[Computer Vision]
        R4[Voice Command Processing]
        R5[Autonomous Exploration]
    end

    %% Connections
    E5 --> Raspberry_Pi
    M1 --> RP6_Main_Board
    M2 --> RP6_Main_Board
    M3 --> RP6_Main_Board
    M4 --> RP6_Main_Board
    M5 --> RP6_Main_Board

    E1 --> RP6_Extension_Board
    E2 --> RP6_Extension_Board
    E3 --> RP6_Extension_Board
    E4 --> RP6_Extension_Board

    RP6_Extension_Board --> RP6_Main_Board
