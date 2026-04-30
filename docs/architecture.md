# Architecture

1. SITL is launched using Python subprocess
2. MAVSDK connects via UDP
3. Mission is executed automatically
4. PX4 logs are generated
5. Logs are extracted and analyzed

## Data Flow

PX4 SITL → MAVSDK → Mission → Log → Python Analysis
