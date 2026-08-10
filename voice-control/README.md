# Voice Control

Voice-controlled drone flight using ArduPilot SITL + MAVSDK + speech recognition.

## Structure
- `wsl_side/` — runs in WSL alongside ArduPilot SITL
  - `command_server.py` — receives voice commands over HTTP, executes them via MAVSDK (arm, takeoff, land, hold, return home, named locations, altitude control, status, emergency stop)
  - `fly_mission.py` — standalone scripted mission (arm → takeoff → goto → land), no voice involved
- `windows_side/` — runs on Windows (mic access isn't available directly in WSL)
  - `voice_listener.py` — push-to-talk mic capture, sends recognized speech to `command_server.py`, speaks back the result

## Setup
1. Start ArduPilot SITL: `sim_vehicle.py --console --map`
2. In MAVProxy console: `output add 127.0.0.1:14551`
3. In WSL: `python3 wsl_side/command_server.py`
4. On Windows: `python voice_listener.py`
5. Press Enter, speak a command (e.g. "take off", "go to point a", "land"). Say "help" for the full list.

## Requirements
- WSL: `mavsdk`, `flask`
- Windows: `SpeechRecognition`, `PyAudio`, `requests`, `pyttsx3`
