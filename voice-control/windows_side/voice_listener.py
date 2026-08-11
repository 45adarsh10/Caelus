"""
Push-to-talk voice command capture (run this on WINDOWS, in PowerShell)
Press Enter, then speak one command clearly. Repeats until you type 'q' + Enter to quit.
More reliable than always-on listening since it avoids picking up background noise.
"""

import speech_recognition as sr
import requests

recognizer = sr.Recognizer()

print("Available microphones:")
for i, name in enumerate(sr.Microphone.list_microphone_names()):
    print(f"  {i}: {name}")

MIC_INDEX = 1  # change this if it doesn't pick up your mic correctly
SERVER_URL = "http://localhost:5005/command"


def send_command(text):
    try:
        response = requests.post(SERVER_URL, json={"command": text}, timeout=10)
        print(f"  -> Sent. Server response: {response.json()}")
    except requests.exceptions.ConnectionError:
        print("  -> Could not reach the WSL command server. Is command_server.py running?")


print(f"\nUsing microphone index {MIC_INDEX}.")
print("Calibrating for ambient noise, please stay quiet for a moment...")

with sr.Microphone(device_index=MIC_INDEX) as source:
    recognizer.adjust_for_ambient_noise(source, duration=1.5)
    recognizer.pause_threshold = 0.8

print(f"Energy threshold set to: {recognizer.energy_threshold}")
print("\nReady. Press ENTER then speak one command. Type 'q' + ENTER to quit.\n")

while True:
    user_input = input(">>> Press ENTER to speak (or 'q' to quit): ")
    if user_input.strip().lower() == "q":
        print("Exiting.")
        break

    with sr.Microphone(device_index=MIC_INDEX) as source:
        print("Listening...")
        audio = recognizer.listen(source, timeout=5, phrase_time_limit=6)

    print("Recognizing...")
    try:
        text = recognizer.recognize_google(audio)
        print(f"You said: {text}")
        send_command(text.lower().strip())

    except sr.WaitTimeoutError:
        print("No speech detected -- try again.")
    except sr.UnknownValueError:
        print("Couldn't understand that -- try speaking more clearly.")
    except sr.RequestError as e:
        print(f"Speech API error: {e}")

    print()  # blank line for readability between attempts