---
id: voice-recognition-nlp
title: Voice Recognition and Natural Language Processing
sidebar_label: Voice Recognition and NLP
---

# Voice Recognition and Natural Language Processing

To make a robot truly interactive, it needs to hear and understand. In this chapter, we build a **Voice Command Stack** using OpenAI Whisper (Ears) and GPT-4 (Brain).

## The Voice Pipeline

```mermaid
graph LR
    A[Microphone] -->|Audio Stream| B[Wake Word Engine]
    B -->|Detect 'Hey Robot'| C[Speech-to-Text (Whisper)]
    C -->|Transcribed Text| D[LLM (GPT-4)]
    D -->|JSON Command| E[ROS 2 Node]
```

## 1. Wake Word Detection

We don't want the robot listening 24/7 (privacy & cost). We use a lightweight "Wake Word" engine running locally.

**Tools:**
- **Porcupine (Picovoice)**: High accuracy, easy to use.
- **OpenWakeWord**: Open source, runs on CPU.

**Python Example (Porcupine):**
```python
import pvporcupine
import pyaudio

porcupine = pvporcupine.create(keywords=["jarvis"])
pa = pyaudio.PyAudio()
audio_stream = pa.open(
    rate=porcupine.sample_rate,
    channels=1,
    format=pyaudio.paInt16,
    input=True,
    frames_per_buffer=porcupine.frame_length
)

while True:
    pcm = audio_stream.read(porcupine.frame_length)
    keyword_index = porcupine.process(pcm)
    if keyword_index >= 0:
        print("Wake Word Detected!")
        # Trigger Whisper...
```

## 2. Speech-to-Text (Whisper)

Once awake, we record audio and send it to **OpenAI Whisper**.

**Why Whisper?**
- It's robust to accents and background noise.
- It handles technical jargon well.

**Implementation:**
```python
import openai
import sounddevice as sd
import scipy.io.wavfile as wav

def record_audio(duration=5, fs=44100):
    print("Listening...")
    recording = sd.rec(int(duration * fs), samplerate=fs, channels=1)
    sd.wait()
    wav.write("command.wav", fs, recording)
    return "command.wav"

def transcribe(filename):
    audio_file = open(filename, "rb")
    transcript = openai.Audio.transcribe("whisper-1", audio_file)
    return transcript["text"]
```

## 3. Natural Language Understanding (LLM)

Now we have text: *"Go to the kitchen and find me a soda."*
We need to turn this into a ROS 2 command.

**Prompt Engineering for Robotics:**
We give the LLM a "System Prompt" that defines its persona and available tools.

```python
SYSTEM_PROMPT = """
You are a robot assistant. You can control the robot using the following JSON commands:
1. navigate(location: str)
2. pick_up(object: str)
3. say(text: str)

User input will be natural language. You must output ONLY a valid JSON list of commands.

Example:
User: "Get me a soda from the kitchen."
Output: [
    {"action": "navigate", "location": "kitchen"},
    {"action": "pick_up", "object": "soda"},
    {"action": "navigate", "location": "user"},
    {"action": "say", "text": "Here is your soda."}
]
"""

def get_plan(user_text):
    response = openai.ChatCompletion.create(
        model="gpt-4",
        messages=[
            {"role": "system", "content": SYSTEM_PROMPT},
            {"role": "user", "content": user_text}
        ]
    )
    return response.choices[0].message.content
```

## 4. The ROS 2 Voice Node

We wrap everything into a ROS 2 node.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

class VoiceCommander(Node):
    def __init__(self):
        super().__init__('voice_commander')
        self.publisher_ = self.create_publisher(String, 'robot_commands', 10)
        self.listen_loop()

    def listen_loop(self):
        # 1. Wait for Wake Word
        # 2. Record & Transcribe
        text = transcribe("command.wav")
        self.get_logger().info(f"Heard: {text}")
        
        # 3. Get Plan from LLM
        plan_json = get_plan(text)
        
        # 4. Publish Command
        msg = String()
        msg.data = plan_json
        self.publisher_.publish(msg)

def main():
    rclpy.init()
    node = VoiceCommander()
    rclpy.spin(node)
```

## Handling Ambiguity

What if the user says *"Go there"*? The robot doesn't know where "there" is.

**Chain of Thought Prompting:**
Update the system prompt to allow the robot to ask questions.

```python
SYSTEM_PROMPT += """
If the command is ambiguous or missing information, output a JSON with action "ask":
{"action": "ask", "question": "Where would you like me to go?"}
"""
```

## Summary

You have built a **Natural Language Interface** for your robot.
- **Wake Word**: Efficient listening.
- **Whisper**: Accurate hearing.
- **LLM**: Intelligent understanding.
- **JSON**: Structured action.

---

**Next:** Now that we have the components, let's design the final system in [Capstone Project Overview](../Capstone-Project/capstone-overview.md).