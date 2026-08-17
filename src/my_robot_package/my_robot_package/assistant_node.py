#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import speech_recognition as sr
from openai import OpenAI
from std_msgs.msg import String as StringMsg
from pathlib import Path
from cv_bridge import CvBridge
import os
import json
from ddgs import DDGS
import sys
import threading
from contextlib import contextmanager
from datetime import datetime
import cv2
import base64
import subprocess
import time

from my_robot_package.assistant_logic import command_for_robot_mode


@contextmanager
def suppress_c_warnings():
    """Temporarily redirect OS-level stderr to /dev/null to hide ALSA and JACK spam."""
    sys.stderr.flush()
    null_fd = os.open(os.devnull, os.O_RDWR)
    save_fd = os.dup(2)
    os.dup2(null_fd, 2)
    try:
        yield
    finally:
        sys.stderr.flush()
        os.dup2(save_fd, 2)
        os.close(null_fd)
        os.close(save_fd)


class AssistantNode(Node):
    def __init__(self):
        super().__init__('assistant_node')

        # Initialize OpenAI (Requires OPENAI_API_KEY environment variable)
        api_key = os.environ.get("OPENAI_API_KEY")
        if not api_key:
            raise RuntimeError(
                "OPENAI_API_KEY is not set. Load it through the startup environment."
            )
        self.declare_parameter(
            "openai_model", os.environ.get("OPENAI_MODEL", "gpt-4o")
        )
        self.declare_parameter("allow_shell_commands", False)
        self.declare_parameter(
            "microphone_sample_rate",
            int(os.environ.get("ROBOT_MIC_SAMPLE_RATE", "16000")),
        )
        self.openai_model = str(self.get_parameter("openai_model").value)
        self.allow_shell_commands = bool(
            self.get_parameter("allow_shell_commands").value
        )
        self.microphone_sample_rate = int(
            self.get_parameter("microphone_sample_rate").value
        )
        self.client = OpenAI(api_key=api_key)
        self.recognizer = sr.Recognizer()
        self.cv_bridge = CvBridge()

        # --- PERFORMANCE TUNING ---
        # Lower threshold makes the mic more sensitive to catch the wake word
        self.recognizer.energy_threshold = 1000
        # Shorter pause threshold for faster response after user stops talking
        self.recognizer.pause_threshold = 1.5

        # Initialize chat history with a smarter system prompt
        self.chat_history = [
            {"role": "system", "content": self._system_prompt()}
        ]

        # --- VISION: Subscribe to the raw camera image feed and detections ---
        self.create_subscription(
            Image,
            '/oakd/color/image_raw',
            self.image_callback,
            10
        )
        self.latest_frame = None
        self.last_frame_time = 0
        self.image_lock = threading.Lock()
        self.get_logger().info("Subscribed to /oakd/color/image_raw for VLM analysis.")

        self.tools = [
            {
                "type": "function",
                "function": {
                    "name": "search_web",
                    "description": "Search the internet for real-time information, news, weather, or live sports scores.",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "query": {"type": "string", "description": "The search query to use."}
                        },
                        "required": ["query"]
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "execute_bash_command",
                    "description": "Execute a bash command on the robot's Linux system. Use this for general system tasks, file operations, or running scripts.",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "command": {"type": "string", "description": "The bash command to run. Remember to use '&' for long-running launch files."}
                        },
                        "required": ["command"]
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "set_robot_mode",
                    "description": "Control person following, framing, photos, and video recording.",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "mode": {
                                "type": "string",
                                "description": "The desired mode.",
                                "enum": [
                                    "follow", "keep_frame", "take_photo",
                                    "start_recording", "stop_recording",
                                    "enroll_target", "clear_enrollment",
                                    "orbit", "stop"
                                ]
                            }
                        },
                        "required": ["mode"]
                    }
                }
            },
            {
                "type": "function",
                "function": {
                    "name": "analyze_visual_scene",
                    "description": "Analyzes the robot's camera view to answer a specific question about the environment. Use this for any visual questions like 'What do you see?', 'Describe the room.', 'What kind of plant is that?', or 'Is there a person here?'.",
                    "parameters": {
                        "type": "object",
                        "properties": {
                            "question": {"type": "string", "description": "The specific question to answer about the scene."}
                        },
                        "required": ["question"]
                    }
                }
            }
        ]
        if not self.allow_shell_commands:
            self.tools = [
                tool for tool in self.tools
                if tool["function"]["name"] != "execute_bash_command"
            ]

        # Publisher for sending commands to the follower node
        self.command_publisher = self.create_publisher(
            StringMsg,
            '/assistant/command',
            10)

        # Start the listening loop in a background thread
        self.listen_thread = threading.Thread(target=self.listening_loop)
        self.listen_thread.daemon = True
        self.listen_thread.start()

    def _system_prompt(self, now=None):
        prompt = (
            "You are Bob, a helpful companion robot voice assistant. Use "
            "`analyze_visual_scene` for questions about the camera view, `search_web` "
            "for current information, and `set_robot_mode` for following, framing, "
            "photos, recording, and enrolling or clearing the tracked subject. "
            "Explain that orbit mode is not yet available if "
            "the user requests it. Keep spoken answers brief and in plain English."
        )
        if self.allow_shell_commands:
            prompt += " Use `execute_bash_command` for requested robot system tasks."
        if now:
            prompt += " The current local date and time is %s." % now
        return prompt

    # --- VISION: Callback to store the latest camera frame ---
    def image_callback(self, msg):
        with self.image_lock:
            self.latest_frame = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            self.last_frame_time = time.time()

    # --- VISION: Function that the AI tool will call ---
    def analyze_visual_scene(self, question):
        self.get_logger().info(f"Analyzing scene with question: '{question}'")
        with self.image_lock:
            if self.latest_frame is None or (time.time() - self.last_frame_time) > 3.0:
                return "I'm not receiving a camera feed right now. Please check if the camera node is running."

            # Encode the image to base64
            _, buffer = cv2.imencode('.jpg', self.latest_frame)
            base64_image = base64.b64encode(buffer).decode('utf-8')

        try:
            # Call OpenAI's Vision API
            response = self.client.chat.completions.create(
                model=self.openai_model,
                messages=[
                    {"role": "system", "content": "You are a highly intelligent robotic vision system. Analyze the image to answer the user's question. If asked to identify objects, provide specific details like exact brand names, models, or plant species. If there is foreign text in the image, you must translate it to English. Keep answers brief (1-2 sentences) and conversational."},
                    {"role": "user", "content": [
                        {"type": "text", "text": question},
                        {"type": "image_url", "image_url": {"url": f"data:image/jpeg;base64,{base64_image}", "detail": "low"}}
                    ]}
                ],
                max_tokens=150
            )
            description = response.choices[0].message.content
            self.get_logger().info(f"VLM Analysis: {description}")
            return description
        except Exception as e:
            self.get_logger().error(f"Error calling VLM API: {e}")
            return "I'm having trouble analyzing the image right now."

    def listening_loop(self):
        mic = None
        source = None
        while rclpy.ok():
            try:
                # PipeWire handles shared access when available. The 16 kHz
                # fallback also works with the EMEET USB device's raw ALSA mode.
                with suppress_c_warnings():
                    mic = sr.Microphone(
                        device_index=None,
                        sample_rate=self.microphone_sample_rate,
                    )
                    source = mic.__enter__()

                if source.stream is None:
                    raise OSError(
                        "PyAudio returned no input stream for the default device."
                    )
                self.get_logger().info(
                    "Microphone connected at %d Hz."
                    % self.microphone_sample_rate
                )
                break
            except Exception as error:
                if getattr(mic, 'stream', None) is not None:
                    mic.__exit__(None, None, None)
                mic = None
                source = None
                self.get_logger().error(
                    "Microphone init failed: %s. Retrying in 5 seconds."
                    % error
                )
                time.sleep(5)

        if source is None:
            return

        try:
            self.get_logger().info("Bob Online. Waiting for wake word 'Hi Bob'...")
            # Calibrate briefly for background noise
            self.recognizer.adjust_for_ambient_noise(source, duration=1)

            while rclpy.ok():
                try:
                    # Listen in shorter bursts for the wake word for better responsiveness
                    audio = self.recognizer.listen(source, timeout=0.5, phrase_time_limit=3)
                    text = self.recognizer.recognize_google(audio).lower()

                    # Phonetic variations in case the STT engine misinterprets "Bob"
                    wake_words = ["hi bob", "hey bob", "high bob", "hello bob"]
                    if any(w in text for w in wake_words):
                        self.get_logger().info(f"Wake word detected! (Heard: '{text}')")
                        subprocess.run(
                            ["espeak-ng", "Yes?"],
                            check=False,
                            stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL,
                        )

                        # Enter an active conversation loop
                        conversation_active = True
                        while conversation_active and rclpy.ok():
                            self.get_logger().info("Listening for command...")
                            try:
                                # Wait up to 8 seconds for the user to reply before ending the active conversation
                                audio_cmd = self.recognizer.listen(source, timeout=8, phrase_time_limit=25)
                                cmd_text = self.recognizer.recognize_google(audio_cmd)
                                self.get_logger().info(f"You said: {cmd_text}")

                                self.process_and_respond(cmd_text)
                            except sr.WaitTimeoutError:
                                self.get_logger().info("Conversation ended due to silence.")
                                conversation_active = False
                            except sr.UnknownValueError:
                                # End active conversation if we hear indistinguishable noise
                                conversation_active = False
                        self.get_logger().info("Waiting for wake word 'Hi Bob'...")
                    # No 'else' block to avoid spamming logs with background noise

                except sr.WaitTimeoutError:
                    pass  # Normal timeout while waiting for wake word, loop again
                except sr.UnknownValueError:
                    pass  # Background noise, ignore
                except Exception as e:
                    self.get_logger().error(f"Audio Error: {e}")
        finally:
            if getattr(source, 'stream', None) is not None:
                mic.__exit__(None, None, None)

    def process_and_respond(self, text):
        try:
            # Update the system prompt with the current exact date and time
            now = datetime.now().strftime("%A, %B %d, %Y %I:%M %p")
            self.chat_history[0]["content"] = self._system_prompt(now)

            # Add user input to history
            self.chat_history.append({"role": "user", "content": text})

            # Keep history manageable: System prompt + 30 recent messages
            # (Increased because web searches require saving extra messages)
            if len(self.chat_history) > 31:
                self.chat_history = [self.chat_history[0]] + self.chat_history[-30:]

            response = self.client.chat.completions.create(
                model=self.openai_model,
                messages=self.chat_history,
                tools=self.tools,
                tool_choice="auto"
            )

            message = response.choices[0].message

            # Did the AI decide it needs to search the web?
            if message.tool_calls:
                self.get_logger().info("AI is using a tool...")
                # Store plain JSON-compatible data, not an SDK-specific object.
                self.chat_history.append(
                    message.model_dump(exclude_none=True)
                )

                for tool_call in message.tool_calls:
                    try:
                        args = json.loads(tool_call.function.arguments)
                    except (json.JSONDecodeError, TypeError) as error:
                        self.get_logger().warning(
                            "Invalid arguments for tool %s: %s"
                            % (tool_call.function.name, error)
                        )
                        self.chat_history.append({
                            "role": "tool",
                            "tool_call_id": tool_call.id,
                            "content": "The tool arguments were invalid JSON."
                        })
                        continue

                    if tool_call.function.name == "search_web":
                        query = str(args.get("query", "")).strip()
                        self.get_logger().info(f"[DuckDuckGo Searching: {query}]")
                        try:
                            if not query:
                                raise ValueError("Search query is empty")
                            results = list(DDGS().text(query, max_results=3))
                            search_result = json.dumps(results)
                        except Exception as e:
                            search_result = f"Search failed: {e}"

                        self.chat_history.append({
                            "role": "tool",
                            "tool_call_id": tool_call.id,
                            "content": search_result
                        })
                    elif tool_call.function.name == "execute_bash_command":
                        cmd = args.get('command', '')
                        self.get_logger().info(f"[Executing Bash: {cmd}]")
                        try:
                            # Run the command with a 10s timeout so the AI doesn't hang forever
                            result = subprocess.run(cmd, shell=True, text=True, capture_output=True, timeout=10)
                            output = result.stdout if result.stdout else result.stderr
                            if not output:
                                output = "Command executed successfully with no output."
                            output = output[:2000]  # Truncate massive logs
                        except subprocess.TimeoutExpired:
                            output = "Command timed out after 10 seconds. (If this was a launch file, it is running in the background)."
                        except Exception as e:
                            output = f"Command failed: {e}"

                        self.chat_history.append({"role": "tool", "tool_call_id": tool_call.id, "content": output})

                    elif tool_call.function.name == "set_robot_mode":
                        mode = args.get('mode', 'stop')
                        self.get_logger().info(f"[Setting Robot Mode: {mode}]")
                        command_str = command_for_robot_mode(mode)
                        self.command_publisher.publish(StringMsg(data=command_str))
                        tool_output = f"Command '{mode}' sent to the robot's motion controller."
                        self.chat_history.append({"role": "tool", "tool_call_id": tool_call.id, "content": tool_output})

                    elif tool_call.function.name == "analyze_visual_scene":
                        question = args.get('question', 'What do you see?')
                        camera_data = self.analyze_visual_scene(question=question)
                        self.chat_history.append({
                            "role": "tool",
                            "tool_call_id": tool_call.id,
                            "content": camera_data
                        })
                    else:
                        self.chat_history.append({
                            "role": "tool",
                            "tool_call_id": tool_call.id,
                            "content": "Unknown tool requested."
                        })

                # Call OpenAI a second time, this time with the search results included!
                response = self.client.chat.completions.create(
                    model=self.openai_model,
                    messages=self.chat_history
                )
                message = response.choices[0].message

            reply = message.content or "I couldn't form a response. Please try again."
            self.get_logger().info(f"Robot says: {reply}")

            # Add the AI's response to history so it remembers the conversation
            self.chat_history.append({"role": "assistant", "content": reply})

            # --- UPGRADE: Use OpenAI's high-quality TTS for a natural voice ---
            self.speak(reply)

        except Exception as e:
            self.get_logger().error(f"AI/Network Error: {e}")

    def speak(self, text):
        try:
            # It requires an internet connection and will incur small API costs.
            # NOTE: You may need to install an MP3 player: sudo apt-get install mpg123
            speech_file_path = Path("/tmp/assistant_reply.mp3")
            with self.client.audio.speech.with_streaming_response.create(
                model="tts-1",
                voice="alloy",  # Other voices: echo, fable, onyx, nova, shimmer
                input=text
            ) as response:
                response.stream_to_file(speech_file_path)

            # Play the generated audio file
            subprocess.run(
                ["mpg123", str(speech_file_path)],
                check=False,
            )

        except Exception as e:
            self.get_logger().error(f"AI/Network Error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = AssistantNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
