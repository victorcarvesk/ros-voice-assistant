import threading

import rclpy
import rclpy.logging
from rclpy.node import Node

from std_msgs.msg import String
from assistant_interfaces.srv import Text

import speech_recognition as sr
import sounddevice as _sounddevice  # Required to avoid microphone warnings


class MicNode(Node):

    def __init__(self):
        super().__init__('mic_node')

        self.declare_parameter('language', 'en-US')
        self.declare_parameter('keyword', 'Lucy')
        
        self.language = self.get_parameter('language').get_parameter_value().string_value
        self.keyword = self.get_parameter('keyword').get_parameter_value().string_value.lower()
        
        self.mic_pub_ = self.create_publisher(String, 'mic/transcription', 10)
        self.nlp_cli_ = self.create_client(Text, 'nlp/prompt_input')

        self.transcription = str()
        self.mic_msg = String()
        self.nlp_msg = Text.Request()

        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone(chunk_size=4096)

        
        self.listening_active = True
        self.listen_thread = threading.Thread(target=self.listening_thread, daemon=True)
        self.listen_thread.start()

    def listening_thread(self):
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source)
            self.get_logger().info("mic ready.")

            while rclpy.ok():
                try:
                    voice_input = self.recognizer.listen(source)
                    self.get_logger().info("Generating transcripion...")

                    self.transcription = self.recognizer.recognize_google(
                        voice_input, language=self.language).lower()

                    if self.keyword in self.transcription:
                        self.transcription = self.transcription.replace(self.keyword.lower(), "").strip()
                        self.nlp_msg.data = self.transcription
                        self.mic_msg.data = self.transcription

                        self.mic_pub_.publish(self.mic_msg)

                        try:
                            while not self.nlp_cli_.wait_for_service(timeout_sec=1.0):
                                self.get_logger().info("Waiting for 'nlp/prompt_input' service to be available...")
                        except KeyboardInterrupt:
                            pass

                        nlp_future = self.nlp_cli_.call_async(self.nlp_msg)
                        rclpy.spin_until_future_complete(self, nlp_future)

                except sr.WaitTimeoutError:
                    self.get_logger().warning("Timeout exceeded, no speech detected.")
                except sr.UnknownValueError:
                    self.get_logger().warning("Could not understand the audio.")
                except sr.RequestError as e:
                    self.get_logger().error(f"Error connecting to the speech recognition service: {e}.")
                except KeyboardInterrupt:
                    self.get_logger().info("Keyboard interrupt.")
                except Exception as e:
                    self.get_logger().error(f"An unexpected error occurred: {e}.")


def main(args=None):
    rclpy.init(args=args)
    mic_node = MicNode()

    try:
        rclpy.spin(mic_node)
    except KeyboardInterrupt:
        pass
    finally:
        mic_node.destroy_node()


if __name__ == '__main__':
    main()
