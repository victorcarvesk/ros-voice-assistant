import os

import rclpy
from rclpy.node import Node

from assistant_interfaces.srv import Text

os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = '1'

from gtts import gTTS
import pygame
from time import sleep


class SpeakerNode(Node):
    
    def __init__(self):
        super().__init__('speaker_node')

        self.declare_parameter('sync_mode', True)
        
        self.sync_mode = self.get_parameter('sync_mode').get_parameter_value().bool_value

        pygame.mixer.init()

        self.speaker_srv = self.create_service(
            Text, 'speaker/text_input', self.speaker_callback
            )

    def speaker_callback(self, request, response):
        tts = gTTS(text=request.data, lang='en')
        audio_file = "/tmp/speaker_buffer.mp3"
        tts.save(audio_file)

        pygame.mixer.music.load(audio_file)
        pygame.mixer.music.play()

        if self.sync_mode:
            while pygame.mixer.music.get_busy():
                sleep(0.1)

        os.remove(audio_file)

        return response


def main(args=None):
    rclpy.init(args=args)
    speaker_node = SpeakerNode()

    try:
        rclpy.spin(speaker_node)
    except KeyboardInterrupt:
        pass
    finally:
        speaker_node.destroy_node()


if __name__ == '__main__':
    main()
