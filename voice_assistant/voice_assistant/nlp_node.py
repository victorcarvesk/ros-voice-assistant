from threading import Event

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from assistant_interfaces.srv import Text


class NlpNode(Node):
    
    def __init__(self):
        super().__init__('nlp_node')

        self.callback_group = ReentrantCallbackGroup()
        
        self.speaker_cli = self.create_client(Text, 'speaker/text_input', callback_group=self.callback_group)

        self.nlp_srv = self.create_service(Text, 'nlp/prompt_input', self.nlp_callback)

        self.prompt_output = Text.Request()

    def nlp_callback(self, request, response):
        match request.data.lower():

            case greeting if any(tag in greeting for tag in ["hello", "hi", "hey", "greetings", "what's up"]):
                answer = "Hey there! How's it going today?"

            case "how are you" | "how's it going" | "how do you feel":
                answer = "I'm just a bot, but I'm feeling pretty good! Thanks for asking. How about you?"

            case "tell me a joke" | "make me laugh":
                answer = "Why did the robot go on vacation? Because it needed to recharge its batteries!"

            case "what's the weather" | "what's the weather like":
                answer = "I wish I could check the weather for you, but my sensors aren't that advanced yet!"

            case "what can you do" | "what are your abilities":
                answer = "I can chat with you, tell jokes, and maybe even learn new things if you teach me! What would you like to talk about?"

            case "goodbye" | "see you later" | "bye":
                answer = "Goodbye! It was nice chatting with you. Come back soon!"

            case "who are you" | "what are you":
                answer = "I'm your friendly chatbot! My job is to keep you entertained and assist you. Anything you'd like to know?"

            case _:
                answer = "Hmm, I'm not sure I understand. Could you repeat that, please?"

        self.prompt_output.data = answer

        try:
            while not self.speaker_cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info("Waiting for speaker service to be available...")
        except KeyboardInterrupt:
            pass

        event = Event()

        def done_callback(future):
            nonlocal event
            event.set()

        future = self.speaker_cli.call_async(self.prompt_output)
        future.add_done_callback(done_callback)

        event.wait()
        return response


def main(args=None):
    rclpy.init(args=args)
    nlp_node = NlpNode()
    executor = MultiThreadedExecutor()

    try:
        rclpy.spin(nlp_node, executor)
    except KeyboardInterrupt:
        pass
    finally:
        nlp_node.destroy_node()


if __name__ == '__main__':
    main()
