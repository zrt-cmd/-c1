"""
Automatic Speech Recognition (ASR) Node

Converts speech to text using cloud or offline ASR engines.
Supports: iFlytek, Baidu, Google, Whisper, etc.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool


class ASRNode(Node):
    def __init__(self):
        super().__init__('asr_node')

        # Parameters
        self.declare_parameter('engine', 'iflytek')  # iflytek, baidu, google, whisper
        self.declare_parameter('language', 'zh_cn')
        self.declare_parameter('api_key', '')
        self.declare_parameter('api_secret', '')
        self.declare_parameter('app_id', '')

        self.engine = self.get_parameter('engine').value
        self.language = self.get_parameter('language').value
        self.api_key = self.get_parameter('api_key').value
        self.api_secret = self.get_parameter('api_secret').value
        self.app_id = self.get_parameter('app_id').value

        # Subscribers
        self.wake_word_sub = self.create_subscription(
            Bool, '/interaction/wake_word/detected',
            self._wake_word_callback, 10)

        # Publishers
        self.text_pub = self.create_publisher(
            String, '/interaction/asr/text', 10)
        self.is_listening_pub = self.create_publisher(
            Bool, '/interaction/asr/is_listening', 10)

        self.is_listening = False

        self.get_logger().info(f'ASR node started, engine: {self.engine}')
        self.get_logger().warn('TODO: Implement ASR engine integration')

        # TODO: Initialize ASR engine
        # self._init_asr_engine()

    def _wake_word_callback(self, msg):
        """Start listening after wake word is detected."""
        if msg.data and not self.is_listening:
            self.get_logger().info('Wake word detected, starting ASR...')
            self._start_listening()

    def _init_asr_engine(self):
        """Initialize ASR engine based on configured type."""
        # TODO: Implement engine initialization
        # - iFlytek: https://www.xfyun.cn/doc/asr/voicedictation/API.html
        # - Baidu: https://ai.baidu.com/ai-doc/SPEECH/Vk38lxily
        # - Google: https://cloud.google.com/speech-to-text
        # - Whisper (local): https://github.com/openai/whisper
        pass

    def _start_listening(self):
        """Start capturing audio and recognizing speech."""
        self.is_listening = True

        # Publish listening state
        msg = Bool()
        msg.data = True
        self.is_listening_pub.publish(msg)

        # TODO: Implement audio capture + ASR
        # 1. Capture audio (mic input)
        # 2. Send to ASR engine
        # 3. Get recognition result
        # 4. Publish text
        # 5. Stop listening (VAD or timeout)

        # Mock result for now
        self.get_logger().warn('Mock ASR result (TODO: implement real ASR)')
        self._publish_result("导航到厨房")

    def _publish_result(self, text):
        """Publish ASR result."""
        self.get_logger().info(f'ASR result: "{text}"')

        msg = String()
        msg.data = text
        self.text_pub.publish(msg)

        # Stop listening
        self.is_listening = False
        listening_msg = Bool()
        listening_msg.data = False
        self.is_listening_pub.publish(listening_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ASRNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
