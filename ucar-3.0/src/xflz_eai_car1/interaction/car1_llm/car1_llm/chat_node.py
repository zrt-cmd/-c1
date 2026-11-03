"""
LLM Chat Node

Handles conversation with Large Language Models (OpenAI, Claude, local models, etc.)
Integrates with ASR/TTS for voice interaction.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json


class ChatNode(Node):
    def __init__(self):
        super().__init__('chat_node')

        # Parameters
        self.declare_parameter('llm_provider', 'openai')  # openai, anthropic, ollama, spark
        self.declare_parameter('model', 'gpt-4')
        self.declare_parameter('api_key', '')
        self.declare_parameter('api_base', '')
        self.declare_parameter('temperature', 0.7)
        self.declare_parameter('max_tokens', 500)
        self.declare_parameter('system_prompt', 'You are a helpful robot assistant.')

        self.llm_provider = self.get_parameter('llm_provider').value
        self.model = self.get_parameter('model').value
        self.api_key = self.get_parameter('api_key').value
        self.api_base = self.get_parameter('api_base').value
        self.temperature = self.get_parameter('temperature').value
        self.max_tokens = self.get_parameter('max_tokens').value
        self.system_prompt = self.get_parameter('system_prompt').value

        # Conversation history
        self.messages = [
            {"role": "system", "content": self.system_prompt}
        ]

        # Subscribers
        self.asr_sub = self.create_subscription(
            String, '/interaction/asr/text',
            self._asr_callback, 10)
        self.user_input_sub = self.create_subscription(
            String, '/interaction/llm/user_input',
            self._user_input_callback, 10)

        # Publishers
        self.reply_pub = self.create_publisher(
            String, '/interaction/llm/assistant_reply', 10)
        self.tts_pub = self.create_publisher(
            String, '/interaction/tts/speak', 10)
        self.skill_request_pub = self.create_publisher(
            String, '/interaction/llm/skill_request', 10)

        self.get_logger().info(f'Chat node started, provider: {self.llm_provider}, model: {self.model}')
        self.get_logger().warn('TODO: Implement LLM API integration')

        # TODO: Initialize LLM client
        # self._init_llm_client()

    def _init_llm_client(self):
        """Initialize LLM client based on provider."""
        # TODO: Implement LLM client initialization
        # - OpenAI: pip install openai
        # - Anthropic: pip install anthropic
        # - Ollama: pip install ollama
        # - iFlytek Spark: https://www.xfyun.cn/doc/spark/Web.html
        pass

    def _asr_callback(self, msg):
        """Receive ASR text and send to LLM."""
        user_text = msg.data
        if not user_text:
            return

        self.get_logger().info(f'Received ASR: "{user_text}"')
        self._process_user_input(user_text)

    def _user_input_callback(self, msg):
        """Receive direct text input and send to LLM."""
        user_text = msg.data
        if not user_text:
            return

        self.get_logger().info(f'Received user input: "{user_text}"')
        self._process_user_input(user_text)

    def _process_user_input(self, user_text):
        """Process user input and generate LLM response."""
        # Add user message to history
        self.messages.append({"role": "user", "content": user_text})

        # TODO: Call LLM API
        # response = self._call_llm(self.messages)

        # Mock response for now
        response = self._mock_llm_response(user_text)

        # Add assistant message to history
        self.messages.append({"role": "assistant", "content": response})

        # Publish response
        self._publish_response(response)

        # Check if response contains skill request
        self._check_for_skill_request(response)

    def _mock_llm_response(self, user_text):
        """Generate a mock response (TODO: replace with real LLM)."""
        self.get_logger().warn(f'Mock LLM response for: "{user_text}" (TODO: implement real LLM)')

        # Simple pattern matching for demo
        user_lower = user_text.lower()
        if '导航' in user_lower or 'navigate' in user_lower:
            return '好的，我现在开始导航。[SKILL:navigate_to|location:kitchen]'
        elif '抓取' in user_lower or 'grasp' in user_lower:
            return '好的，我将开始抓取物体。[SKILL:grasp_object|color:red]'
        elif '你好' in user_lower or 'hello' in user_lower:
            return '你好！我是小飞机器人，有什么可以帮助你的吗？'
        else:
            return f'收到您的指令：{user_text}。我会尽力完成。'

    def _call_llm(self, messages):
        """Call LLM API and return response."""
        # TODO: Implement actual LLM API call
        # Example for OpenAI:
        # from openai import OpenAI
        # client = OpenAI(api_key=self.api_key)
        # response = client.chat.completions.create(
        #     model=self.model,
        #     messages=messages,
        #     temperature=self.temperature,
        #     max_tokens=self.max_tokens
        # )
        # return response.choices[0].message.content
        pass

    def _publish_response(self, response):
        """Publish LLM response."""
        self.get_logger().info(f'Assistant reply: "{response}"')

        # Publish to reply topic
        msg = String()
        msg.data = response
        self.reply_pub.publish(msg)

        # Also send to TTS (remove skill markers first)
        clean_response = self._remove_skill_markers(response)
        tts_msg = String()
        tts_msg.data = clean_response
        self.tts_pub.publish(tts_msg)

    def _check_for_skill_request(self, response):
        """Check if response contains skill request marker."""
        # Format: [SKILL:skill_name|param1:value1|param2:value2]
        if '[SKILL:' in response:
            import re
            matches = re.findall(r'\[SKILL:(.*?)\]', response)
            for match in matches:
                self.get_logger().info(f'Skill request detected: {match}')
                skill_msg = String()
                skill_msg.data = match
                self.skill_request_pub.publish(skill_msg)

    def _remove_skill_markers(self, text):
        """Remove skill markers from text for TTS."""
        import re
        return re.sub(r'\[SKILL:.*?\]', '', text).strip()


def main(args=None):
    rclpy.init(args=args)
    node = ChatNode()
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
