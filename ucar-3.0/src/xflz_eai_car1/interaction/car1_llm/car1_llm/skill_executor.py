"""
Skill Executor Node

Listens for skill requests from LLM and executes robot actions.
Maps natural language intents to ROS actions/services.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped
import yaml


class SkillExecutor(Node):
    def __init__(self):
        super().__init__('skill_executor')

        # Parameters
        self.declare_parameter('skills_config', '')

        skills_config = self.get_parameter('skills_config').value

        # Load skills configuration
        self.skills = {}
        if skills_config:
            self._load_skills_config(skills_config)

        # Subscriber
        self.skill_request_sub = self.create_subscription(
            String, '/interaction/llm/skill_request',
            self._skill_request_callback, 10)

        # Service clients (created on demand)
        self.service_clients = {}

        self.get_logger().info(f'Skill executor started, loaded {len(self.skills)} skills')

    def _load_skills_config(self, config_path):
        """Load skills configuration from YAML file."""
        try:
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
                self.skills = config.get('skills', {})
                self.get_logger().info(f'Loaded skills config from {config_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to load skills config: {e}')

    def _skill_request_callback(self, msg):
        """Parse and execute skill request."""
        # Format: skill_name|param1:value1|param2:value2
        request = msg.data
        self.get_logger().info(f'Skill request: {request}')

        try:
            parts = request.split('|')
            skill_name = parts[0]
            params = {}
            for part in parts[1:]:
                if ':' in part:
                    key, value = part.split(':', 1)
                    params[key] = value

            self._execute_skill(skill_name, params)

        except Exception as e:
            self.get_logger().error(f'Failed to parse skill request: {e}')

    def _execute_skill(self, skill_name, params):
        """Execute a skill with given parameters."""
        if skill_name not in self.skills:
            self.get_logger().error(f'Unknown skill: {skill_name}')
            return

        skill_config = self.skills[skill_name]
        action_type = skill_config.get('type', 'service')

        self.get_logger().info(f'Executing skill: {skill_name}, type: {action_type}, params: {params}')

        if action_type == 'service':
            self._call_service(skill_config, params)
        elif action_type == 'action':
            self._call_action(skill_config, params)
        elif action_type == 'topic':
            self._publish_topic(skill_config, params)
        else:
            self.get_logger().error(f'Unknown action type: {action_type}')

    def _call_service(self, skill_config, params):
        """Call a ROS service."""
        service_name = skill_config.get('service')
        service_type_str = skill_config.get('service_type', 'std_srvs/srv/Trigger')

        if not service_name:
            self.get_logger().error('Service name not specified in skill config')
            return

        self.get_logger().info(f'Calling service: {service_name}')

        # TODO: Implement actual service call
        # For now, just log
        self.get_logger().warn(f'TODO: Call {service_name} with params {params}')

        # Example for Trigger service:
        # if service_name not in self.service_clients:
        #     self.service_clients[service_name] = self.create_client(Trigger, service_name)
        #
        # client = self.service_clients[service_name]
        # if not client.wait_for_service(timeout_sec=3.0):
        #     self.get_logger().error(f'Service {service_name} not available')
        #     return
        #
        # request = Trigger.Request()
        # future = client.call_async(request)

    def _call_action(self, skill_config, params):
        """Call a ROS action."""
        action_name = skill_config.get('action')
        self.get_logger().info(f'Calling action: {action_name}')
        self.get_logger().warn(f'TODO: Implement action call for {action_name}')

    def _publish_topic(self, skill_config, params):
        """Publish to a ROS topic."""
        topic_name = skill_config.get('topic')
        self.get_logger().info(f'Publishing to topic: {topic_name}')
        self.get_logger().warn(f'TODO: Implement topic publish for {topic_name}')


def main(args=None):
    rclpy.init(args=args)
    node = SkillExecutor()
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
