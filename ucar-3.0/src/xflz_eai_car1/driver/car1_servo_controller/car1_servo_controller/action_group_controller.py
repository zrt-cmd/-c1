#!/usr/bin/env python3
# encoding: utf-8
"""Action group controller: plays back pre-recorded servo action sequences."""
import os
import time
import sqlite3
from car1_servo_controller_msgs.msg import ServosPosition, ServoPosition


class ActionGroupController:
    """Plays action groups stored as .d6a SQLite files."""

    def __init__(self, publisher, action_path):
        """
        Args:
            publisher: ROS2 publisher for ServosPosition messages.
            action_path: Directory path containing .d6a action files.
        """
        self.servo_controller_pub = publisher
        self.action_path = action_path
        self.running_action = False
        self.stop_running = False

    def stop_action_group(self):
        """Request stop of currently running action group."""
        self.stop_running = True

    def run_action(self, action_name):
        """
        Run a named action group.

        Args:
            action_name: Name of the action (without .d6a extension).
        """
        if action_name is None:
            return

        file_path = os.path.join(self.action_path, action_name + ".d6a")
        self.stop_running = False

        if not os.path.exists(file_path):
            self.running_action = False
            return

        if self.running_action:
            return

        self.running_action = True
        try:
            conn = sqlite3.connect(file_path)
            cursor = conn.cursor()
            cursor.execute("select * from ActionGroup")

            while True:
                act = cursor.fetchone()
                if self.stop_running:
                    self.stop_running = False
                    break
                if act is None:
                    break

                msg = ServosPosition()
                msg.position_unit = 'pulse'
                msg.duration = float(act[1]) / 1000.0

                for i in range(0, len(act) - 2):
                    servo = ServoPosition()
                    # Column mapping: servo 6 maps to ID 10 (gripper)
                    if i + 1 == 6:
                        servo.id = 10
                    else:
                        servo.id = i + 1
                    servo.position = float(act[2 + i])
                    msg.position.append(servo)

                self.servo_controller_pub.publish(msg)
                time.sleep(msg.duration)

            cursor.close()
            conn.close()
        finally:
            self.running_action = False
