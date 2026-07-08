#!/usr/bin/env python3
"""
LLM Bridge — 笔记本端 ROS2 桥接节点
用法:
  source /opt/ros/galactic/setup.bash
  source install/setup.bash   # 要访问 cyberdog_race.srv.LLMAsk
  python3 llm_bridge.py

狗端通过 ROS2 Service /llm_ask 发送 prompt，
桥接节点转发到云端 LLM API 并返回结果。
"""

import rclpy
from rclpy.node import Node
from cyberdog_race.srv import LLMAsk
import requests
import os

BACKEND = os.environ.get("LLM_BACKEND", "https://api.deepseek.com/v1/chat/completions")
API_KEY = os.environ.get("LLM_API_KEY", "sk-your-key-here")
MODEL   = os.environ.get("LLM_MODEL", "deepseek-chat")

SYSTEM_PROMPT = (
    "You are a CyberDog quadruped robot control expert. "
    "Output JSON only, no explanation: "
    '{"action":"forward/back/left/right/stop/push/jump","speed":0.0-0.5,"yaw":-0.5-0.5}'
)

class LLMBridge(Node):
    def __init__(self):
        super().__init__('llm_bridge')
        self.srv = self.create_service(LLMAsk, '/llm_ask', self.cb)
        self.get_logger().info(f'LLM Bridge ready -> {BACKEND}')

    def cb(self, req, resp):
        self.get_logger().info(f'LLM: {req.prompt[:80]}...')
        body = {
            "model": MODEL,
            "messages": [
                {"role": "system", "content": SYSTEM_PROMPT},
                {"role": "user", "content": req.prompt}
            ],
            "temperature": 0.1,
            "max_tokens": 200
        }
        try:
            r = requests.post(BACKEND, json=body,
                headers={"Authorization": f"Bearer {API_KEY}"}, timeout=15)
            r.raise_for_status()
            resp.response = r.json()["choices"][0]["message"]["content"]
            resp.success = True
        except Exception as e:
            self.get_logger().error(str(e))
            resp.success = False
        return resp

def main():
    rclpy.init()
    rclpy.spin(LLMBridge())

if __name__ == '__main__':
    main()
