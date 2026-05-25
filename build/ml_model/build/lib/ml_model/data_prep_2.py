#!/usr/bin/env python3
"""
Ambulance Spawner + Data Collector (FIXED VERSION)
===================================================
- ROS 2 safe async Gazebo spawn/delete
- No nested spinning (fixes crash)
- Deterministic delete → spawn pipeline
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge

import cv2
import numpy as np
import os
import csv
import math
import time
import threading

# ───────────────── CONFIG ─────────────────
IMAGE_FOLDER = os.path.expanduser('~/data_collection/images')
CSV_PATH = os.path.expanduser('~/data_collection/data.csv')

SDF_CONTENT = """<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="ambulance">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry>
          <mesh>
            <scale>0.0254 0.0254 0.0254</scale>
            <uri>model://ambulance/meshes/ambulance.obj</uri>
          </mesh>
        </geometry>
      </visual>
    </link>
  </model>
</sdf>"""


# ───────────────── MAIN NODE ─────────────────
class AmbulanceController(Node):
    def __init__(self):
        super().__init__('ambulance_controller')

        # ROS interfaces
        self.bridge = CvBridge()
        self.current_image = None

        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')

        while not self.spawn_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("Waiting for spawn service...")
        while not self.delete_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("Waiting for delete service...")

        # state
        self.amb_pos = None
        self.amb_yaw = None
        self.amb_spawned = False

        self.click1 = None
        self.click1_world = None

        self.pending_spawn = False
        self.pending_pose = None

        # camera
        self.create_subscription(
            Image,
            '/rosbot/camera/image_raw',
            self.image_callback,
            10
        )

        os.makedirs(IMAGE_FOLDER, exist_ok=True)
        if not os.path.exists(CSV_PATH):
            with open(CSV_PATH, 'w') as f:
                csv.writer(f).writerow(['img', 'x', 'y', 'ax', 'ay', 'yaw'])

        self.get_logger().info("Controller ready")

        threading.Thread(target=self.map_loop, daemon=True).start()
        threading.Thread(target=self.keyboard_loop, daemon=True).start()

    # ───────── CAMERA ─────────
    def image_callback(self, msg):
        self.current_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    # ───────── GAZEBO DELETE ─────────
    def delete_ambulance(self):
        req = DeleteEntity.Request()
        req.name = 'ambulance'

        print("Requesting delete...")

        future = self.delete_client.call_async(req)
        future.add_done_callback(self.delete_callback)

    def delete_callback(self, future):
        try:
            future.result()
            print("Delete complete")
        except Exception as e:
            print("Delete failed:", e)

        self.amb_spawned = False

        # trigger spawn AFTER delete
        if self.pending_spawn:
            wx, wy, yaw = self.pending_pose
            self.pending_spawn = False
            self._spawn_ambulance(wx, wy, yaw)

    # ───────── SPAWN ENTRY ─────────
    def spawn_ambulance(self, wx, wy, yaw):
        print("Spawn requested")

        self.pending_spawn = True
        self.pending_pose = (wx, wy, yaw)

        self.delete_ambulance()

    # ───────── ACTUAL SPAWN ─────────
    def _spawn_ambulance(self, wx, wy, yaw):
        print("Spawning ambulance...")

        qz = math.sin(yaw / 2)
        qw = math.cos(yaw / 2)

        req = SpawnEntity.Request()
        req.name = 'ambulance'
        req.xml = SDF_CONTENT

        req.initial_pose.position.x = float(wx)
        req.initial_pose.position.y = float(wy)
        req.initial_pose.position.z = 0.0

        req.initial_pose.orientation.z = qz
        req.initial_pose.orientation.w = qw

        future = self.spawn_client.call_async(req)
        future.add_done_callback(self.spawn_callback)

        self.amb_spawned = True
        self.amb_pos = (wx, wy)
        self.amb_yaw = yaw

    def spawn_callback(self, future):
        try:
            future.result()
            print("Spawn complete")
        except Exception as e:
            print("Spawn failed:", e)

    # ───────── MAP LOOP ─────────
    def map_loop(self):
        cv2.namedWindow("Map")
        cv2.setMouseCallback("Map", self.mouse)

        while True:
            img = np.zeros((600, 600, 3), dtype=np.uint8)

            if self.amb_pos:
                x, y = self.amb_pos
                px = int(300 + x * 10)
                py = int(300 - y * 10)
                cv2.circle(img, (px, py), 10, (0, 0, 255), -1)

            cv2.imshow("Map", img)
            if cv2.waitKey(30) == 27:
                break

        cv2.destroyAllWindows()

    # ───────── CLICK ─────────
    def mouse(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return

        wx = (x - 300) / 10
        wy = (300 - y) / 10

        if self.click1 is None:
            self.click1 = (wx, wy)
            print("First click set")
        else:
            dx = wx - self.click1[0]
            dy = wy - self.click1[1]
            yaw = math.atan2(dy, dx)

            self.spawn_ambulance(self.click1[0], self.click1[1], yaw)

            self.click1 = None

    # ───────── LOOP ─────────
    def keyboard_loop(self):
        while rclpy.ok():
            input("Press Enter to log sample...")


def main():
    rclpy.init()
    node = AmbulanceController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()