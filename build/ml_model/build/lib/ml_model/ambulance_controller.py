#!/usr/bin/env python3
"""
Ambulance Spawner + Data Collector
===================================
- Shows a top-down 2D map of the world
- Click 1: set ambulance spawn position
- Click 2: set ambulance orientation (yaw toward second click)
- Ambulance spawns in Gazebo, old one gets deleted first
- Press SPACEBAR in map window to capture image + save data
- theta computed automatically from robot and ambulance positions
- Images named by timestamp, CSV appended across sessions
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge
from sympy import symbols, Eq, solve

import cv2
import numpy as np
import csv
import os
import threading
import math
import time
from datetime import datetime

# ── CONFIGURATION ─────────────────────────────────────────────────────────────
ROBOT_X      = 0.0
ROBOT_Y      = 0.0
WORLD_RANGE  = 15.0
MAP_SIZE     = 700
FOV_DEG      = 30.0
FOV_DEPTH    = 10.0
CX           = 320.5
FX           = 554.38
LINEAR_VEL   = 0.5

IMAGE_FOLDER = os.path.expanduser('~/mtre4820-final-project/src/ml_model/ml_model/data/images')
CSV_PATH     = os.path.expanduser('~/mtre4820-final-project/src/ml_model/ml_model/data/data.csv')

SDF_CONTENT = """<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="ambulance">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry>
          <mesh>
            <scale>0.0254 0.0254 0.0254</scale>
            <uri>model://ambulance/meshes/ambulance.obj</uri>
          </mesh>
        </geometry>
      </collision>
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

# ── MATH HELPERS ──────────────────────────────────────────────────────────────
t_sym, s_sym = symbols('t s')

def midpoint(x1, y1, x2, y2):
    return np.array([(x1+x2)/2, (y1+y2)/2])

def dist(x1, y1, x2, y2):
    return np.sqrt((x2-x1)**2 + (y2-y1)**2)

def compute_p3(x1, y1, x2, y2, s1, d1, theta):
    dx = x2 - x1
    dy = y2 - y1
    u  = np.array([dx/d1, dy/d1])
    n  = np.array([-u[1], u[0]])
    offset = (d1/4) * abs(math.sin(theta))
    side   = 1 if theta >= 0 else -1
    return s1 + side * offset * n

def perp(x, y):
    return np.array([-y, x])

def find_center(s1, s2, n1, n2):
    r1 = s1 + n1*t_sym
    r2 = s2 + n2*s_sym
    sol = solve((Eq(r1[0], r2[0]), Eq(r1[1], r2[1])), (t_sym, s_sym))
    if isinstance(sol, list):
        if len(sol) == 0:
            return None
        sol = sol[0]
    sv = sol[s_sym]
    c  = s2 + n2*sv
    return float(c[0]), float(c[1])

def compute_omega(x1, y1, x2, y2, theta, v=LINEAR_VEL):
    if abs(math.sin(theta)) < 1e-6:
        return 0.0
    s1 = midpoint(x1, y1, x2, y2)
    d1 = dist(x1, y1, x2, y2)
    p3 = compute_p3(x1, y1, x2, y2, s1, d1, theta)
    s2 = midpoint(x1, y1, p3[0], p3[1])
    n1 = perp(x2-x1, y2-y1)
    n2 = perp(p3[0]-x1, p3[1]-y1)
    center = find_center(s1, s2, n1, n2)
    if center is None:
        return None
    h, k  = center
    r     = math.sqrt((x1-h)**2 + (y1-k)**2)
    cross = (x2-x1)*(k-y1) - (y2-y1)*(h-x1)
    sign  = 1 if cross > 0 else -1
    return sign * v / r

# ── MAP HELPERS ───────────────────────────────────────────────────────────────
def world_to_map(wx, wy):
    px = int((wx + WORLD_RANGE) / (2 * WORLD_RANGE) * MAP_SIZE)
    py = int((WORLD_RANGE - wy) / (2 * WORLD_RANGE) * MAP_SIZE)
    return px, py

def map_to_world(px, py):
    wx =  (px / MAP_SIZE) * (2 * WORLD_RANGE) - WORLD_RANGE
    wy = -(py / MAP_SIZE) * (2 * WORLD_RANGE) - (-WORLD_RANGE)
    return wx, wy

def yaw_to_quaternion(yaw):
    corrected = yaw + math.pi / 2
    return 0.0, 0.0, math.sin(corrected/2), math.cos(corrected/2)

def draw_map(amb_pos=None, amb_yaw=None, click1=None):
    img = np.ones((MAP_SIZE, MAP_SIZE, 3), dtype=np.uint8) * 40

    for m in range(-int(WORLD_RANGE), int(WORLD_RANGE)+1, 5):
        px, _ = world_to_map(m, 0)
        _, py  = world_to_map(0, m)
        cv2.line(img, (px, 0), (px, MAP_SIZE), (60, 60, 60), 1)
        cv2.line(img, (0, py), (MAP_SIZE, py), (60, 60, 60), 1)
        if m != 0:
            cv2.putText(img, str(m), (px+2, MAP_SIZE-4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.3, (80,80,80), 1)

    ox, oy = world_to_map(0, 0)
    cv2.line(img, (ox, 0), (ox, MAP_SIZE), (80, 80, 80), 1)
    cv2.line(img, (0, oy), (MAP_SIZE, oy), (80, 80, 80), 1)

    robot_heading = 0.0
    fov_rad  = FOV_DEG * math.pi / 180
    rx, ry   = world_to_map(ROBOT_X, ROBOT_Y)
    depth_px = int(FOV_DEPTH / (2 * WORLD_RANGE) * MAP_SIZE)

    left_angle  = robot_heading + fov_rad
    right_angle = robot_heading - fov_rad
    lx  = int(rx + depth_px * math.cos(left_angle))
    ly  = int(ry - depth_px * math.sin(left_angle))
    fx2 = int(rx + depth_px * math.cos(right_angle))
    fy2 = int(ry - depth_px * math.sin(right_angle))

    fov_pts = np.array([[rx, ry], [lx, ly], [fx2, fy2]], dtype=np.int32)
    cv2.fillPoly(img, [fov_pts], (30, 60, 30))
    cv2.line(img, (rx, ry), (lx, ly),   (0, 180, 80), 1)
    cv2.line(img, (rx, ry), (fx2, fy2), (0, 180, 80), 1)
    cv2.putText(img, 'FOV', (rx + depth_px//2, ry - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,180,80), 1)

    cv2.circle(img, (rx, ry), 8, (120, 80, 255), -1)
    cv2.putText(img, 'robot', (rx+10, ry+4),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (120,80,255), 1)

    if click1 is not None:
        cv2.circle(img, click1, 5, (0, 200, 255), -1)
        cv2.putText(img, 'pos', (click1[0]+6, click1[1]),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0,200,255), 1)

    if amb_pos is not None:
        ax, ay = world_to_map(amb_pos[0], amb_pos[1])
        cv2.circle(img, (ax, ay), 8, (0, 80, 255), -1)
        cv2.putText(img, 'AMB', (ax+8, ay+4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,80,255), 1)
        if amb_yaw is not None:
            arrow_len = 20
            ex = int(ax + arrow_len * math.cos(amb_yaw))
            ey = int(ay - arrow_len * math.sin(amb_yaw))
            cv2.arrowedLine(img, (ax, ay), (ex, ey), (0,200,255), 2, tipLength=0.4)

    cv2.putText(img, 'Click 1: spawn position',  (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200,200,200), 1)
    cv2.putText(img, 'Click 2: facing direction', (8, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200,200,200), 1)
    cv2.putText(img, 'Space: capture sample',     (8, 56), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200,200,200), 1)

    return img


# ── MAIN NODE ─────────────────────────────────────────────────────────────────
class AmbulanceController(Node):
    def __init__(self):
        super().__init__('ambulance_controller')

        self.bridge        = CvBridge()
        self.current_image = None
        self.amb_spawned   = False
        self.amb_pos       = None
        self.amb_yaw       = None
        self.click1        = None
        self.click1_world  = None
        self.last_click_time = 0.0

        self._spawn_request  = None
        self._delete_request = False
        self._spawn_lock     = threading.Lock()

        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(Image, '/rosbot/camera/image_raw',
                                 self.image_callback, 10)

        self.spawn_client  = self.create_client(SpawnEntity,  '/spawn_entity')
        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')

        while not self.spawn_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for /spawn_entity...')
        while not self.delete_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for /delete_entity...')

        self.create_timer(0.2, self.process_spawn_queue)

        # Create directories
        os.makedirs(IMAGE_FOLDER, exist_ok=True)
        print(f'DEBUG: IMAGE_FOLDER = {os.path.abspath(IMAGE_FOLDER)}')
        print(f'DEBUG: CSV_PATH     = {os.path.abspath(CSV_PATH)}')

        # Only write header if CSV does not exist yet — otherwise append
        if not os.path.exists(CSV_PATH):
            with open(CSV_PATH, 'w', newline='') as f:
                csv.writer(f).writerow([
                    'Image', 'Date', 'Time',
                    'RobotX', 'RobotY',
                    'AmbX', 'AmbY', 'AmbYaw',
                    'theta', 'omega'
                ])
            print('DEBUG: CSV created with header')
        else:
            print('DEBUG: CSV exists — new samples will be appended')

        self.get_logger().info('Ready! Press SPACE in map window to capture')
        threading.Thread(target=self.map_loop, daemon=True).start()

    # ── CALLBACKS ─────────────────────────────────────────────────────────────
    def image_callback(self, msg):
        self.current_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    # ── SPAWN QUEUE ───────────────────────────────────────────────────────────
    def process_spawn_queue(self):
        with self._spawn_lock:
            do_delete  = self._delete_request
            spawn_args = self._spawn_request
            self._delete_request = False
            self._spawn_request  = None

        if do_delete and self.amb_spawned:
            req = DeleteEntity.Request()
            req.name = 'ambulance'
            future = self.delete_client.call_async(req)
            future.add_done_callback(lambda f: self.get_logger().info('Ambulance deleted'))
            self.amb_spawned = False

        if spawn_args is not None:
            wx, wy, yaw = spawn_args
            qx, qy, qz, qw = yaw_to_quaternion(yaw)
            req = SpawnEntity.Request()
            req.name            = 'ambulance'
            req.xml             = SDF_CONTENT
            req.robot_namespace = ''
            req.initial_pose.position.x    = float(wx)
            req.initial_pose.position.y    = float(wy)
            req.initial_pose.position.z    = 0.0
            req.initial_pose.orientation.x = qx
            req.initial_pose.orientation.y = qy
            req.initial_pose.orientation.z = qz
            req.initial_pose.orientation.w = qw
            req.reference_frame = 'world'
            future = self.spawn_client.call_async(req)
            future.add_done_callback(lambda f: self.get_logger().info(
                f'Ambulance spawned at ({wx:.2f}, {wy:.2f}) yaw={math.degrees(yaw):.1f}°'))
            self.amb_spawned = True
            self.amb_pos     = (wx, wy)
            self.amb_yaw     = yaw
            print(f'\nAmbulance spawned at ({wx:.2f}, {wy:.2f}) yaw={math.degrees(yaw):.1f}°')
            print('Press SPACE in map window to capture')

    def request_spawn(self, wx, wy, yaw):
        with self._spawn_lock:
            self._delete_request = True
            self._spawn_request  = (wx, wy, yaw)

    # ── MAP LOOP ──────────────────────────────────────────────────────────────
    def map_loop(self):
        cv2.namedWindow('World Map')
        cv2.setMouseCallback('World Map', self.map_click)
        while True:
            frame = draw_map(amb_pos=self.amb_pos, amb_yaw=self.amb_yaw, click1=self.click1)
            cv2.imshow('World Map', frame)
            key = cv2.waitKey(50) & 0xFF
            if key == ord('q'):
                break
            if key == ord(' '):
                print('DEBUG: spacebar pressed')
                threading.Thread(target=self.capture_sample, daemon=True).start()
        cv2.destroyAllWindows()

    def map_click(self, event, px, py, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        now = time.time()
        if now - self.last_click_time < 0.5:
            return
        self.last_click_time = now

        wx, wy = map_to_world(px, py)
        if self.click1 is None:
            self.click1       = (px, py)
            self.click1_world = (wx, wy)
            print(f'\nPosition set: ({wx:.2f}, {wy:.2f}) — now click facing direction...')
        else:
            dx  = wx - self.click1_world[0]
            dy  = wy - self.click1_world[1]
            yaw = math.atan2(dy, dx)
            spawn_x, spawn_y  = self.click1_world
            self.click1       = None
            self.click1_world = None
            self.request_spawn(spawn_x, spawn_y, yaw)

    # ── CAPTURE ───────────────────────────────────────────────────────────────
    def get_robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
            return tf.transform.translation.x, tf.transform.translation.y
        except Exception as e:
            print(f'TF error: {e}')
            return None, None

    def capture_sample(self):
        print('DEBUG: capture_sample called')
        print(f'DEBUG: current_image = {self.current_image is not None}')
        print(f'DEBUG: amb_pos = {self.amb_pos}')

        if self.current_image is None:
            print('No camera image yet — is the simulation running?')
            return
        if self.amb_pos is None:
            print('No ambulance spawned yet — click the map first')
            return

        robot_x, robot_y = self.get_robot_pose()
        if robot_x is None:
            return

        print(f'DEBUG: robot pose = ({robot_x:.3f}, {robot_y:.3f})')

        # Timestamp for filename and CSV
        now       = datetime.now()
        date_str  = now.strftime('%Y-%m-%d')
        time_str  = now.strftime('%H-%M-%S')
        stamp     = now.strftime('%Y%m%d_%H%M%S')

        # Compute theta from world coordinates
        dx    = self.amb_pos[0] - robot_x
        dy    = self.amb_pos[1] - robot_y
        theta = math.atan2(dy, dx)

        print(f'DEBUG: theta={math.degrees(theta):.2f} deg')

        omega = compute_omega(robot_x, robot_y,
                              self.amb_pos[0], self.amb_pos[1], theta)

        print(f'DEBUG: omega={omega}')

        if omega is None:
            print('Could not compute omega')
            return

        # Save image named by timestamp
        img_name    = f'image_{stamp}.png'
        img_path    = os.path.join(IMAGE_FOLDER, img_name)
        img_save    = self.current_image.copy()
        save_result = cv2.imwrite(img_path, img_save)
        print(f'DEBUG: cv2.imwrite result={save_result}, path={img_path}')

        # Append row to CSV
        with open(CSV_PATH, 'a', newline='') as f:
            csv.writer(f).writerow([
                img_name,
                date_str,
                time_str,
                round(robot_x, 6), round(robot_y, 6),
                round(self.amb_pos[0], 6), round(self.amb_pos[1], 6),
                round(self.amb_yaw, 6),
                round(float(theta), 6),
                round(float(omega), 6)
            ])

        print(f'\nSample saved!')
        print(f'  Image:  {img_name}')
        print(f'  Date:   {date_str}  Time: {time_str}')
        print(f'  Robot:  ({robot_x:.3f}, {robot_y:.3f})')
        print(f'  Amb:    ({self.amb_pos[0]:.3f}, {self.amb_pos[1]:.3f})'
              f' yaw={math.degrees(self.amb_yaw):.1f}°')
        print(f'  theta:  {math.degrees(theta):.2f} deg')
        print(f'  omega:  {omega:.4f} rad/s')


def main(args=None):
    rclpy.init(args=args)
    node = AmbulanceController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()