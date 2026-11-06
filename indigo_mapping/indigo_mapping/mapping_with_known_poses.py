#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, MapMetaData
from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
import math

PRIOR_PROB = 0.5
OCC_PROB = 0.9
FREE_PROB = 0.35

class Pose:
    def __init__(self, px=0, py=0):
        self.x = px
        self.y = py

def prob2logodds(p):
    return math.log(p / (1 - p))

def logodds2prob(l):
    try:
        return 1 - (1 / (1 + math.exp(l)))
    except OverflowError:
        return 1.0 if l > 0 else 0.0

def poseToCell(pose: Pose, map_info: MapMetaData):
    return map_info.width * pose.y + pose.x

def coordinatesToPose(px, py, map_info: MapMetaData):
    pose = Pose()
    pose.x = round((px - map_info.origin.position.x) / map_info.resolution)
    pose.y = round((py - map_info.origin.position.y) / map_info.resolution)
    return pose

def poseOnMap(pose: Pose, map_info: MapMetaData):
    return pose.x < map_info.width and pose.x >= 0 and pose.y < map_info.height and pose.y >= 0

def bresenham(start: Pose, end: Pose):
    line = []

    dx = end.x - start.x
    dy = end.y - start.y

    xsign = 1 if dx > 0 else -1
    ysign = 1 if dy > 0 else -1

    dx = abs(dx)
    dy = abs(dy)

    if dx > dy:
        xx = xsign
        xy = 0
        yx = 0
        yy = ysign
    else:
        tmp = dx
        dx = dy
        dy = tmp
        xx = 0
        xy = ysign
        yx = xsign
        yy = 0

    D = 2 * dy - dx
    y = 0

    for i in range(dx + 1):
        line.append(Pose(start.x + i * xx + y * yx, start.y + i * xy + y * yy))
        if D >= 0:
            y += 1
            D -= 2 * dx
        D += 2 * dy

    return line

def inverseSensorModel(p_robot: Pose, p_beam: Pose):
    occ_values = []
    line = bresenham(p_robot, p_beam)

    for pose in line[:-1]:
        occ_values.append((pose, FREE_PROB))  # FREE

    occ_values.append((line[-1], OCC_PROB))  # OCCUPIED
    return occ_values

class MappingWithKnownPoses(Node):
    def __init__(self, name):
        super().__init__(name)
        self.declare_parameter("width", 10.0)
        self.declare_parameter("height", 10.0)
        self.declare_parameter("resolution", 0.05)

        width = self.get_parameter("width").value
        height = self.get_parameter("height").value
        resolution = self.get_parameter("resolution").value

        self.map_ = OccupancyGrid()
        self.map_.info.resolution = resolution
        self.map_.info.width = round(width / resolution)
        self.map_.info.height = round(height / resolution)
        self.map_.info.origin.position.x = float(-round(width / 2.0))
        self.map_.info.origin.position.y = float(-round(height / 2.0))
        self.map_.header.frame_id = "odom"
        self.map_.data = [-1] * (self.map_.info.width * self.map_.info.height)
        self.probability_map_ = [prob2logodds(PRIOR_PROB)] * (self.map_.info.width * self.map_.info.height)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.scan_sub = self.create_subscription(LaserScan, "scan", self.scanCallback, qos_profile_sensor_data)
        self.map_pub = self.create_publisher(OccupancyGrid, "map", 1)
        self.timer = self.create_timer(1.0, self.timerCallback)

    def scanCallback(self, scan: LaserScan):
        try:
            t = self.tf_buffer.lookup_transform(self.map_.header.frame_id, scan.header.frame_id, rclpy.time.Time())
        except TransformException as e:
            self.get_logger().warn(f"TF not ready: {e}")
            return

        robot_p = coordinatesToPose(t.transform.translation.x, t.transform.translation.y, self.map_.info)
        if not poseOnMap(robot_p, self.map_.info):
            self.get_logger().error("The robot is out of the map!")
            return

        q = t.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        
        for i in range(len(scan.ranges)):
            r = scan.ranges[i]
            if math.isinf(r) or math.isnan(r) or r < scan.range_min or r > scan.range_max:
                continue
            # Polar to cartesian conversion
            angle = scan.angle_min + (i * scan.angle_increment) + yaw
            px = r * math.cos(angle)
            py = r * math.sin(angle)
            px += t.transform.translation.x
            py += t.transform.translation.y

            beam_p = coordinatesToPose(px, py, self.map_.info)
            if not poseOnMap(beam_p, self.map_.info):
                continue

            poses = inverseSensorModel(robot_p, beam_p)

            for pose, value in poses:
                if poseOnMap(pose, self.map_.info):
                    cell = poseToCell(pose, self.map_.info)
                    self.probability_map_[cell] += prob2logodds(value) - prob2logodds(PRIOR_PROB)

    def timerCallback(self):
        self.map_.data = [int(logodds2prob(value) * 100) for value in self.probability_map_]
        self.map_.header.stamp = self.get_clock().now().to_msg()
        self.map_pub.publish(self.map_)

def main(args=None):
    rclpy.init(args=args)
    node = MappingWithKnownPoses("mapping_with_known_poses")
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()