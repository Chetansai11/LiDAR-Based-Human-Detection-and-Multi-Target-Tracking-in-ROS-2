#!/usr/bin/env python3
import math
from typing import List
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray, Pose, Point
from rclpy.time import Time

class LidarDetector(Node):
    def __init__(self):
        super().__init__('lidar_detector')
        self.declare_parameter('gap_threshold', 0.3)   # meter jump to start new cluster
        self.declare_parameter('min_cluster_points', 3)
        self.declare_parameter('max_cluster_width', 1.0)  # meters

        self.gap_threshold = self.get_parameter('gap_threshold').value
        self.min_cluster_points = self.get_parameter('min_cluster_points').value
        self.max_cluster_width = self.get_parameter('max_cluster_width').value

        self.pub = self.create_publisher(PoseArray, '/person_detections', 10)
        self.sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)

        self.get_logger().info('LidarDetector started')

    def scan_cb(self, scan: LaserScan):
        # Convert ranges -> points
        angles = []
        points = []
        ang = scan.angle_min
        for r in scan.ranges:
            if math.isfinite(r) and r > 0.0 and r < scan.range_max:
                x = r * math.cos(ang)
                y = r * math.sin(ang)
                points.append((x, y, r))
                angles.append(ang)
            else:
                points.append(None)
                angles.append(ang)
            ang += scan.angle_increment

        # cluster contiguous valid points
        clusters = []
        current = []
        for idx, p in enumerate(points):
            if p is None:
                if current:
                    clusters.append(current)
                    current = []
            else:
                if not current:
                    current = [ (idx, p) ]
                else:
                    # previous index was valid: check gap
                    prev_r = current[-1][1][2]
                    if abs(p[2] - prev_r) > self.gap_threshold:
                        # start new cluster
                        clusters.append(current)
                        current = [(idx, p)]
                    else:
                        current.append((idx, p))
        if current:
            clusters.append(current)

        # Filter clusters and compute centroids
        pose_array = PoseArray()
        pose_array.header.stamp = scan.header.stamp
        pose_array.header.frame_id = scan.header.frame_id if scan.header.frame_id else 'base_link'

        for c in clusters:
            if len(c) < self.min_cluster_points:
                continue
            xs = [pt[1][0] for pt in c]
            ys = [pt[1][1] for pt in c]
            width = math.hypot(max(xs)-min(xs), max(ys)-min(ys))
            if width > self.max_cluster_width:
                # likely wall or large object -> skip
                continue
            cx = sum(xs)/len(xs)
            cy = sum(ys)/len(ys)
            p = Pose()
            p.position.x = cx
            p.position.y = cy
            p.position.z = 0.0
            pose_array.poses.append(p)

        self.pub.publish(pose_array)

def main(args=None):
    rclpy.init(args=args)
    node = LidarDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
