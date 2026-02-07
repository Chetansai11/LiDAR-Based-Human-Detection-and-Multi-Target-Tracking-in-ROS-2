#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Point
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.time import Time
import math
import random
import time

class Track:
    def __init__(self, tid, pos, now):
        self.id = tid
        self.last_pos = pos  # (x,y)
        self.history = [Point(x=pos[0], y=pos[1], z=0.0)]
        self.last_seen = now
        self.color = (random.random(), random.random(), random.random(), 1.0)

    def update(self, pos, now):
        self.last_pos = pos
        self.history.append(Point(x=pos[0], y=pos[1], z=0.0))
        self.last_seen = now

class PersonTracker(Node):
    def __init__(self):
        super().__init__('person_tracker')
        self.declare_parameter('assoc_distance', 0.8)   # meters
        self.declare_parameter('lost_time', 2.0)       # seconds before considered lost
        self.declare_parameter('expire_time', 10.0)    # seconds before track deleted
        self.declare_parameter('publish_hz', 10.0)

        self.assoc_distance = self.get_parameter('assoc_distance').value
        self.lost_time = self.get_parameter('lost_time').value
        self.expire_time = self.get_parameter('expire_time').value
        self.publish_hz = self.get_parameter('publish_hz').value

        self.sub = self.create_subscription(PoseArray, '/person_detections', self.detections_cb, 10)
        self.pub = self.create_publisher(MarkerArray, '/person_markers', 10)

        self.tracks = {}  # id -> Track
        self.next_id = 1
        self.frame_id = 'base_link'
        self.last_detections_header = None

        self.create_timer(1.0/self.publish_hz, self.publish_markers)

        self.get_logger().info('PersonTracker started')

    def detections_cb(self, pose_array: PoseArray):
        now = self.get_clock().now().to_msg()
        # extract detections as (x,y)
        dets = []
        for p in pose_array.poses:
            dets.append((p.position.x, p.position.y))
        if pose_array.header.frame_id:
            self.frame_id = pose_array.header.frame_id

        # association: greedy nearest neighbor
        unmatched_dets = set(range(len(dets)))
        matched_tracks = set()
        # build distance table
        dist_table = []
        for tid, track in self.tracks.items():
            for di, det in enumerate(dets):
                d = math.hypot(track.last_pos[0]-det[0], track.last_pos[1]-det[1])
                if d <= self.assoc_distance:
                    dist_table.append((d, tid, di))
        dist_table.sort(key=lambda x: x[0])
        for d, tid, di in dist_table:
            if di in unmatched_dets and tid in self.tracks:
                # update track
                self.tracks[tid].update(dets[di], self.get_clock().now())
                unmatched_dets.remove(di)
                matched_tracks.add(tid)

        # create new tracks for unmatched detections
        for di in list(unmatched_dets):
            det = dets[di]
            tid = self.next_id
            self.next_id += 1
            self.tracks[tid] = Track(tid, det, self.get_clock().now())
            unmatched_dets.remove(di)

        # mark not-seen tracks by not updating last_seen (we'll expire them in publish loop)

    def publish_markers(self):
        now = self.get_clock().now()
        marker_array = MarkerArray()
        to_delete = []
        for tid, track in list(self.tracks.items()):
            age = (now - track.last_seen).nanoseconds / 1e9
            if age > self.expire_time:
                # delete track entirely
                to_delete.append(tid)
                continue

            # only publish if we have history (we may publish even if temporarily occluded)
            m = Marker()
            m.header.frame_id = self.frame_id
            m.header.stamp = now.to_msg()
            m.ns = 'people'
            m.id = tid
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.points = track.history[:]  # whole history
            # linewidth
            m.scale.x = 0.08
            # color: deterministic alpha
            r, g, b, a = track.color
            m.color.r = float(r)
            m.color.g = float(g)
            m.color.b = float(b)
            m.color.a = float(a)
            m.lifetime = rclpy.duration.Duration(seconds=0).to_msg()  # persistent in rviz until removed
            marker_array.markers.append(m)

        # remove expired tracks
        for tid in to_delete:
            del self.tracks[tid]

        self.pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = PersonTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
