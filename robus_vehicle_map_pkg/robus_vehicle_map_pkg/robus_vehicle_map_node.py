import rclpy
import math
import numpy as np
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point, PoseStamped, Vector3, Quaternion
from v2x.msg import CAM
from nav_msgs.msg import Odometry
from collections import defaultdict
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from geographiclib.geodesic import Geodesic

# Reference position (Kronach)
LAT_0 = 50.235990
LON_0 = 11.331048
ALT_0 = 0.0

def geodetic2enu(lat, lon, alt, lat_ref, lon_ref, alt_ref):
    geo = Geodesic.WGS84
    result = geo.Inverse(lat_ref, lon_ref, lat, lon)
    azimuth_rad = math.radians(result['azi1'])
    distance = result['s12']
    east = distance * math.sin(azimuth_rad)
    north = distance * math.cos(azimuth_rad)
    up = alt - alt_ref
    return east, north, up

class VehicleVisualizer(Node):
    def __init__(self):
        super().__init__('vehicle_visualizer')

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self.ego_marker_pub  = self.create_publisher(MarkerArray, '/ego_car_marker', qos)
        self.nearby_vehicle_marker_pub = self.create_publisher(MarkerArray, '/nearby_vehicle_marker', qos)
       

        self.create_subscription(Odometry, '/ego_odom', self.ego_odom_callback, qos)
        self.create_subscription(CAM,      '/cam',     self.cam_callback,     qos)

        self.ego_marker      = None
        self.nearby_vehicles   = defaultdict(dict)
        self.marker_id_count = 1

        self.init_ego_marker()

        # Publish loop only
        self.publish_timer = self.create_timer(0.1, self.publish_all_markers)

        self.get_logger().info("Vehicle Map is Ready")

    def init_ego_marker(self):
        m = Marker()
        m.header.frame_id = "map"
        m.ns  = "ego_vehicle"
        m.id  = 0
        m.type = Marker.MESH_RESOURCE
        m.mesh_resource = "file:///home/vaithish/v2x_ws/src/vehicle_map/models/car3.stl"
        m.mesh_use_embedded_materials = True
        m.action = Marker.ADD
        m.scale  = Vector3(x=0.001, y=0.001, z=0.001)
        m.color  = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)
        self.ego_marker = m

    def ego_odom_callback(self, msg: Odometry):
        if not self.ego_marker:
            return

        p = msg.pose.pose.position
        o = msg.pose.pose.orientation

        roll, pitch, yaw = euler_from_quaternion([o.x, o.y, o.z, o.w])
        yaw -= math.pi/2  # model alignment
        qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)

        self.ego_marker.pose.position    = Point(x=p.x, y=p.y, z=p.z)
        self.ego_marker.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)
        self.ego_marker.header.stamp     = self.get_clock().now().to_msg()

    def cam_callback(self, msg: CAM):
        try:
            lat = msg.cam.cam_parameters.basic_container.reference_position.latitude   / 1e7
            lon = msg.cam.cam_parameters.basic_container.reference_position.longitude  / 1e7
            alt = msg.cam.cam_parameters.basic_container.reference_position.altitude.altitude_value / 1e2
            raw_heading = msg.cam.cam_parameters.high_frequency_container.basic_vehicle_container_high_frequency.heading.heading_value
            station_id  = msg.header.station_id

            if station_id == 7:
                return

            x, y, _ = geodetic2enu(lat, lon, alt, LAT_0, LON_0, ALT_0)

            # orientation from heading
            yaw_rad = math.radians(raw_heading / 10.0)
            qz = round(math.sin(yaw_rad/2), 4)
            qw = round(math.cos(yaw_rad/2), 4)

            x, y = round(x,2), round(y,2)
            z = 0.0

            # marker creation/update
            if station_id not in self.nearby_vehicles:
                mid = self.marker_id_count
                self.marker_id_count += 1
                m = Marker()
                m.header.frame_id = "map"
                m.ns   = "nearby_vehicles"
                m.id   = mid
                m.type = Marker.MESH_RESOURCE
                m.mesh_resource = "file:///home/vaithish/v2x_ws/src/vehicle_map/models/car3.stl"
                m.mesh_use_embedded_materials = True
                m.action = Marker.ADD
                m.scale  = Vector3(x=0.001, y=0.001, z=0.001)
                m.color  = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
                self.nearby_vehicles[station_id] = m
                self.get_logger().info(f"Nearby vehicle marker {station_id}")

            m = self.nearby_vehicles[station_id]
            m.pose.position    = Point(x=x, y=y, z=z)
            m.pose.orientation = Quaternion(x=0.0, y=0.0, z=qz, w=qw)
            m.header.stamp     = self.get_clock().now().to_msg()

        except Exception as e:
            self.get_logger().error(f"cam_callback error: {e}")

    def publish_all_markers(self):
        # ego
        self.ego_marker.header.stamp = self.get_clock().now().to_msg()
        self.ego_marker_pub.publish(MarkerArray(markers=[self.ego_marker]))

        # peers
        arr = MarkerArray()
        for m in self.nearby_vehicles.values():
            m.header.stamp = self.get_clock().now().to_msg()
            arr.markers.append(m)
        self.nearby_vehicle_marker_pub.publish(arr)

def main(args=None):
    rclpy.init(args=args)
    n = VehicleVisualizer()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()

if __name__=="__main__":
    main()


