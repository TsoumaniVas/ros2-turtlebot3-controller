import rclpy
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions, StorageFilter
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
import pandas as pd
import numpy as np

def quaternion_to_yaw(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return np.arctan2(siny_cosp, cosy_cosp)


bag_path = 'askisi3_bag'  
output_csv = 'odom3_output.csv'  

rclpy.init()

reader = SequentialReader()
reader.open(
    StorageOptions(uri=bag_path, storage_id='sqlite3'),
    ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
)

reader.set_filter(StorageFilter(topics=['/odom']))

records = []

while reader.has_next():
    topic, data, t = reader.read_next()
    if topic == '/odom':
        msg = deserialize_message(data, Odometry)

        q = msg.pose.pose.orientation
        theta = quaternion_to_yaw(q.x, q.y, q.z, q.w)

        records.append({
            'timestamp': t * 1e-9,
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'theta': theta,
            'vx': msg.twist.twist.linear.x,
            'wz': msg.twist.twist.angular.z
        })

df = pd.DataFrame(records)
df.to_csv(output_csv, index=False)


