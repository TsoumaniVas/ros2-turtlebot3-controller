import rclpy
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
import pandas as pd
import os
import struct
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from rosbag2_py import StorageFilter


bag_path = 'askisi2_bag'
output_csv = 'odom2_output.csv'

#arxikopoiisi ROS2
rclpy.init()

#setting for rosbag2 reader
storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')

reader = SequentialReader()
reader.open(storage_options, converter_options)

#find topics
topic_types = reader.get_all_topics_and_types()
type_map = {topic.name: topic.type for topic in topic_types}

reader.set_filter(StorageFilter(topics=['/odom']))

#lista gia dedomena
records = []

#anagnosi minimaton
while reader.has_next():
    topic, data, t = reader.read_next()
    if topic == '/odom':
        msg = deserialize_message(data, Odometry)
        record = {
            'timestamp': t * 1e-9,
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'vx': msg.twist.twist.linear.x,
            'wz': msg.twist.twist.angular.z
        }
        records.append(record)

#save to CSV
df = pd.DataFrame(records)
df.to_csv(output_csv, index=False)


