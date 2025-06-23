import rclpy
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions, StorageFilter
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import Twist
import pandas as pd


bag_path = 'askisi1_bag'  
output_csv = 'odom_output.csv'

rclpy.init()


storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
converter_options = ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')


reader = SequentialReader()
reader.open(storage_options, converter_options)


reader.set_filter(StorageFilter(topics=['/cmd_vel']))


records = []


while reader.has_next():
    topic, data, t = reader.read_next()
    if topic == '/cmd_vel':
        msg = deserialize_message(data, Twist)
        record = {
            'timestamp': t * 1e-9,  
            'v': msg.linear.x,      
            'w': msg.angular.z      
        }
        records.append(record)


df = pd.DataFrame(records)
df.to_csv(output_csv, index=False)



