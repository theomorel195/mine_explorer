from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from rclpy.serialization import deserialize_message
import numpy as np
import tf_transformations

def extract_odom_data(bag_path):
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions('', '')

    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    vx_list, vy_list, vz_list = [], [], []
    vroll_list, vpitch_list, vyaw_list = [], [], []

    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic == '/base_controller/odom':
            msg = deserialize_message(data, Odometry)
            vx_list.append(msg.twist.twist.linear.x)
            vy_list.append(msg.twist.twist.linear.y)
            vz_list.append(msg.twist.twist.linear.z)
            vroll_list.append(msg.twist.twist.angular.x)
            vpitch_list.append(msg.twist.twist.angular.y)
            vyaw_list.append(msg.twist.twist.angular.z)

    return {
        'vx': np.var(vx_list),
        'vy': np.var(vy_list),
        'vz': np.var(vz_list),
        'vroll': np.var(vroll_list),
        'vpitch': np.var(vpitch_list),
        'vyaw': np.var(vyaw_list)
    }


def extract_imu_data(bag_path):
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions('', '')

    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    ax_list, ay_list, az_list = [], [], []
    roll_list, pitch_list, yaw_list = [], [], []

    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic == '/sensors/imu/data':
            msg = deserialize_message(data, Imu)
            ax_list.append(msg.linear_acceleration.x)
            ay_list.append(msg.linear_acceleration.y)
            az_list.append(msg.linear_acceleration.z)

            q = msg.orientation
            r, p, y = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
            roll_list.append(r)
            pitch_list.append(p)
            yaw_list.append(y)

    return {
        'ax': np.var(ax_list),
        'ay': np.var(ay_list),
        'az': np.var(az_list),
        'roll': np.var(roll_list),
        'pitch': np.var(pitch_list),
        'yaw': np.var(yaw_list)
    }


if __name__ == "__main__":
    bag_path = "rosbag/bag_imu_odom_variance"

    odom_var = extract_odom_data(bag_path)
    imu_var = extract_imu_data(bag_path)

    print("Variances odom:")
    for k, v in odom_var.items():
        print(f"{k} : {v}")

    print("\nVariances IMU:")
    for k, v in imu_var.items():
        print(f"{k} : {v}")
