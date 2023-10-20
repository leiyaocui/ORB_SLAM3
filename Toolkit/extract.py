import os

import cv2
import numpy as np
import open3d as o3d
from cv_bridge import CvBridge
from rosbags.highlevel import AnyReader
from tqdm import tqdm

import point_cloud2 as pc2
from config import DATA_PATH

input_path = DATA_PATH
output_path = DATA_PATH
use_timestamp = True

include_topic = "/depth"  # /rgb /depth /imu /lidar_scan /camera_info /tf_imu2cam /tf_lidar2cam /tf_gt

if include_topic == "/tf_gt":
    fd = open(os.path.join(output_path, "tf_gt.txt"), "w")
else:
    output_path = output_path / include_topic[1:]
    output_path.mkdir(parents=True, exist_ok=True)

cv_bridge = CvBridge()

# vis = o3d.visualization.Visualizer()
# vis.create_window()
# pointcloud3d = o3d.geometry.PointCloud()
# render_opt = vis.get_render_option()
# render_opt.point_size = 1
# render_opt.background_color = np.array([0, 0, 0])
# vis.add_geometry(pointcloud3d)


with AnyReader([input_path]) as reader:
    connections = [x for x in reader.connections if x.topic == include_topic]
    for idx, (connection, timestamp, rawdata) in tqdm(
        enumerate(reader.messages(connections=connections)),
        dynamic_ncols=True,
        desc=include_topic,
    ):
        msg = reader.deserialize(rawdata, connection.msgtype)
        if "/tf" not in include_topic:
            timestamp = int(msg.header.stamp.sec * 10e9 + msg.header.stamp.nanosec)
        if connection.msgtype == "sensor_msgs/msg/Imu":
            linear_acceleration = np.array(
                [
                    msg.linear_acceleration.x,
                    msg.linear_acceleration.y,
                    msg.linear_acceleration.z,
                ],
                dtype=np.float64,
            )
            angular_velocity = np.array(
                [
                    msg.angular_velocity.x,
                    msg.angular_velocity.y,
                    msg.angular_velocity.z,
                ],
                dtype=np.float64,
            )

            if use_timestamp:
                save_path = output_path / f"{timestamp}.txt"
            else:
                save_path = output_path / f"{idx:05d}.txt"
            np.savetxt(
                str(save_path),
                np.concatenate([linear_acceleration, angular_velocity]).reshape(1, -1),
                delimiter=" ",
                fmt="%.15f",
            )
        elif connection.msgtype == "sensor_msgs/msg/Image":
            image = cv_bridge.imgmsg_to_cv2(msg, desired_encoding=msg.encoding)
            if msg.encoding == "rgb8":
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            if msg.encoding == "32FC1":
                image = (image * 1000).astype(np.uint16)

            if use_timestamp:
                save_path = output_path / f"{timestamp}.png"
            else:
                save_path = output_path / f"{idx:05d}.png"
            cv2.imwrite(str(save_path), image)
            # cv2.imshow(include_topic, image)
            # cv2.waitKey(30)
        elif connection.msgtype == "sensor_msgs/msg/PointCloud2":
            field_names = ["x", "y", "z"]
            pointcloud = pc2.read_points(msg, field_names=field_names, skip_nans=True)
            # pointcloud3d.points = o3d.utility.Vector3dVector(pointcloud)
            # vis.update_geometry(pointcloud3d)
            # vis.poll_events()
            # vis.update_renderer()
        elif connection.msgtype == "tf2_msgs/msg/TFMessage":
            msg = msg.transforms[0]
            timestamp = (
                float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) / 10e9
            )
            translation, rotation = msg.transform.translation, msg.transform.rotation
            # TUM format
            print(
                f"{timestamp:.9f} {translation.x:.9f} {translation.y:.9f} {translation.z:.9f} {rotation.x:.9f} {rotation.y:.9f} {rotation.z:.9f} {rotation.w:.9f}",
                file=fd,
            )
        else:
            raise NotImplementedError(
                f"Not supported message type [{connection.msgtype}] in topic [{include_topic}] "
            )

if include_topic == "/tf_gt":
    fd.close()
# print("Press any key to exit.")
# cv2.waitKey(0)
# cv2.destroyAllWindows()
