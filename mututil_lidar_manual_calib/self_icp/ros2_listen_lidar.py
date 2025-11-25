#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import open3d as o3d
import numpy as np
import os


class PointCloudSubscriber(Node):
    def __init__(self):
        super().__init__('pointcloud_subscriber')

        # 订阅两个点云（你可以换 topic 名）
        self.create_subscription(PointCloud2, "/livox/lidar", self.cb_target, 10)
        # self.create_subscription(PointCloud2, "/cloud_source", self.cb_source, 10)

        self.pcd_target = None
        # self.pcd_source = None
        self.save_once = False

    def cb_target(self, msg):
        self.get_logger().info("接收到 target 点云")
        self.pcd_target = self.ros2_to_o3d(msg)

    # def cb_source(self, msg):
    #     self.get_logger().info("接收到 source 点云")
    #     self.pcd_source = self.ros2_to_o3d(msg)

    #     # 当两个都收到了再执行配准
    #     if self.pcd_target is not None:
    #         self.do_registration()

    def ros2_to_o3d(self, cloud_msg: PointCloud2):
        """ROS2 PointCloud2 → Open3D point cloud"""

        # --------------------------
        # 1. 解析 XYZ
        # --------------------------
        points_list = []
        for p in pc2.read_points(cloud_msg, skip_nans=True):
            points_list.append([p[0], p[1], p[2]])

        np_points = np.array(points_list, dtype=np.float32)
        o3d_cloud = o3d.geometry.PointCloud()
        o3d_cloud.points = o3d.utility.Vector3dVector(np_points)

        # --------------------------
        # 2. ROS2 时间戳转浮点秒
        # --------------------------
        stamp = cloud_msg.header.stamp  # builtin_interfaces/Time
        t = stamp.sec + stamp.nanosec * 1e-9  # float seconds

        # 格式化成 "1732435346.123456"
        timestamp_str = f"{t:.6f}"

        # --------------------------
        # # 3. 自动按时间戳保存点云
        # # --------------------------
        # save_path = f"/home/zchuang/ubt_calicate/calicate/scripts/Data/factory_c1/cloud_{timestamp_str}.pcd"

        # # 目录不存在自动创建
   
        # os.makedirs(os.path.dirname(save_path), exist_ok=True)

        # o3d.io.write_point_cloud(save_path, o3d_cloud)
        # # print(f"点云已保存：{save_path}")

        return o3d_cloud

    # def do_registration(self):
    #     self.get_logger().info("执行点云配准...")

    #     # 使用 Open3D ICP
    #     threshold = 0.5
    #     trans_init = np.eye(4)

    #     reg = o3d.pipelines.registration.registration_icp(
    #         self.pcd_source,
    #         self.pcd_target,
    #         threshold,
    #         trans_init,
    #         o3d.pipelines.registration.TransformationEstimationPointToPoint()
    #     )

    #     self.get_logger().info(f"ICP 完成，转换矩阵：\n{reg.transformation}")





