# coding:utf-8

import open3d as o3d
import numpy as np
import math
import copy
from ICP import lidar16_lidar32_T

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import os
from ros2_listen_lidar import PointCloudSubscriber
space_pressed = False  # 全局变量，记录空格键状态

def key_space_down(vis):
    global space_pressed
    space_pressed = True
    print("空格按下")
    # return False

def key_space_up(vis):
    global space_pressed
    space_pressed = False
    print("空格抬起")
    return False

def R33_to_T(R1):
    R1_4x4 = np.eye(4)
    t = np.array([1.0, 1.0, 1.0])
    R1_4x4[:3, :3] = R1  # 放入旋转部分
    R1_4x4[:3, 3] = t
    print("旋转矩阵：\n", R1_4x4)
    # lidar16_lidar32_T(pcd_sourse_init, pcd_target, R1_4x4)

def eulerAnglesToRotationMatrix(theta):
    rx = np.asarray([[1.0, 0.0, 0.0],
                     [0.0, math.cos(theta[0]), -math.sin(theta[0])],
                     [0.0, math.sin(theta[0]), math.cos(theta[0])]])

    ry = np.asarray([[math.cos(theta[1]), 0.0, math.sin(theta[1])],
                     [0.0, 1.0, 0.0],
                     [-math.sin(theta[1]), 0.0, math.cos(theta[1])]])

    rz = np.asarray([[math.cos(theta[2]), -math.sin(theta[2]), 0.0],
                     [math.sin(theta[2]), math.cos(theta[2]), 0.0],
                     [0.0, 0.0, 1.0]])
    r = rz @ ry @ rx
    # print(r)
    return r
    # r = np.matmul(np.matmul(rz,ry),rx)


def key_call(x):
    global to_reset, yaw_deg, roll_deg, pitch_deg, trans_init, angel_pixel, distance_pixel
    # a = keyboard.KeyboardEvent('down',28,'enter')
    if x.event_type == 'down' and x.name == 'q':
        yaw_deg = yaw_deg + angel_pixel
        # // 转化为弧度
        roll_arc = roll_deg * DEG_TO_ARC;  # // 绕X轴
        pitch_arc = pitch_deg * DEG_TO_ARC;  # // 绕Y轴
        yaw_arc = yaw_deg * DEG_TO_ARC;  # // 绕Z轴
        print("你按下了 q 键", yaw_deg, roll_deg, pitch_deg)
        print("你按下了 q 键", roll_arc, pitch_arc, yaw_arc)

        rr = eulerAnglesToRotationMatrix([roll_arc, pitch_arc, yaw_arc])
        trans_init[0:3, 0:3] = rr
        # print(trans_init)
        pcd_sourse.transform(trans_init)


        o3d.visualization.draw_geometries([pcd_target, pcd_sourse])


def key_q(vis):
    global yaw_deg, roll_deg, pitch_deg, trans_init, angel_pixel, temp_r
    pcd_sourse.rotate(temp_r)

    yaw_deg = yaw_deg + angel_pixel
    # // 转化为弧度
    roll_arc = roll_deg * DEG_TO_ARC;  # // 绕X轴
    pitch_arc = pitch_deg * DEG_TO_ARC;  # // 绕Y轴
    yaw_arc = yaw_deg * DEG_TO_ARC;  # // 绕Z轴
    print("你按下了 q 键", yaw_deg, roll_deg, pitch_deg)

    R1 = pcd_sourse.get_rotation_matrix_from_xyz((roll_arc, pitch_arc, yaw_arc))
    # r2 = o3d.geometry.get_rotation_matrix_from_xyz((roll_arc, pitch_arc, yaw_arc))
    pcd_sourse.rotate(R1)  # 不指定旋转中心
    # print("旋转矩阵：\n", R1)\
    R33_to_T(R1)
    # if space_pressed:
    #     R33_to_T(R1)
    vis.update_geometry(pcd_sourse)
    R1 = np.linalg.inv(R1)
    temp_r = R1


def key_a(vis):
    global yaw_deg, roll_deg, pitch_deg, trans_init, angel_pixel, temp_r
    pcd_sourse.rotate(temp_r)
    yaw_deg = yaw_deg - angel_pixel
    # // 转化为弧度
    roll_arc = roll_deg * DEG_TO_ARC;  # // 绕X轴
    pitch_arc = pitch_deg * DEG_TO_ARC;  # // 绕Y轴
    yaw_arc = yaw_deg * DEG_TO_ARC;  # // 绕Z轴
    print("你按下了 a 键", yaw_deg, roll_deg, pitch_deg)
    # print("你按下了 q 键",roll_arc, pitch_arc, yaw_arc)

    R1 = pcd_sourse.get_rotation_matrix_from_xyz((roll_arc, pitch_arc, yaw_arc))
    pcd_sourse.rotate(R1)  # 不指定旋转中心
    # print("旋转矩阵：\n", R1)
    R33_to_T(R1)
    # if space_pressed:
    #     R33_to_T(R1)
    vis.update_geometry(pcd_sourse)
    R1 = np.linalg.inv(R1)
    temp_r = R1


def key_w(vis):
    global yaw_deg, roll_deg, pitch_deg, trans_init, angel_pixel, temp_r
    pcd_sourse.rotate(temp_r)

    roll_deg = roll_deg + angel_pixel
    # // 转化为弧度
    roll_arc = roll_deg * DEG_TO_ARC;  # // 绕X轴
    pitch_arc = pitch_deg * DEG_TO_ARC;  # // 绕Y轴
    yaw_arc = yaw_deg * DEG_TO_ARC;  # // 绕Z轴
    print("你按下了 w 键", yaw_deg, roll_deg, pitch_deg)
    # print("你按下了 q 键",roll_arc, pitch_arc, yaw_arc)

    R1 = pcd_sourse.get_rotation_matrix_from_xyz((roll_arc, pitch_arc, yaw_arc))
    pcd_sourse.rotate(R1)  # 不指定旋转中心
    # print("旋转矩阵：\n", R1)
    R33_to_T(R1)
    # if space_pressed:
    #     R33_to_T(R1)
    vis.update_geometry(pcd_sourse)
    R1 = np.linalg.inv(R1)
    temp_r = R1


def key_s(vis):
    global yaw_deg, roll_deg, pitch_deg, trans_init, angel_pixel, temp_r
    pcd_sourse.rotate(temp_r)

    roll_deg = roll_deg - angel_pixel
    # // 转化为弧度
    roll_arc = roll_deg * DEG_TO_ARC;  # // 绕X轴
    pitch_arc = pitch_deg * DEG_TO_ARC;  # // 绕Y轴
    yaw_arc = yaw_deg * DEG_TO_ARC;  # // 绕Z轴
    print("你按下了 s 键", yaw_deg, roll_deg, pitch_deg)
    # print("你按下了 q 键",roll_arc, pitch_arc, yaw_arc)

    R1 = pcd_sourse.get_rotation_matrix_from_xyz((roll_arc, pitch_arc, yaw_arc))
    pcd_sourse.rotate(R1)  # 不指定旋转中心
    # print("旋转矩阵：\n", R1)
    R33_to_T(R1)
    # if space_pressed:
    #     R33_to_T(R1)
    vis.update_geometry(pcd_sourse)
    R1 = np.linalg.inv(R1)
    temp_r = R1


def key_e(vis):
    global yaw_deg, roll_deg, pitch_deg, trans_init, angel_pixel, temp_r
    pcd_sourse.rotate(temp_r)

    pitch_deg = pitch_deg + angel_pixel
    # // 转化为弧度
    roll_arc = roll_deg * DEG_TO_ARC;  # // 绕X轴
    pitch_arc = pitch_deg * DEG_TO_ARC;  # // 绕Y轴
    yaw_arc = yaw_deg * DEG_TO_ARC;  # // 绕Z轴
    print("你按下了 e 键", yaw_deg, roll_deg, pitch_deg)
    # print("你按下了 q 键",roll_arc, pitch_arc, yaw_arc)

    R1 = pcd_sourse.get_rotation_matrix_from_xyz((roll_arc, pitch_arc, yaw_arc))
    pcd_sourse.rotate(R1)  # 不指定旋转中心
    # print("旋转矩阵：\n", R1)
    R33_to_T(R1)
    # if space_pressed:
    #     R33_to_T(R1)
    vis.update_geometry(pcd_sourse)
    R1 = np.linalg.inv(R1)
    temp_r = R1


def key_d(vis):
    global yaw_deg, roll_deg, pitch_deg, trans_init, angel_pixel, temp_r
    pcd_sourse.rotate(temp_r)

    pitch_deg = pitch_deg - angel_pixel
    # // 转化为弧度
    roll_arc = roll_deg * DEG_TO_ARC;  # // 绕X轴
    pitch_arc = pitch_deg * DEG_TO_ARC;  # // 绕Y轴
    yaw_arc = yaw_deg * DEG_TO_ARC;  # // 绕Z轴
    print("你按下了 d 键", yaw_deg, roll_deg, pitch_deg)

    R1 = pcd_sourse.get_rotation_matrix_from_xyz((roll_arc, pitch_arc, yaw_arc))
    pcd_sourse.rotate(R1)  # 不指定旋转中心
    R33_to_T(R1)
    # if space_pressed:
    #     R33_to_T(R1)
    vis.update_geometry(pcd_sourse)
    R1 = np.linalg.inv(R1)
    temp_r = R1


# xyz move
def key_r(vis):
    global xd, yd, zd, trans_init, temp_t, distance_pixel
    # pcd_sourse.transform(trans_init)
    pcd_sourse.translate((temp_t[0], temp_t[1], temp_t[2]))

    xd = xd + distance_pixel
    print("你按下了 r 键; xyz偏移为 ", xd, yd, zd)

    pcd_sourse.translate((xd, yd, zd))
    vis.update_geometry(pcd_sourse)
    temp_t[0] = -xd
    temp_t[1] = -yd
    temp_t[2] = -zd


def key_f(vis):
    global xd, yd, zd, trans_init, temp_t, distance_pixel
    pcd_sourse.translate((temp_t[0], temp_t[1], temp_t[2]))

    xd = xd - distance_pixel
    print("你按下了 f 键; xyz偏移为 ", xd, yd, zd)
    pcd_sourse.translate((xd, yd, zd))
    vis.update_geometry(pcd_sourse)
    temp_t[0] = -xd
    temp_t[1] = -yd
    temp_t[2] = -zd


def key_t(vis):
    global xd, yd, zd, trans_init, temp_t, distance_pixel
    pcd_sourse.translate((temp_t[0], temp_t[1], temp_t[2]))

    yd = yd + distance_pixel
    print("你按下了 t 键; xyz偏移为 ", xd, yd, zd)

    pcd_sourse.translate((xd, yd, zd))
    vis.update_geometry(pcd_sourse)
    temp_t[0] = -xd
    temp_t[1] = -yd
    temp_t[2] = -zd


def key_g(vis):
    global xd, yd, zd, trans_init, temp_t, distance_pixel
    pcd_sourse.translate((temp_t[0], temp_t[1], temp_t[2]))

    yd = yd - distance_pixel
    print("你按下了 g 键; xyz偏移为 ", xd, yd, zd)

    pcd_sourse.translate((xd, yd, zd))
    vis.update_geometry(pcd_sourse)
    temp_t[0] = -xd
    temp_t[1] = -yd
    temp_t[2] = -zd


def key_y(vis):
    global xd, yd, zd, trans_init, temp_t, distance_pixel
    pcd_sourse.translate((temp_t[0], temp_t[1], temp_t[2]))

    zd = zd + distance_pixel
    print("你按下了 y 键; xyz偏移为 ", xd, yd, zd)

    pcd_sourse.translate((xd, yd, zd))
    vis.update_geometry(pcd_sourse)
    temp_t[0] = -xd
    temp_t[1] = -yd
    temp_t[2] = -zd


def key_h(vis):
    global xd, yd, zd, trans_init, temp_t, distance_pixel
    pcd_sourse.translate((temp_t[0], temp_t[1], temp_t[2]))

    zd = zd - distance_pixel
    print("你按下了 h 键; xyz偏移为 ", xd, yd, zd)

    pcd_sourse.translate((xd, yd, zd))
    vis.update_geometry(pcd_sourse)
    temp_t[0] = -xd
    temp_t[1] = -yd
    temp_t[2] = -zd


if __name__ == '__main__':
    # rclpy.init(args=None)
    # node = PointCloudSubscriber()
   
    trans_init = np.eye(4)
    temp_r = np.asarray([[1.0, 0.0, 0.0],
                         [0.0, 1.0, 0.0],
                         [0.0, 0.0, 1.0]]).astype(np.float64)
    temp_t = np.asarray([0.0, 0.0, 0.0]).astype(np.float64)

    # 分辨率
    angel_pixel = 1.0
    distance_pixel = 0.05

    path2 = r"/home/zchuang/ubt_calicate/calicate/scripts/Data/factory_c1/factory.ply"  #主
    path1 = r"/home/zchuang/ubt_calicate/calicate/scripts/Data/factory_c1/mid360.pcd"

    pcd_target = o3d.io.read_point_cloud(path2)  #目标点云 1-16 目标点云为16-pcd
    pcd_sourse = o3d.io.read_point_cloud(path1)  #需要配准点云 1-16 目标点云为1-pcd
    # # 等待 target 点云
    # while node.pcd_target is None:
    #     rclpy.spin_once(node)
    # pcd_sourse = node.pcd_target

    pcd_sourse = pcd_sourse.voxel_down_sample(voxel_size=0.1)
    pcd_target = pcd_target.voxel_down_sample(voxel_size=0.1)
    pcd_sourse_init = pcd_sourse
    lidar_16 = np.array(pcd_target.points)
  
    # 建立一个o3d的点云对象
    pcd_target = o3d.geometry.PointCloud()
    # 使用Vector3dVector方法转换
    pcd_target.points = o3d.utility.Vector3dVector(lidar_16)

    pcd_target.paint_uniform_color([0, 0, 1])
    pcd_sourse.paint_uniform_color([1, 0, 0])
    pcd_sourse.transform(trans_init)

    vis = o3d.visualization.Visualizer()

    ARC_TO_DEG = 57.29577951308238
    DEG_TO_ARC = 0.0174532925199433
    # // 设定车体欧拉角(角度)，绕固定轴
    roll_deg = 0.00001;  # // 绕X轴
    pitch_deg = 0.00001;  # // 绕Y轴
    yaw_deg = 0.00001;  # // 绕Z轴
    xd = 0.000001
    yd = 0.000001
    zd = 0.000001

    key_to_callback = {}

    # --- 普通按键 ---
    key_to_callback[ord("Q")] = key_q
    key_to_callback[ord("A")] = key_a
    key_to_callback[ord("W")] = key_w
    key_to_callback[ord("S")] = key_s
    key_to_callback[ord("E")] = key_e
    key_to_callback[ord("D")] = key_d

    key_to_callback[ord("R")] = key_r
    key_to_callback[ord("F")] = key_f
    key_to_callback[ord("T")] = key_t
    key_to_callback[ord("G")] = key_g
    key_to_callback[ord("Y")] = key_y
    key_to_callback[ord("H")] = key_h

    # --- 空格按下(space down) ---
    key_to_callback[32] = key_space_down      # 空格键按下：ASCII 32

    # --- 空格释放(space up) ---
    key_to_callback[32 + 256] = key_space_up  # Open3D 的释放事件 = ASCII + 256


    # o3d.visualization.draw_geometries_with_key_callbacks([pcd_target, pcd_sourse], key_to_callback)   #初始外参获取
       #精细化标定
    # trans_init = np.array([
    #    [ 0.37437856 , 0.92661898,  0.03489932 , 1.        ],
    #     [ 0.92727074, -0.37398498, -0.0174416,   1.        ],
    #     [-0.0031099,   0.03889088 ,-0.99923862 , 1.        ],
    #     [ 0.   ,       0.     ,     0.    ,      1.        ]
    # ])
    # lidar16_lidar32_T(pcd_sourse, pcd_target, trans_init)
    # rclpy.spin(node)
    # node.destroy_node()
    # rclpy.shutdown()
 