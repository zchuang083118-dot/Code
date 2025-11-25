# coding:utf-8
import open3d as o3d
import numpy as np
import math
import copy
from ICP import lidar16_lidar32_T  # 引入自定义ICP函数，用于点云配准

# ---------------- 全局变量 ----------------
space_pressed = False                 # 空格键状态（可扩展使用）
trans_init = np.eye(4)               # 初始外参矩阵 4x4
ARC_TO_DEG = 57.29577951308238       # 弧度转角度
DEG_TO_ARC = 0.0174532925199433      # 角度转弧度

# 欧拉角（单位：度）
roll_deg = pitch_deg = yaw_deg = 0.0
# 平移量（单位：米）
xd = yd = zd = 0.0

# 每次按键改变的角度或位移
angel_pixel = 1.0       # 旋转步长（度）
distance_pixel = 0.05   # 平移步长（米）

# 点云对象
pcd_sourse_init = None  # 原始点云，用于每次恢复
pcd_sourse = None       # 当前显示的点云
pcd_target = None       # 目标点云

# ---------------- 工具函数 ----------------
def eulerAnglesToRotationMatrix(theta):
    """
    将欧拉角转换为旋转矩阵
    theta: [roll, pitch, yaw] 单位：弧度
    返回3x3旋转矩阵
    """
    rx = np.array([[1,0,0],
                   [0, math.cos(theta[0]), -math.sin(theta[0])],
                   [0, math.sin(theta[0]), math.cos(theta[0])]])
    ry = np.array([[math.cos(theta[1]),0,math.sin(theta[1])],
                   [0,1,0],
                   [-math.sin(theta[1]),0,math.cos(theta[1])]])
    rz = np.array([[math.cos(theta[2]), -math.sin(theta[2]), 0],
                   [math.sin(theta[2]), math.cos(theta[2]),0],
                   [0,0,1]])
    return rz @ ry @ rx  # ZYX顺序旋转（Yaw-Pitch-Roll）

def update_trans_init():
    """
    根据当前欧拉角和位移更新全局外参矩阵
    """
    global trans_init, roll_deg, pitch_deg, yaw_deg, xd, yd, zd
    r_mat = eulerAnglesToRotationMatrix([roll_deg*DEG_TO_ARC, 
                                         pitch_deg*DEG_TO_ARC, 
                                         yaw_deg*DEG_TO_ARC])
    trans_init[:3,:3] = r_mat
    trans_init[:3,3] = [xd, yd, zd]
    print("当前外参矩阵:\n", trans_init)

def apply_transform(vis):
    """
    根据当前欧拉角和位移对源点云进行旋转和平移，并刷新可视化
    """
    global pcd_sourse, pcd_sourse_init, roll_deg, pitch_deg, yaw_deg, xd, yd, zd
    # 复制原始点云，避免累积误差
    pcd_copy = copy.deepcopy(pcd_sourse_init)
    # 计算旋转矩阵
    r_mat = eulerAnglesToRotationMatrix([roll_deg*DEG_TO_ARC, 
                                         pitch_deg*DEG_TO_ARC, 
                                         yaw_deg*DEG_TO_ARC])
    # 应用旋转
    pcd_copy.rotate(r_mat)
    # 应用平移
    pcd_copy.translate([xd, yd, zd])
    # 更新当前显示点云
    pcd_sourse.points = pcd_copy.points
    vis.update_geometry(pcd_sourse)
    # 更新全局外参矩阵
    update_trans_init()

# ---------------- 按键回调 ----------------
# 欧拉角旋转按键
def key_q(vis):  # +yaw（绕Z轴）
    global yaw_deg
    yaw_deg += angel_pixel
    apply_transform(vis)

def key_a(vis):  # -yaw
    global yaw_deg
    yaw_deg -= angel_pixel
    apply_transform(vis)

def key_w(vis):  # +roll（绕X轴）
    global roll_deg
    roll_deg += angel_pixel
    apply_transform(vis)

def key_s(vis):  # -roll
    global roll_deg
    roll_deg -= angel_pixel
    apply_transform(vis)

def key_e(vis):  # +pitch（绕Y轴）
    global pitch_deg
    pitch_deg += angel_pixel
    apply_transform(vis)

def key_d(vis):  # -pitch
    global pitch_deg
    pitch_deg -= angel_pixel
    apply_transform(vis)

# 平移按键
def key_r(vis):  # +x
    global xd
    xd += distance_pixel
    apply_transform(vis)

def key_f(vis):  # -x
    global xd
    xd -= distance_pixel
    apply_transform(vis)

def key_t(vis):  # +y
    global yd
    yd += distance_pixel
    apply_transform(vis)

def key_g(vis):  # -y
    global yd
    yd -= distance_pixel
    apply_transform(vis)

def key_y(vis):  # +z
    global zd
    zd += distance_pixel
    apply_transform(vis)

def key_h(vis):  # -z
    global zd
    zd -= distance_pixel
    apply_transform(vis)

# ---------------- 主程序 ----------------
if __name__ == '__main__':
    # 读取目标点云和源点云
    path_target = "/home/zchuang/ubt_calicate/calicate/scripts/Data/factory_c1/factory.ply"   # 目标点云
    path_source = "/home/zchuang/ubt_calicate/calicate/scripts/Data/factory_c1/mid360.pcd"   # 需要配准的点云

    pcd_target = o3d.io.read_point_cloud(path_target)
    pcd_sourse_init = o3d.io.read_point_cloud(path_source)

    # 对点云进行体素下采样，提高显示和计算效率
    voxel_size = 0.1
    pcd_target = pcd_target.voxel_down_sample(voxel_size)
    pcd_sourse_init = pcd_sourse_init.voxel_down_sample(voxel_size)

    # 复制一份源点云用于可视化旋转/平移
    pcd_sourse = copy.deepcopy(pcd_sourse_init)

    # 设置点云颜色，目标蓝色，源红色
    pcd_target.paint_uniform_color([0,0,1])
    pcd_sourse.paint_uniform_color([1,0,0])

    # 设置按键映射
    key_to_callback = {
        ord("Q"): key_q,
        ord("A"): key_a,
        ord("W"): key_w,
        ord("S"): key_s,
        ord("E"): key_e,
        ord("D"): key_d,
        ord("R"): key_r,
        ord("F"): key_f,
        ord("T"): key_t,
        ord("G"): key_g,
        ord("Y"): key_y,
        ord("H"): key_h
    }

    # 打开Open3D可视化窗口，并绑定按键回调
    o3d.visualization.draw_geometries_with_key_callbacks([pcd_target, pcd_sourse], key_to_callback)

    # 完成手动标定后调用ICP函数，使用当前外参矩阵进行精细配准
    lidar16_lidar32_T(pcd_sourse_init, pcd_target, trans_init)
