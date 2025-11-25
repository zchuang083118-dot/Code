import open3d as o3d
import numpy as np
import copy
#读取电脑中的 ply 点云文件

def lidar16_lidar32_T(pcd16,pcd32,trans_init):
    #为两个点云上上不同的颜色
    print("初始变换矩阵:",trans_init)
    pcd16.paint_uniform_color([0, 0, 1])    #source 为黄色
    pcd32.paint_uniform_color([0, 1, 0])#target 为蓝色
    # o3d.visualization.draw_geometries([pcd16,pcd32], window_name="点云初始位置1",
    #                                   width=1024, height=768,
    #                                   left=50, top=50,
    #                                   mesh_show_back_face=False)  # 可视化点云初始位置
    # #为两个点云分别进行outlier removal
    processed_source =pcd16
    processed_target = pcd32#.voxel_down_sample#(voxel_size=0.1)
    # print("16线雷达点云的个数：",processed_source)
    # print("32线雷达点云数量：",processed_target)
    threshold = 2.0 #移动范围的阀值

    # trans_init = np.linalg.inv(trans_init)
    #运行icp
    reg_p2p = o3d.pipelines.registration.registration_icp(
            processed_source, processed_target, threshold, trans_init,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=1000))

    #将我们的矩阵依照输出的变换矩阵进行变换
    print(reg_p2p.transformation)

    # 提取旋转矩阵 R (3x3)
    results = np.linalg.inv(reg_p2p.transformation)
    print(results)
    # R = reg_p2p.transformation[:3, :3]
    R = results[:3, :3]

    # 计算滚转 (Roll), 俯仰 (Pitch), 偏航 (Yaw)
    roll = np.arctan2(R[2, 1], R[2, 2])
    pitch = np.arctan2(-R[2, 0], np.sqrt(R[2, 1]**2 + R[2, 2]**2))
    yaw = np.arctan2(R[1, 0], R[0, 0])

    # 将弧度转换为度数（如果需要）
    roll_deg = np.degrees(roll)
    pitch_deg = np.degrees(pitch)
    yaw_deg = np.degrees(yaw)

    # 输出结果
    print(f"Roll (radian): {roll}, Pitch (radian): {pitch}, Yaw (radian): {yaw}")
    print(f"Roll (degree): {roll_deg}, Pitch (degree): {pitch_deg}, Yaw (degree): {yaw_deg}")
    result = copy.deepcopy(processed_source)
    result=result.transform(reg_p2p.transformation)
    o3d.visualization.draw_geometries([processed_target, result], window_name="ICP算法配准效果",
                                      width=1024, height=768,
                                      left=50, top=50,
                                      mesh_show_back_face=False)
    print(reg_p2p.fitness,reg_p2p.inlier_rmse)
     # 保存融合后的点云
    output_path = "aligned_point_cloud.pcd"
    o3d.io.write_point_cloud(output_path, result)
    print(f"融合后的点云已保存到 {output_path}")
    return reg_p2p.transformation                                                                       

# target = o3d.io.read_point_cloud("/home/zchuang/ubt_calicate/calicate/scripts/Data/factory_c1/factory.ply", remove_nan_points=True, remove_infinite_points=True) #  #source 为需要配准的点云
# source = o3d.io.read_point_cloud("/home/zchuang/ubt_calicate/calicate/scripts/Data/factory_c1/mid360.pcd", remove_nan_points=True, remove_infinite_points=True) #主雷达

# target = target.voxel_down_sample(voxel_size=0.05)
# source = source.voxel_down_sample(voxel_size=0.05)

#需要标定雷达到主雷达的外参
trans_init = np.asarray([    
 [0.38859083,  0.91546218, -0.10452829,1.0],
 [ 0.92136942, -0.38713526 , 0.03470849,1.0],
 [-0.00869228 ,-0.10979657, -0.99391607,1.0],
 [0.000000,  0.000000,  0.000000,  1.000000]
    ])

# lidar16_lidar32_T(source,target,trans_init)


