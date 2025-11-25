#include <iostream>
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>
#include <cmath>
#include <thread>
#include <chrono>

Eigen::Matrix4f trans_init = Eigen::Matrix4f::Identity();
float roll_deg = 0.0f, pitch_deg = 0.0f, yaw_deg = 0.0f;
float xd = 0.0f, yd = 0.0f, zd = 0.0f;

const float DEG_TO_RAD = M_PI / 180.0f;
const float ANGLE_STEP = 1.0f;       // 每次旋转步长（度）
const float DIST_STEP = 0.05f;       // 每次平移步长（米）

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_init(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target(new pcl::PointCloud<pcl::PointXYZ>);

pcl::visualization::PCLVisualizer::Ptr viewer;

// ---------------- 欧拉角转旋转矩阵 ----------------
Eigen::Matrix3f eulerToRotationMatrix(float roll, float pitch, float yaw)
{
    float cr = cos(roll), sr = sin(roll);
    float cp = cos(pitch), sp = sin(pitch);
    float cy = cos(yaw), sy = sin(yaw);

    Eigen::Matrix3f Rx; Rx << 1,0,0, 0,cr,-sr, 0,sr,cr;
    Eigen::Matrix3f Ry; Ry << cp,0,sp, 0,1,0, -sp,0,cp;
    Eigen::Matrix3f Rz; Rz << cy,-sy,0, sy,cy,0, 0,0,1;

    return Rz * Ry * Rx; // ZYX顺序
}

// ---------------- 应用变换 ----------------
void apply_transform()
{
    // 计算源点云质心
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud_source_init, centroid);

    // 平移到原点
    Eigen::Matrix4f T_center = Eigen::Matrix4f::Identity();
    T_center(0,3) = -centroid[0];
    T_center(1,3) = -centroid[1];
    T_center(2,3) = -centroid[2];

    // 旋转矩阵
    Eigen::Matrix3f R = eulerToRotationMatrix(roll_deg*DEG_TO_RAD,
                                              pitch_deg*DEG_TO_RAD,
                                              yaw_deg*DEG_TO_RAD);
    Eigen::Matrix4f T_rot = Eigen::Matrix4f::Identity();
    T_rot.block<3,3>(0,0) = R;

    // 平移回原点 + 用户平移
    Eigen::Matrix4f T_back = Eigen::Matrix4f::Identity();
    T_back(0,3) = centroid[0] + xd;
    T_back(1,3) = centroid[1] + yd;
    T_back(2,3) = centroid[2] + zd;

    // 总变换矩阵
    Eigen::Matrix4f T = T_back * T_rot * T_center;

    // 应用变换
    pcl::transformPointCloud(*cloud_source_init, *cloud_source, T);
    trans_init = T;

    // 更新可视化点云
    viewer->updatePointCloud(cloud_source, "source");

    std::cout << "当前外参矩阵:\n" << trans_init << std::endl;
}

// ---------------- 键盘事件回调 ----------------
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void*)
{
    if(!event.keyDown()) return;

    int key = event.getKeyCode(); // 使用 ASCII 码避免冲突
    switch(key)
    {
        case 'a': yaw_deg += ANGLE_STEP; break;
        case 'z': yaw_deg -= ANGLE_STEP; break;
        case 'w': roll_deg += ANGLE_STEP; break;
        case 's': roll_deg -= ANGLE_STEP; break;
        case 'e': pitch_deg += ANGLE_STEP; break;
        case 'd': pitch_deg -= ANGLE_STEP; break;
        case 'r': xd += DIST_STEP; break;
        case 'f': xd -= DIST_STEP; break;
        case 't': yd += DIST_STEP; break;
        case 'g': yd -= DIST_STEP; break;
        case 'y': zd += DIST_STEP; break;
        case 'h': zd -= DIST_STEP; break;
        default: return; // 其他键不处理
    }

    apply_transform();
}

// ---------------- 主函数 ----------------
int main()
{
    // 读取点云
    if(pcl::io::loadPLYFile("/home/zchuang/ubt_calicate/calicate/scripts/Data/factory_c1/factory.ply", *cloud_target) == -1)
    {
        PCL_ERROR("无法读取目标点云文件！\n");
        return -1;
    }
    if(pcl::io::loadPCDFile("/home/zchuang/ubt_calicate/calicate/scripts/Data/factory_c1/mid360.pcd", *cloud_source_init) == -1)
    {
        PCL_ERROR("无法读取源点云文件！\n");
        return -1;
    }

    *cloud_source = *cloud_source_init;

    // 初始化可视化
    viewer.reset(new pcl::visualization::PCLVisualizer("Cloud Registration"));
    viewer->setBackgroundColor(0,0,0);
    viewer->addPointCloud(cloud_target, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud_target,0,0,255), "target");
    viewer->addPointCloud(cloud_source, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(cloud_source,255,0,0), "source");

    viewer->registerKeyboardCallback(keyboardEventOccurred);

    // 主循环
    while(!viewer->wasStopped())
    {
        viewer->spinOnce(50);
        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 减少 CPU 占用
    }

    return 0;
}
