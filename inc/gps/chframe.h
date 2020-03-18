#pragma once

#include "global_map_common.h"

namespace gm 
{
class MapPoint;
class Frame 
{
public:
    
    Frame();
    long unsigned int id;
    bool isfix;
    bool isborder;//?
    bool doBA;
    bool doGraphOpti;
    bool doMatch;
    bool willDel;
    double time_stamp;
    std::string frame_file_name;
    std::vector<cv::KeyPoint> kps;
    Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> descriptors;
    std::vector<std::shared_ptr<MapPoint>> obss;//该帧能观测到的所有特征点
    std::vector<long unsigned int> obss_ids; //temp variable used in loading data
    std::vector<Eigen::Vector3d> acces;//该帧对应的imu数据的加速度
    std::vector<Eigen::Vector3d> gyros;//该帧对应的Imu数据的角速度
    std::vector<double> imu_times;//该帧对应的Imu数据的时间戳
    std::shared_ptr<Frame> imu_next_frame;
    long unsigned int imu_next_frame_id; //temp variable used in loading data
    Eigen::Vector3d position;//该帧的平移twc，融合gps之后参考坐标系将转换至gps世界系上，与gps_position为一个参考系
    
    Eigen::Quaternion<double,Eigen::DontAlign> direction;//该帧的旋转Rwc，融合gps之后参考坐标系将转换至gps世界系上
    Eigen::Vector3d gps_position;//该帧图像对应的gps坐标（utm投影坐标系下的）
    float gps_accu;//对应的gps坐标位置置信度
    int gps_avg_count;
    
    Eigen::Vector3d Tbc_posi;//平移外参
    Eigen::Quaternion<double,Eigen::DontAlign> Tbc_qua;//旋转外参

    float fx;
    float fy;
    float cx;
    float cy;
    float k1;  // radian
    float k2;  // radian
    float p1;  // tan
    float p2;  // tan
    int width;
    int height;

    Eigen::Matrix<double, 4,4,Eigen::DontAlign> getPose();
    void setPose(Eigen::Matrix4d pose);

    Eigen::Matrix<double, 3, 3> getKMat();
    Eigen::Matrix<double, 3, 4> getProjMat();

    void getDesc(int ind, Eigen::Matrix<unsigned char, Eigen::Dynamic, 1>& desc_out);
    void AddKPAndDesc(cv::KeyPoint kp,
                      Eigen::Matrix<unsigned char, Eigen::Dynamic, 1>& desc,
                      std::shared_ptr<MapPoint> mp);
    void LoadDesc(std::string desc_file);
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace gm
