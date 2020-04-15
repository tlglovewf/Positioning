#ifndef _GPSFUSEMAP_H_H
#define _GPSFUSEMAP_H_H
#include <memory>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>

namespace gps {
     
    class GpsFrame;
    class TrackItem 
    {
    public:
        std::shared_ptr<GpsFrame> frame;//mapPoint所在的帧
        int kp_ind;//mapPoint在该帧中能观测到的地图点中的序号
    };

    class GpsMapPoint {
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    GpsMapPoint(){}
    long unsigned int id = -1;
    Eigen::Vector3d position;
    std::vector<TrackItem> track;//存储该MapPoint所在的所有帧
    };

    class GpsMapPoint;
    class GpsFrame 
    {
      public:

        GpsFrame();
        long unsigned int id;

        double time_stamp;
        std::vector<cv::KeyPoint> kps;
        std::vector<std::shared_ptr<GpsMapPoint>> obss;//该帧能观测到的所有特征点
        Eigen::Vector3d position;//该帧的平移twc，融合gps之后参考坐标系将转换至gps世界系上，与gps_position为一个参考系

        Eigen::Quaternion<double,Eigen::DontAlign> direction;//该帧的旋转Rwc，融合gps之后参考坐标系将转换至gps世界系上
        Eigen::Vector3d gps_position;//该帧图像对应的gps坐标（utm投影坐标系下的）
        float gps_accu;//对应的gps坐标位置置信度
        int gps_avg_count;

        float fx;
        float fy;
        float cx;
        float cy;

        Eigen::Matrix<double, 4,4,Eigen::DontAlign> getPose();
        void setPose(Eigen::Matrix4d pose);

        Eigen::Matrix<double, 3, 4> getProjMat();

        public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    class GpsGlobalMap 
    {
      public:

        std::vector<std::shared_ptr<GpsFrame>> frames;//地图中的帧
        std::vector<std::shared_ptr<GpsMapPoint>> mappoints;//地图中的地图点
        std::vector<std::shared_ptr<GpsFrame>> pose_graph_v1;//共视帧v1
        std::vector<std::shared_ptr<GpsFrame>> pose_graph_v2;//共视帧v2
        std::vector<Eigen::Vector3d> pose_graph_e_posi;//v1到v2的平移
        std::vector<Eigen::Matrix3d> pose_graph_e_rot;//v1到v2的旋转
        std::vector<double> pose_graph_e_scale;//共视尺度，用来干嘛？
        std::vector<double> pose_graph_weight;//v1和v2共视的权重，即0.5*共视mapPoint数量
        void CheckConnections();
        void CheckConsistence();
        void AssignKpToMp();

        void UpdatePoseEdge();

        void AddConnection(std::shared_ptr<GpsFrame> v1,
                           std::shared_ptr<GpsFrame> v2,
                           Eigen::Vector3d& posi,
                           Eigen::Matrix3d& rot,
                           double scale,
                           double weight);
        void CalConnections();
        void FilterTrack(); //remove the track that connect to the same frame. Keep the one with smallest reprojection error.
        //std::set<unsigned int> GetNearBlock(std::vector<unsigned int> block_ids);
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}
#endif