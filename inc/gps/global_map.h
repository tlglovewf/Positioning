#pragma once
#include "chframe.h"
#include "map_point.h"
#include "global_map_common.h"

namespace gm {
    
void get_map_block_id_from_gps(unsigned int& block_id, Eigen::Vector3d gps_latlon);
void get_new_global_id(long unsigned int& new_id, Eigen::Vector3d gps_latlon);
void get_map_block_id_from_id(unsigned int& block_id, long unsigned int id);
void get_gps_from_block_id(Eigen::Vector3d& gps_latlon, unsigned int block_id);
void get_blockids_frome_gps_list(std::vector<Eigen::Vector3d>& gps_list, std::vector<unsigned int>& blockid_list);
    
class GlobalMap 
{
public:
    
    void ReleaseMap();
    std::vector<std::shared_ptr<Frame>> frames;//地图中的帧
    std::vector<std::shared_ptr<MapPoint>> mappoints;//地图中的地图点
    unsigned int map_id;
    Eigen::Vector3d gps_anchor;
    std::vector<std::shared_ptr<Frame>> pose_graph_v1;//共视帧v1
    std::vector<std::shared_ptr<Frame>> pose_graph_v2;//共视帧v2
    std::vector<Eigen::Vector3d> pose_graph_e_posi;//v1到v2的平移
    std::vector<Eigen::Matrix3d> pose_graph_e_rot;//v1到v2的旋转
    std::vector<double> pose_graph_e_scale;//共视尺度，用来干嘛？
    std::vector<double> pose_graph_weight;//v1和v2共视的权重，即0.5*共视mapPoint数量
    bool checkFrameExist(std::shared_ptr<gm::Frame> query_frame);
    bool checkMPExist(std::shared_ptr<gm::MapPoint> query_mp);
    void CheckConnections();
    void DelMappoint(long unsigned int id);
    bool DelFrame(long unsigned int id);
    void GetMPPosiList(std::vector<Eigen::Vector3d>& mp_posis);
    std::shared_ptr<MapPoint> getMPById(long unsigned int id);
    std::shared_ptr<Frame> getFrameById(long unsigned int id);
    void CheckConsistence();
    void GetCovisi(std::shared_ptr<Frame> frame_p, std::map<std::shared_ptr< Frame>, int>& connections);
    void getFrameChildren(std::vector<std::shared_ptr<gm::Frame>>& children, std::shared_ptr<gm::Frame> cur_frame);
    void AssignKpToMp();
    void CalPoseEdgeVal();
    void UpdatePoseEdge();
    void CreateSubMap(int startframe_id, int endframe_id, GlobalMap& submap);
    void AddConnection(std::shared_ptr<Frame> v1,
                       std::shared_ptr<Frame> v2,
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
}  // namespace gm
