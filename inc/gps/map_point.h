#pragma once
#include "global_map_common.h"

namespace gm {
class Frame;

class TrackItem 
{
public:
    std::shared_ptr<Frame> frame;//mapPoint所在的帧
    int kp_ind;//mapPoint在该帧中能观测到的地图点中的序号
    void getUV(float& x, float& y, int& octave);
};

class MapPoint {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    MapPoint();
    long unsigned int id = -1;
    bool isfix;
    bool isbad;
    bool doBA;
    bool isborder;
    int match_count=0;
    Eigen::Vector3d position;
    std::vector<TrackItem> track;//存储该MapPoint所在的所有帧
    int calDescDiff(Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& query_desc);
    void getALLDesc(Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>& descs);
};

}  // namespace gm
