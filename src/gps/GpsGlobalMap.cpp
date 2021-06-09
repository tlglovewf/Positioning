#include "GpsGlobalMap.h"
#include <set>
#include <vector>
#include <iostream>
using namespace std;
namespace gps{ 

    GpsFrame::GpsFrame()
    {
        gps_avg_count=0;
    }
    //返回当前帧位姿Twc
    Eigen::Matrix<double ,4,4,Eigen::DontAlign> GpsFrame::getPose()
    {
        Eigen::Matrix3d temp_rot(direction);
        Eigen::Matrix<double, 4,4,Eigen::DontAlign> temp_pose = Eigen::Matrix4d::Identity();
        temp_pose.block(0, 0, 3, 3) = temp_rot;
        temp_pose.block(0, 3, 3, 1) = position;
        return temp_pose;
    }

    void GpsFrame::setPose(Eigen::Matrix4d pose)
    {
        position = pose.block(0, 3, 3, 1);
        Eigen::Matrix3d tempRot = pose.block(0, 0, 3, 3);
        Eigen::Quaterniond temp_qua(tempRot);
        direction = temp_qua;
    }

    Eigen::Matrix<double, 3, 4> GpsFrame::getProjMat()
    {
        Eigen::Matrix<double, 3, 4> k_mat = Eigen::Matrix<double, 3, 4>::Zero();
        k_mat(0, 0) = fx;
        k_mat(1, 1) = fy;
        k_mat(0, 2) = cx;
        k_mat(1, 2) = cy;
        k_mat(2, 2) = 1;
        Eigen::Matrix<double, 3, 4> proj = k_mat * getPose().inverse();
        return proj;
    }



    //检查共视关系是否正确
    void GpsGlobalMap::CheckConnections()
    {
            std::set<std::shared_ptr<gps::GpsFrame>> all_frame_set;
            for(int i=0; i<frames.size(); i++)
            {
                all_frame_set.insert(frames[i]);
            }
            for(int i=0; i<pose_graph_v1.size(); i++)
            {
                if(all_frame_set.find(pose_graph_v1[i])==all_frame_set.end()||all_frame_set.find(pose_graph_v2[i])==all_frame_set.end())
                {
                    std::cout<<"non exist frame!!"<<std::endl;
                }
            }
    }
    
    //为地图计算共视关系，并保存在相关变量中
    void gps::GpsGlobalMap::CalConnections()
    {
        pose_graph_v1.clear();
        pose_graph_v2.clear();
        pose_graph_weight.clear();
        pose_graph_e_scale.clear();
        pose_graph_e_posi.clear();
        pose_graph_e_rot.clear();
        for(int i=0; i<frames.size(); i++)//遍历地图中的所有帧
        {
            std::map<std::shared_ptr<gps::GpsFrame>, int> frame_list;//<与当前帧有共视的frame,共视点的数量>
            for(int j=0; j<frames[i]->obss.size(); j++)//遍历该帧能观测到的每一个mp
            {
                
                if(frames[i]->obss[j]!=nullptr)
                {

                    for(int k=0; k<frames[i]->obss[j]->track.size(); k++)//遍历该特征点所在的所有帧
                    {
                        frame_list[frames[i]->obss[j]->track[k].frame]++;
                    }
                }
            }
            //std::cout<<"=========="<<frames[i]->id<<"=========="<<std::endl;
            for(std::map<std::shared_ptr<gps::GpsFrame>,int>::iterator mit=frame_list.begin(), mend=frame_list.end(); mit!=mend; mit++)//遍历frame_list，即遍历所有与当前帧共视的帧
            {
                if(mit->second<=20)
                {
                    continue;
                }
                //1:mit 2:frame[i]
                Eigen::Matrix<double,4,4,Eigen::DontAlign> T_2_1=frames[i]->getPose().inverse()*mit->first->getPose();//Tfrom mit to frame[i]
                Eigen::Matrix3d rot=T_2_1.block(0,0,3,3);
                Eigen::Vector3d posi=T_2_1.block(0,3,3,1);
                AddConnection(mit->first, frames[i] , posi, rot, 1, mit->second*0.5);//添加共视关系
            }
        }
    }

    //添加共视关系
    //v1,v2是直接共视的两帧
    //posi:v1到v2的平移
    //rot:v1到v2的旋转
    void GpsGlobalMap::AddConnection(std::shared_ptr<GpsFrame> v1, std::shared_ptr<GpsFrame> v2, Eigen::Vector3d& posi, Eigen::Matrix3d& rot,
        double scale, double weight)
    {
        bool dup=false;//标记该共视关系是否已经存在
        for(int i=0; i<pose_graph_v1.size(); i++)
        {
            if(pose_graph_v1[i]->id== v1->id && pose_graph_v2[i]->id== v2->id)
            {
                dup=true;
                break;
            }
        }
        if(dup)
        {
            return;
        }
        pose_graph_v1.push_back(v1);
        pose_graph_v2.push_back(v2);
        pose_graph_weight.push_back(weight);
        pose_graph_e_scale.push_back(scale);
        pose_graph_e_posi.push_back(posi);
        pose_graph_e_rot.push_back(rot);
    }
    
    void GpsGlobalMap::UpdatePoseEdge()
    {
        for(size_t i=0; i<pose_graph_v1.size(); i++)
        {
            Eigen::Matrix4d r_pose = pose_graph_v2[i]->getPose().inverse()*pose_graph_v1[i]->getPose();
            pose_graph_e_posi[i]=r_pose.block(0,3,3,1);
            pose_graph_e_rot[i]=r_pose.block(0,0,3,3);
        }
    }
    
    void GpsGlobalMap::AssignKpToMp()
    {
        for(size_t i=0; i<mappoints.size(); i++)
        {
            mappoints[i]->track.clear();
        }
        for(size_t i=0; i<frames.size(); i++)//遍历地图中的每一帧
        {
            for(size_t k=0; k<frames[i]->obss.size(); k++)//遍历该帧能观测到的所有地图点
            {
                if(frames[i]->obss[k]!=nullptr)
                {
                    TrackItem temp_track;
                    temp_track.frame=frames[i];
                    temp_track.kp_ind= k;
                    frames[i]->obss[k]->track.push_back(temp_track);
                }
            }
        }
    }

    //检查track是否正确
    void GpsGlobalMap::CheckConsistence()
    {
        std::set<std::shared_ptr<gps::GpsFrame>> all_frame_set;//地图中所有的帧
        for(int i=0; i<frames.size(); i++)
        {
            all_frame_set.insert(frames[i]);
        }
        for(size_t i=0; i<mappoints.size(); i++)
        {
            for(size_t j=0; j<mappoints[i]->track.size(); j++)
            {
                std::shared_ptr< GpsFrame> frame_p = mappoints[i]->track[j].frame;
                if(all_frame_set.find(frame_p)==all_frame_set.end())
                {
                    //std::cout<<"[CheckConsistence]non exist frame!!"<<std::endl;
                }
                if(frame_p==nullptr)
                {
                    //cout << "frame_p==nullptr" << endl;
                }
                std::shared_ptr< GpsMapPoint> mp = frame_p->obss[mappoints[i]->track[j].kp_ind];
                if(mp==nullptr)
                {
                    //cout << "mp==nullptr" << endl;
                }
                else
                {
                    if(mp->id!=mappoints[i]->id)
                    {
                        //cout <<"mp->id!=mappoints[i]->id: "<<mp->id<<" : "<<mappoints[i]->id;
                    }
                }
            }       
        }
        
        for(size_t i=0; i<frames.size(); i++)
        {
            for(size_t j=0; j<frames[i]->obss.size(); j++)
            {
                if(frames[i]->obss[j]!=nullptr)
                {
                    std::shared_ptr< GpsMapPoint> mp=frames[i]->obss[j];
                    bool find_one=false;
                    for(size_t k=0; k<mp->track.size(); k++)
                    {
                        if(mp->track[k].frame->id==frames[i]->id && (size_t)mp->track[k].kp_ind==j)
                        {
                            find_one=true;
                        }
                    }
                    if(find_one==false)
                    {
                        //cout << "find_one==false" << endl;
                        frames[i]->obss[j]==nullptr;
                    }
                }
            }
        }
        
    }
    
    void GpsGlobalMap::FilterTrack()
    {
        int dup_count=0;
        for(int i=0; i<mappoints.size(); i++)
        {
            std::map<std::shared_ptr< GpsFrame>, std::vector<int>> frame_mask;
            for(int j=0; j<mappoints[i]->track.size(); j++)
            {
                if(frame_mask.count(mappoints[i]->track[j].frame)!=0)
                {
                    frame_mask[mappoints[i]->track[j].frame].push_back(j);
                    dup_count++;
                }else{
                    std::vector<int> temp;
                    temp.push_back(j);
                    frame_mask[mappoints[i]->track[j].frame]=temp;
                }
            }
            std::map<std::shared_ptr< GpsFrame>, std::vector<int>>::iterator it;
            for ( it = frame_mask.begin(); it != frame_mask.end(); it++ ){
                if(it->second.size()>1){
                    float min_err=11111;
                    int min_trackid;
                    for(int j=0; j<it->second.size(); j++){
                        Eigen::Matrix<double, 3,4> proj_mat = it->first->getProjMat();
                        Eigen::Vector4d posi_homo;
                        posi_homo.block(0,0,3,1)=mappoints[i]->position;
                        posi_homo(3)=1;
                        Eigen::Vector3d proj_homo = proj_mat*posi_homo;
                        double u=proj_homo(0)/proj_homo(2);
                        double v=proj_homo(1)/proj_homo(2);
                        int kp_ind = mappoints[i]->track[it->second[j]].kp_ind;
                        cv::Point2f uv= it->first->kps[kp_ind].pt;
                        float proj_err=sqrt((uv.x-u)*(uv.x-u)+(uv.y-v)*(uv.y-v));
                        //std::cout<<kp_ind<<":"<<proj_err<<std::endl;
                        if(proj_err<min_err){
                            min_err=proj_err;
                            min_trackid=it->second[j];
                        }
                    }
                    for(int j=0; j<it->second.size(); j++){
                        if(it->second[j]!=min_trackid){
                            int kp_ind = mappoints[i]->track[it->second[j]].kp_ind;
                            //std::cout<<"del: "<<kp_ind<<std::endl;
                            it->first->obss[kp_ind]=nullptr;
                        }
                    }
                }
            }
        }
    }

}
