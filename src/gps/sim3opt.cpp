#include "sim3opt.h"
#include <sim3_match.h>


//使用sim3对in_pose进行变换得到out_pose
static void transformPoseUseSim3(Eigen::Matrix4d& sim3, Eigen::Matrix4d& in_pose,  Eigen::Matrix4d& out_pose)
{
    Eigen::Matrix3d R_tran=sim3.block(0,0,3,3);
    double scale=R_tran.block(0,0,3,1).norm();
    R_tran=R_tran/scale;
    Eigen::Matrix3d R_in=in_pose.block(0,0,3,3);
    Eigen::Matrix3d R_out=R_tran*R_in;
    Eigen::Vector4d t_out=sim3*in_pose.block(0,3,4,1);
    out_pose= Eigen::Matrix4d::Identity();
    out_pose.block(0,0,3,3) = R_out;
    out_pose.block(0,3,4,1) = t_out;
}

Sim3Opt::Sim3Opt()
{
   initGPS = false;
   mGlobalMap.ReleaseMap();
   //mGlobalMap = NULL;
}

Sim3Opt::~Sim3Opt()
{
    
}

//要求消息的是间t已与视觉部分同步
void Sim3Opt::GPS_input(const GPSData& GPS_msg)
{
    double xyz[3];
    double latitude = GPS_msg.lat;
    double longitude = GPS_msg.lon;
    double altitude = GPS_msg.alt;
    double time = GPS_msg.t;
    double pos_accuracy = GPS_msg.conf;

    GPS2XYZ(latitude, longitude, altitude, xyz);

    Eigen::Vector3d gps_position(xyz[0],xyz[1],xyz[2]);
    for(int i = 0 ;i < mGlobalMap.frames.size(); i++)
    {
        if (mGlobalMap.frames[i]->time_stamp - time < 0.01 && mGlobalMap.frames[i]->time_stamp - time > -0.01)
        {
            mGlobalMap.frames[i]->gps_position = gps_position;
            mGlobalMap.frames[i]->gps_accu = 1;

            break;
        }
        
    }
}

void Sim3Opt::beginOpt()
{
    //1.计算sim3 2.优化
    ComputeSim3();
    pose_graph_opti_se3_hu();
}

void Sim3Opt::GPS2XYZ(double latitude, double longitude, double altitude, double* xyz)
{
    if(!initGPS)
    {
        geoConverter.Reset(latitude, longitude, altitude);
        initGPS = true;
    }
    geoConverter.Forward(latitude, longitude, altitude, xyz[0], xyz[1], xyz[2]);
    printf("la: %f lo: %f al: %f\n", latitude, longitude, altitude);
    printf("gps x: %f y: %f z: %f\n", xyz[0], xyz[1], xyz[2]);
}

//将gps世界坐标系下的姿态转为gps绝对坐标
void Sim3Opt::XYZ2GPS(double* xyz , double& latitude, double& longitude, double& altitude)
{
    //void Reverse(real x, real y, real z, real& lat, real& lon, real& h)
    geoConverter.Reverse(xyz[0],xyz[1],xyz[2],latitude,longitude,altitude);
}


void Sim3Opt::getGlobalGPS(double time,double& latitude, double& longitude, double& altitude)
{
    for(int i = 0 ;i < mGlobalMap.frames.size(); i++)
    {
        if (mGlobalMap.frames[i]->time_stamp - time < 0.01 && mGlobalMap.frames[i]->time_stamp - time > -0.01)
        {
            Eigen::Vector3d position = mGlobalMap.frames[i]->getPose().block(0,3,3,1);
            double xyz[3] = {0};
            xyz[0] = position[0];
            xyz[1] = position[1];
            xyz[2] = position[2];
            XYZ2GPS(xyz,latitude,longitude,altitude);
            break;
        }
        
    }

}

void Sim3Opt::ComputeSim3()
{
    std::vector<gm::GlobalMap> out_maps;
    int last_frame_id=0;
    
    for(int nn=10; nn<mGlobalMap.frames.size(); nn++)//从第10帧开始遍历地图中所有帧
    {
        //std::cout<<"mGlobalMap.frames.size() = "<<mGlobalMap.frames.size()<<std::endl;
        int cur_frame_id=nn;

        std::vector<Eigen::Vector3d> pc_frame;//视觉得到的平移twc
        std::vector<Eigen::Vector3d> pc_gps;//对应的gps给出的平移，这里只统计gps数据置信度小于30的tgwc
        
        for(int i=last_frame_id; i <= cur_frame_id; i++)//遍历当前帧之前的所有帧，统计gps数据置信度小于30的帧
        {
            std::shared_ptr<gm::Frame> frame = mGlobalMap.frames[i];
            if(frame->gps_accu<30)//gps_accu越小表示精确度越高，这里选取精确度较高的gps数据
            {
                //std::cout<<"frame->position = "<<frame->position<<std::endl;
                //std::cout<<"frame->gps_position = "<<frame->gps_position<<std::endl;
                pc_frame.push_back(frame->position);
                pc_gps.push_back(frame->gps_position);
            }
            else
            {
                //std::cout<<"cannot find frame name in pose list: "<<std::endl;
            }
        }

        if(pc_gps.size()>=10)
        {
            double scale_12;
            Eigen::Matrix4d T12;
            //计算pc_gps和pc_frame之间的sim3变换，即是计算gps和视觉参考系两个坐标系之间的sim3变换？
            chamo::ComputeSim3(pc_gps, pc_frame , T12, scale_12);//这里得到的T12已经含有尺度scale_12
            std::cout<<"sim3 = "<<T12<<std::endl;
            std::cout<<"scale_12 = "<<scale_12<<std::endl;
            Eigen::Matrix3d R_tran=T12.block(0,0,3,3);
            double scale=R_tran.block(0,0,3,1).norm();
            std::cout<<"scale = "<<scale<<std::endl;
            float avg_err=0;
            std::vector<Eigen::Vector3d> pc_frame_transformed_temp;
            //计算sim3变换的平均误差：通过sim3变换将视觉平移变换到gps下，再相减得到
            for(int i=0; i<pc_gps.size(); i++)
            {
                Eigen::Vector4d posi_homo;//视觉平移齐次坐标twc
                posi_homo.block(0,0,3,1)=pc_frame[i];
                posi_homo(3)=1;
                Eigen::Vector4d posi_gps_homo = T12*posi_homo;
                float err = (pc_gps[i]-posi_gps_homo.block(0,0,3,1)).norm();
                avg_err=avg_err+err/pc_gps.size();
                pc_frame_transformed_temp.push_back(posi_gps_homo.block(0,0,3,1));
            }
            std::cout<<"avg_err: "<<cur_frame_id<<" | "<<pc_gps.size()<<" | "<<avg_err<<std::endl;
            //if(avg_err>FLAGS_err_thres || cur_frame_id==map.frames.size()-1){
            if(cur_frame_id==mGlobalMap.frames.size()-1)//如果到了最后一帧
            {
                //创建从last_frame_id到cur_frame_id的子图
                //gm::GlobalMap submap;
                //mGlobalMap.CreateSubMap(last_frame_id, cur_frame_id, submap);

                for(int j=0; j<mGlobalMap.frames.size(); j++)//遍历子图的每一帧
                {
                    //使用sim3对每一帧位姿进行变换
                    Eigen::Matrix4d pose_transformed_temp;
                    Eigen::Matrix4d temp_pose=mGlobalMap.frames[j]->getPose();
                    transformPoseUseSim3(T12, temp_pose, pose_transformed_temp);
                    mGlobalMap.frames[j]->setPose(pose_transformed_temp);//Tgwc
                    std::cout<<"sim3转换后的pose"<<std::endl;
                    std::cout<<"map.frames[j]->direction = "<<mGlobalMap.frames[j]->direction.w()<<","<<mGlobalMap.frames[j]->direction.x()<<","<<mGlobalMap.frames[j]->direction.y()<<","<<mGlobalMap.frames[j]->direction.z()<<std::endl;
                    
                }
                int huhu = 0;
                //std::cin>>huhu;
                for(int j=0; j<mGlobalMap.mappoints.size(); j++)//遍历子图的每一个地图点
                {
                    //使用sim3对地图点坐标进行变换
                    Eigen::Vector4d posi_homo;
                    posi_homo.block(0,0,3,1)=mGlobalMap.mappoints[j]->position;
                    posi_homo(3)=1;
                    Eigen::Vector4d posi_gps_homo = T12*posi_homo;
                    mGlobalMap.mappoints[j]->position=posi_gps_homo.block(0,0,3,1);                       
                }
                //out_maps.push_back(submap);
                last_frame_id=cur_frame_id;
            }
        }
    }
    
    
}



void Sim3Opt::pose_graph_opti_se3_hu()
{
    mGlobalMap.AssignKpToMp();
    mGlobalMap.CalConnections();
    mGlobalMap.CheckConsistence();
    //计算共视关系
    //std::cout<<"12"<<std::endl;
    mGlobalMap.CalConnections();
    //std::cout<<"cccc"<<std::endl;
    //检查共视关系是否正确
    mGlobalMap.CheckConnections();
    mGlobalMap.FilterTrack();

    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(true);
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(
        new g2o::BlockSolverX(
            new g2o::LinearSolverEigen<g2o::BlockSolverX::PoseMatrixType>()));

    //solver->setUserLambdaInit(1e-16);
    optimizer.setAlgorithm(solver);
    
    for (size_t i = 0; i < mGlobalMap.frames.size(); i++)
    {
        std::cout<<"map.frames[i]->gps_position ="<<mGlobalMap.frames[i]->gps_position<<std::endl;
        std::cout<<"map.frames[i]->position = "<<mGlobalMap.frames[i]->position<<std::endl;
        std::cout<<"map.frames[i]->direction = "<<mGlobalMap.frames[i]->direction.w()<<","<<mGlobalMap.frames[i]->direction.x()<<","<<mGlobalMap.frames[i]->direction.y()<<","<<mGlobalMap.frames[i]->direction.z()<<std::endl;
    }
    int hu = 0;
    //std::cin>>hu;
    std::vector<g2o::VertexSE3Expmap*> v_se3_list;//存放所有优化的顶点
    
    //存放frame和其姿态顶点的对应关系
    std::map<std::shared_ptr<gm::Frame>, g2o::VertexSE3Expmap*> frame_to_vertex;
    std::map<g2o::VertexSE3Expmap*, std::shared_ptr<gm::Frame>> vertex_to_frame;
//     std::vector<Eigen::Vector3d> debug_points;
//     for(int n=0; n<map.frames.size(); n++){
//         if(map.frames[n]->isborder==true){
//             debug_points.push_back(map.frames[n]->position);
//         }
//     }
//     show_mp_as_cloud(debug_points, "debug1");
//     ros::spin();
    bool any_fix_frame=false;
    for(int i=0; i<mGlobalMap.frames.size(); i++)//遍历所有帧
    {
        g2o::VertexSE3Expmap* vSE3 = new g2o::VertexSE3Expmap();//顶点，待优化的frame
        Eigen::Matrix<double,3,3> R(mGlobalMap.frames[i]->direction);
        Eigen::Matrix<double,3,1> t=mGlobalMap.frames[i]->position;
        vSE3->setEstimate(g2o::SE3Quat(R,t).inverse());//Tcgw
        vSE3->setFixed(mGlobalMap.frames[i]->isborder);
        if(mGlobalMap.frames[i]->isborder==true)
        {
            any_fix_frame=true;
        }
        
        vSE3->setId(i);
        vSE3->setMarginalized(false);
        
        optimizer.addVertex(vSE3);
        v_se3_list.push_back(vSE3);
        frame_to_vertex[mGlobalMap.frames[i]]=vSE3;
        vertex_to_frame[vSE3]=mGlobalMap.frames[i];
    }
    if(any_fix_frame==false)
    {
        if(v_se3_list.size()>0)
        {
            v_se3_list[0]->setFixed(true);
        }
    }
    //添加gps的边，视觉(已通过sim3转换至gps的世界坐标系下)和gps得到的平移之差
    std::vector<g2o::EdgePosiPreSE3*> gps_edges;//存储所有gps的边
    for(int i=0; i<mGlobalMap.frames.size(); i++)
    {
        if(mGlobalMap.frames[i]->isborder==true)
        {
            continue;
        }
        if(mGlobalMap.frames[i]->gps_accu<30 && mGlobalMap.frames[i]->gps_accu>0)
        {
            g2o::EdgePosiPreSE3* e = new g2o::EdgePosiPreSE3();//一元边
            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(frame_to_vertex[mGlobalMap.frames[i]]));//Tcgw
            e->setMeasurement(mGlobalMap.frames[i]->gps_position);//tgwc
            Eigen::Matrix<double, 3, 3> con_mat = Eigen::Matrix<double, 3, 3>::Identity()*0.6;
            con_mat(2,2)=0.0000001;
            e->information()=con_mat;

            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            e->setRobustKernel(rk);
            rk->setDelta(2.3);

            optimizer.addEdge(e);
            gps_edges.push_back(e);
            e->computeError();
            
        }
    }
    std::cout<<"add gps edge: "<<gps_edges.size()<<std::endl;
    
    std::vector<g2o::EdgeSE3*> se3_edge_list;
    for(int i=0; i<mGlobalMap.pose_graph_v1.size(); i++)
    {
        //std::cout<<Eigen::Matrix3d(map.pose_graph_e_rot[i])<<std::endl;
        //std::cout<<map.pose_graph_e_posi[i].transpose()<<std::endl;
        g2o::SE3Quat Sji(mGlobalMap.pose_graph_e_rot[i],mGlobalMap.pose_graph_e_posi[i]);
        g2o::EdgeSE3* e = new g2o::EdgeSE3();//二元边
        //std::cout<<"v1: "<<frame_to_vertex[map.pose_graph_v1[i]]->estimate().inverse().translation().transpose()<<std::endl;
        //std::cout<<"obs: "<<(frame_to_vertex[map.pose_graph_v2[i]]->estimate().inverse().rotation().toRotationMatrix()*Sji.translation()+frame_to_vertex[map.pose_graph_v2[i]]->estimate().inverse().translation()).transpose()<<std::endl;
        if(mGlobalMap.pose_graph_v1.size() != mGlobalMap.pose_graph_v2.size())
        {
            std::cout<<"connection frame null!!"<<std::endl;
            exit(0);
        }
        if(mGlobalMap.pose_graph_v1[i]==nullptr ||mGlobalMap.pose_graph_v2[i]==nullptr)
        {
            std::cout<<"connection frame null!!"<<std::endl;
            exit(0);
        }
        if(frame_to_vertex.find(mGlobalMap.pose_graph_v1[i])==frame_to_vertex.end()||frame_to_vertex.find(mGlobalMap.pose_graph_v2[i])==frame_to_vertex.end())
        {
            std::cout<<"non exist frame!!"<<std::endl;
            exit(0);
        }
        e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(frame_to_vertex[mGlobalMap.pose_graph_v1[i]]));
        e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(frame_to_vertex[mGlobalMap.pose_graph_v2[i]]));
        e->setMeasurement(Sji);
        //std::cout<<map.pose_graph_weight[i]<<std::endl;
        if(mGlobalMap.pose_graph_weight[i]<1)
        {
            mGlobalMap.pose_graph_weight[i]=100;
        }
        Eigen::Matrix<double,6,6> matLambda = Eigen::Matrix<double,6,6>::Identity()*mGlobalMap.pose_graph_weight[i]*2;
        //Eigen::Matrix<double,6,6> matLambda = Eigen::Matrix<double,6,6>::Identity();
        e->information() = matLambda;

        g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
        e->setRobustKernel(rk);
        rk->setDelta(2.3);

        e->computeError();

        optimizer.addEdge(e);

        se3_edge_list.push_back(e);
    }
    std::cout<<"add sim3 edge: "<<se3_edge_list.size()<<std::endl;
    
    float avg_error=0;
    for(int i=0; i<gps_edges.size(); i++)
    {
        gps_edges[i]->computeError();
        avg_error=avg_error+sqrt(gps_edges[i]->chi2())/gps_edges.size();
    }
    std::cout<<"gps edge err before: "<<avg_error<<std::endl;
    avg_error=0;
    for(int i=0; i<se3_edge_list.size(); i++)
    {
        se3_edge_list[i]->computeError();
        avg_error=avg_error+sqrt(se3_edge_list[i]->chi2())/se3_edge_list.size();
        if(avg_error<0)
        {
            return;
        }
    }
    std::cout<<"sim3 edge err before: "<<avg_error<<std::endl;
    
    optimizer.initializeOptimization();
    //optimizer.computeInitialGuess();
    optimizer.optimize(100);
    
    avg_error=0;
    for(int i=0; i<gps_edges.size(); i++)
    {
        gps_edges[i]->computeError();
        avg_error=avg_error+sqrt(gps_edges[i]->chi2())/gps_edges.size();
    }
    std::cout<<"gps edge err after: "<<avg_error<<std::endl;
    avg_error=0;
    for(int i=0; i<se3_edge_list.size(); i++)
    {
        se3_edge_list[i]->computeError();
        avg_error=avg_error+sqrt(se3_edge_list[i]->chi2())/se3_edge_list.size();
        if(avg_error<0)
        {
            return;
        }
        
    }
    std::cout<<"sim3 edge err after: "<<avg_error<<std::endl;
    for(int i=0; i<v_se3_list.size(); i++)
    {
        g2o::SE3Quat CorrectedSiw =  v_se3_list[i]->estimate();
        Eigen::Matrix3d eigR = CorrectedSiw.rotation().toRotationMatrix();
        Eigen::Vector3d eigt = CorrectedSiw.translation();
        Eigen::Matrix4d Tiw=Eigen::Matrix4d::Identity();
        Tiw.block(0,0,3,3)=eigR;
        Tiw.block(0,3,3,1)=eigt;
        vertex_to_frame[v_se3_list[i]]->setPose(Tiw.inverse());
    }
}


void Sim3Opt::setOrbMap(gm::GlobalMap& globalmap)//为融合优化器添加orbslam地图
{
    for (size_t i = 0; i < globalmap.frames.size(); i++)
    {
        std::cout<<"globalmap.frames[i]->direction = "<<globalmap.frames[i]->direction.w()<<","<<globalmap.frames[i]->direction.x()<<","<<globalmap.frames[i]->direction.y()<<","<<globalmap.frames[i]->direction.z()<<std::endl;
    }
    mGlobalMap = globalmap;
    for (size_t i = 0; i < mGlobalMap.frames.size(); i++)
    {
        std::cout<<"mGlobalMap.frames[i]->direction = "<<mGlobalMap.frames[i]->direction.w()<<","<<mGlobalMap.frames[i]->direction.x()<<","<<mGlobalMap.frames[i]->direction.y()<<","<<mGlobalMap.frames[i]->direction.z()<<std::endl;
    }
    int n=0;
    //std::cin>>n;
    std::cout<<"finish setOrbMap"<<std::endl;
    std::cout<<"finish setOrbMap"<<std::endl;
}


