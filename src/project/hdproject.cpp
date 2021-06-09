#include "project/hdproject.h"
#include "P_IOHelper.h"

HdConfig::HdConfig(const std::string &path):
HdCamFx(0),
HdCamFy(0),
HdCamCx(0),
HdCamCy(0),
HdCamk1(0),
HdCamk2(0),
HdCamk3(0),
HdCamp1(0),
HdCamp2(0)
{
    PUSH_MAP(HdCamFx);
    PUSH_MAP(HdCamFy);
    PUSH_MAP(HdCamCx);
    PUSH_MAP(HdCamCy);
    PUSH_MAP(HdCamk1);
    PUSH_MAP(HdCamk2);
    PUSH_MAP(HdCamk3);
    PUSH_MAP(HdCamp1);
    PUSH_MAP(HdCamp2);
    load(path);
}

//其他信息加载
void HdConfig::loadmore() 
{
    READ_VALUE(HdCamFx);
    READ_VALUE(HdCamFy);
    READ_VALUE(HdCamCx);
    READ_VALUE(HdCamCy);
    READ_VALUE(HdCamk1);
    READ_VALUE(HdCamk2);
    READ_VALUE(HdCamk3);
    READ_VALUE(HdCamp1);
    READ_VALUE(HdCamp2);
}

    
HdData::HdData(const std::shared_ptr<Position::IConfig> &pcfg):PFrameData(pcfg)
{
    assert(dynamic_cast<HdConfig*>(pcfg.get()));
    float fx = GETCFGVALUE(pcfg,HdCamFx,float);
    float fy = GETCFGVALUE(pcfg,HdCamFx,float);
    float cx = GETCFGVALUE(pcfg,HdCamCx,float);
    float cy = GETCFGVALUE(pcfg,HdCamCy,float);

    Position::CameraParam mCamera;

    mCamera.K = (Mat_<MATTYPE>(3,3) << fx, 0 , cx,
                                       0 , fy, cy,
                                       0 , 0 ,  1);
    mCamera.rgb = 1;

    mCamera.fps = 20;

    float k1 = GETCFGVALUE(pcfg, HdCamk1,float);
    float k2 = GETCFGVALUE(pcfg, HdCamk2,float);
    float p1 = GETCFGVALUE(pcfg, HdCamp1,float);
    float p2 = GETCFGVALUE(pcfg, HdCamp2,float);
    float k3 = GETCFGVALUE(pcfg, HdCamk3,float);

    mCamera.D = cv::Mat(4,1,MATCVTYPE);

    mCamera.D.at<MATTYPE>(0) = k1;
    mCamera.D.at<MATTYPE>(1) = k2;
    mCamera.D.at<MATTYPE>(2) = p1;
    mCamera.D.at<MATTYPE>(3) = p2;

    if(fabs(k3) < 1e-6)
    {
        mCamera.D.resize(5);
        mCamera.D.at<MATTYPE>(4) = k3;
    }
    mpCfg->setCamera(mCamera);
}

//预处理数据
bool HdData::loadDatas()
{
    const std::string pstpath = GETCFGVALUE(mpCfg,PosPath,string);
    const int         stno    = max(0,GETCFGVALUE(mpCfg,StNo,int));
    const int         edno    = max(stno,GETCFGVALUE(mpCfg,EdNo,int));
    if(pstpath.empty())
        return false;
        
    PROMT_V("postt path ",pstpath.c_str());

    try
    {
        LOG_INFO("Begin load datas ...");
        ifstream pstfile, imufile;
        pstfile.open(pstpath);
        int index = 0;
        bool allimg = (stno >= edno);
        std::string pststr;

        //read headline
        while(index++ < 12)
        {
            getline(pstfile, pststr);
        }
        index = 0;
        //load img pst file
        while (!pstfile.eof() && (allimg || index++ < edno))
        {
            getline(pstfile, pststr);
            if(index < stno)
                continue;
            if(pststr.empty())
                continue;
            char filename[255] = {0};
            Position::FrameData *framedata = new Position::FrameData;
            Position::PoseData &pose = framedata->_pos;

            int sqnum;
            int type;
            double x,y,z;
            double r11,r12,r13;
            double r21,r22,r23;
            double r31,r32,r33;
            double gs,sp;
            sscanf(pststr.c_str(),"%d %s %lf %d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
            &sqnum,filename,&pose._t,&type,&pose.pos.lat,&pose.pos.lon,&x,&y,&z,
            &r11,&r12,&r13,
            &r21,&r22,&r23,
            &r31,&r32,&r33,
            &gs,&sp);

            // PROMTD_V("Load",filename);

            framedata->_name = string(filename) + ".jpg";

            mFrameDatas.emplace_back(framedata);
        }
        pstfile.close();
        LOG_INFO_F("Data loaded successfully:%d",mFrameDatas.size());
        return true;
    }
    catch (const std::exception &e)
    {
        LOG_ERROR(e.what());
    }
    return false;
}

#define BEGINFILEREGION(PATH,MD)  try                               \
                                  {                                 \
                                     if(open(PATH,std::ios::MD))    \
                                     {

#define ENDFILEREGION()                 mfile.close();              \
                                     }                              \
                                  }                                 \
                                  catch(const std::exception& e)    \
                                  {                                 \
                                      std::cerr << e.what() << '\n';\
                                  }




 //加载项目列表
void HdPosePrj::loadPrjList(const std::string &path)
{
    assert(!path.empty());
    BEGINFILEREGION(path,in)
    
    while(!mfile.eof())
    {
        std::string line;
        getline(mfile,line);
        char batchname[20] = {0};
        int  n;
        sscanf(line.c_str(),"%s %d",batchname,&n);
        if(n < 1)
            continue;
        int index = 0;
        
        Position::BatchItem *pv = new Position::BatchItem(batchname,n);
        //read batch files name
        for(int i = 0;i < n; ++i)
        {
            getline(mfile,line);
            Position::FrameData *fms = new Position::FrameData;
            fms->_name = line;
            pv->insert(fms);
        }
        mBatches.emplace_back(pv);
    }
    ENDFILEREGION()
}

//加载地图
void HdPosePrj::load(const std::string &path)
{
    //add more ...
}
//保存地图
void HdPosePrj::save(const std::string &path)
{
    // assert(!path.empty());
    // BEGINFILEREGION(path,out)
    // PROMTD_V("save hd prj result to : ",path.c_str());
    // //head    
    // mfile << std::setiosflags(std::ios::fixed) << std::setiosflags(std::ios::right)
    //       << std::setw(9)  << "ImageName"
    //       << std::setw(7)  << "valid"
    //       << std::setw(15) << "R11"
    //       << std::setw(15) << "R12"
    //       << std::setw(15) << "R13"
    //       << std::setw(15) << "R21"
    //       << std::setw(15) << "R22"
    //       << std::setw(15) << "R23"
    //       << std::setw(15) << "R31"
    //       << std::setw(15) << "R32"
    //       << std::setw(15) << "R33"
    //       << std::setw(15) << "T1"
    //       << std::setw(15) << "T2"
    //       << std::setw(15) << "T3"
    //       << std::endl;
    
    // Position::PrjBatchVIter it = mBatches.begin();
    // Position::PrjBatchVIter ed = mBatches.end();
    // for(;it != ed; ++it)
    // {
    //     //batch info
    //     mfile << (*it)->_btname.c_str() << " " << (*it)->_n << std::endl;
    //     Mat pose = Mat::eye(4,4,MATCVTYPE);
    //     for(int i = 0;i < (*it)->_n; ++i)
    //     {
    //         Mat T =  (*it)->getFramePose(i);
    //         int vaild = !T.empty();
    //         //pose
    //         mfile << std::setiosflags(std::ios::fixed) << std::setiosflags(std::ios::right)
    //               << std::setw(9)  << (*it)->_fmsdata[i].first->_name.c_str()
    //               << std::setw(7)  << vaild;
    //         if(vaild)
    //         {
    //             if(i==0)
    //                 pose = Mat::eye(4,4,MATCVTYPE);
    //             else
    //             {
    //                 if(!(*it)->_poses[i-1].empty())
    //                 {
    //                     Mat Rfuse = T.rowRange(0,3).colRange(0,3).t();
    //                     Mat tfuse = -Rfuse*T.rowRange(0,3).col(3);
    //                     Rfuse.copyTo(pose.rowRange(0,3).colRange(0,3));
    //                     tfuse.copyTo(pose.rowRange(0,3).col(3));
    //                 }       
    //                 else
    //                     pose = Mat::eye(4,4,MATCVTYPE);   
    //             }
    //         }
    //         else
    //             pose = Mat::zeros(4,4,MATCVTYPE);
                
            
    //         //尺度信息
    //         double scale = 1.0;
    //         mfile << std::setiosflags(std::ios::fixed) << std::setprecision(9) << std::setiosflags(std::ios::right)
    //               << std::setw(15) << pose.at<MATTYPE>(0,0)
    //               << std::setw(15) << pose.at<MATTYPE>(0,1)
    //               << std::setw(15) << pose.at<MATTYPE>(0,2)
    //               << std::setw(15) << pose.at<MATTYPE>(1,0)
    //               << std::setw(15) << pose.at<MATTYPE>(1,1)
    //               << std::setw(15) << pose.at<MATTYPE>(1,2)
    //               << std::setw(15) << pose.at<MATTYPE>(2,0)
    //               << std::setw(15) << pose.at<MATTYPE>(2,1)
    //               << std::setw(15) << pose.at<MATTYPE>(2,2)
    //               << std::setw(15) << pose.at<MATTYPE>(0,3) * scale
    //               << std::setw(15) << pose.at<MATTYPE>(1,3) * scale
    //               << std::setw(15) << pose.at<MATTYPE>(2,3) * scale
    //               << std::endl;
    //     }
    // }
    // PROMTD_S("save successfully!!!");
    // ENDFILEREGION()
}