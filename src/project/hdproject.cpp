#include "project/hdproject.h"
#include "P_Writer.h"

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

    
HdData::HdData(const std::shared_ptr<Position::IConfig> &pcfg):PData(pcfg)
{
    assert(dynamic_cast<HdConfig*>(pcfg.get()));
    float fx = GETCFGVALUE(pcfg,HdCamFx,float);
    float fy = GETCFGVALUE(pcfg,HdCamFx,float);
    float cx = GETCFGVALUE(pcfg,HdCamCx,float);
    float cy = GETCFGVALUE(pcfg,HdCamCy,float);

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
        PROMT_S("begin to load datas.");
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
        while (!pstfile.eof() && (allimg || index++ <= edno))
        {
            getline(pstfile, pststr);
            if(index < stno)
                continue;
            if(pststr.empty())
                continue;
            char filename[255] = {0};
            Position::FrameData framedata;
            Position::PoseData &pose = framedata._pos;

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

            framedata._name = string(filename) + ".jpg";

            mFrameDatas.emplace_back(framedata);
        }
        pstfile.close();
        PROMT_V("data loaded successfully.",mFrameDatas.size());
        return true;
    }
    catch (const std::exception &e)
    {
        PROMT_S(e.what());
    }
    return false;
}