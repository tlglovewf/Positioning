#include "hd/hdproject.h"
#include "P_Writer.h"

HdConfig::HdConfig(const std::string &path):
HdCamFx(0),
HdCamFy(0),
HdCamCx(0),
HdCamCy(0)
{
    PUSH_MAP(HdCamFx);
    PUSH_MAP(HdCamFy);
    PUSH_MAP(HdCamCx);
    PUSH_MAP(HdCamCy);

    load(path);
}

//其他信息加载
void HdConfig::loadmore() 
{
    READ_VALUE(HdCamFx);
    READ_VALUE(HdCamFy);
    READ_VALUE(HdCamCx);
    READ_VALUE(HdCamCy);
}

    
HdData::HdData(const std::shared_ptr<Position::IConfig> &pcfg):PData(pcfg)
{
    float fx = GETCFGVALUE(pcfg,HdCamFx,float);
    float fy = GETCFGVALUE(pcfg,HdCamFx,float);
    float cx = GETCFGVALUE(pcfg,HdCamCx,float);
    float cy = GETCFGVALUE(pcfg,HdCamCy,float);

    mCamera.K = (Mat_<MATTYPE>(3,3) << fx, 0 , cx,
                                       0 , fy, cy,
                                       0 , 0 ,  1);
    mCamera.rgb = 1;

    mCamera.fps = 20;
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

            PROMT_V("Load",filename);
            framedata._name = filename;
            mFrameDatas.emplace_back(framedata);
        }
        pstfile.close();
        PROMT_S("data loaded successfully.");
        return true;
    }
    catch (const std::exception &e)
    {
        PROMT_S(e.what());
    }
    return false;
}