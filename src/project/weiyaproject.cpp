#include "project/weiyaproject.h"
#include "P_IOHelper.h"
#include "P_Utils.h"
WeiyaConfig::WeiyaConfig(const std::string &path):
                               ExtriPath(""),
                               BsPath("")
{
    PUSH_MAP(ExtriPath);
    PUSH_MAP(BsPath);
    load(path);
}

void WeiyaConfig::loadmore()
{
    READ_VALUE(ExtriPath);
    READ_VALUE(BsPath);
}


WeiyaData::WeiyaData(const std::shared_ptr<Position::IConfig> &pcfg):Position::PData(pcfg)
{
    assert(dynamic_cast<WeiyaConfig*>(pcfg.get()));
    const std::string expath = GETCFGVALUE(pcfg,ExtriPath,string);
    const std::string bspath = GETCFGVALUE(pcfg,BsPath,string);
	FileStorage intr(expath, FileStorage::READ);
	FileStorage bs  (bspath, FileStorage::READ);
	if (intr.isOpened() && bs.isOpened())
	{
		intr["P1"] >> mCamera.K;
		bs["RotateMatrixCam2IMU"] >> mCamera.RCam2Imu;
		bs["TranslationVectorCam2IMU"] >> mCamera.TCam2Imu;
		if (mCamera.K.cols > 3)
		{
			mCamera.K = mCamera.K.colRange(0, 3);
		}
        if(mCamera.K.type() != MATCVTYPE)
        {
            mCamera.K.convertTo(mCamera.K,MATCVTYPE);
            mCamera.RCam2Imu.convertTo(mCamera.RCam2Imu,MATCVTYPE);
            mCamera.TCam2Imu.convertTo(mCamera.TCam2Imu,MATCVTYPE);
        }
        mCamera.fps = GETCFGVALUE(mpCfg,ImgFps,int);
        mCamera.rgb = GETCFGVALUE(mpCfg,ImgRgb,int);
        PROMT_S("Camera params >>>>>>>>>>>>>>>>>>>>>>>>>");
        PROMT_V("K ",mCamera.K);
        PROMT_V("Fps ",mCamera.fps);
        PROMT_V("Rgb ",mCamera.rgb);
        PROMT_S("<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");
       
       
        //add
	}
	else
	{
        PROMT_S("Read config failed!!!")
	}
}
//根据图像名取时间(天秒)
double WeiyaData::getTimeFromName(const std::string &name) 
{
    if(name.empty())
    {
        return 0;
    }
    else
    {
        size_t n = name.find_last_of('/');
        if (n == string::npos)
        {
            return Position::PUtils::HMS2DaySec(name.substr(9, 12));
        }
        else
        {
            string date = name.substr(++n).substr(9, 12);
            return Position::PUtils::HMS2DaySec(date);
        }
    }
}

//处理数据
bool WeiyaData::loadDatas()
{
    const std::string pstpath = GETCFGVALUE(mpCfg,PosPath,string);
    if(pstpath.empty())
        return false;
        
    PROMT_V("postt path ",pstpath.c_str());
    try
    {
        PROMT_S("begin to load datas.");
        ifstream pstfile, imufile;
        pstfile.open(pstpath);
        int index = 0;
        std::string pststr;
        //read headline
        getline(pstfile, pststr);
        //load img pst file
        while (!pstfile.eof())
        {
            getline(pstfile, pststr);
            if(pststr.empty())
                continue;
            char filename[255] = {0};
            Position::FrameData *framedata = new Position::FrameData();
            Position::PoseData &pose = framedata->_pos;
            sscanf(pststr.c_str(),"%s %lf %lf %lf %lf %lf %lf %lf", filename,
                                                                    &pose._t,
                                                                    &pose.pos.lon,
                                                                    &pose.pos.lat,
                                                                    &pose.pos.alt,
                                                                    &pose._pitch,
                                                                    &pose._yaw,
                                                                    &pose._roll);
            framedata->_name = filename;
            mFrameDatas.emplace_back(framedata);
        }
        pstfile.close();
        LOG_INFO_F("%s-%d","Frame Datas Finished.",mFrameDatas.size());
        return true;
    }
    catch (const std::exception &e)
    {
        PROMT_S(e.what());
    }
    return false;
}