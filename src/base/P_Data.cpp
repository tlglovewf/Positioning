#include "P_Data.h"
#include "P_Checker.h"
#include "P_Writer.h"

namespace Position
{

    PData::PData(std::shared_ptr<IConfig> pcfg):mpCfg(pcfg)
    {
        mpChecker = std::unique_ptr<IChecker>(new PChecker());
    }
 
    WeiyaData::WeiyaData(std::shared_ptr<IConfig> pcfg):PData(pcfg)
    {
        const std::string expath = GETCFGVALUE((*pcfg)["ExtriPath"],string);
        const std::string bspath = GETCFGVALUE((*pcfg)["BsPath"],string);

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
            PROMT_S("Camera params >>>>>>>>>>>>>>>>>>>>>>>>>");
            PROMT_V("K ",mCamera.K);
            PROMT_S("<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");
		}
		else
		{
			cout << "Read config failed!!!" << endl;
		}
    }

    //处理数据
    bool WeiyaData::loadDatas()
    {
        const std::string imgpath = GETCFGVALUE((*mpCfg)["ImgPath"],string);
        const std::string pstpath = GETCFGVALUE((*mpCfg)["PosPath"],string);
        const int         stno    = max(0,GETCFGVALUE((*mpCfg)["StNo"],int));
        const int         edno    = max(stno,GETCFGVALUE((*mpCfg)["EdNo"],int));

        if(imgpath.empty() || pstpath.empty())
            return false;
            
        PROMT_V("image path ",imgpath.c_str());
        PROMT_V("postt path ",pstpath.c_str());

        try
        {
            PROMT_S("begin to load datas.");
            ifstream pstfile, imufile;
            pstfile.open(pstpath);

            int index = stno;
            bool allimg = (stno == edno);
            std::string pststr;
            //read headline
            getline(pstfile, pststr);

            //load img pst file
            while (!pstfile.eof() && (allimg || index++ <= edno))
            {
                getline(pstfile, pststr);
                if(pststr.empty())
                    continue;
                char filename[255] = {0};
                FrameData framedata;
                PoseData &pose = framedata._pos;
                sscanf(pststr.c_str(),"%s %lf %lf %lf %lf %lf %lf %lf", filename,
                                                                        &pose._t,
                                                                        &pose.pos.lon,
                                                                        &pose.pos.lat,
                                                                        &pose.pos.alt,
                                                                        &pose._pitch,
                                                                        &pose._yaw,
                                                                        &pose._roll);

                string picpath = imgpath + "/" + filename;
                //PROMT_S(picpath);
                framedata._img = imread(picpath);
                if(mpChecker->check(framedata))
                {
                    mFrameDatas.emplace_back(framedata);
                }
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
}