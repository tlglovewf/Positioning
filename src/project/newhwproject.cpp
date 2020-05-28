#include "newhwproject.h"
#include "P_IOHelper.h"
#include "P_Checker.h"
#include "Thirdparty/rapidjson/document.h"
#include <regex>

namespace Position
{
    //预处理数据
    bool NewHwProjectData::loadDatas()
    {

        //load camera paramers
        //get path
        const std::string expath = GETCFGVALUE(mpCfg, PrjPath, string);

        if (expath.empty())
            return false;

        const std::string trkjson = expath + "/Trace/trace.json";
        const std::string campath = expath + "/camera.sdb";
        SETCFGVALUE(GETGLOBALCONFIG(), ImgPath, expath + "/Pic");
        loadCameraParams(campath);

        if (!loadTrackJson(trkjson, mFrameDatas))
        {
            LOG_ERROR_F("%s Read Failed!!!", trkjson.c_str());
            return false;
        }
    }

    void NewHwProjectData::loadCameraParams(const std::string &path)
    {
        if (mCameraDB.open(path))
        {
            mCamera = mCameraDB.getCameraParams();
            PROMT_S("Camera params >>>>>>>>>>>>>>>>>>>>>>>>>");
            PROMT_V("K", mCamera.K);
            PROMT_V("D",mCamera.D);
            // PROMT_V("Fps ", (int)mCamera.fps);
            // PROMT_V("Rgb ", (int)mCamera.rgb);
            PROMT_S("<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");
        }
        else
        {
            LOG_ERROR("CameraParams Read Error!!!");
        }
    }

    //加载trackjson
    bool NewHwProjectData::loadTrackJson(const std::string &path, FrameDataPtrVector &framedatas)
    {
        if (!PATHCHECK(path))
        {
            LOG_ERROR_F("%s not found.", path.c_str());
            return false;
        }

        LOG_INFO_F("%s Load ...", path.c_str());

        ifstream json_file;
        json_file.open(path.c_str());
        string line, json;
        if (!json_file.is_open())
        {
            LOG_CRIT("Track json can not be open.");
            return false;
        }
        rapidjson::Document doc;

        while(!json_file.eof())
        {
            string line;
            getline(json_file,line);
            if(line.empty())
                continue;
            if(doc.Parse(line.c_str()).HasParseError())
            {
                LOG_ERROR_F("%s parse error!",line.c_str());
                continue;
            }
            FrameData *pFData = new FrameData();

            if(doc.HasMember("altitude"))
            {
                pFData->_pos.pos.alt = doc["altitude"].GetFloat();
            } 
            if(doc.HasMember("roll"))
            {
                pFData->_pos._roll = doc["roll"].GetFloat();
            }
            if(doc.HasMember("pitch"))
            {
                pFData->_pos._pitch = doc["pitch"].GetFloat();
            }
            if(doc.HasMember("direction"))
            {
                pFData->_pos._yaw = doc["direction"].GetFloat();
            }
            if(doc.HasMember("geometry"))
            {
                string str = doc["geometry"].GetString();
                static std::regex re(string("POINT(.* .*)"));
                bool ret = std::regex_match(str,re);
                if(ret)
                {
                    sscanf(str.c_str(), "POINT(%lf %lf)",&pFData->_pos.pos.lon, &pFData->_pos.pos.lat);
                }
                else
                {
                    PROMT_S("geometry format error！！！");
                }
            }
            if(doc.HasMember("imageName"))
            {
                pFData->_name = doc["imageName"].GetString();
            }
            mFrameDatas.emplace_back(pFData);
        }
        json_file.close();
        return true;
    }
} // namespace Position