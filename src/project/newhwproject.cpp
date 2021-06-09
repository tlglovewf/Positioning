#include "newhwproject.h"
#include "P_IOHelper.h"
#include "P_Checker.h"
#include "Thirdparty/rapidjson/document.h"
#include <regex>


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
    return true;
}

void NewHwProjectData::loadCameraParams(const std::string &path)
{
    if (mCameraDB.open(path))
    {
        Position::CameraParam mCamera = mCameraDB.getCameraParams();
        mpCfg->setCamera(mCamera);
        PROMT_S("Camera params >>>>>>>>>>>>>>>>>>>>>>>>>");
        PROMT_V("K", mCamera.K);
        PROMT_V("D", mCamera.D);
        // PROMT_V("Fps ", (int)mCamera.fps);
        // PROMT_V("Rgb ", (int)mCamera.rgb);
        PROMT_S("<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");
    }
    else
    {
        LOG_ERROR("CameraParams Read Error!!!");
    }
}

#define PI 3.14159265358979324

double transformLat(double x, double y)
{
    auto ret = -100.0 + 2.0 * x + 3.0 * y + 0.2 * y * y + 0.1 * x * y + 0.2 * sqrt(fabs(x));
    ret += (20.0 * sin(6.0 * x * PI) + 20.0 * sin(2.0 * x * PI)) * 2.0 / 3.0;
    ret += (20.0 * sin(y * PI) + 40.0 * sin(y / 3.0 * PI)) * 2.0 / 3.0;
    ret += (160.0 * sin(y / 12.0 * PI) + 320 * sin(y * PI / 30.0)) * 2.0 / 3.0;
    return ret;
}

double transformLon(double x, double y)
{
    auto ret = 300.0 + x + 2.0 * y + 0.1 * x * x + 0.1 * x * y + 0.1 * sqrt(fabs(x));
    ret += (20.0 * sin(6.0 * x * PI) + 20.0 * sin(2.0 * x * PI)) * 2.0 / 3.0;
    ret += (20.0 * sin(x * PI) + 40.0 * sin(x / 3.0 * PI)) * 2.0 / 3.0;
    ret += (150.0 * sin(x / 12.0 * PI) + 300.0 * sin(x / 30.0 * PI)) * 2.0 / 3.0;
    return ret;
}

static Position::BLHCoordinate delta(double lat, double lon)
{
    auto a = 6378245.0;               //  a: 卫星椭球坐标投影到平面地图坐标系的投影因子
    auto ee = 0.00669342162296594323; //  ee: 椭球的偏心率。
    auto dLat = transformLat(lon - 105.0, lat - 35.0);
    auto dLon = transformLon(lon - 105.0, lat - 35.0);
    auto radLat = lat / 180.0 * PI;
    auto magic = sin(radLat);
    magic = 1 - ee * magic * magic;
    auto sqrtMagic = sqrt(magic);
    dLat = (dLat * 180.0) / ((a * (1 - ee)) / (magic * sqrtMagic) * PI);
    dLon = (dLon * 180.0) / (a / sqrtMagic * cos(radLat) * PI);
    Position::BLHCoordinate blh;
    blh.lat = dLat;
    blh.lon = dLon;
    blh.alt = 0;
    return blh;
}

//WGS-84 to GCJ-02
Position::BLHCoordinate transWgs84to02(double wgsLat, double wgsLon)
{
    auto d = delta(wgsLat, wgsLon);
    Position::BLHCoordinate blh;
    blh.lon = wgsLon + d.lon;
    blh.lat = wgsLat + d.lat;
    return blh;
}

//GCJ-02 to WGS-84
Position::BLHCoordinate trans02toWgs84(double gcjLat, double gcjLon)
{
    auto d = delta(gcjLat, gcjLon);
    Position::BLHCoordinate blh;
    blh.lon = gcjLon - d.lon;
    blh.lat = gcjLat - d.lat;
    blh.alt = 0;
    return blh;
}

//加载trackjson
bool NewHwProjectData::loadTrackJson(const std::string &path, Position::FrameDataPtrVector &framedatas)
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
        exit(-1);
    }
    rapidjson::Document doc;

    while (!json_file.eof())
    {
        string line;
        getline(json_file, line);
        if (line.empty())
            continue;
        if (doc.Parse(line.c_str()).HasParseError())
        {
            LOG_ERROR_F("%s parse error!", line.c_str());
            continue;
        }
        Position::FrameData *pFData = new Position::FrameData();

        if (doc.HasMember("altitude"))
        {
            pFData->_pos.pos.alt = doc["altitude"].GetFloat();
        }
        if (doc.HasMember("roll"))
        {
            pFData->_pos._roll = doc["roll"].GetFloat();
        }
        if (doc.HasMember("pitch"))
        {
            pFData->_pos._pitch = doc["pitch"].GetFloat();
        }
        if (doc.HasMember("direction"))
        {
            pFData->_pos._yaw = doc["direction"].GetFloat();
        }
        if (doc.HasMember("geometry"))
        {
            string str = doc["geometry"].GetString();
            static std::regex re(string("POINT(.* .*)"));
            bool ret = std::regex_match(str, re);
            if (ret)
            {
                sscanf(str.c_str(), "POINT(%lf %lf)", &pFData->_pos.pos.lon, &pFData->_pos.pos.lat);

                pFData->_pos.pos = trans02toWgs84(pFData->_pos.pos.lat, pFData->_pos.pos.lon);
            }
            else
            {
                PROMT_S("geometry format error！！！");
            }
        }
        if (doc.HasMember("imageName"))
        {
            pFData->_name = doc["imageName"].GetString();
        }
        mFrameDatas.emplace_back(pFData);
    }
    LOG_INFO_F("Load %d Frame Datas ...", mFrameDatas.size());
    json_file.close();
    return true;
}
