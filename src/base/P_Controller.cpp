#include "P_Controller.h"
#include "P_Config.h"
#include "P_Data.h"
#include "P_Detector.h"
#include "P_Writer.h"

//设置数据解析类型
PositionController::PositionController(const string &cfgpath, DataParseType datatype,ImgDetectType imgtype /*= eSSDType*/ )
{
    assert(!cfgpath.empty());

    PROMT_V("data parse type is ", datatype);
    switch(datatype)
    {
        case eWeiyaType:
            mpConfig = std::make_shared<Position::WeiyaConfig>(); 
            mpConfig->load(cfgpath);
            mpData   =  std::unique_ptr<Position::IData>(new Position::WeiyaData(mpConfig));
            break;
        default:
            assert(NULL);
            break;
    }

    PROMT_V("image detect type is ",imgtype);
    switch(imgtype)
    {
        case eSSDType:
            mpDetector = std::unique_ptr<Position::IDetector>(new Position::SSDDetector());
            break;
        default:
            assert(NULL);
            break;
    }
}

//初始化 
bool PositionController::init()
{
    //add more
    return mpData->loadDatas();
}