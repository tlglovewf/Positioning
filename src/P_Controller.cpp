#include "P_Controller.h"
#include "P_Factory.h"
#include "P_Map.h"

//设置数据解析类型
PositionController::PositionController(const shared_ptr<Position::IDetector> &pdetecter, 
                                       const shared_ptr<Position::IData> &pdata,
                                       const shared_ptr<Position::IConfig> &pcfg):
                                        mpConfig(pcfg),mpData(pdata),mpDetector(pdetecter)
{
    assert(pdata);
    assert(pcfg);
    assert(pdetecter);

    mpMap = std::make_shared<Position::PMap>();
    if(GETCFGVALUE(mpConfig,ViewEnable,int))
    {
        mpViewer = std::unique_ptr<Position::IViewer>(Position::PFactory::CreateViewer(Position::eVPangolin,pcfg,mpMap));
    }
    std::shared_ptr<Position::ITracker>(Position::PFactory::CreateTracker(Position::eUniformSpeed,mpMap));
    mpChecker = std::unique_ptr<Position::IChecker>(Position::PFactory::CreateChecker(Position::eNormalChecker));
}

//初始化 
bool PositionController::init()
{
    //add more
    bool bol = mpData->loadDatas() && mpDetector->init();

    return bol ;   
}

//运行
void PositionController::run()
{
    if(init())
    {
       Position::FrameDataVIter it = mpData->begin();
       Position::FrameDataVIter ed = mpData->end();
       const std::string imgpath = GETCFGVALUE(mpConfig,ImgPath ,string) + "/";
       for(;it != ed ;++it)
       {
           const std::string picpath = imgpath + it->_name;
           it->_img = imread(picpath,IMREAD_UNCHANGED);
           if(it->_img.channels() > 1)
           {//通道数量>1 转为灰度图
               cvtColor(it->_img,it->_img,CV_RGB2GRAY);
           }
           it->_targets = mpDetector->detect(it->_img);
           if(it->_targets.empty())
           {
               continue;
           }
       }
    }
}

//处理位姿
void PositionController::handlePose()
{

}

//是否能创建新帧
bool PositionController::needCreateNewKeyFrame()
{
    return true;
}