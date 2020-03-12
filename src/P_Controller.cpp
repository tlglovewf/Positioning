#include "P_Controller.h"
#include "P_Factory.h"
#include "P_Map.h"


//间隔最大的帧数
const int maxThFrameCnt = 3;

//设置数据解析类型
PositionController::PositionController(const shared_ptr<Position::IDetector> &pdetecter, 
                                       const shared_ptr<Position::IData> &pdata,
                                       const shared_ptr<Position::IConfig> &pcfg):
                                        mpConfig(pcfg),mpData(pdata),mpDetector(pdetecter)
{
    assert(pdata);
    assert(pcfg);
    assert(pdetecter);

    mpTrajProcesser.reset(Position::PFactory::CreateTrajProcesser(Position::eUniformSpeed,pcfg,pdata));
    mpMap = mpTrajProcesser->getMap();
    assert(mpMap);
    if(GETCFGVALUE(mpConfig,ViewEnable,int))
    {
        mpViewer = std::unique_ptr<Position::IViewer>(Position::PFactory::CreateViewer(Position::eVPangolin,pcfg,mpMap));
    }
    
    mpChecker  = std::unique_ptr<Position::IChecker>(Position::PFactory::CreateChecker(Position::eNormalChecker));
}

//初始化 
bool PositionController::init()
{
    //add more
    bool bol = mpData->loadDatas() && mpDetector->init();
    const Position::CameraParam &cam = mpData->getCamera();
    mpMulPositioner     = std::unique_ptr<Position::IPositioning>(Position::PFactory::CreatePositioning(Position::ePMultiImage,cam)); 
    mpSinglePositioner  = std::unique_ptr<Position::IPositioning>(Position::PFactory::CreatePositioning(Position::ePSingleImage,cam));

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
       bool hasTarget =  false;
       
        while( it != ed)
        {//帧循环 构建局部场景
            int  oThs = 0;
            Position::FrameDataVector framedatas; 
            mpTrajProcesser->reset();//重置位姿推算器
            for(;it != ed ;++it)
            {//遍历帧
                if(!mpChecker->check(*it))
                    continue;
                const std::string picpath = imgpath + it->_name;
                it->_img = imread(picpath,IMREAD_UNCHANGED);
                it->_targets = mpDetector->detect(it->_img);
                framedatas.push_back(*it);
                if(it->_targets.empty())
                {
                    hasTarget = false;

                    if(oThs++ > maxThFrameCnt)
                    {//连续帧没有探测到物体 闭合定位场景
                        break;
                    }

                }
                else
                {   
                    oThs = 0;
                    hasTarget = true;
                }
            }

            if(framedatas.empty())
            {
                ;//不做处理
            }
            else if(framedatas.size() < 2)
            {
                //单张直接定位
                // mpSinglePositioner->positioin
            }
            else
            {
                //先处理轨迹
                if(mpTrajProcesser->process(framedatas))
                {
                    //轨迹处理完 进行目标定位
                    // mpMulPositioner->positioin();
                }
                else
                {
                    cout << "!!!!!!!!!!!!!!!!" << endl;
                    cout << "position failed." << endl;
                    cout << "!!!!!!!!!!!!!!!!" << endl;
                }
                
            }

            saveResult();
        }
    }
}


void PositionController::saveResult()
{

}

