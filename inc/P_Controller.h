/**
 *   P_Controller.h
 *   
 *   add by tu li gen   2020.2.7
 * 
 */
#ifndef __PCONTROLLER_H_H_
#define __PCONTROLLER_H_H_
#include "P_Interface.h"

// position controller class
class PositionController
{
public:
    //构造函数
    PositionController(const shared_ptr<Position::IDetector> &pdetected,
                       const shared_ptr<Position::IData> &pdata,
                       const shared_ptr<Position::IConfig> &pcfg);

    //初始化
    bool init(); 

    //运行
    void run();

    //处理位姿
    void handlePose();

protected:
    std::shared_ptr<Position::IConfig>    mpConfig;
    std::shared_ptr<Position::IData>      mpData;
    std::shared_ptr<Position::IDetector>  mpDetector;
    std::shared_ptr<Position::IMap>       mpMap;
    std::unique_ptr<Position::ITracker>   mpTracker;

    std::unique_ptr<Position::IViewer>    mpViewer;
    std::unique_ptr<Position::IChecker>   mpChecker;

};

#endif