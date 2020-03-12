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

    //运行
    void run();

protected:
    //输出
    void saveResult();

protected:
    std::shared_ptr<Position::IConfig>          mpConfig;
    std::shared_ptr<Position::IData>            mpData;
    std::shared_ptr<Position::IDetector>        mpDetector;
    std::shared_ptr<Position::IMap>             mpMap;
    std::shared_ptr<Position::ITrajProcesser>   mpTrajProcesser;
    std::shared_ptr<Position::IViewer>          mpViewer;
    
    std::unique_ptr<Position::IChecker>         mpChecker;
    std::unique_ptr<Position::IPositioning>     mpMulPositioner;
    std::unique_ptr<Position::IPositioning>     mpSinglePositioner;

};

#endif