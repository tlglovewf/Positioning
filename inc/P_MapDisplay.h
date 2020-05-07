/**
 *   P_MapDisplay.h
 *   
 *   add by tu li gen   2020.2.7
 * 
 */
#ifndef __PMAPDISPLAY_H_H_
#define __PMAPDISPLAY_H_H_
#include "P_Interface.h"
#include "P_PoseEstimator.h"
#include <thread>

//轨迹地图处理  仅为调试测试
class PMapDisplay
{
public:
    //构造函数
    PMapDisplay(const shared_ptr<Position::IData> &pdata,
                       const shared_ptr<Position::IConfig> &pcfg);

    //运行
    void run();

protected:
    //输出
    void saveResult();

protected:
    std::shared_ptr<Position::IConfig>          mpConfig;
    std::shared_ptr<Position::IData>            mpData;
#ifdef USE_VIEW
    std::shared_ptr<Position::IViewer>          mpViewer;
#endif
    std::unique_ptr<Position::PoseEstimator>    mpTrajProSelector;
    std::unique_ptr<Position::IGpsFusion>       mpGpsFunsion;

    std::unique_ptr<std::thread>                mptViewer; 
};

#endif