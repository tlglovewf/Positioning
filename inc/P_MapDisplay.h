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

namespace Position
{
    //轨迹地图处理  仅为调试测试
    class PMapDisplay
    {
    public:
        //构造函数
        //@param eTjtype  1 multi  0 uniform
        //@param useThread display in thread
        PMapDisplay(const shared_ptr<Position::IFrameData> &pdata,
                    const shared_ptr<Position::IConfig> &pcfg,
                    int eTjtype = 1,
                    bool useThread = true);
        //构造函数,直接可视化轨迹
        PMapDisplay(const shared_ptr<Position::IConfig> &pcfg,
                    const shared_ptr<Position::IMap> &pmap);
        //运行 轨迹处理
        void run();

    protected:
        //输出
        void saveResult();

    protected:
        bool mbUseThread;
        std::shared_ptr<Position::IConfig> mpConfig;
        std::shared_ptr<Position::IFrameData> mpData;
    #ifdef USE_VIEW
        std::shared_ptr<Position::IViewer> mpViewer;
    #endif
        std::unique_ptr<Position::PoseEstimator> mpTrajProSelector;
        std::unique_ptr<Position::IGpsFusion> mpGpsFunsion;

        std::unique_ptr<std::thread> mptViewer;
    };
} // namespace Position
#endif