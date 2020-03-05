/**
 *   Pangolin_Viewer.h
 *   
 *   add by tu li gen   2020.2.7
 * 
 */
#ifndef __PANGOLINVIWER_H_H_
#define __PANGOLINVIWER_H_H_
#include "P_Interface.h"
#include <pangolin/pangolin.h>

namespace Position
{
    class IFrameDrawer : public IBase
    {
    public:
        virtual void updateState(eTrackStatus eStatus) = 0;
        //绘制帧
        virtual Mat drawFrame( IFrame *frame)  = 0;
    };

    //绘制帧
    class CVFrameDrawer : public IFrameDrawer
    {
    public:
        //绘制帧
        virtual Mat drawFrame( IFrame *frame) ;

        //更新状态
        virtual void updateState(eTrackStatus eStatus)
        {
            mStatus = eStatus;
        }
    protected:
        //绘制文本信息
        void drawTextInfo(cv::Mat &img, cv::Mat &txtimg);
    protected:
        eTrackStatus mStatus;
    };
    //pangolin可视化
    class Pangolin_Viwer : public IViewer
    {
    public:
        //构造函数
        Pangolin_Viwer(const std::shared_ptr<IConfig> &pCfg ,const std::shared_ptr<IMap> &pMap);

        //初始化
        virtual void init();

        //绘制
        virtual bool render();
    
    protected:
        //绘制帧
        void drawFrames();
        //绘制地图点
        void drawMapPoints();
        //绘制关联线
        void drawRelLines();
    protected:
        std::shared_ptr<IConfig>        mCfg;
        std::shared_ptr<IMap>           mMap;

        std::unique_ptr<IFrameDrawer>   mFDrawer;

        bool                            mbInit;
        int                             mWinW;
        int                             mWinH;
    };
}

#endif