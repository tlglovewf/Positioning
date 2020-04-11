/**
 *   P_PangolinViewer.h
 *   
 *   add by tu li gen   2020.2.7
 * 
 */
#ifndef __PANGOLINVIWER_H_H_
#define __PANGOLINVIWER_H_H_
#include "P_Interface.h"
#include "P_FrameViewer.h"
#include <pangolin/pangolin.h>

namespace Position
{
    //pangolin可视化
    class Pangolin_Viewer : public IViewer
    {
    public:
        //构造函数
        Pangolin_Viewer(const std::shared_ptr<IConfig> &pCfg);

        //设置显示地图
        virtual void setMap(const std::shared_ptr<IMap> &pMap) 
        {
            mMap = pMap;
            mFrameViewer->setMap(mMap);
        }
         //初始化
        virtual void init();
        //绘制一次
        virtual bool renderOnce();
        //绘制循环
        virtual void renderLoop();
         //绘制状态
        virtual bool isRender()const 
        {
            return mbRender;
        }
    protected:
        //绘制帧
        void drawFrames();
        //绘制轨迹线
        void drawTraceLine();
        //绘制地图点
        void drawMapPoints();
        //绘制关联线
        void drawRelLines();
    protected:
        std::shared_ptr<IConfig>        mCfg;
        std::shared_ptr<IMap>           mMap;
        std::unique_ptr<PFrameViewer>   mFrameViewer;

        pangolin::View                 *mpView;
        pangolin::OpenGlRenderState     mCam;

        float                           mViewF;
        float                           mViewX;
        float                           mViewY;
        float                           mViewZ;

        bool                            mbInit;
        bool                            mbRender;
        int                             mWinW;
        int                             mWinH;
    };
}

#endif