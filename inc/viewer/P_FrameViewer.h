/**
 *   P_FrameViewer.h
 *   
 *   add by tu li gen   2020.4.9
 * 
 */
#ifndef _IFrameVIEWER_H_H_
#define _IFrameVIEWER_H_H_

#include "P_Interface.h"

namespace Position
{   
    //图片显示
    class PFrameViewer : public IBase
    {
    public:
        PFrameViewer();

        //设置地图
        void setMap(const std::shared_ptr<IMap> &pmap)
        {
            mMap = pmap;
        }

        //绘制帧
        void drawFrame();

        //绘制帧文字
        void drawFrameText(Mat &img,eTrackStatus status, Mat &imText);

    protected:
        //更新
        void drawPts( IKeyFrame *pkf, Mat &im);

    protected:
        eTrackStatus            mStatus;
        size_t                  mTrackPts;
        std::shared_ptr<IMap>   mMap;
    };
}



#endif