/**
 *   P_Tracker.h
 *   
 *   add by tu li gen   2020.3.3
 * 
 */
#ifndef __PTRACKER_H_H_
#define __PTRACKER_H_H_
#include "P_Interface.h"

namespace Position
{
    //跟踪对象
    class PTracker : public ITracker
    {
    public:
        //构造
        PTracker( const std::shared_ptr<IMap> &pmap):mpMap(pmap)
        {

        }
        //跟踪
        virtual cv::Mat track(const FrameData &data)
        {
            assert(NULL);
        }

    protected:
        std::shared_ptr<IMap> mpMap;
    };

    //匀速跟踪
    class UniformSpeedTracker : public PTracker
    {
    public:
        //构造
        UniformSpeedTracker(const std::shared_ptr<IMap> &pmap):PTracker(pmap){}

         //跟踪
        virtual cv::Mat track(const FrameData &data);
    };
}

#endif