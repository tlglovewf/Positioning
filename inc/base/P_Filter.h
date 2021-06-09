/**
 *   P_Filter.h
 *   
 *   add by tu li gen   2020.6.4
 * 
 */
#ifndef __PFILTER_H_H_
#define __PFILTER_H_H_
#include "P_Interface.h"
namespace Position
{
    // 帧数组过滤
    class FrameDataFilter  : public IFilter<FrameDataPtrVector>
    {
    public:
        //过滤
        virtual void doFilter(FrameDataPtrVector &t);
    };
}

#endif