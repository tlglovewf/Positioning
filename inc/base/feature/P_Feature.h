/**
 *   P_Feature.h
 *   
 *   add by tu li gen   2020.2.13
 * 
 */
#ifndef __PFEATURE_H_H_
#define __PFEATURE_H_H_
#include "P_Interface.h"

namespace Position
{
    //特征提取
    class PFeature : public IFeature
    {
    public:
        //计算特征点
        virtual bool detect(const FrameData &frame,FeatureInfo &info)
        {
            assert(NULL);
            return false;
        }
    };
}

#endif