/**
 *   P_Detector.h
 *   
 *   add by tu li gen   2020.2.7
 * 
 */
#ifndef __PDETECTOR_H_H_
#define __PDETECTOR_H_H_
#include "P_Interface.h"

namespace Position
{


    //ssd 检测
    class SSDDetector : public IDetector
    {
    public:

        //识别
        virtual TargetVector detect(const Mat &img);

    protected:   

    };
}

#endif