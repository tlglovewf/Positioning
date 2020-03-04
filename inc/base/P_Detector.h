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

        //初始化
        virtual bool init() 
        {
            //add more .
            return true;
        }
        
        //识别
        virtual TargetVector detect(const Mat &img);

    protected:   

    };
}

#endif