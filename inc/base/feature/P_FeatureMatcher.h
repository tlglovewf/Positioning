/**
 *   P_FeatureMatcher.h
 *   
 *   add by tu li gen   2020.2.13
 * 
 */
#ifndef __PFEATUREMATCHER_H_H_
#define __PFEATUREMATCHER_H_H_
#include "P_Interface.h"

namespace Position
{
    //特征匹配对象
    class PFeatureMatcher : public IFeatureMatcher
    {
    public:
        //構造函數  fratio閾值 越大匹配約束越小 checkori 是否檢查旋轉
        PFeatureMatcher(float fratio,bool bcheckori = true):mbCheckOrientation(true),mfNNratio(fratio){}
        //匹配  返回匹配对
        virtual MatchVector match(IFrame *preframe, IFrame *curframe, int windowsize); 
        
    protected:
        bool    mbCheckOrientation;
        float   mfNNratio;
    };

    //cv 匹配  汉明距离 knn近临匹配
    class PKnnMatcher : public IFeatureMatcher
    {
    public:  
        PKnnMatcher();

          //匹配  返回匹配对
        virtual MatchVector match(IFrame *preframe, IFrame *curframe, int windowsize); 

    protected:
        cv::Ptr<DescriptorMatcher> mMatcher;
    };
}

#endif