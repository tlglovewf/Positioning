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
    //特征匹配对象 汉明距离
    class HanMingMatcher : public IFeatureMatcher
    {
    public:
        //構造函數  fratio閾值 越大匹配約束越小 checkori 是否檢查旋轉
        HanMingMatcher(float fratio = 0.8):mbCheckOrientation(true),mfNNratio(fratio){}
        //匹配  返回匹配对
        virtual MatchVector match(IFrame *preframe, IFrame *curframe, int windowsize); 
    protected:
        bool    mbCheckOrientation;
        float   mfNNratio;
    };

    //cv 匹配  knn近临匹配
    class KnnMatcher : public IFeatureMatcher
    {
    public:  
        KnnMatcher(float fratio = 0.5);

          //匹配  返回匹配对
        virtual MatchVector match(IFrame *preframe, IFrame *curframe, int windowsize); 
    protected:
        cv::Ptr<DescriptorMatcher> mMatcher;
        float   mfNNratio;
    };
    
    DECLAREIFACTORY(IFeatureMatcher, HanMingMatcher,HanMing)
    DECLAREIFACTORY(IFeatureMatcher, KnnMatcher    ,Knn)
}

#endif