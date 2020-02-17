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

        PFeatureMatcher():mbCheckOrientation(true),mfNNratio(0.8){}
        //匹配  返回匹配对
        virtual MatchVector match(IFrame *preframe, IFrame *curframe, int windowsize); 

        //匹配  返回匹配对特征序号对
        virtual MatchPairs  search(IFrame *preframe, IFrame *curframe, int windowsize);

    protected:
        bool    mbCheckOrientation;
        float   mfNNratio;
    };
}

#endif