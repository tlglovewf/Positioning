/**
 *   P_TargetBatchesGenerator.h
 *   
 *   add by tu li gen   2020.2.7
 * 
 */
#ifndef _PTARGETBATCHESGENERATOR_H_H_
#define _PTARGETBATCHESGENERATOR_H_H_

#include "P_Interface.h"

namespace Position
{

    //一个目标对应一个batch  最简单的生成方式
    class TargetBatchesGenerator : public IBatchesGenerator
    {
    public:
    //生成批处理对象
        virtual PrjBatchVector generate(const std::shared_ptr<IFrameData> &pdata,
                                        TrackerItemVector      &trackitems);
    };
}

#endif