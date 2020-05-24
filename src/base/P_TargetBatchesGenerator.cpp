#include "P_TargetBatchesGenerator.h"
#include "P_IOHelper.h"
#include "P_Utils.h"

namespace Position
{
    PrjBatchVector TargetBatchesGenerator::generate(const std::shared_ptr<IData> &pdata,
                                                    TrackerItemVector      &trackitems)
    {
        PrjBatchVector batches;
        batches.reserve(trackitems.size());
        for(size_t i = 0; i < trackitems.size(); ++i)
        {
           const InfoIndex bg = trackitems[i].stno;
           const InfoIndex ed = trackitems[i].edno;

           std::shared_ptr<BatchItem> batch(new BatchItem(std::to_string(trackitems[i].id),ed.first - bg.first + 1));
            trackitems[i].batch = batch;
           batch->_fmsdata = FrameDataPtrVector(pdata->begin() + bg.first,pdata->begin() + ed.first + 1);
           //默认将trackitem的值 赋成识别结果出现的最后一帧
           trackitems[i].blh = (*(pdata->begin() + ed.first))->_pos.pos;
           batches.emplace_back(batch);
        }
        LOG_INFO_F("%s:%d","Batches size",batches.size());
        return batches;
    }
}