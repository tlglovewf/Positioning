#include "PoseEst.h"


PrjBatchVector TargetBatchesGenerator::generate(const std::shared_ptr<IData> &pdata,
                                                const TrackerItemVector      &trackitems)
{
    PrjBatchVector batches;
    batches.reserve(trackitems.size());
    for(size_t i = 0; i < trackitems.size(); ++i)
    {
       const InfoIndex bg = trackitems[i].stno;
       const InfoIndex ed = trackitems[i].edno;
       
       BatchItem batch(std::to_string(trackitems[i].id),ed.first - bg.first + 1);

       std::transform(pdata->begin() + bg.first,pdata->begin() + ed.first + 1,back_inserter(batch._fmsdata),
       [](FrameData &data)->FrameData*
       {
           return &data;
       });

       batches.emplace_back(batch);
    }
    cout << "batches " << batches.size() << endl;
    return batches;
}


//批位姿估计器, 估算每个批的位姿,都是相对于初始化成功的第一帧率
bool BatchPoseEstimator::poseEstimate( PrjBatchVector &prjbatch)
{
    return true;
}


//批定位器
bool BatchVisualPositioner::position(TrackerItem &item,const PrjBatchVector &batches)
{
    return true;
}
