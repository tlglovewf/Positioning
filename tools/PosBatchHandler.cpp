#include "PosBatchHandler.h"
#include "P_Writer.h"
#include "project/imgautoproject.h"
#include "P_Checker.h"
#include "P_Factory.h"
#include "P_Utils.h"
void SimpleBatchGenerator::generateBatches(const std::shared_ptr<IData> &pdata, const TrackerItemVector &trkItems,PrjBatchVector &batches)
{
    // const string batchLine = "target_";
    batches.reserve(trkItems.size());
    for(size_t i = 0; i < trkItems.size(); ++i)
    {
       const InfoIndex bg = trkItems[i].stno;
       const InfoIndex ed = trkItems[i].edno;
       
       BatchItem batch(std::to_string(trkItems[i].id),ed.first - bg.first + 1);
       batch._fmsdata.assign(pdata->begin() + bg.first,pdata->begin() + ed.first + 1);
       batches.emplace_back(batch);
    }
}


PosBatchHandler::PosBatchHandler( const std::shared_ptr<IConfig> &pcfg,const std::shared_ptr<IData> &pdata ):
mpConfig(pcfg),
mpData(pdata),
mPrjList(new ImgAutoPrjList(pdata)),
mpPos(Position::PFactory::CreatePositioning(Position::ePMultiImage,pdata->getCamera())),
mTrjSelector(pcfg,pdata)
{
     
}

//加载轨迹文件文件
bool PosBatchHandler::loadTrackerInfos(const std::string &path)
{
    if(!PATHCHECK(path))
        return false;
    assert(mPrjList);

    mPrjList->loadPrjList(path);

    // const Position::PrjBatchVector &batches = mPrjList->getPrjList();
    // cout << "batches size : " << batches.size() << endl;
    // for(auto item : batches)
    // {
    //     PROMT_V("batch",item._btname.c_str(),item._n);
    //     for(size_t i = 0; i < item._n; ++i)
    //     {
    //         cout << item._fmsdata[i]._name.c_str() << endl;
    //         cout << item._fmsdata[i]._targets.size() << endl;
    //     }
    // } 
    return !mPrjList->getPrjList().empty();
}

void PosBatchHandler::saveResult(const std::string &path)
{
    PROMT_V("Saving result to ",path.c_str());
    mPrjList->saveMap(path);
    PROMT_S("Over! Thank you!");

}
const int test_count = 1;
//估算batch 位姿
void PosBatchHandler::poseEstimate()
{
    TrackerItemVector &targets = mPrjList->trackInfos();
    PrjBatchVector    &batches = mPrjList->getPrjList();
    const std::string imgpath  = GETCFGVALUE(mpConfig,ImgPath,string);
    if(targets.empty())
        return;
    assert(targets.size()  == batches.size());
    PROMT_S("begin to position the targets");
    
    for(size_t i = 0; i < targets.size(); ++i)
    {
        PROMT_V("handle batch",targets[i].id);
//        if(i >= test_count)
//            break;
        assert(std::to_string(targets[i].id) == batches[i]._btname);
        //先赋值为最后一帧出现的位置
        targets[i].blh = batches[i]._fmsdata.rbegin()->_pos.pos;
        if(batches[i]._fmsdata.size() >= 2)
        {//仅在位姿估算成功后,进行量测定位 
            if(mTrjSelector.process(batches[i]._fmsdata,imgpath))
            {//位姿估算成功
                KeyFrameVector frames = mTrjSelector.getMap()->getAllFrames();
               
                assert(frames.size() == batches[i]._fmsdata.size());
                for(size_t m = 0; m < frames.size(); ++m)
                {
                    assert(batches[i]._fmsdata[m]._name == frames[m]->getData()._name);
                    batches[i]._fmsdata[m] = std::move(frames[m]->getData());
                    batches[i]._poses.emplace_back(frames[m]->getPose());
                }
            }
        }
        PROMT_V("batch handle over.",batches[i]._n);
        mTrjSelector.reset();
    }

    PROMT_S("end.");
}

//选帧 策略 待优化
static inline void selectFrame(const FrameDataVector &frames,const std::vector<Mat> &poses, int &idx1, int &idx2)
{
    if(frames.size() == 2)
    {
        idx1 = 0;
        idx2 = 1;
    }
    else
    {//取中间帧
        std::vector<Mat>::const_iterator iter = std::find_if(poses.begin(),poses.end(),[](const Mat &pse)->bool
        {
            return !pse.empty();
        });

        if(iter != poses.end())
        {
            int st = iter - poses.begin();
            int mid = (poses.end() - iter) / 2;
            idx1 = st + (mid - 1);
            
            idx2 = frames.size() - 1;
        }
        else
        {
            idx1 = 0;
            idx2 = 0;
        }
    }
}
//通过id获取目标
static inline const TargetData& getTarget(const FrameData &frame,int id)
{
    TargetVector::const_iterator iter = 
    std::find_if(frame._targets.begin(),frame._targets.end(),[&](const TargetData &t)->bool
    {
        return t._type == id;
    });
    if(iter != frame._targets.end())
        return *iter;
    else
    {
        static TargetData error;
        return error;
    }
}



const BLHCoordinate& checkBlh(const BLHCoordinate &frame, const BLHCoordinate &rst)
{
    Point3d gaus1 = PCoorTrans::BLH_to_GaussPrj(frame);
    Point3d gaus2 = PCoorTrans::BLH_to_GaussPrj(rst);
    double dlon = gaus2.x - gaus1.x;
    double dlat = gaus2.y - gaus2.y;
    double dt = fabs(dlon) + fabs(dlat);
    //暂定  横纵绝对值之和 > 10m为计算无效
    if( dt > 10.0 )
    {
        return frame;
    }
    else
    {
        return rst;
    }
}



//目标定位
void PosBatchHandler::targetPositioning()
{
    TrackerItemVector &targets = mPrjList->trackInfos();
    PrjBatchVector    &batches = mPrjList->getPrjList();
    assert(targets.size()  == batches.size());
    PROMT_S("Calc Target absolute position.");

    for(size_t i = 0; i < targets.size(); ++i)
    {
        TrackerItem &target = targets[i];
        PROMT_V("get target",target.id,"position.");
        if(target.maxsize >= 2)
        {//只有在关联帧数大于2 才进入量测赋值,否则保持原有当前帧的值
            int idx1, idx2;
            if(batches[i]._fmsdata.size() != batches[i]._poses.size())
                continue;
            selectFrame(batches[i]._fmsdata,batches[i]._poses,idx1,idx2);
            if(idx1 >= idx2)
            {//选帧 帧数不足 位姿推算失败 取当前位置
                continue;
            }

            FrameData &frame1 = batches[i]._fmsdata[idx1];
            FrameData &frame2 = batches[i]._fmsdata[idx2];
            Mat &pose1 = batches[i]._poses[idx1];
            Mat &pose2 = batches[i]._poses[idx2];
            const TargetData &targ1 = getTarget(frame1,target.id);
            const TargetData &targ2 = getTarget(frame2,target.id);

            if(!TargetData::isValid(targ1) || !TargetData::isValid(targ2))
            {
                PROMT_V("target get error : ",frame1._name.c_str()," ", frame2._name.c_str()," ", target.id);
            }

            //此处暂时先直接以跟踪到的中心点为同名点 判断条件
            Mat pose = pose2 * pose1.inv();
            Mat R = pose.rowRange(0,3).colRange(0,3);
            Mat t = pose.rowRange(0,3).col(3);
            Mat x3d = mpPos->position(R,t,targ1.center(),targ2.center());
            
            cout.precision(15);
            cout << frame1._pos.pos.lon << " " << frame1._pos.pos.lat << endl;
            cout << batches[i]._fmsdata[idx2 ]._pos.pos.lon << " " 
                 << batches[i]._fmsdata[idx2 ]._pos.pos.lat << endl;
            //此处暂用定位第二帧作为方向计算
            Mat trans = PUtils::CalcTransBLH(frame1._pos, frame2._pos);

            resize(x3d,x3d,Size(1,4));
            x3d.at<MATTYPE>(3) = 1.0;

            Mat wdpt = trans * x3d;
            //坐标转换
            BLHCoordinate rstblh  =  PCoorTrans::XYZ_to_BLH(XYZCoordinate(wdpt.at<MATTYPE>(0),
                                                                          wdpt.at<MATTYPE>(1),
                                                                          wdpt.at<MATTYPE>(2)));

            target.blh = checkBlh(frame1._pos.pos,rstblh);

            PROMT_V("target",target.id,target.blh.lon,target.blh.lat);
        }             
    }

}