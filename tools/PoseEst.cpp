#include "PoseEst.h"
#include "P_Writer.h"
#include "P_Utils.h"
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
       std::transform(pdata->begin() + bg.first,pdata->begin() + ed.first + 1,back_inserter(batch->_fmsdata),
       [](FrameData &data)->FrameData*
       {
           return &data;
       });

       batches.emplace_back(batch);
    }
    cout << "batches " << batches.size() << endl;
    return batches;
}

//选帧 策略 待优化
static inline void selectFrame(const FrameDataPtrVector &frames,const std::vector<Mat> &poses, int &idx1, int &idx2)
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

//批定位器
bool BatchVisualPositioner::position(TrackerItem &target)
{
    PROMT_V("get target",target.id,"position.");
    if(target.maxsize >= 2)
    {//只有在关联帧数大于2 才进入量测赋值,否则保持原有当前帧的值
        assert(target.batch);
        int idx1, idx2;
        if(target.batch->_fmsdata.size() != target.batch->_poses.size())
            return false;
        // selectFrame(batches[i]->_fmsdata,batches[i]->_poses,idx1,idx2);
        if(idx1 >= idx2)
        {//选帧 帧数不足 位姿推算失败 取当前位置
            return false;
        }

        FrameData &frame1 = *target.batch->_fmsdata[idx1];
        FrameData &frame2 = *target.batch->_fmsdata[idx2];
        Mat &pose1 = target.batch->_poses[idx1];
        Mat &pose2 = target.batch->_poses[idx2];
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
     
        //三角测量
        cv::Mat P1(3,4,MATCVTYPE,cv::Scalar(0));
        mCamera.K.copyTo(P1.rowRange(0,3).colRange(0,3));

        // Camera 2 Projection Matrix K[R|t]
        cv::Mat P2(3,4,MATCVTYPE);
        R.copyTo(P2.rowRange(0,3).colRange(0,3));
        t.copyTo(P2.rowRange(0,3).col(3));
        P2 = mCamera.K * P2;
        Mat x3d;
        PUtils::Triangulate(targ1.center(),targ2.center(),P1,P2,x3d);


        cout.precision(15);
        cout << frame1._pos.pos.lon << " " << frame1._pos.pos.lat << endl;
        cout << target.batch->_fmsdata[idx2 ]->_pos.pos.lon << " " 
             << target.batch->_fmsdata[idx2 ]->_pos.pos.lat << endl;
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
    return true;
}

//重置
void BatchVisualPositioner::reset() 
{

}