#include "P_Positioning.h"
#include "P_Utils.h"
#include "P_Writer.h"
#include "P_Checker.h"
#include "P_Factory.h"

namespace Position
{
#define EPILINESEARCHLEN    800  //极线搜索距离
#define ALLOWSCORE          0.90 //块匹配 评分阀值

     //获取极线
    EpLine Positioning::computeEpLine(const cv::Mat &R, const cv::Mat &t,const cv::Point2f &pt)
    {
        assert(!R.empty() && !t.empty());

        cv::Mat F = PUtils::ComputeFFromRT(R,t,mCamera.K);

        double a,b,c;
        // PUtils::CreateEpiline(F,pt,a,b,c);
        PUtils::CalcEpiline(F,pt,a,b,c);

        PROMT_V("ep line",a, b, c);
        return EpLine(a,b,c);
    }

    //极线匹配(基于目标包围盒)
    TargetData Positioning::eplineMatch(const EpLine &epline,const TargetData &item, const TargetVector &targets)
    {
        if(targets.empty())
        {
            return TargetData();
        }
        else
        {
            PROMT_S("begin target matching.");
            Point2f bg = item.center();
            Point2f ed(bg.x + EPILINESEARCHLEN,0);
            ed.y = PUtils::ComputeY(ed.x, epline.a, epline.b, epline.c);
            
            bg.x -= EPILINESEARCHLEN;
            bg.y = PUtils::ComputeY(bg.x, epline.a, epline.b, epline.c);
            
            
            for( auto target : targets)
            {
                if( !BLHCoordinate::isValid(target._pos) &&
                     (target._type == item._type) )
                {//检测到类型相同且为赋值 才进行下一步判断
                    if(PUtils::IsIntersect(bg, ed, target._box ))
                    {//相交则认为是同一个物体,并不再进行下一步判断
                        PROMT_S("Found a correct target.");
                        return target;
                    }
                }
            }
            PROMT_S("target not found.");
            return TargetData();
        }
    }
    //极线匹配(基于块)
    Point2f Positioning::eplineMatch(const EpLine &epline,const FrameData &preframe, const FrameData &curframe,const Point2f &pt) 
    {
        //左移 还是右移动  基于寻找同名点的x值 与 1/2 图像大小进行判断
        int  moveleft = (pt.x > (preframe._img.cols >>1) ) ? 1 : -1;
        Point2f bg = pt;
        Point2f ed(bg.x,0);
        ed.y = PUtils::ComputeY(ed.x, epline.a, epline.b, epline.c);
        
        float best_score    = -1.0;
        Point2f best_px_curr ;
        const int space = 1;
        const int searchlen = moveleft * EPILINESEARCHLEN  + bg.x;
        
        int st_x = bg.x;
        int ed_x = searchlen;
        
        if(moveleft < 0)
        {
            st_x = searchlen;
            ed_x = bg.x;
        }
        
        Ptr<IBlockMatcher> pBlock(PFactory::CreateBlockMatcher(eBMNCC,preframe._img, pt));
        Time_Interval timer;
        timer.start();
        for(int i = st_x; i < ed_x; i += space)
        {
            Point2f px_curr(i, PUtils::ComputeY(i, epline.a,epline.b,epline.c));
            
            double dscore =  pBlock->score(curframe._img, px_curr);
            
            if(dscore > best_score)
            {
                best_score = dscore;
                best_px_curr = px_curr;
            }
    // #if OUTPUTRESULT
    //         circle(img, px_curr, 3, CV_RGB(255,0,0));//绘制匹配线
    // #endif
        }
        
        PROMT_V("The best score ", best_score);

        timer.prompt("BlockMatching cost ");

        if(best_score > ALLOWSCORE)
        {

    // #if OUTPUTRESULT
    // 		char tmp[64] = { 0 };

    // 		sprintf(tmp, "%f", best_score);
            
    // 		putText(img, tmp, best_px_curr, cv::FONT_HERSHEY_SIMPLEX, 1, CV_RGB(255, 0, 0), 3);
    // #endif

            return best_px_curr;
        }
        else
        {
            return Point2f(WRONGDATA, 0);
        }
    }

    //反投
    cv::Point2f Positioning::backProject(const FrameData &frame,const BLHCoordinate &blh, cv::Mat &outimg)
    {
        //calc target xyz coordinate
	    Point3d  xyz = PCoorTrans::BLH_to_XYZ(blh);

        //calc cam xyz coordinate
	    Point3d  cam = PCoorTrans::BLH_to_XYZ(frame._pos.pos);

        Point3d dt = xyz - cam;

        //trans pt -> mat
        Mat tmp = (Mat_<double>(3,1) << dt.x ,dt.y,dt.z);

        //xyz -> enu
	    cv::Mat XYZ2Enu1 = PCoorTrans::XYZ_to_ENU(frame._pos.pos.lat, frame._pos.pos.lon);

        //计算imu到enu 转换矩阵
	    cv::Mat Rimu2Enu1 = PCoorTrans::IMU_to_ENU(-frame._pos._yaw, frame._pos._pitch, frame._pos._roll);

        //xyz->enu->imu
        Mat rst = Rimu2Enu1.t() * XYZ2Enu1 * tmp;

        //imu-> cam
        rst = rst - mCamera.TCam2Imu;
        rst =  mCamera.RCam2Imu.t() *  rst ;

        Mat t = mCamera.K * rst ;

        Point2f pt;
        pt.x = t.at<double>(0,0) / t.at<double>(2,0);
        pt.y = t.at<double>(1,0) / t.at<double>(2,0);

	    circle(outimg, pt, 4, CV_RGB(0, 255, 0), LINE_AA);

        return pt;
    }

    //计算速度
    static inline cv::Mat computeVelocity(const cv::Mat &prepose, const cv::Mat &curpose)
    {
       return curpose * prepose.inv();
    }

     //定位场景地图关键帧中目标
    void MultiImgPositioning::position(const std::shared_ptr<IMap> &pMap)
    {
        //ADD MORE...
    }


    Mat MultiImgPositioning::position(const Mat &R, const Mat &t, const Point2f &pt1, const Point2f &pt2)
    {
        if(R.empty() || t.empty())
            return Mat();
        cv::Mat P1(3,4,MATCVTYPE,cv::Scalar(0));
        mCamera.K.copyTo(P1.rowRange(0,3).colRange(0,3));

        // Camera 2 Projection Matrix K[R|t]
        cv::Mat P2(3,4,MATCVTYPE);
        R.copyTo(P2.rowRange(0,3).colRange(0,3));
        t.copyTo(P2.rowRange(0,3).col(3));
        P2 = mCamera.K * P2;
        Mat rst;
        PUtils::Triangulate(pt1,pt2,P1,P2,rst);
        return rst;
    }

    //定位
    void MultiImgPositioning::position(IKeyFrame *frame)
    {
        assert(frame);
        IKeyFrame *pnxt = frame->getNext();

        if(NULL == pnxt)
            return;
        while(pnxt->isBad())
        {//若为坏点,一直往下关键帧
            pnxt = pnxt->getNext();
            if(NULL == pnxt)
                return ;
        }  
        
        TargetVector &preTargets = frame->getTargets();

        const TargetVector &curTargets = pnxt->getTargets();
        if(preTargets.empty() || curTargets.empty())
            return;

        Mat prepose = frame->getPose();
        Mat curpose = pnxt->getPose();
        Mat dpose = computeVelocity(prepose,curpose);
        Mat R,t;
        PUtils::GetRTFromFramePose(dpose,R,t);

        // Camera 1 Projection Matrix K[I|0]
        cv::Mat P1(3,4,MATCVTYPE,cv::Scalar(0));
        mCamera.K.copyTo(P1.rowRange(0,3).colRange(0,3));

        cv::Mat O1 = cv::Mat::zeros(3,1,MATCVTYPE);

        // Camera 2 Projection Matrix K[R|t]
        cv::Mat P2(3,4,MATCVTYPE);
        R.copyTo(P2.rowRange(0,3).colRange(0,3));
        t.copyTo(P2.rowRange(0,3).col(3));
        P2 = mCamera.K * P2;
        // calculate world trans matrix
        Mat trans = PUtils::CalcTransBLH(IFRAME(frame)->getData()->_pos,
                                         IFRAME(pnxt)->getData()->_pos);
        //遍历目标
        for(TargetData &data : preTargets)
        {
            Point2f ct = data.center();
            EpLine epline = computeEpLine(R,t,ct);
            TargetData dt =  std::move(eplineMatch(epline,data,curTargets));
            Point2f dpt;
            if(TargetData::isValid(dt))
            {//匹配到了物体
                dpt = dt.center();
            }
            else
            {//基于物体包围盒匹配失败,开始极线搜索
                Point2f dpt = eplineMatch(epline,*IFRAME(frame)->getData(),*IFRAME(pnxt)->getData(),ct);
                if(dpt.x < 0)
                {
                    PROMT_V("not found. ",data._type);
                    return;
                }
            }
            Mat rst;
            //三角测量
            PUtils::Triangulate(ct,dpt,P1,P2,rst);
            //相对->绝对
            Mat wdpt = trans * rst;
            //坐标转换
            data._pos =  PCoorTrans::XYZ_to_BLH(XYZCoordinate(wdpt.at<MATTYPE>(0),
                                                              wdpt.at<MATTYPE>(1),
                                                              wdpt.at<MATTYPE>(2)));
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
        selectFrame(target,idx1,idx2);
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

        
        if(ResultCheckStrategy::Instance()->check(target,idx1,rstblh))
        {//检查通过 赋值目标位置
            target.blh = rstblh;
        }
        

        PROMT_V("target",target.id,target.blh.lon,target.blh.lat);
    }
    return true;
    }

    //重置
    void BatchVisualPositioner::reset() 
    {

    }


    //选择用于量测的帧
    void BatchVisualPositioner::selectFrame(const TrackerItem &item,int &idx1, int &idx2)
    {
        if(item.batch->_fmsdata.size() == 2)
        {//只有两帧的情况
            idx1 = 0;
            idx2 = 1;
        }
        else
        {//取中间帧
            std::vector<Mat>::const_iterator iter = std::find_if(item.batch->_poses.begin(),
                                                                 item.batch->_poses.end(),[](const Mat &pse)->bool
            {
                return !pse.empty();
            });

            if(iter != item.batch->_poses.end())
            {
                int st = iter - item.batch->_poses.begin();
                int mid = (item.batch->_poses.end() - iter) / 2;
                idx1 = st + (mid - 1);

                idx2 = item.batch->_fmsdata.size() - 1;
            }
            else
            {
                idx1 = 0;
                idx2 = 0;
            }
        }
    }
}