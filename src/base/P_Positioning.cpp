#include "P_Positioning.h"
#include "P_Utils.h"
#include "P_Writer.h"
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
        PUtils::CreateEpiline(F,pt,a,b,c);

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
        
        Ptr<IBlockMatcher> pBlock(PFactory::CreateBlockMatcher(eNCC,preframe._img, pt));
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

    static inline cv::Mat computeVelocity(const cv::Mat &prepose, const cv::Mat &curpose)
    {
       return curpose * prepose.inv();
    }

    //定位
    void Positioning::position(IKeyFrame *frame)
    {
        assert(frame);
        IKeyFrame *pnxt = frame->getNext();
        if(NULL == frame->getNext())
            return ;
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

        Mat trans = PUtils::CalcTransBLH(frame->getData()._pos,
                                         pnxt->getData()._pos);
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
                Point2f dpt = eplineMatch(epline,frame->getData(),pnxt->getData(),ct);
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

    //定位
    void Positioning::position(IKeyFrame *frame, const Point2f &pt)
    {
        assert(NULL);
        //add more 
    }
}