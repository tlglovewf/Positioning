#include "P_Positioner.h"
#include "P_Utils.h"
#include "P_Writer.h"
#include "P_Checker.h"
#include "P_Factory.h"

namespace Position
{
#define EPILINESEARCHLEN    800  //极线搜索距离
#define ALLOWSCORE          0.90 //块匹配 评分阀值



    //极线匹配(基于目标包围盒)
    int TargetPositioner::eplineMatch(const EpLine &epline,const TargetData &item, const TargetVector &targets)
    {
        if(targets.empty())
        {
            return -1;
        }
        else
        {
            PROMT_S("begin target matching.");
            Point2f bg = item.center();
            Point2f ed(bg.x + EPILINESEARCHLEN,0);
            ed.y = PUtils::ComputeY(ed.x, epline.a, epline.b, epline.c);
            
            bg.x -= EPILINESEARCHLEN;
            bg.y = PUtils::ComputeY(bg.x, epline.a, epline.b, epline.c);
            
            int index=0;
            for( auto target : targets)
            {
                if(target._type == item._type)
                {//检测到类型相同且为赋值 才进行下一步判断
                    if(PUtils::IsIntersect(bg, ed, target._box ))
                    {//相交则认为是同一个物体,并不再进行下一步判断
                        PROMT_S("Found a correct target.");
                        return index;
                    }
                }
                ++index;
            }
            PROMT_S("target not found.");
            return -1;
        }
    }
    //极线匹配(基于块)
    Point2f TargetPositioner::eplineMatch(const EpLine &epline,const FrameData &preframe, const FrameData &curframe,const Point2f &pt) 
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

    bool TargetPositioner::position(KeyFrameVector &frame)
    {
        const size_t len = frame.size();
        if(len < 2)
        {
            //add single frame positioning.
        }
        else
        {
            for(size_t i = 1; i < len; ++i)
            {
                Mat prepose = frame[ i - 1 ]->getPose();
                Mat curpose = frame[ i ]->getPose();
                //计算帧间R，t
                Mat dpose   = PUtils::computeVelocity(prepose,curpose);           
                Mat R,t;
                PUtils::GetRTFromFramePose(dpose,R,t);
                // calculate world trans matrix
                Mat trans = PUtils::CalcTransBLH(IFRAME(frame[i-1])->getData()->_pos,
                                                 IFRAME(frame[i])->getData()->_pos);

                for(TargetData &target : frame[i-1]->getTargets())
                {
                    //目标已经有定位信息
                    if(TargetData::isValid(target))
                        continue;
                    Point2f ct = target.center();
                    EpLine epline =   PUtils::ComputeEpLine(R,t,mCamera,ct);
                    Point2f corpt;
                    int idx = eplineMatch(epline,target,frame[i]->getTargets());
                    if(idx < 0)
                    {//匹配失败 进入块匹配
                        corpt = eplineMatch(epline,*frame[i-1]->getData(),*frame[i]->getData(),target.center());
                        if(corpt.x < 0)
                            continue;
                    }
                    else
                    {//成功匹配到 取中心点
                        corpt = frame[i]->getTargets()[idx].center();
                    }
                    //计算相对坐标
                    cv::Mat cpos = PUtils::CorPtsMeasure(mCamera,R,t,ct,corpt);
                    //相对->绝对
                    Mat wdpt = trans * cpos;
                    //坐标转换
                    target._pos =  PCoorTrans::XYZ_to_BLH(XYZCoordinate(wdpt.at<MATTYPE>(0),
                                                                        wdpt.at<MATTYPE>(1),
                                                                        wdpt.at<MATTYPE>(2)));

                }   
            }
        }
        

        return true;
    }

    //通过id获取目标
    static inline const TargetData& GetTargetFromFrame(const FrameData &frame,int id)
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
            LOG_WARNING_F("Target %d not found in %s",id , frame._name.c_str());
            return error;
        }
    }


    //批定位器
    bool BatchVisualPositioner::position(TrackerItem &target)
    {
        LOG_INFO_F("Calc Target %d Pos.",target.id);
        if(target.maxsize >= 2)
        {//只有在关联帧数大于2 才进入量测赋值,否则保持原有当前帧的值
            assert(target.batch);
            int idx1, idx2;
            if(target.batch->_fmsdata.size() != target.batch->_poses.size())
            {
                LOG_WARNING_F("Target %d Batch Pose Not Enough.",target.id);
                LOG_INFO_F("Target %d Pos:%f,%f",target.id,target.blh.lon,target.blh.lat);
                return false;
            }
                
            selectFrame(target,idx1,idx2);
            if(idx1 >= idx2)
            {//选帧 帧数不足 位姿推算失败 取当前位置
                LOG_WARNING_F("Calc %d Position Failed!!",target.id);
                return false;
            }
            FrameData &frame1 = *target.batch->_fmsdata[idx1];
            FrameData &frame2 = *target.batch->_fmsdata[idx2];
            Mat &pose1 = target.batch->_poses[idx1];
            Mat &pose2 = target.batch->_poses[idx2];
            const TargetData &targ1 = GetTargetFromFrame(frame1,target.id);
            const TargetData &targ2 = GetTargetFromFrame(frame2,target.id);

            if(!TargetData::isValid(targ1) || !TargetData::isValid(targ2))
            {
                return false;
            }
            //此处暂时先直接以跟踪到的中心点为同名点 判断条件
            Mat pose = pose2 * pose1.inv();
            Mat R = pose.rowRange(0,3).colRange(0,3);
            Mat t = pose.rowRange(0,3).col(3);
            
            Mat img1 = frame1._img;
            Mat img2 = frame2._img;

#if 1
            EpLine line = PUtils::ComputeEpLine(R,t,mCamera,targ1.center());
            cvtColor(img2, img2, CV_GRAY2BGR);
            PUtils::DrawEpiLine(line.a,line.b,line.c,targ2.center(),img2);

            string outname = "/media/tu/Work/Datas/TracePath/" + std::to_string(target.id) + "_epline.jpg";
            imwrite(outname,img2);
#endif
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

            //暂用取出姿态的两帧gps计算绝对坐标的转换矩阵额
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
                LOG_INFO_F("Calc Target %d Pos Successfully. PosBe:%f,%f",target.id,target.blh.lon,target.blh.lat);
                
                target.blh = rstblh;

                LOG_INFO_F("Calc Target %d Pos Successfully. PosAf:%f,%f",target.id,target.blh.lon,target.blh.lat);
            }
            else
            {
                LOG_WARNING_F("Calc Target %d Pos Failed.Pos Not Changed.",target.id);
            }
               
        }
        else
        {
            LOG_WARNING_F("Frame Not Enough.Target %d Not Changed.",target.id);
        }
        return true;
    }

    //重置
    void BatchVisualPositioner::reset() 
    {

    }


    //选择用于量测的帧
    //此处处理 建立在1target -> 1batch
    void BatchVisualPositioner::selectFrame(const TrackerItem &item,int &idx1, int &idx2)
    {
        if(item.batch->_fmsdata.size() == 2)
        {//只有两帧的情况
            idx1 = 0;
            idx2 = 1;
        }
        else
        {//取中间有位姿的帧
            std::vector<Mat>::const_iterator iter = std::find_if(item.batch->_poses.begin(),
                                                                 item.batch->_poses.end(),[](const Mat &pse)->bool
            {
                return !pse.empty();
            });

            
            assert(item.batch->_fmsdata.size() == item.batch->_poses.size());

            if(iter != item.batch->_poses.end())
            {
                int st = iter - item.batch->_poses.begin();
                int mid = (item.batch->_poses.end() - iter) / 2;
                idx1 = st + (mid - 1);     //取中间帧

                idx2 = item.batch->_fmsdata.size() - 1;//取最后一帧

                PROMTD_V("Select Frame ",idx1, " ", idx2);
            }
            else
            {
                idx1 = 0;
                idx2 = 0;
            }
        }
    }
}