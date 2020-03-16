
#include "P_ORBLocalMapping.h"
#include "P_ORBmatcher.h"
#include "P_ORBOptimizer.h"
#include "P_ORBKeyFrame.h"
#include "P_ORBLoopClosing.h"
#include "P_ORBTracking.h"
#include "P_Writer.h"
#include <unistd.h>
namespace Position
{

#define  SEARCHFMNO   10

ORBLocalMapping::ORBLocalMapping(const std::shared_ptr<IMap> &pMap):
    mbResetRequested(false), mbFinishRequested(false), mbFinished(true), mpMap(pMap),
    mbAbortBA(false), mbStopped(false), mbStopRequested(false), mbNotStop(false), mbAcceptKeyFrames(true)
{
}

void ORBLocalMapping::SetLoopCloser(const std::shared_ptr<ORBLoopClosing>& pLoopCloser)
{
    mpLoopCloser = pLoopCloser;
}

void ORBLocalMapping::SetTracker(const std::shared_ptr<ORBTracking>& pTracker)
{
    mpTracker=pTracker;
}

void ORBLocalMapping::Run()
{

    mbFinished = false;

    while(1)
    {
        // Tracking will see that Local Mapping is busy
        SetAcceptKeyFrames(false);

        // Check if there are keyframes in the queue
        if(CheckNewKeyFrames())
        {
            // BoW conversion and insertion in Map
            //处理新帧,主要是计算bow 建立各种连接
            ProcessNewKeyFrame();

            // Check recent MapPoints
            //剔除部分地图点
            MapPointCulling();

            // Triangulate new MapPoints
            //查询共视帧,通过极线匹配,三角化新地图点
            CreateNewMapPoints();

            if(!CheckNewKeyFrames())
            {
                // Find more matches in neighbor keyframes and fuse point duplications
                //搜索共视帧,融合地图点
                SearchInNeighbors();
            }

            mbAbortBA = false;

            if(!CheckNewKeyFrames() && !stopRequested())
            {
                // Local BA
                //局部地图优化(与当前帧有共视关系的,简历优化图，进行优化)
                if(mpMap->keyFrameInMap() >2)
                    Optimizer::LocalBundleAdjustment(mpCurrentKeyFrame,&mbAbortBA, mpMap);

                // Check redundant local Keyframes
                //剔除 地图点重复率较高的关键帧(90%的点关联至少3个关键帧)
                KeyFrameCulling();
            }

            // mpLoopCloser->InsertKeyFrame(mpCurrentKeyFrame);  //close close loop first 
        }
        else if(Stop())
        {
            // Safe area to stop
            while(isStopped() && !CheckRequestFinish())
            {
                usleep(3000);
            }
            if(CheckRequestFinish())
                break;
        }

        ResetIfRequested();

        // Tracking will see that Local Mapping is busy
        SetAcceptKeyFrames(true);

        if(CheckRequestFinish())
            break;

        // usleep(3000);
    }

    SetFinish();
}

void ORBLocalMapping::InsertKeyFrame(ORBKeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexNewKFs);
    mlNewKeyFrames.push_back(pKF);
    mbAbortBA=true;
}


bool ORBLocalMapping::CheckNewKeyFrames()
{
    unique_lock<mutex> lock(mMutexNewKFs);
    return(!mlNewKeyFrames.empty());
}
//有新的关键生成时 先处理新帧
//遍历所有有效的地图点,检查与当前新帧的关联性,如果没有关联上 建立关联关系 并刷新mappoint的状态
void ORBLocalMapping::ProcessNewKeyFrame()
{
    {
        unique_lock<mutex> lock(mMutexNewKFs);
        mpCurrentKeyFrame = mlNewKeyFrames.front();
        mlNewKeyFrames.pop_front();
    }

    // Compute Bags of Words structures
    mpCurrentKeyFrame->ComputeBoW();

    // Associate MapPoints to the new keyframe and update normal and descriptor
    const MapPtVector& vpMapPointMatches = mpCurrentKeyFrame->getPoints();

    for(size_t i=0; i<vpMapPointMatches.size(); i++)
    {
        ORBMapPoint* pMP = ORBMAPPOINT(vpMapPointMatches[i]);
        if(pMP)
        {
            if(!pMP->isBad())
            {
                if(!pMP->isInFrame(mpCurrentKeyFrame))
                {
                    pMP->addObservation(mpCurrentKeyFrame, i);
                    pMP->UpdateNormalAndDepth();
                    pMP->ComputeDistinctiveDescriptors();
                }
                else // this can only happen for new stereo points inserted by the Tracking
                {//地图点已经与当前帧建立了关联
                    mlpRecentAddedMapPoints.push_back(pMP);
                }
            }
        }
    }    

    // Update links in the Covisibility Graph
    mpCurrentKeyFrame->UpdateConnections();

    // Insert Keyframe in Map
    mpMap->addKeyFrame(mpCurrentKeyFrame);
}

//地图点剔除
void ORBLocalMapping::MapPointCulling()
{
    // Check Recent Added MapPoints
    //遍历原来就和新帧关联上的地图点
    list<ORBMapPoint*>::iterator lit = mlpRecentAddedMapPoints.begin();
    const unsigned long int nCurrentKFid = mpCurrentKeyFrame->mnId;

    const int cnThObs = 2;

    while(lit!=mlpRecentAddedMapPoints.end())
    {
        ORBMapPoint* pMP = *lit;
        if(pMP->isBad())
        {
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if(pMP->GetFoundRatio()<0.25f )
        {//地图点所属/能被观测到的地图比例小于 1/4认为为坏点, 及地图点被太少的帧观测到了
            pMP->setBadFlag();
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if(((int)nCurrentKFid-(int)pMP->mnFirstKFid)>=2 && pMP->observations()<=cnThObs)
        {//当前帧与地图点第一次观测到的帧距离超过2帧,且当前点关联的帧少于2帧
            pMP->setBadFlag();
            lit = mlpRecentAddedMapPoints.erase(lit);
        }
        else if(((int)nCurrentKFid-(int)pMP->mnFirstKFid)>=3)
            lit = mlpRecentAddedMapPoints.erase(lit);
        else
            lit++;
    }
}

//创建新地图点
//遍历所有与当前帧公视的关键帧,通过bow检查出新的特征点
//计算该点的空间坐标,并检查其尺度、极线约束、重投误差后方可生成新地图点
void ORBLocalMapping::CreateNewMapPoints()
{
    ORBmatcher matcher(0.6,false);

    cv::Mat Rcw1 = mpCurrentKeyFrame->getRotation();
    cv::Mat Rwc1 = Rcw1.t();
    cv::Mat tcw1 = mpCurrentKeyFrame->getTranslation();
    cv::Mat Tcw1(3,4,MATCVTYPE);
    Rcw1.copyTo(Tcw1.colRange(0,3));
    tcw1.copyTo(Tcw1.col(3));
    cv::Mat Ow1 = mpCurrentKeyFrame->GetCameraCenter();

    const float &fx1 = mpCurrentKeyFrame->fx;
    const float &fy1 = mpCurrentKeyFrame->fy;
    const float &cx1 = mpCurrentKeyFrame->cx;
    const float &cy1 = mpCurrentKeyFrame->cy;
    const float &invfx1 = mpCurrentKeyFrame->invfx;
    const float &invfy1 = mpCurrentKeyFrame->invfy;

    const float ratioFactor = /*1.5f* */mpCurrentKeyFrame->mfScaleFactor;

    int nnew=0;

     // Retrieve neighbor keyframes in covisibility graph
    int nn = SEARCHFMNO;
    //获取当前帧 共视系数最好的前n帧
    const vector<ORBKeyFrame*> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);

    // Search matches with epipolar restriction and triangulate
    //遍历这些关联的公视帧, 通过bow信息 和对极约束  生成新的地图点
    for(size_t i=0; i<vpNeighKFs.size(); i++)
    {
        if(i>0 && CheckNewKeyFrames())
            return;

        ORBKeyFrame* pKF2 = vpNeighKFs[i];

        // Check first that baseline is not too short
        cv::Mat Ow2 = pKF2->GetCameraCenter();
        //获取与当前帧基线长
        cv::Mat vBaseline = Ow2-Ow1;
        //范化
        const float baseline = cv::norm(vBaseline);

        {
            //计算pKF2帧的中深度
            const float medianDepthKF2 = pKF2->ComputeSceneMedianDepth(2);
            
            const float ratioBaselineDepth = baseline/medianDepthKF2;
            // 基线 / 中深度  小于某个阈值 视为无效 进入下一帧
            // 如果地图深度太大(点都特别远) 剔除  
            if(ratioBaselineDepth< 0.01) 
                continue;
        }

        // Compute Fundamental Matrix
        cv::Mat F12 = ComputeF12(mpCurrentKeyFrame,pKF2);

        // Search matches that fullfil epipolar constraint
        vector<pair<size_t,size_t> > vMatchedIndices;
        //匹配curfm 和 pkf2 满足极线约束的匹配对
        matcher.SearchForTriangulation(mpCurrentKeyFrame,pKF2,F12,vMatchedIndices);

        if(vMatchedIndices.empty())
            continue;

        cv::Mat Rcw2 = pKF2->getRotation();
        cv::Mat Rwc2 = Rcw2.t();
        cv::Mat tcw2 = pKF2->getTranslation();
        cv::Mat Tcw2(3,4,MATCVTYPE);
        Rcw2.copyTo(Tcw2.colRange(0,3));
        tcw2.copyTo(Tcw2.col(3));

        const float &fx2 = pKF2->fx;
        const float &fy2 = pKF2->fy;
        const float &cx2 = pKF2->cx;
        const float &cy2 = pKF2->cy;
        const float &invfx2 = pKF2->invfx;
        const float &invfy2 = pKF2->invfy;

        // Triangulate each match
        const int nmatches = vMatchedIndices.size();
        for(int ikp=0; ikp<nmatches; ikp++)
        {
            const int &idx1 = vMatchedIndices[ikp].first;
            const int &idx2 = vMatchedIndices[ikp].second;

            const cv::KeyPoint &kp1 = mpCurrentKeyFrame->mvKeysUn[idx1];

            const cv::KeyPoint &kp2 = pKF2->mvKeysUn[idx2];

            // Check parallax between rays
            cv::Mat xn1 = (cv::Mat_<MATTYPE>(3,1) << (kp1.pt.x-cx1)*invfx1, (kp1.pt.y-cy1)*invfy1, 1.0);
            cv::Mat xn2 = (cv::Mat_<MATTYPE>(3,1) << (kp2.pt.x-cx2)*invfx2, (kp2.pt.y-cy2)*invfy2, 1.0);

            cv::Mat ray1 = Rwc1*xn1;
            cv::Mat ray2 = Rwc2*xn2;
            const float cosParallaxRays = ray1.dot(ray2)/(cv::norm(ray1)*cv::norm(ray2));

            cv::Mat x3D;
            if( cosParallaxRays>0 &&  (cosParallaxRays<0.9998))
            {
                // Linear Triangulation Method
                cv::Mat A(4,4,MATCVTYPE);
                A.row(0) = xn1.at<MATTYPE>(0)*Tcw1.row(2)-Tcw1.row(0);
                A.row(1) = xn1.at<MATTYPE>(1)*Tcw1.row(2)-Tcw1.row(1);
                A.row(2) = xn2.at<MATTYPE>(0)*Tcw2.row(2)-Tcw2.row(0);
                A.row(3) = xn2.at<MATTYPE>(1)*Tcw2.row(2)-Tcw2.row(1);

                cv::Mat w,u,vt;
                cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);

                x3D = vt.row(3).t();

                if(x3D.at<MATTYPE>(3)==0)
                    continue;

                // Euclidean coordinates
                x3D = x3D.rowRange(0,3)/x3D.at<MATTYPE>(3);

            }
            else
                continue; //No stereo and very low parallax

            cv::Mat x3Dt = x3D.t();

            //Check triangulation in front of cameras
            float z1 = Rcw1.row(2).dot(x3Dt)+tcw1.at<MATTYPE>(2);
            if(z1<=0)
                continue;

            float z2 = Rcw2.row(2).dot(x3Dt)+tcw2.at<MATTYPE>(2);
            if(z2<=0)
                continue;

            //Check reprojection error in first keyframe
            const float &sigmaSquare1 = mpCurrentKeyFrame->mvLevelSigma2[kp1.octave];
            const float x1 = Rcw1.row(0).dot(x3Dt)+tcw1.at<MATTYPE>(0);
            const float y1 = Rcw1.row(1).dot(x3Dt)+tcw1.at<MATTYPE>(1);
            const float invz1 = 1.0/z1;

            {
                float u1 = fx1*x1*invz1+cx1;
                float v1 = fy1*y1*invz1+cy1;
                float errX1 = u1 - kp1.pt.x;
                float errY1 = v1 - kp1.pt.y;
                if((errX1*errX1+errY1*errY1)>CHITH*sigmaSquare1)
                    continue;
            }

            //Check reprojection error in second keyframe
            const float sigmaSquare2 = pKF2->mvLevelSigma2[kp2.octave];
            const float x2 = Rcw2.row(0).dot(x3Dt)+tcw2.at<MATTYPE>(0);
            const float y2 = Rcw2.row(1).dot(x3Dt)+tcw2.at<MATTYPE>(1);
            const float invz2 = 1.0/z2;

            {
                float u2 = fx2*x2*invz2+cx2;
                float v2 = fy2*y2*invz2+cy2;
                float errX2 = u2 - kp2.pt.x;
                float errY2 = v2 - kp2.pt.y;
                if((errX2*errX2+errY2*errY2)>CHITH*sigmaSquare2)
                    continue;
            }

            //Check scale consistency
            cv::Mat normal1 = x3D-Ow1;
            float dist1 = cv::norm(normal1);

            cv::Mat normal2 = x3D-Ow2;
            float dist2 = cv::norm(normal2);

            if(dist1==0 || dist2==0)
                continue;

            //d2 / d1 的比例
            const float ratioDist = dist2/dist1;
            //特征点层级的比例
            const float ratioOctave = mpCurrentKeyFrame->mvScaleFactors[kp1.octave]/pKF2->mvScaleFactors[kp2.octave];

            // 实际空间比例系数要介于 特征点成绩比 /与* 缩放系数 之间
            if(ratioDist*ratioFactor<ratioOctave || ratioDist>ratioOctave*ratioFactor)
                continue;

            // Triangulation is succesfull
            ORBMapPoint* pMP = new ORBMapPoint(x3D,mpCurrentKeyFrame,mpMap);

            pMP->addObservation(mpCurrentKeyFrame,idx1);            
            pMP->addObservation(pKF2,idx2);

            mpCurrentKeyFrame->addMapPoint(pMP,idx1);
            pKF2->addMapPoint(pMP,idx2);

            pMP->ComputeDistinctiveDescriptors();

            pMP->UpdateNormalAndDepth();

            mpMap->addMapPoint(pMP);
            mlpRecentAddedMapPoints.push_back(pMP);

            nnew++;
        }
    }
    PROMTD_V(mpCurrentKeyFrame->getData()._name,"Create New Points", nnew);
}

//搜索邻近帧,取所有共视帧以及共视帧关联的最最佳的几个共视帧,简历邻近搜索列表
//先用当前帧地图点集合与所有目标帧进行融合, 在将所有领近帧的所有地图点与当前帧进行一次匹配融合
void ORBLocalMapping::SearchInNeighbors()
{
    // Retrieve neighbor keyframes
    int nn = SEARCHFMNO;
    const vector<ORBKeyFrame*> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);
    vector<ORBKeyFrame*> vpTargetKFs;
    for(vector<ORBKeyFrame*>::const_iterator vit=vpNeighKFs.begin(), vend=vpNeighKFs.end(); vit!=vend; vit++)
    {
        ORBKeyFrame* pKFi = *vit;
        if(pKFi->isBad() || pKFi->mnFuseTargetForKF == mpCurrentKeyFrame->mnId)
            continue;
        vpTargetKFs.push_back(pKFi);
        pKFi->mnFuseTargetForKF = mpCurrentKeyFrame->mnId;

        // Extend to some second neighbors
        const vector<ORBKeyFrame*> vpSecondNeighKFs = pKFi->GetBestCovisibilityKeyFrames(5);
        //遍历此帧共视的帧,寻找融合帧id不是当前帧的帧 加入到目标列表
        for(vector<ORBKeyFrame*>::const_iterator vit2=vpSecondNeighKFs.begin(), vend2=vpSecondNeighKFs.end(); vit2!=vend2; vit2++)
        {
            ORBKeyFrame* pKFi2 = *vit2;
            if(pKFi2->isBad() || pKFi2->mnFuseTargetForKF==mpCurrentKeyFrame->mnId || pKFi2->mnId==mpCurrentKeyFrame->mnId)
                continue;
            vpTargetKFs.push_back(pKFi2);
        }
    }


    // Search matches by projection from current KF in target KFs
    ORBmatcher matcher;
    const MapPtVector& vpMapPointMatches = mpCurrentKeyFrame->getPoints();
    //遍历所有关键帧,进行点的融合
    for(vector<ORBKeyFrame*>::iterator vit=vpTargetKFs.begin(), vend=vpTargetKFs.end(); vit!=vend; vit++)
    {
        ORBKeyFrame* pKFi = *vit;
        //用当前帧的地图点,与目标帧进行融合操作
        matcher.Fuse(pKFi,vpMapPointMatches);
    }

    // Search matches by projection from target KFs in current KF
    MapPtVector vpFuseCandidates;
    vpFuseCandidates.reserve(vpTargetKFs.size()*vpMapPointMatches.size());
    //再次遍历目标帧,获取每帧的地图点集
    //将所有点都加入到候选融合点列表中
    for(vector<ORBKeyFrame*>::iterator vitKF=vpTargetKFs.begin(), vendKF=vpTargetKFs.end(); vitKF!=vendKF; vitKF++)
    {
        ORBKeyFrame* pKFi = *vitKF;

        const MapPtVector& vpMapPointsKFi = pKFi->getPoints();

        for(MapPtVector::const_iterator vitMP=vpMapPointsKFi.begin(), vendMP=vpMapPointsKFi.end(); vitMP!=vendMP; vitMP++)
        {
            ORBMapPoint* pMP = ORBMAPPOINT(*vitMP);
            if(!pMP)
                continue;
            if(pMP->isBad() || pMP->mnFuseCandidateForKF == mpCurrentKeyFrame->mnId)
                continue;
            pMP->mnFuseCandidateForKF = mpCurrentKeyFrame->mnId;
            vpFuseCandidates.push_back(pMP);
        }
    }
    //将所有点与当前帧进行一次融合
    matcher.Fuse(mpCurrentKeyFrame,vpFuseCandidates);


    // Update points
     const MapPtVector& vpMapts = mpCurrentKeyFrame->getPoints();
    for(size_t i=0, iend=vpMapts.size(); i<iend; i++)
    {
        ORBMapPoint* pMP = ORBMAPPOINT(vpMapts[i]);
        if(pMP)
        {
            if(!pMP->isBad())
            {
                pMP->ComputeDistinctiveDescriptors();
                pMP->UpdateNormalAndDepth();
            }
        }
    }

    // Update connections in covisibility graph
    mpCurrentKeyFrame->UpdateConnections();
}

cv::Mat ORBLocalMapping::ComputeF12(IKeyFrame *pKF1, IKeyFrame *pKF2)
{
    cv::Mat R1w = pKF1->getRotation();
    cv::Mat t1w = pKF1->getTranslation();
    cv::Mat R2w = pKF2->getRotation();
    cv::Mat t2w = pKF2->getTranslation();

    cv::Mat R12 = R1w*R2w.t();
    cv::Mat t12 = -R1w*R2w.t()*t2w+t1w;

    cv::Mat t12x = SkewSymmetricMatrix(t12);

    const cv::Mat &K1 = ORBKEYFRAME(pKF1)->mK;
    const cv::Mat &K2 = ORBKEYFRAME(pKF2)->mK;


    return K1.t().inv()*t12x*R12*K2.inv();
}

void ORBLocalMapping::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopRequested = true;
    unique_lock<mutex> lock2(mMutexNewKFs);
    mbAbortBA = true;
}

bool ORBLocalMapping::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(mbStopRequested && !mbNotStop)
    {//请求暂停或者设置停止
        mbStopped = true;
        cout << "Local Mapping STOP" << endl;
        return true;
    }

    return false;
}

bool ORBLocalMapping::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool ORBLocalMapping::stopRequested()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopRequested;
}

void ORBLocalMapping::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);
    if(mbFinished)
        return;
    mbStopped = false;
    mbStopRequested = false;
    for(list<ORBKeyFrame*>::iterator lit = mlNewKeyFrames.begin(), lend=mlNewKeyFrames.end(); lit!=lend; lit++)
        delete *lit;
    mlNewKeyFrames.clear();

    cout << "Local Mapping RELEASE" << endl;
}

bool ORBLocalMapping::AcceptKeyFrames()
{
    unique_lock<mutex> lock(mMutexAccept);
    return mbAcceptKeyFrames;
}

void ORBLocalMapping::SetAcceptKeyFrames(bool flag)
{
    unique_lock<mutex> lock(mMutexAccept);
    mbAcceptKeyFrames=flag;
}

bool ORBLocalMapping::SetNotStop(bool flag)
{
    unique_lock<mutex> lock(mMutexStop);

    if(flag && mbStopped)
        return false;

    mbNotStop = flag;

    return true;
}

void ORBLocalMapping::InterruptBA()
{
    mbAbortBA = true;
}

//主要剔除,当前帧中90%的点有至少三帧其他帧观测到的帧
void ORBLocalMapping::KeyFrameCulling()
{
    // Check redundant keyframes (only local keyframes)
    // A keyframe is considered redundant if the 90% of the MapPoints it sees, are seen
    // in at least other 3 keyframes (in the same or finer scale)
    // We only consider close stereo points
    // 获取当前帧的所有公视关键帧
    vector<ORBKeyFrame*> vpLocalKeyFrames = mpCurrentKeyFrame->GetVectorCovisibleKeyFrames();

    for(vector<ORBKeyFrame*>::iterator vit=vpLocalKeyFrames.begin(), vend=vpLocalKeyFrames.end(); vit!=vend; vit++)
    {
        ORBKeyFrame* pKF = *vit;
        if(pKF->mnId==0)
            continue;
        const MapPtVector& vpMapPoints = pKF->getPoints();

        int nObs = 3;
        const int thObs=nObs;
        int nRedundantObservations=0;
        int nMPs=0;
        //遍历每个除世界第一帧以外,每一帧关联的所有地图点
        for(size_t i=0, iend=vpMapPoints.size(); i<iend; i++)
        {
            IMapPoint* pMP = vpMapPoints[i];
            if(pMP)
            {
                if(!pMP->isBad())
                {
                    nMPs++;
                    //地图点观测的帧数量大于阈值
                    if(pMP->observations()>thObs)
                    {
                        //取当前关联的的特征点的层级
                        const int &scaleLevel = pKF->mvKeysUn[i].octave;
                        const KeyFrameMap& observations = pMP->getObservations();
                        int nObs=0;
                        //遍历所有观测到该点的关键帧
                        for(KeyFrameMap::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
                        {
                            ORBKeyFrame* pKFi = ORBKEYFRAME(mit->first);
                            if(pKFi==pKF)
                                continue;
                            const int &scaleLeveli = pKFi->mvKeysUn[mit->second].octave;
                            
                            if(scaleLeveli<=scaleLevel+1)
                            {//关联该点其他特征点的层级应该小于或者等于这帧的层级+1
                                nObs++;
                                if(nObs>=thObs)
                                    break;
                            }
                        }
                        //如果当前点所被观测到的帧数高于阈值 则多余点计数+1
                        if(nObs>=thObs)
                        {
                            nRedundantObservations++;
                        }
                    }
                }
            }
        }  
        //当前帧关联的有效地图点 有90%的点至少有三个关联有效帧  则认为此帧为无效帧
        if(nRedundantObservations>0.9*nMPs)
        {
            PROMT_V("Set Bad From RedundataObs,No.",pKF->mnId);
            pKF->setBadFlag();
        }
            
    }
}

cv::Mat ORBLocalMapping::SkewSymmetricMatrix(const cv::Mat &v)
{
    return (cv::Mat_<MATTYPE>(3,3) <<             0, -v.at<MATTYPE>(2), v.at<MATTYPE>(1),
            v.at<MATTYPE>(2),               0,-v.at<MATTYPE>(0),
            -v.at<MATTYPE>(1),  v.at<MATTYPE>(0),              0);
}

void ORBLocalMapping::RequestReset()
{
    {
        unique_lock<mutex> lock(mMutexReset);
        mbResetRequested = true;
    }

    while(1)
    {
        {
            unique_lock<mutex> lock2(mMutexReset);
            if(!mbResetRequested)
                break;
        }
        usleep(3000);
    }
}

void ORBLocalMapping::ResetIfRequested()
{
    unique_lock<mutex> lock(mMutexReset);
    if(mbResetRequested)
    {
        mlNewKeyFrames.clear();
        mlpRecentAddedMapPoints.clear();
        mbResetRequested=false;
    }
}

void ORBLocalMapping::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool ORBLocalMapping::CheckRequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void ORBLocalMapping::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;    
    unique_lock<mutex> lock2(mMutexStop);
    mbStopped = true;
}

bool ORBLocalMapping::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

}
