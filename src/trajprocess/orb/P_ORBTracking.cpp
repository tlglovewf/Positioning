#include "P_ORBTracking.h"
#include "P_ORBmatcher.h"
#include "P_Converter.h"
#include "P_ORBInitializer.h"
#include "P_ORBOptimizer.h"
#include "P_ORBPnPsolver.h"
#include "P_IOHelper.h"
#include "P_Utils.h"
#include <unistd.h>
#include "P_SemanticGraph.h"

using namespace std;

namespace Position
{

ORBTracking::ORBTracking(const std::shared_ptr<ORBVocabulary>& pVoc, 
                         const std::shared_ptr<IMap>& pMap, 
                         const std::shared_ptr<ORBKeyFrameDatabase>& pKFDB,
                         const std::shared_ptr<IConfig> &pcfg,
                         const CameraParam &camparam):
    mState(eTrackNoImage),mpORBVocabulary(pVoc),
    mpKeyFrameDB(pKFDB), mpInitializer(static_cast<Initializer*>(NULL)),mpMap(pMap), mnLastRelocFrameId(0)
{
    // Load camera parameters from settings file
    mK = camparam.K;

    mDistCoef = camparam.D;

    // Max/Min Frames to insert keyframes and to check relocalisation
    mMinFrames = 0;
    mMaxFrames = camparam.fps;

    mbRGB = 1;// camparam.rgb;

    // if(mbRGB)
    //     PROMTD_S("- color order: RGB (ignored if grayscale)")
    // else
    //     PROMTD_S("- color order: BGR (ignored if grayscale)")

    // Load ORB parameters

    int     nFeatures       = GETCFGVALUE(pcfg,FeatureCnt,int); 
    float   fScaleFactor    = GETCFGVALUE(pcfg,ScaleFactor,float);
    int     nLevels         = GETCFGVALUE(pcfg,PyramidLevel,int);
    mnSearchRadius          = GETCFGVALUE(pcfg,SearchRadius,int);
    initMode                = 1;//GETCFGVALUE(pcfg,InitializationMode,int);
    initStep                = 5;//GETCFGVALUE(pcfg,InitImgLength,int);
    int     fIniThFAST      = 20;
    int     fMinThFAST      = 7;

    mfForInitRatio =  0.9;// GETCFGVALUE(pcfg,MatchRatio,float);

    mpORBextractorLeft = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    mpIniORBextractor = new ORBextractor(2*nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);
}

void ORBTracking::SetLocalMapper(const std::shared_ptr<ORBLocalMapping>& pLocalMapper)
{
    mpLocalMapper=pLocalMapper;
}

void ORBTracking::SetLoopClosing(const std::shared_ptr<ORBLoopClosing>& pLoopClosing)
{
    mpLoopClosing=pLoopClosing;
}

cv::Mat ORBTracking::InitMode(const FrameDataPtrVector &framedatas, const int imgnum)
{
    cv::Mat initMat;
    if(mState != eTrackOk)
    {
        if(initMode == 0)
        {
            initMat = track(framedatas[imgnum]);    
        }
        else if(initMode == 1)//增加初始化策略1-2、1-3、1-4……，若失败继续2-3、2-4……，一直循环下去
        {  
            if(imgnum == framedatas.size()-1)//最后一帧还未初始化成功处理
            {     
                delete mpInitializer;
                mpInitializer = static_cast<Initializer*>(NULL);
                return initMat;  
            }
            initMat = track(framedatas[imgnum]);
            if(mpInitializer)
            {
                int searchLen = imgnum+1+initStep<framedatas.size()? imgnum+1+initStep: framedatas.size();
                for(size_t j = imgnum+1; j<searchLen; j++)
                {
                    initMat = track(framedatas[j]);
                    if(mState == eTrackOk)
                    {
                        break;
                    }
                }
                if(mState != eTrackOk && mpInitializer)
                {
                    delete mpInitializer;
                    mpInitializer = static_cast<Initializer*>(NULL);
                }
                    
            }
        }
        else//其他初始化
        {
            //to do...
        }    
    }
    if(mState == eTrackOk)
        initMat = track(framedatas[imgnum]);
    return initMat;
}

cv::Mat ORBTracking::track(FrameData *data)
{
    if(!mDistCoef.empty())
    {
        undistort(data->_img,mImGray,mK,mDistCoef);
    }
    else
    {
        mImGray = data->_img;
    }

    if(data->_img.channels()==3)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGB2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGR2GRAY);
    }
    else if(mImGray.channels()==4)
    {
        if(mbRGB)
            cvtColor(mImGray,mImGray,CV_RGBA2GRAY);
        else
            cvtColor(mImGray,mImGray,CV_BGRA2GRAY);
    }

    data->_img = mImGray;
    
    if(mState == eTrackNoReady || mState == eTrackNoImage)
        mCurrentFrame = ORBFrame(data,mpIniORBextractor,mpORBVocabulary.get(),mK,mDistCoef);
    else
        mCurrentFrame = ORBFrame(data,mpORBextractorLeft,mpORBVocabulary.get(),mK,mDistCoef);

    Track();

    return mCurrentFrame.mTcw.clone();
}

void ORBTracking::Track()
{
    while(!mpLocalMapper->AcceptKeyFrames())
    {
        usleep(500);
    }
    if(mState==eTrackNoImage)
    {
        mState = eTrackNoReady;
    }

    // Get Map Mutex -> Map cannot be changed
    unique_lock<mutex> lock(mpMap->mapUpdateMutex());

    if(mState==eTrackNoReady)
    {
        MonocularInitialization();

        if(mState != eTrackOk)
            return;
    }
    else
    {
        // System is initialized. Track ORBFrame.
        bool bOK;

        // Local Mapping is activated. This is the normal behaviour, unless
        // you explicitly activate the "only tracking" mode.
        if(mState == eTrackOk)
        {
            // Local Mapping might have changed some MapPoints tracked in last frame
            CheckReplacedInLastFrame();
            if(mVelocity.empty() || mCurrentFrame.mnId<mnLastRelocFrameId+2)
            {
                bOK = TrackReferenceKeyFrame();
            }
            else
            {
                bOK = TrackWithMotionModel();
                if(!bOK)
                    bOK = TrackReferenceKeyFrame();
            }
        }
        else
        {
            bOK = Relocalization();
        }

        mCurrentFrame.mpReferenceKF = mpReferenceKF;

        // If we have an initial estimation of the camera pose and matching. Track the local map.
        if(bOK)
            bOK = TrackLocalMap();

        if(bOK)
            mState = eTrackOk;
        else
            mState = eTrackLost;

        // If tracking were good, check if we insert a keyframe
        if(bOK)
        {
            // Update motion model
            if(!mLastFrame.mTcw.empty())
            {
                cv::Mat LastTwc = cv::Mat::eye(4,4,MATCVTYPE);
                mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0,3).colRange(0,3));
                mLastFrame.getCameraCenter().copyTo(LastTwc.rowRange(0,3).col(3));
                mVelocity = mCurrentFrame.mTcw*LastTwc;
            }
            else
                mVelocity = cv::Mat();

            // Clean VO matches
            for(int i=0; i<mCurrentFrame.N; i++)
            {
                IMapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(pMP)
                    if(pMP->observations()<1)
                    {
                        mCurrentFrame.mvbOutlier[i] = false;
                        mCurrentFrame.mvpMapPoints[i]=static_cast<ORBMapPoint*>(NULL);
                       
                        delete pMP;
                    }
            }

            // Check if we need to insert a new keyframe
            if(NeedNewKeyFrame())
                CreateNewKeyFrame();

            // We allow points with high innovation (considererd outliers by the Huber Function)
            // pass to the new keyframe, so that bundle adjustment will finally decide
            // if they are outliers or not. We don't want next frame to estimate its position
            // with those points so we discard them in the frame.
            for(int i=0; i<mCurrentFrame.N;i++)
            {
                if(mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
                    mCurrentFrame.mvpMapPoints[i]=static_cast<ORBMapPoint*>(NULL);
            }
        }

        // Reset if the camera get lost soon after initialization
        if(mState==eTrackLost)
        {
            if(mpMap->keyFrameInMap() <=5)
            {
                LOG_WARNING("Track Lost,Not enough frame can track again. reseting ...");
                Reset();
                return;
            }
        }

        if(!mCurrentFrame.mpReferenceKF)
            mCurrentFrame.mpReferenceKF = mpReferenceKF;

        mLastFrame = ORBFrame(mCurrentFrame);
    }

    // Store frame pose information to retrieve the complete camera trajectory afterwards.
    if(!mCurrentFrame.mTcw.empty())
    {
        cv::Mat Tcr = mCurrentFrame.mTcw*mCurrentFrame.mpReferenceKF->GetPoseInverse();
        mlRelativeFramePoses.push_back(Tcr);
        mlpReferences.push_back(mpReferenceKF);
        mlbLost.push_back(mState==eTrackLost);
    }
    else
    {
        // This can happen if tracking is lost
        mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
        mlpReferences.push_back(mlpReferences.back());
        mlbLost.push_back(mState==eTrackLost);
    }

}


void ORBTracking::MonocularInitialization()
{

    if(!mpInitializer)
    {
        // Set Reference ORBFrame
        if(mCurrentFrame.mvKeys.size()>100)
        {
            mInitialFrame = ORBFrame(mCurrentFrame);
            mLastFrame = ORBFrame(mCurrentFrame);
            mvbPrevMatched.resize(mCurrentFrame.mvKeysUn.size());
            for(size_t i=0; i<mCurrentFrame.mvKeysUn.size(); i++)
                mvbPrevMatched[i]=mCurrentFrame.mvKeysUn[i].pt;

            if(mpInitializer)
                delete mpInitializer;

            mpInitializer =  new Initializer(mCurrentFrame,2.0,300);

            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);

            return;
        }
    }
    else
    {
        // Try to initialize
        if((int)mCurrentFrame.mvKeys.size()<=100)
        {
            delete mpInitializer;
            mpInitializer = static_cast<Initializer*>(NULL);
            fill(mvIniMatches.begin(),mvIniMatches.end(),-1);
            return;
        }
        PROMTD_S("Try to initialize.")
        // Find correspondences
        ORBmatcher matcher(mfForInitRatio,true);
        int nmatches = matcher.SearchForInitialization(mInitialFrame,mCurrentFrame,mvbPrevMatched,mvIniMatches,mnSearchRadius);
        // Check if there are enough correspondences
        if(nmatches < 80)
        {
            PROMTD_V("Initalize Number Of Points",nmatches)
            PROMTD_S("not enough for initializing. retry.")
            delete mpInitializer;
            mpInitializer = static_cast<Initializer*>(NULL);
            return;
        }

        cv::Mat Rcw; // Current Camera Rotation
        cv::Mat tcw; // Current Camera Translation
        BolVector vbTriangulated; // Triangulated Correspondences (mvIniMatches)

        if(mpInitializer->Initialize(mCurrentFrame, mvIniMatches, Rcw, tcw, mvIniP3D, vbTriangulated))
        {
            for(size_t i=0, iend=mvIniMatches.size(); i<iend;i++)
            {
                if(mvIniMatches[i]>=0 && !vbTriangulated[i])
                {
                    mvIniMatches[i]=-1;
                    nmatches--;
                }
            }

            // Set ORBFrame Poses
            mInitialFrame.setPose(cv::Mat::eye(4,4,MATCVTYPE));
            cv::Mat Tcw = cv::Mat::eye(4,4,MATCVTYPE);
            Rcw.copyTo(Tcw.rowRange(0,3).colRange(0,3));
            tcw.copyTo(Tcw.rowRange(0,3).col(3));
            mCurrentFrame.setPose(Tcw);

            mVelocity = Tcw;

            CreateInitialMapMonocular();
        }
        else
        {

            LOG_INFO("Initialize Failed !!!");
#if 0 // test init 
            MatchVector matches;
            for(int i = 0;i < mvIniMatches.size();++i)
            {
                if(mvIniMatches[i] >= 0)
                {
                    cv::DMatch m;
                    m.queryIdx = i;
                    m.trainIdx = mvIniMatches[i];
                    m.distance = 0;
                    matches.push_back(m);
                }
            }
            PROMT_S("Begin save match image.");
            cv::Mat otimg = Position::PUtils::DrawFeatureMatch(mLastFrame.getData()->_img,
                                                mCurrentFrame.getData()->_img,
                                                mLastFrame.getKeys(),
                                                mCurrentFrame.getKeys(),
                                                matches);
            const string text = "match size :" + std::to_string(matches.size());
            putText(otimg, text , Point(50, 50), CV_FONT_HERSHEY_COMPLEX, 2, Scalar(0, 0, 255), 3, CV_AA);
            PROMTD_S("save init image. ");
            cv::imwrite("/media/tlg/work/tlgfiles/HDData/result/orbinit.jpg",otimg);
#endif
            
        }
    }
}

void ORBTracking::CreateInitialMapMonocular()
{
    // Create KeyFrames
    ORBKeyFrame* pKFini = new ORBKeyFrame(mInitialFrame,mpMap,mpKeyFrameDB.get());
    ORBKeyFrame* pKFcur = new ORBKeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB.get());

    pKFini->ComputeBoW();
    pKFcur->ComputeBoW();

    // Insert KFs in the map
    mpMap->addKeyFrame(pKFini);
    mpMap->addKeyFrame(pKFcur);

    // Create MapPoints and asscoiate to keyframes
    for(size_t i=0; i<mvIniMatches.size();i++)
    {
        if(mvIniMatches[i]<0)
            continue;

        //Create ORBMapPoint.
        cv::Mat worldPos(mvIniP3D[i]);

        ORBMapPoint* pMP = new ORBMapPoint(worldPos,pKFcur,mpMap);

        pKFini->addMapPoint(pMP,i);
        pKFcur->addMapPoint(pMP,mvIniMatches[i]);

        pMP->addObservation(pKFini,i);
        pMP->addObservation(pKFcur,mvIniMatches[i]);
        //筛选关联特征点最佳的描述子(与其他描述子的距离具有最小中距离)
        pMP->ComputeDistinctiveDescriptors();
        pMP->UpdateNormalAndDepth();

        //Fill Current ORBFrame structure
        mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;
        mCurrentFrame.mvbOutlier[mvIniMatches[i]] = false;

        //Add to Map
        mpMap->addMapPoint(pMP);
    }

    // Update Connections
    pKFini->UpdateConnections();
    pKFcur->UpdateConnections();

    PROMTD_V("New Map created points",mpMap->mapPointsInMap());

    Optimizer::GlobalBundleAdjustemnt(mpMap,20);

    // Set median depth to 1
    float medianDepth = pKFini->ComputeSceneMedianDepth(2);
    cv::Mat tmp = pKFcur->getPose().col(3);
   //add by tu  初始化第二帧距离第一帧位置
    float len = sqrt(tmp.at<MATTYPE>(0) * tmp.at<MATTYPE>(0) + 
                     tmp.at<MATTYPE>(1) * tmp.at<MATTYPE>(1) +
                     tmp.at<MATTYPE>(2) * tmp.at<MATTYPE>(2));
    float invMedianDepth = 1.0f / len ;

    //取点中位深度
    // float invMedianDepth = 1.0f/medianDepth;

    if(medianDepth < 0 || pKFcur->TrackedMapPoints(1) < 50)
    {
        LOG_WARNING_F("Tracked Map Points size:%s, resetting",pKFcur->TrackedMapPoints(1));
        Reset();
        return;
    }

    // Scale initial baseline
    cv::Mat Tc2w = pKFcur->getPose();
    Tc2w.col(3).rowRange(0,3) = Tc2w.col(3).rowRange(0,3)*invMedianDepth;
    pKFcur->setPose(Tc2w);

    // Scale points
    const MapPtVector& vpAllMapPoints = pKFini->getWorldPoints();
    for(size_t iMP=0; iMP<vpAllMapPoints.size(); iMP++)
    {
        if(vpAllMapPoints[iMP])
        {
            IMapPoint* pMP = vpAllMapPoints[iMP];
            pMP->setWorldPos(pMP->getWorldPos()*invMedianDepth);
        }
    }

    mpLocalMapper->InsertKeyFrame(pKFini);
    mpLocalMapper->InsertKeyFrame(pKFcur);

    mCurrentFrame.setPose(pKFcur->getPose());
    mnLastKeyFrameId=mCurrentFrame.mnId;
    mpLastKeyFrame = pKFcur;

    mvpLocalKeyFrames.push_back(pKFcur);
    mvpLocalKeyFrames.push_back(pKFini);
    // mvpLocalMapPoints=mpMap->getAllMapPts();
    mpReferenceKF = pKFcur;
    mCurrentFrame.mpReferenceKF = pKFcur;

    mLastFrame = ORBFrame(mCurrentFrame);

    mpMap->setReferenceMapPoints(mvpLocalMapPoints);

    // mpMap->mvpKeyFrameOrigins.push_back(pKFini); //for loop closing  

    mVelocity = mCurrentFrame.getPose();

    mState = eTrackOk;

    PROMTD_S("Monocular Initialize successfully.")
}

void ORBTracking::CheckReplacedInLastFrame()
{
    for(int i =0; i<mLastFrame.N; i++)
    {
        ORBMapPoint* pMP = ORBMAPPOINT(mLastFrame.mvpMapPoints[i]);

        if(pMP)
        {
            ORBMapPoint* pRep = pMP->GetReplaced();
            if(pRep)
            {
                mLastFrame.mvpMapPoints[i] = pRep;
            }
        }
    }
}

//通过关联帧 获取当前帧位姿
bool ORBTracking::TrackReferenceKeyFrame()
{
    // Compute Bag of Words vector
    mCurrentFrame.ComputeBoW();

    // We perform first an ORB matching with the reference keyframe
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.85,true);
    MapPtVector vpMapPointMatches;

    int nmatches = matcher.SearchByBoW(mpReferenceKF,mCurrentFrame,vpMapPointMatches);

    if(nmatches<15)
        return false;

    mCurrentFrame.mvpMapPoints = vpMapPointMatches;
    mCurrentFrame.setPose(mLastFrame.mTcw);

    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                ORBMapPoint* pMP = ORBMAPPOINT(mCurrentFrame.mvpMapPoints[i]);

                mCurrentFrame.mvpMapPoints[i]=static_cast<ORBMapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;

                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;

                nmatches--;
            }
            else if(mCurrentFrame.mvpMapPoints[i]->observations()>0)
                nmatchesMap++;
        }
    }

    return nmatchesMap>=10;
}

void ORBTracking::UpdateLastFrame()
{
    // Update pose according to reference keyframe
    ORBKeyFrame* pRef = mLastFrame.mpReferenceKF;
    cv::Mat Tlr = mlRelativeFramePoses.back();

    mLastFrame.setPose(Tlr*pRef->getPose());
}

bool ORBTracking::TrackWithMotionModel()
{
    ORBmatcher matcher(0.9,true);
    
    // Update last frame pose according to its reference keyframe
    // Create "visual odometry" points if in Localization Mode
    UpdateLastFrame();

    mCurrentFrame.setPose(mVelocity*mLastFrame.mTcw);

    fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<ORBMapPoint*>(NULL));

    // Project points seen in previous frame
    int th = 10;//50;
    
    int nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,th,true);

    // If few matches, uses a wider window search
    if(nmatches<15)
    {
        fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<ORBMapPoint*>(NULL));
        nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,2*th,true);
    }

    if(nmatches<20)
    {
        PROMTD_V("Track Motion Lost",nmatches);
        return false;
    }
        

    // Optimize frame pose with all matches
    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    int nmatchesMap = 0;
    for(int i =0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(mCurrentFrame.mvbOutlier[i])
            {
                ORBMapPoint* pMP = ORBMAPPOINT(mCurrentFrame.mvpMapPoints[i]);

                mCurrentFrame.mvpMapPoints[i]=static_cast<ORBMapPoint*>(NULL);
                mCurrentFrame.mvbOutlier[i]=false;
                pMP->mbTrackInView = false;
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                nmatches--;
            }
            else if(mCurrentFrame.mvpMapPoints[i]->observations()>0)
                nmatchesMap++;
        }
    }   
    // PROMTD_V("track with motion ",nmatchesMap);
    return nmatchesMap>=10;
}

bool ORBTracking::TrackLocalMap()
{
    // We have an estimation of the camera pose and some map points tracked in the frame.
    // We retrieve the local map and try to find matches to points in the local map.

    UpdateLocalMap();

    SearchLocalPoints();

    // Optimize Pose
    //上一步有新点加入了,要重新单帧优化
    Optimizer::PoseOptimization(&mCurrentFrame);
    mnMatchesInliers = 0;

    // Update MapPoints Statistics
    for(int i=0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            if(!mCurrentFrame.mvbOutlier[i])
            {   
                ORBMAPPOINT(mCurrentFrame.mvpMapPoints[i])->IncreaseFound();
                if(mCurrentFrame.mvpMapPoints[i]->observations()>0)
                    mnMatchesInliers++;
            }
        }
    }

    // Decide if the tracking was succesful
    // More restrictive if there was a relocalization recently
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && mnMatchesInliers<50)
        return false;

    if(mnMatchesInliers<30)
        return false;
    else
        return true;
}


bool ORBTracking::NeedNewKeyFrame()
{
    return true;
    // If Local Mapping is freezed by a Loop Closure do not insert keyframes
    if(mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
    {
         PROMTD_S("Can Not Create KeyFrame.")
         return false;
    }
       

    //如果fps为0 则认为所有帧都未关键帧
    if(0 == mMaxFrames)
        return true;

    const int nKFs = mpMap->keyFrameInMap();

    // Do not insert keyframes if not enough frames have passed from last relocalisation
    if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && nKFs>mMaxFrames)
        return false;

    // Tracked MapPoints in the reference keyframe
    int nMinObs = 3;
    //若地图中关键数量不足 降低阈值
    if(nKFs<=2)
        nMinObs=2;
    //统计相连帧所有地图点中有关联帧 大于minobs数的 地图点
    int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);

    // Local Mapping accept keyframes?
    bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

    // Check how many "close" points are being tracked and how many could be potentially created.
    int nNonTrackedClose = 0;
    int nTrackedClose= 0;

    bool bNeedToInsertClose = (nTrackedClose<100) && (nNonTrackedClose>70);

    // Thresholds
    float thRefRatio = 0.75f;
    if(nKFs<2)
        thRefRatio = 0.4f;

    thRefRatio = 0.9f;//monocular

    // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
    const bool c1a = mCurrentFrame.mnId>=mnLastKeyFrameId+mMaxFrames;
    // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
    const bool c1b = (mCurrentFrame.mnId>=mnLastKeyFrameId+mMinFrames && bLocalMappingIdle);
    //Condition 1c: tracking is weak
    const bool c1c = false;
    // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
    const bool c2 = ((mnMatchesInliers<nRefMatches*thRefRatio|| bNeedToInsertClose) && mnMatchesInliers>15);

    if((c1a||c1b||c1c)&&c2)
    {
        // If the mapping accepts keyframes, insert keyframe.
        // Otherwise send a signal to interrupt BA
        if(bLocalMappingIdle)
        {
            return true;
        }
        else
        {
            mpLocalMapper->InterruptBA();
          
            return false;
        }
    }
    else
        return false;
}

void ORBTracking::CreateNewKeyFrame()
{
    if(!mpLocalMapper->SetNotStop(true))
        return;

    // PROMTD_V("Creat New Frame ",mCurrentFrame.getData()->_name);

    ORBKeyFrame* pKF = new ORBKeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB.get());
    
    mpReferenceKF = pKF;
    mCurrentFrame.mpReferenceKF = pKF;

    mpLocalMapper->InsertKeyFrame(pKF);

    mpLocalMapper->SetNotStop(false);

    mnLastKeyFrameId = mCurrentFrame.mnId;
    mpLastKeyFrame = pKF;
}
//遍历当前帧地图点列表
//
void ORBTracking::SearchLocalPoints()
{
    // Do not search map points already matched
    for(MapPtVIter vit=mCurrentFrame.mvpMapPoints.begin(), vend=mCurrentFrame.mvpMapPoints.end(); vit!=vend; vit++)
    {
        ORBMapPoint* pMP = ORBMAPPOINT(*vit);
        if(pMP)
        {
            if(pMP->isBad())
            {
                *vit = static_cast<ORBMapPoint*>(NULL);
            }
            else
            {
                pMP->IncreaseVisible();
                pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                pMP->mbTrackInView = false;
            }
        }
    }

    int nToMatch=0;

    // Project points in frame and check its visibility
    //遍历所有局部地图点,判断在当前帧视锥体点的数量
    for(MapPtVIter vit=mvpLocalMapPoints.begin(), vend=mvpLocalMapPoints.end(); vit!=vend; vit++)
    {
        ORBMapPoint* pMP = ORBMAPPOINT(*vit);
        if(pMP->mnLastFrameSeen == mCurrentFrame.mnId)
            continue;
        if(pMP->isBad())
            continue;
        // Project (this fills ORBMapPoint variables for matching)
        if(mCurrentFrame.isInFrustum(pMP,0.5))
        {
            pMP->IncreaseVisible();
            nToMatch++;
        }
    }
    //若查询到有匹配的点,则用局部地图点与当前帧进行匹配 查找新点
    if(nToMatch>0)
    {
        ORBmatcher matcher(0.8);
        int th = 1;
        // If the camera has been relocalised recently, perform a coarser search
        if(mCurrentFrame.mnId<mnLastRelocFrameId+2)
            th=5;
        matcher.SearchByProjection(mCurrentFrame,mvpLocalMapPoints,th);
    }
}

void ORBTracking::UpdateLocalMap()
{
    mpMap->setReferenceMapPoints(mvpLocalMapPoints);

    // Update
    UpdateLocalKeyFrames();
    UpdateLocalPoints();
}

//根据刷新的共视关键帧列表
//将所有地图点加入到当前本地地图点列表
void ORBTracking::UpdateLocalPoints()
{
    mvpLocalMapPoints.clear();

    for(KeyFrameVector::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        ORBKeyFrame* pKF = ORBKEYFRAME(*itKF);
        const MapPtVector& vpMPs = pKF->getWorldPoints();

        for(MapPtVector::const_iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end(); itMP!=itEndMP; itMP++)
        {
            ORBMapPoint* pMP = ORBMAPPOINT(*itMP);
            if(!pMP)
                continue;
            if(pMP->mnTrackReferenceForFrame==mCurrentFrame.mnId)
                continue;
            if(!pMP->isBad())
            {
                mvpLocalMapPoints.push_back(pMP);
                pMP->mnTrackReferenceForFrame=mCurrentFrame.mnId;
            }
        }
    }
}

//根据当前帧共视关系,刷新localkeyframe列表
void ORBTracking::UpdateLocalKeyFrames()
{
    // Each map point vote for the keyframes in which it has been observed
    KeyFrameMap keyframeCounter;
    //遍历当前帧地图点,获取共视关键帧的权值(共视点数)
    for(int i=0; i<mCurrentFrame.N; i++)
    {
        if(mCurrentFrame.mvpMapPoints[i])
        {
            IMapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
            if(!pMP->isBad())
            {
                const KeyFrameMap observations = pMP->getObservations();
                for(KeyFrameMap::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
                    keyframeCounter[it->first]++;
            }
            else
            {
                mCurrentFrame.mvpMapPoints[i]=NULL;
            }
        }
    }

    if(keyframeCounter.empty())
        return;

    int max=0;
    ORBKeyFrame* pKFmax= static_cast<ORBKeyFrame*>(NULL);

    mvpLocalKeyFrames.clear();
    mvpLocalKeyFrames.reserve(3*keyframeCounter.size());
    
    // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
    for(KeyFrameMap::const_iterator it=keyframeCounter.begin(), itEnd=keyframeCounter.end(); it!=itEnd; it++)
    {
        ORBKeyFrame* pKF = ORBKEYFRAME(it->first);

        if(pKF->isBad())
            continue;

        if(it->second>max)
        {
            max=it->second;
            pKFmax=pKF;
        }

        mvpLocalKeyFrames.push_back(ORBKEYFRAME(it->first));
        pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;
    }


    // Include also some not-already-included keyframes that are neighbors to already-included keyframes
    for(KeyFrameVector::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
    {
        // Limit the number of keyframes
        //限制共视关键帧的数量
        if(mvpLocalKeyFrames.size()>80)
            break;

        ORBKeyFrame* pKF = ORBKEYFRAME(*itKF);

        const vector<ORBKeyFrame*> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);

        for(vector<ORBKeyFrame*>::const_iterator itNeighKF=vNeighs.begin(), itEndNeighKF=vNeighs.end(); itNeighKF!=itEndNeighKF; itNeighKF++)
        {
            ORBKeyFrame* pNeighKF = *itNeighKF;
            if(!pNeighKF->isBad())
            {
                if(pNeighKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pNeighKF);
                    pNeighKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        const set<ORBKeyFrame*>& spChilds = pKF->GetChilds();
        for(set<ORBKeyFrame*>::const_iterator sit=spChilds.begin(), send=spChilds.end(); sit!=send; sit++)
        {
            ORBKeyFrame* pChildKF = *sit;
            if(!pChildKF->isBad())
            {
                if(pChildKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pChildKF);
                    pChildKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }
        }

        ORBKeyFrame* pParent = pKF->GetParent();
        if(pParent)
        {
            if(pParent->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
            {
                mvpLocalKeyFrames.push_back(pParent);
                pParent->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                break;
            }
        }

    }

    if(pKFmax)
    {
        mpReferenceKF = pKFmax;
        mCurrentFrame.mpReferenceKF = mpReferenceKF;
    }
}
//重定位(当运动模型跟踪丢失 且追参考帧追踪也丢失了 才进行)
bool ORBTracking::Relocalization()
{
    LOG_WARNING_F("Track lost Relocalization.%s",mCurrentFrame.getData()->_name.c_str());

    mState = eTrackNoReady;

    return false;

    if(!mpKeyFrameDB)
        return false;
    
    // Compute Bag of Words Vector
    mCurrentFrame.ComputeBoW();

    // Relocalization is performed when tracking is lost
    // Track Lost: Query ORBKeyFrame Database for keyframe candidates for relocalisation
    //通过当前帧的词袋模型中 筛选重定位的候选关键帧
    vector<ORBKeyFrame*> vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame);

    if(vpCandidateKFs.empty())
        return false;

    const int nKFs = vpCandidateKFs.size();

    // We perform first an ORB matching with each candidate
    // If enough matches are found we setup a PnP solver
    ORBmatcher matcher(0.75,true);

    vector<ORBPnPsolver*> vpPnPsolvers;
    vpPnPsolvers.resize(nKFs);

    vector< MapPtVector > vvpMapPointMatches;
    vvpMapPointMatches.resize(nKFs);

    BolVector vbDiscarded;
    vbDiscarded.resize(nKFs);

    int nCandidates=0;
    //遍历候选帧率, 根据词袋模型搜索 有效帧中 匹配点数量大于阈值建立pnp算法对象
    for(int i=0; i<nKFs; i++)
    {
        ORBKeyFrame* pKF = vpCandidateKFs[i];
        if(pKF->isBad())
            vbDiscarded[i] = true;
        else
        {
            int nmatches = matcher.SearchByBoW(pKF,mCurrentFrame,vvpMapPointMatches[i]);
            if(nmatches<15)
            {
                vbDiscarded[i] = true;
                continue;
            }
            else
            {
                ORBPnPsolver* pSolver = new ORBPnPsolver(mCurrentFrame,vvpMapPointMatches[i]);
                pSolver->SetRansacParameters(0.99,10,300,4,0.5,CHITH);
                vpPnPsolvers[i] = pSolver;
                nCandidates++;
            }
        }
    }

    // Alternatively perform some iterations of P4P RANSAC
    // Until we found a camera pose supported by enough inliers
    bool bMatch = false;
    ORBmatcher matcher2(0.9,true);

    while(nCandidates>0 && !bMatch)
    {
        for(int i=0; i<nKFs; i++)
        {
            if(vbDiscarded[i])
                continue;

            // Perform 5 Ransac Iterations
            BolVector vbInliers;
            int nInliers;
            bool bNoMore;

            ORBPnPsolver* pSolver = vpPnPsolvers[i];
            cv::Mat Tcw = pSolver->iterate(5,bNoMore,vbInliers,nInliers);

            // If Ransac reachs max. iterations discard keyframe
            if(bNoMore)
            {
                vbDiscarded[i]=true;
                nCandidates--;
            }

            // If a Camera Pose is computed, optimize
            if(!Tcw.empty())
            {
                Tcw.copyTo(mCurrentFrame.mTcw);

                MapPtSet sFound;

                const int np = vbInliers.size();

                for(int j=0; j<np; j++)
                {
                    if(vbInliers[j])
                    {
                        mCurrentFrame.mvpMapPoints[j]=vvpMapPointMatches[i][j];
                        sFound.insert(vvpMapPointMatches[i][j]);
                    }
                    else
                        mCurrentFrame.mvpMapPoints[j]=NULL;
                }

                int nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                if(nGood<10)
                    continue;

                for(int io =0; io<mCurrentFrame.N; io++)
                    if(mCurrentFrame.mvbOutlier[io])
                        mCurrentFrame.mvpMapPoints[io]=static_cast<ORBMapPoint*>(NULL);

                // If few inliers, search by projection in a coarse window and optimize again
                if(nGood<50)
                {
                    //查询候选帧中其他内点 在当前帧中新的匹配点数量
                    int nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,10,100);

                    if(nadditional+nGood>=50)
                    {
                        nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                        // If many inliers but still not enough, search by projection again in a narrower window
                        // the camera has been already optimized with many points
                        if(nGood>30 && nGood<50)
                        {
                            sFound.clear();
                            for(int ip =0; ip<mCurrentFrame.N; ip++)
                                if(mCurrentFrame.mvpMapPoints[ip])
                                    sFound.insert(mCurrentFrame.mvpMapPoints[ip]);
                            nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,3,64);

                            // Final optimization
                            if(nGood+nadditional>=50)
                            {
                                nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                                for(int io =0; io<mCurrentFrame.N; io++)
                                    if(mCurrentFrame.mvbOutlier[io])
                                        mCurrentFrame.mvpMapPoints[io]=NULL;
                            }
                        }
                    }
                }


                // If the pose is supported by enough inliers stop ransacs and continue
                if(nGood>=50)
                {
                    bMatch = true;
                    break;
                }
            }
        }
    }

    if(!bMatch)
    {
        return false;
    }
    else
    {
        mnLastRelocFrameId = mCurrentFrame.mnId;
        return true;
    }

}

void ORBTracking::Reset()
{
    PROMTD_S("System Reseting")

    // Reset Local Mapping
    PROMTD_S("Reseting Local Mapper...")
    mpLocalMapper->RequestReset();
    PROMTD_S(" done")

    // Reset Loop Closing
    // PROMTD_S("Reseting Loop Closing...")
    // mpLoopClosing->RequestReset();
    // PROMTD_S(" done")

    // Clear BoW Database
    PROMTD_S("Reseting Database...")
    mpKeyFrameDB->clear();
    PROMTD_S(" done")

    // Clear Map (this erase MapPoints and KeyFrames)
    PROMTD_S("Clear Map...");
    mpMap->clear();
    PROMTD_S(" done");

    ORBKeyFrame::nNextId = 0;
    ORBFrame::nNextId = 0;
    mState = eTrackNoImage;

    if(mpInitializer)
    {
        delete mpInitializer;
        mpInitializer = static_cast<Initializer*>(NULL);
    }

    mlRelativeFramePoses.clear();
    mlpReferences.clear();
    mlbLost.clear();
}

}
