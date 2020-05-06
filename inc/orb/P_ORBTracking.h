


#ifndef _ORBTRACKING_H_
#define _ORBTRACKING_H_

#include "P_ORBLocalMapping.h"
#include "P_ORBLoopClosing.h"
#include "P_ORBFrame.h"
#include "P_ORBVocabulary.h"
#include "P_ORBKeyFrameDatabase.h"
#include "P_ORBFeature.h"
#include "P_ORBInitializer.h"
#include "P_Interface.h"

namespace Position
{
    class ORBTracking
    {  

    public:
        ORBTracking( const std::shared_ptr<ORBVocabulary>& pVoc, 
                     const std::shared_ptr<IMap>& pMap,
                     const std::shared_ptr<ORBKeyFrameDatabase>& pKFDB,
                     const std::shared_ptr<IConfig> &pcfg,
                     const CameraParam &camparam);
 
        // Preprocess the input and call Track(). Extract features and performs stereo matching.
        cv::Mat track( FrameData *data);

        cv::Mat InitMode(const FrameDataPtrVector &framedatas, const int imgnum);
        
        void SetLocalMapper(const std::shared_ptr<ORBLocalMapping>& pLocalMapper);
        void SetLoopClosing(const std::shared_ptr<ORBLoopClosing>& pLoopClosing);

    public:

        eTrackStatus mState;
        // Current Frame
        ORBFrame mCurrentFrame;
        cv::Mat mImGray;

        // Initialization Variables (Monocular)
        IntVector mvIniLastMatches;
        IntVector mvIniMatches;
        PtVector  mvbPrevMatched;
        Pt3Vector mvIniP3D;
        ORBFrame mInitialFrame;

        // Lists used to recover the full camera trajectory at the end of the execution.
        // Basically we store the reference keyframe for each frame and its relative transformation
        list<cv::Mat> mlRelativeFramePoses;
        list<ORBKeyFrame*> mlpReferences;
        list<bool> mlbLost;

        void Reset();

    protected:

        // Main tracking function. It is independent of the input sensor.
        void Track();

        // Map initialization for monocular
        void MonocularInitialization();
        void CreateInitialMapMonocular();

        void CheckReplacedInLastFrame();
        bool TrackReferenceKeyFrame();
        void UpdateLastFrame();
        bool TrackWithMotionModel();

        bool Relocalization();

        void UpdateLocalMap();
        void UpdateLocalPoints();
        void UpdateLocalKeyFrames();

        bool TrackLocalMap();
        void SearchLocalPoints();

        bool NeedNewKeyFrame();
        void CreateNewKeyFrame();

        //Other Thread Pointers
        std::shared_ptr<ORBLocalMapping> mpLocalMapper;
        std::shared_ptr<ORBLoopClosing> mpLoopClosing;

        //ORB
        ORBextractor* mpORBextractorLeft;
        ORBextractor* mpIniORBextractor;

        //BoW
        std::shared_ptr<ORBVocabulary>       mpORBVocabulary;
        std::shared_ptr<ORBKeyFrameDatabase> mpKeyFrameDB;

        // Initalization (only for monocular)
        Initializer* mpInitializer;

        //Local Map
        ORBKeyFrame* mpReferenceKF;
        KeyFrameVector    mvpLocalKeyFrames;
        MapPtVector       mvpLocalMapPoints;

        //Map
        std::shared_ptr<IMap> mpMap;

        //Calibration matrix
        cv::Mat mK;
        cv::Mat mDistCoef;

        //New ORBKeyFrame rules (according to fps)
        int mMinFrames;
        int mMaxFrames;
        
        //初始化策略选择
        int initMode;
        //初始化搜索步长
        int initStep;
        //for initialize ratio
        float mfForInitRatio;

        //Current matches in frame
        int mnMatchesInliers;

        int mnSearchRadius;
        //Last Frame, ORBKeyFrame and Relocalisation Info
        ORBKeyFrame* mpLastKeyFrame;
        ORBFrame mLastFrame;
        unsigned int mnLastKeyFrameId;
        unsigned int mnLastRelocFrameId;

        //Motion Model
        cv::Mat mVelocity;

        //Color order (true RGB, false BGR, ignored if grayscale)
        bool mbRGB;
    };

} 

#endif // _ORBTRACKING_H_
