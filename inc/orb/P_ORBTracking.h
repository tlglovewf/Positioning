


#ifndef _ORBTRACKING_H_
#define _ORBTRACKING_H_

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

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

    class LocalMapping;
    class LoopClosing;

    class IViewer;
    class Tracking
    {  

    public:
        Tracking(ORBVocabulary* pVoc, IViewer *pViewer, IMap* pMap,
                KeyFrameDatabase* pKFDB, const string &strSettingPath);

        // Preprocess the input and call Track(). Extract features and performs stereo matching.
        cv::Mat track(const FrameData &data);

        void SetLocalMapper(LocalMapping* pLocalMapper);
        void SetLoopClosing(LoopClosing* pLoopClosing);

        // Use this function if you have deactivated local mapping and you only want to localize the camera.
        void InformOnlyTracking(const bool &flag);


    public:

        // Tracking states
        enum eTrackingState{
            SYSTEM_NOT_READY=-1,
            NO_IMAGES_YET=0,
            NOT_INITIALIZED=1,
            OK=2,
            LOST=3
        };

        eTrackingState mState;
        eTrackingState mLastProcessedState;

        // Current Frame
        ORBFrame mCurrentFrame;
        cv::Mat mImGray;

        // Initialization Variables (Monocular)
        std::vector<int> mvIniLastMatches;
        std::vector<int> mvIniMatches;
        std::vector<cv::Point2f> mvbPrevMatched;
        std::vector<cv::Point3f> mvIniP3D;
        ORBFrame mInitialFrame;

        // Lists used to recover the full camera trajectory at the end of the execution.
        // Basically we store the reference keyframe for each frame and its relative transformation
        list<cv::Mat> mlRelativeFramePoses;
        list<ORBKeyFrame*> mlpReferences;
        list<double> mlFrameTimes;
        list<bool> mlbLost;

        // True if local mapping is deactivated and we are performing only localization
        bool mbOnlyTracking;

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

        // In case of performing only localization, this flag is true when there are no matches to
        // points in the map. Still tracking will continue if there are enough matches with temporal points.
        // In that case we are doing visual odometry. The system will try to do relocalization to recover
        // "zero-drift" localization to the map.
        bool mbVO;

        //Other Thread Pointers
        LocalMapping* mpLocalMapper;
        LoopClosing* mpLoopClosing;

        //ORB
        ORBextractor* mpORBextractorLeft;
        ORBextractor* mpIniORBextractor;

        //BoW
        ORBVocabulary* mpORBVocabulary;
        KeyFrameDatabase* mpKeyFrameDB;

        // Initalization (only for monocular)
        Initializer* mpInitializer;

        //Local Map
        ORBKeyFrame* mpReferenceKF;
        KeyFrameVector    mvpLocalKeyFrames;
        MapPtVector       mvpLocalMapPoints;

        //Map
        IMap* mpMap;

        //Calibration matrix
        cv::Mat mK;
        cv::Mat mDistCoef;

        //New ORBKeyFrame rules (according to fps)
        int mMinFrames;
        int mMaxFrames;

        // For RGB-D inputs only. For some datasets (e.g. TUM) the depthmap values are scaled.
        float mDepthMapFactor;

        //Current matches in frame
        int mnMatchesInliers;

        //Last Frame, ORBKeyFrame and Relocalisation Info
        ORBKeyFrame* mpLastKeyFrame;
        ORBFrame mLastFrame;
        unsigned int mnLastKeyFrameId;
        unsigned int mnLastRelocFrameId;

        //Motion Model
        cv::Mat mVelocity;

        //Color order (true RGB, false BGR, ignored if grayscale)
        bool mbRGB;

        list<ORBMapPoint*> mlpTemporalPoints;
    };

} 

#endif // _ORBTRACKING_H_
