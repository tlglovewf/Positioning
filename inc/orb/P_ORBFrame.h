#ifndef _ORBFRAME_H_
#define _ORBFRAME_H_

#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "P_ORBMapPoint.h"
#include "P_ORBVocabulary.h"
#include "P_ORBKeyFrame.h"
#include "P_ORBFeature.h"
#include "P_Interface.h"

namespace Position
{
    #define FRAME_GRID_ROWS 48
    #define FRAME_GRID_COLS 64

    class ORBMapPoint;
    class ORBKeyFrame;

    class ORBFrame : public IFrame
    {
    public:
        ORBFrame();

        // Copy constructor.
        ORBFrame(const ORBFrame &frame);

        // Constructor for Monocular cameras.
        ORBFrame(const cv::Mat &imGray, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef);


        //帧序号
        virtual u64 index()const 
        {
            return mnId;
        }
        //获取帧数据
        virtual FrameData getData()const
        {
            assert(NULL);
        }
        //获取关键点
        virtual const KeyPtVector& getKeys()const {return mvKeysUn;}
        //获取特征点数量
        virtual int getKeySize()const {return mvKeysUn.size();}
        //获取描述子
        virtual const Mat& getDescript()const {return mDescriptors;}

        //获取位置(R|t 矩阵)
        virtual Mat getPose() 
        {
            return mTcw;
        }
        //设置位置(R|t 矩阵)
        virtual void setPose(const cv::Mat &pose);
        //获取中心点
        virtual const Mat& getCameraCenter()const 
        {
            return mOw;
        }

        // Extract ORB on the image. 0 for left image and 1 for right image.
        void ExtractORB(int flag, const cv::Mat &im);

        // Compute Bag of Words representation.
        void ComputeBoW();

        // Computes rotation, translation and camera center matrices from the camera pose.
        void UpdatePoseMatrices();


        // Returns inverse of rotation
        inline cv::Mat GetRotationInverse(){
            return mRwc.clone();
        }

        // Check if a ORBMapPoint is in the frustum of the camera
        // and fill variables of the ORBMapPoint to be used by the tracking
        bool isInFrustum(ORBMapPoint* pMP, float viewingCosLimit);

        // Compute the cell of a keypoint (return false if outside the grid)
        bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);

        SzVector GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel=-1, const int maxLevel=-1) const;

    public:
        // Vocabulary used for relocalization.
        ORBVocabulary* mpORBvocabulary;

        // Feature extractor. The right is used only in the stereo case.
        ORBextractor* mpORBextractorLeft;

        // Frame timestamp.
        double mTimeStamp;

        // Calibration matrix and OpenCV distortion parameters.
        cv::Mat mK;
        static float fx;
        static float fy;
        static float cx;
        static float cy;
        static float invfx;
        static float invfy;
        cv::Mat mDistCoef;

        // Threshold close/far points. Close points are inserted from 1 view.
        // Far points are inserted as in the monocular case from 2 views.

        // Number of KeyPoints.
        int N;

        // Vector of keypoints (original for visualization) and undistorted (actually used by the system).
        // In the stereo case, mvKeysUn is redundant as images must be rectified.
        // In the RGB-D case, RGB images can be distorted.
        KeyPtVector mvKeysUn;
        KeyPtVector mvKeys;

        // Bag of Words Vector structures.
        DBoW2::BowVector mBowVec;
        DBoW2::FeatureVector mFeatVec;

        // ORB descriptor, each row associated to a keypoint.
        cv::Mat mDescriptors;

        // MapPoints associated to keypoints, NULL pointer if no association.
        MapPtVector mvpMapPoints;

        // Flag to identify outlier associations.
        BolVector mvbOutlier;

        // Keypoints are assigned to cells in a grid to reduce matching complexity when projecting MapPoints.
        static float mfGridElementWidthInv;
        static float mfGridElementHeightInv;
        SzVector mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];

        // Camera pose.
        cv::Mat mTcw;

        // Current and Next Frame id.
        static u64 nNextId;
        u64 mnId;

        // Reference Keyframe.
        ORBKeyFrame* mpReferenceKF;

        // Scale pyramid info.
        int mnScaleLevels;
        float mfScaleFactor;
        float mfLogScaleFactor;
        FloatVector mvScaleFactors;
        FloatVector mvInvScaleFactors;
        FloatVector mvLevelSigma2;
        FloatVector mvInvLevelSigma2;

        // Undistorted Image Bounds (computed once).
        static float mnMinX;
        static float mnMaxX;
        static float mnMinY;
        static float mnMaxY;

        static bool mbInitialComputations;


    private:

        // Undistort keypoints given OpenCV distortion parameters.
        // Only for the RGB-D case. Stereo must be already rectified!
        // (called in the constructor).
        void UndistortKeyPoints();

        // Computes image bounds for the undistorted image (called in the constructor).
        void ComputeImageBounds(const cv::Mat &imLeft);

        // Assign keypoints to the grid for speed up feature matching (called in the constructor).
        void AssignFeaturesToGrid();

        // Rotation, translation and camera center
        cv::Mat mRcw;
        cv::Mat mtcw;
        cv::Mat mRwc;
        cv::Mat mOw; //==mtwc
    };

}

#endif // _ORBFRAME_H_
