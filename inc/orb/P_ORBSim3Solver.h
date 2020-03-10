


#ifndef _ORBSIM3SOLVER_H_
#define _ORBSIM3SOLVER_H_

#include <opencv2/opencv.hpp>
#include <vector>

#include "P_ORBKeyFrame.h"



namespace Position
{

    class Sim3Solver
    {
    public:

        Sim3Solver(ORBKeyFrame* pKF1, ORBKeyFrame* pKF2, const MapPtVector &vpMatched12, const bool bFixScale = true);

        void SetRansacParameters(double probability = 0.99, int minInliers = 6 , int maxIterations = 300);

        cv::Mat find(BolVector &vbInliers12, int &nInliers);

        cv::Mat iterate(int nIterations, bool &bNoMore, BolVector &vbInliers, int &nInliers);

        cv::Mat GetEstimatedRotation();
        cv::Mat GetEstimatedTranslation();
        float GetEstimatedScale();


    protected:

        void ComputeCentroid(cv::Mat &P, cv::Mat &Pr, cv::Mat &C);

        void ComputeSim3(cv::Mat &P1, cv::Mat &P2);

        void CheckInliers();

        void Project(const std::vector<cv::Mat> &vP3Dw, std::vector<cv::Mat> &vP2D, cv::Mat Tcw, cv::Mat K);
        void FromCameraToImage(const std::vector<cv::Mat> &vP3Dc, std::vector<cv::Mat> &vP2D, cv::Mat K);


    protected:

        // KeyFrames and matches
        ORBKeyFrame* mpKF1;
        ORBKeyFrame* mpKF2;

        std::vector<cv::Mat> mvX3Dc1;
        std::vector<cv::Mat> mvX3Dc2;
        MapPtVector mvpMapPoints1;
        MapPtVector mvpMatches12;
        SzVector mvnIndices1;
        SzVector mvnMaxError1;
        SzVector mvnMaxError2;

        int N;
        int mN1;

        // Current Estimation
        cv::Mat mR12i;
        cv::Mat mt12i;
        float ms12i;
        cv::Mat mT12i;
        cv::Mat mT21i;
        BolVector mvbInliersi;
        int mnInliersi;

        // Current Ransac State
        int mnIterations;
        BolVector mvbBestInliers;
        int mnBestInliers;
        cv::Mat mBestT12;
        cv::Mat mBestRotation;
        cv::Mat mBestTranslation;
        float mBestScale;

        // Scale is fixed to 1 in the stereo/RGBD case
        bool mbFixScale;

        // Indices for random selection
        SzVector mvAllIndices;

        // Projections
        std::vector<cv::Mat> mvP1im1;
        std::vector<cv::Mat> mvP2im2;

        // RANSAC probability
        double mRansacProb;

        // RANSAC min inliers
        int mRansacMinInliers;

        // RANSAC max iterations
        int mRansacMaxIts;

        // Threshold inlier/outlier. e = dist(Pi,T_ij*Pj)^2 < 5.991*mSigma2
        float mTh;
        float mSigma2;

        // Calibration
        cv::Mat mK1;
        cv::Mat mK2;

    };

} 

#endif // _ORBSIM3SOLVER_H_
