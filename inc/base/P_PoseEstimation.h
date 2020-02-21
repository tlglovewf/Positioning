/**
 *   P_PoseEstimation.h
 *   
 *   add by tu li gen   2020.2.17
 * 
 */
#ifndef __PPOSEESTIMATION_H_H_
#define __PPOSEESTIMATION_H_H_
#include "P_Interface.h"

namespace Position
{
    //位姿推算
    class PPoseEstimation : public IPoseEstimation
    {
    public:
        PPoseEstimation():mPre(NULL),mCur(NULL){}
         //设置相机参数
        virtual void setCamera(const CameraParam &cam) 
        {
            mCam = cam;
        }
        //设置帧
        virtual void setParams( IFrame *pre, IFrame *cur, const MatchVector &matches) 
        {
            assert(pre && cur);
            mPre = pre;
            mCur = cur;
            initParams(matches);
        }
        //推算位姿
        virtual bool estimate(cv::Mat &R, cv::Mat &t)
        {
            assert(NULL);
        }

    protected:
        //初始化
        virtual void initParams(const MatchVector &matches) = 0;


    protected:
        CameraParam              mCam;
        IFrame                  *mPre;
        IFrame                  *mCur;
        MatchPairs               mvMatches12;
    };

    //cv接口推算位姿
    class CVPoseEstimation : public PPoseEstimation
    {
    public:
         //推算位姿
        virtual bool estimate(cv::Mat &R, cv::Mat &t);

     protected:
        //初始化
        virtual void initParams(const MatchVector &matches);

    private:
        PtVector mPrePts;
        PtVector mCurPts;
    };

    //orbslam中 位姿推算
    class ORBPoseEstimation : public PPoseEstimation
    {
    public:
        //构造
        ORBPoseEstimation():mMaxIterations(200),mSigma(2.0),mSigma2(mSigma*mSigma){}

         //推算位姿
        virtual bool estimate(cv::Mat &R, cv::Mat &t);

        //计算单应矩阵
        void FindHomography (BolVector &vbInliers, float &score, cv::Mat &H21);
        cv::Mat ComputeH21(const PtVector &vP1, const PtVector &vP2);
        float CheckHomography(const cv::Mat &H21, const cv::Mat &H12, vector<bool> &vbMatchesInliers, float sigma);

        //计算基础矩阵
        void FindFundamental(BolVector &vbInliers, float &score, cv::Mat &F21);
        cv::Mat ComputeF21(const PtVector &vP1, const PtVector &vP2);
        float CheckFundamental(const cv::Mat &F21, vector<bool> &vbMatchesInliers, float sigma);

        //从基础矩阵恢复位姿
        bool ReconstructF(BolVector &vbMatchesInliers, cv::Mat &F21, cv::Mat &K,
                      cv::Mat &R21, cv::Mat &t21, Pt3Vector &vP3D, BolVector &vbTriangulated, float minParallax, int minTriangulated);

        //从单应矩阵恢复位姿
        bool ReconstructH(BolVector &vbMatchesInliers, cv::Mat &H21, cv::Mat &K,
                      cv::Mat &R21, cv::Mat &t21, Pt3Vector &vP3D, BolVector &vbTriangulated, float minParallax, int minTriangulated);


        //三角化 P1 K[I|0] P2 K[R|t]    x3D 4x1
        void Triangulate(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const cv::Mat &P1, const cv::Mat &P2, cv::Mat &x3D);

        //矩阵分解
        void DecomposeE(const cv::Mat &E, cv::Mat &R1, cv::Mat &R2, cv::Mat &t);

        //检查R t
        int CheckRT(const cv::Mat &R, const cv::Mat &t, const vector<cv::KeyPoint> &vKeys1, const vector<cv::KeyPoint> &vKeys2,
                                    const vector<MatchPair> &vMatches12, vector<bool> &vbMatchesInliers,
                                    const cv::Mat &K, vector<cv::Point3f> &vP3D, float th2, vector<bool> &vbGood, float &parallax);

        //归一化
        void Normalize(const vector<cv::KeyPoint> &vKeys, vector<cv::Point2f> &vNormalizedPoints, cv::Mat &T);
    
    protected:
        //初始化
        virtual void initParams(const MatchVector &matches);

    protected:
        float                       mSigma;
        float                       mSigma2;
        int                         mMaxIterations;
        vector< vector<size_t> >    mvSets;
    };


}

#endif