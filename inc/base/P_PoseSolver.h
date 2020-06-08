/**
 *   P_PoseEstimation.h
 *   
 *   add by tu li gen   2020.2.17
 * 
 */
#ifndef __POSESOLVER_H_H_
#define __POSESOLVER_H_H_
#include "P_Interface.h"

namespace Position
{
    //位姿推算
    class PPoseSolver : public IPoseSolver
    {
    public:
        PPoseSolver():mPre(NULL),mCur(NULL){}
         //设置相机参数
        virtual void setCamera(const CameraParam &cam) 
        {
            mCam = cam;
        }
        //设置帧
        virtual void setFrames( IFrame *pre, IFrame *cur) 
        {
            assert(pre && cur);
            mPre = pre;
            mCur = cur;
        }
        //推算位姿
        virtual bool estimate(cv::Mat &R, cv::Mat &t, MatchVector &matches ,Pt3Vector &vPts)
        {
            assert(NULL);
            return false;
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
    class CVPoseSolver : public PPoseSolver
    {
    public:
         //! 推算位姿
        virtual bool estimate(cv::Mat &R, cv::Mat &t, MatchVector &matches, Pt3Vector &vPts);

     protected:
        //初始化
        virtual void initParams(const MatchVector &matches);

    private:
        PtVector mPrePts;
        PtVector mCurPts;
    };

    //ORBSLAM中 位姿推算
    class ORBPoseSolver : public PPoseSolver
    {
    public:
        //构造
        ORBPoseSolver():mMaxIterations(400),mSigma(2.0),mSigma2(mSigma*mSigma){}

         //推算位姿
        virtual bool estimate(cv::Mat &R, cv::Mat &t,MatchVector &matches, Pt3Vector &vPts);

        //计算单应矩阵
        void FindHomography (BolVector &vbInliers, float &score, cv::Mat &H21);
        cv::Mat ComputeH21(const PtVector &vP1, const PtVector &vP2);
        float CheckHomography(const cv::Mat &H21, const cv::Mat &H12, BolVector &vbMatchesInliers, float sigma);

        //计算基础矩阵
        void FindFundamental(BolVector &vbInliers, float &score, cv::Mat &F21);
        cv::Mat ComputeF21(const PtVector &vP1, const PtVector &vP2);
        float CheckFundamental(const cv::Mat &F21, BolVector &vbMatchesInliers, float sigma);

        //从基础矩阵恢复位姿
        bool ReconstructF(BolVector &vbMatchesInliers, cv::Mat &F21, cv::Mat &K,
                      cv::Mat &R21, cv::Mat &t21, Pt3Vector &vP3D, BolVector &vbTriangulated, float minParallax, int minTriangulated);

        //从单应矩阵恢复位姿
        bool ReconstructH(BolVector &vbMatchesInliers, cv::Mat &H21, cv::Mat &K,
                      cv::Mat &R21, cv::Mat &t21, Pt3Vector &vP3D, BolVector &vbTriangulated, float minParallax, int minTriangulated);

        //矩阵分解
        void DecomposeE(const cv::Mat &E, cv::Mat &R1, cv::Mat &R2, cv::Mat &t);

        //检查R t
        int CheckRT(const cv::Mat &R, const cv::Mat &t, const KeyPtVector &vKeys1, const KeyPtVector &vKeys2,
                                    const MatchPairs &vMatches12, BolVector &vbMatchesInliers,
                                    const cv::Mat &K, Pt3Vector &vP3D, float th2, BolVector &vbGood, float &parallax);

        //归一化
        void Normalize(const KeyPtVector &vKeys, vector<cv::Point2f> &vNormalizedPoints, cv::Mat &T);
    
    protected:
        //初始化
        virtual void initParams(const MatchVector &matches);

    protected:
        int                         mMaxIterations;
        float                       mSigma;
        float                       mSigma2;
        
        vector< SzVector >    mvSets;
    };

    DECLAREIFACTORY(IPoseSolver, CVPoseSolver    ,CVPoseSolver)
    DECLAREIFACTORY(IPoseSolver, ORBPoseSolver   ,ORBPoseSolver)

}

#endif