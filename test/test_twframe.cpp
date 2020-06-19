#include "test.h"
#include "P_Utils.h"
#include "P_Map.h"
#include "P_Factory.h"
#include "P_Task.h"
#include "project/newhwproject.h"

using namespace std;
using namespace cv;

//! 检查基础矩阵 
//! @param sigma 系数,越大越严格
float CheckFundamental(const Position::KeyPtVector &pt1s,const Position::KeyPtVector &pt2s,
                       const Position::MatchVector &mvMatches12, 
                       const cv::Mat &F21, Position::BolVector &vbMatchesInliers, float sigma)
    {
        const int N = mvMatches12.size();

        const MATTYPE f11 = F21.at<MATTYPE>(0,0);
        const MATTYPE f12 = F21.at<MATTYPE>(0,1);
        const MATTYPE f13 = F21.at<MATTYPE>(0,2);
        const MATTYPE f21 = F21.at<MATTYPE>(1,0);
        const MATTYPE f22 = F21.at<MATTYPE>(1,1);
        const MATTYPE f23 = F21.at<MATTYPE>(1,2);
        const MATTYPE f31 = F21.at<MATTYPE>(2,0);
        const MATTYPE f32 = F21.at<MATTYPE>(2,1);
        const MATTYPE f33 = F21.at<MATTYPE>(2,2);

        vbMatchesInliers.resize(N);

        float score = 0;

        float maxdist = 0;
        float mindist = 0;

        const float th = 3.841;
        
        const float thScore = CHITH;

        const float invSigmaSquare = 1.0/(sigma*sigma);

        for(int i=0; i<N; i++)
        {
            bool bIn = true;

            const cv::KeyPoint &kp1 = pt1s[mvMatches12[i].queryIdx];
            const cv::KeyPoint &kp2 = pt2s[mvMatches12[i].trainIdx];

            const float u1 = kp1.pt.x;
            const float v1 = kp1.pt.y;
            const float u2 = kp2.pt.x;
            const float v2 = kp2.pt.y;

            // Reprojection error in second image
            // l2=F21x1=(a2,b2,c2)

            const MATTYPE a2 = f11*u1+f12*v1+f13;
            const MATTYPE b2 = f21*u1+f22*v1+f23;
            const MATTYPE c2 = f31*u1+f32*v1+f33;

            const MATTYPE num2 = a2*u2+b2*v2+c2;

            const MATTYPE squareDist1 = num2*num2/(a2*a2+b2*b2);

            const MATTYPE chiSquare1 = squareDist1*invSigmaSquare;


            MATTYPE d = sqrt(squareDist1);

            if(chiSquare1>th)
                bIn = false;
            else
                score += thScore - chiSquare1;

            // Reprojection error in second image
            // l1 =x2tF21=(a1,b1,c1)

            const MATTYPE a1 = f11*u2+f21*v2+f31;
            const MATTYPE b1 = f12*u2+f22*v2+f32;
            const MATTYPE c1 = f13*u2+f23*v2+f33;

            const MATTYPE num1 = a1*u1+b1*v1+c1;

            const MATTYPE squareDist2 = num1*num1/(a1*a1+b1*b1);

            const MATTYPE chiSquare2 = squareDist2*invSigmaSquare;

            d += sqrt(squareDist2);

            d = d / 2.0;
 
            if(d > maxdist)
                maxdist = d;

            if(i == 0)
                mindist = d;
            else if(mindist > d)
            {
                mindist = d;
            }
            

            if(chiSquare2>th)
                bIn = false;
            else
                score += thScore - chiSquare2;

            if(bIn)
                vbMatchesInliers[i]=true;
            else
                vbMatchesInliers[i]=false;
        }   

        cout << "max distance is : " << maxdist << endl;
        cout << "min distance is : " << mindist << endl;
        cout << "inlier percent : " << std::count_if(vbMatchesInliers.begin(),vbMatchesInliers.end(),[](bool bol)->bool{
            return bol;
        }) / (float)N << endl;
        return score / N;
    }


//! 检查重投误差
//根据R t计算特征对的三维坐标
int CheckRT(const cv::Mat &R, const cv::Mat &t, 
            const Position::KeyPtVector &vKeys1, 
            const Position::KeyPtVector &vKeys2,
            const Position::MatchVector &vMatches12, 
            Position::BolVector &vbMatchesInliers,
            const cv::Mat &K, Position::Pt3Vector &vP3D, 
            Position::BolVector &vbGood,float th2, 
            float &parallax)
{
    // Calibration parameters
    const float thparallax = 1;//0.99998;//大概1°  点到两帧光心夹角最小阈值
    const float fx = K.at<MATTYPE>(0,0);
    const float fy = K.at<MATTYPE>(1,1);
    const float cx = K.at<MATTYPE>(0,2);
    const float cy = K.at<MATTYPE>(1,2);

    vbGood = Position::BolVector(vKeys1.size(),false);
    vP3D.resize(vKeys1.size());

    Position::FloatVector vCosParallax;
    vCosParallax.reserve(vKeys1.size());

    // Camera 1 Projection Matrix K[I|0]
    cv::Mat P1(3,4,MATCVTYPE,cv::Scalar(0));
    K.copyTo(P1.rowRange(0,3).colRange(0,3));

    cv::Mat O1 = cv::Mat::zeros(3,1,MATCVTYPE);

    // Camera 2 Projection Matrix K[R|t]
    cv::Mat P2(3,4,MATCVTYPE);
    R.copyTo(P2.rowRange(0,3).colRange(0,3));
    t.copyTo(P2.rowRange(0,3).col(3));
    P2 = K*P2;

    cv::Mat O2 = -R.t()*t;

    int nGood=0;
    //三角化 地图点
    for(size_t i=0, iend=vMatches12.size();i<iend;i++)
    {
        if(!vbMatchesInliers[i])
            continue;

        const cv::KeyPoint &kp1 = vKeys1[vMatches12[i].queryIdx];
        const cv::KeyPoint &kp2 = vKeys2[vMatches12[i].trainIdx];
        cv::Mat p3dC1;

        Position::PUtils::Triangulate(kp1.pt,kp2.pt,P1,P2,p3dC1);
        //剔除错误点
        if(!isfinite(p3dC1.at<MATTYPE>(0)) || !isfinite(p3dC1.at<MATTYPE>(1)) || !isfinite(p3dC1.at<MATTYPE>(2)))
        {
            vbGood[vMatches12[i].queryIdx]=false;
            continue;
        }

        // Check parallax
        cv::Mat normal1 = p3dC1 - O1;
        float dist1 = cv::norm(normal1);

        cv::Mat normal2 = p3dC1 - O2;
        float dist2 = cv::norm(normal2);
        //计算点到两帧光心的夹角
        float cosParallax = normal1.dot(normal2)/(dist1*dist2);

        // Check depth in front of first camera (only if enough parallax, as "infinite" points can easily go to negative depth)
        if(p3dC1.at<MATTYPE>(2)<=0 && cosParallax < thparallax)
            continue;

        // Check depth in front of second camera (only if enough parallax, as "infinite" points can easily go to negative depth)
        cv::Mat p3dC2 = R*p3dC1+t;

        if(p3dC2.at<MATTYPE>(2)<=0 && cosParallax < thparallax)
            continue;

        // Check reprojection error in first image
        float im1x, im1y;
        float invZ1 = 1.0/p3dC1.at<MATTYPE>(2);
        im1x = fx*p3dC1.at<MATTYPE>(0)*invZ1+cx;
        im1y = fy*p3dC1.at<MATTYPE>(1)*invZ1+cy;
        //计算重投影误差
        float squareError1 = (im1x-kp1.pt.x)*(im1x-kp1.pt.x)+(im1y-kp1.pt.y)*(im1y-kp1.pt.y);

        if(squareError1>th2)
            continue;

        // Check reprojection error in second image
        float im2x, im2y;
        float invZ2 = 1.0/p3dC2.at<MATTYPE>(2);
        im2x = fx*p3dC2.at<MATTYPE>(0)*invZ2+cx;
        im2y = fy*p3dC2.at<MATTYPE>(1)*invZ2+cy;
        
        //计算重投影误差平方和
        float squareError2 = (im2x-kp2.pt.x)*(im2x-kp2.pt.x)+(im2y-kp2.pt.y)*(im2y-kp2.pt.y);

        if(squareError2> th2)
            continue;

        vCosParallax.push_back(cosParallax);
        vP3D[vMatches12[i].queryIdx] = cv::Point3f(p3dC1.at<MATTYPE>(0),p3dC1.at<MATTYPE>(1),p3dC1.at<MATTYPE>(2));
        nGood++;

        if(cosParallax < thparallax)
            vbGood[vMatches12[i].queryIdx]=true;
    }

    if(nGood>0)
    {
        sort(vCosParallax.begin(),vCosParallax.end());

        size_t idx = min(50,int(vCosParallax.size()-1));
        parallax = acos(vCosParallax[idx])*180/CV_PI;
    }
    else
        parallax=0;

    return nGood;
}





TESTBEGIN()

    std::shared_ptr<Position::IConfig> pCfg(new ImgAutoConfig("../config/config_new.yaml"));

    std::shared_ptr<Position::IFrameData>   pData(new NewHwProjectData(pCfg));

    LOG_INITIALIZE(pCfg);
    SETGLOBALCONFIG(pCfg);

#ifdef USE_VIEW
    std::shared_ptr<Position::IViewer>  pv(GETVIEWER());
#endif

    pData->loadDatas();

    int stno = GETCFGVALUE(pCfg,StNo,int);

    Position::FrameDataPtrVIter iter = pData->begin() + 300;
    Position::FrameDataPtrVIter next = iter + 3;

    std::shared_ptr<Position::IMap> pmap(new Position::PMap());

#ifdef USE_VIEW
    pv->setMap(pmap);
#endif
   
    Position::FeatureTask featureTask("SiftEx",pCfg->getCamera());
    Position::MatcherTask mtTask("Knn");
    Position::PoseTask    psTask("CVPoseSolver",pCfg->getCamera());

    Position::IFrame *pf1 = featureTask.run(*iter); 
    Position::IFrame *pf2 = featureTask.run(*next);

    Position::MatcherTask::Item mitem(pf1,pf2);
    int sz = mtTask.run(mitem);
    cout << "match size : " << sz << endl;

    Position::PoseResult result = psTask.run(mitem);

    Position::BolVector bols;
    CheckFundamental(pf1->getKeys(),pf2->getKeys(),mitem.matches,result._F,bols,1);


    cout << mitem.matches.size() << " " << result._match.size() << " " <<  result._rate << endl;




#ifdef USE_VIEW
        // pv->renderLoop();
#endif
TESTEND()