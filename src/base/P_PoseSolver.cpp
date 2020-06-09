#include "P_PoseSolver.h"
#include "P_Utils.h"
#include <thread>
namespace Position
{
    class Random
    {
    public:
        static void Init()
        {
            srand(time(NULL)); 
        }

        static int RandomInt(int min, int max)
        {
	        int d = max - min + 1;
	        return int(((double)rand()/((double)RAND_MAX + 1.0)) * d) + min;
        }
    };

#pragma region ORBPoseSolver
    //计算单应矩阵
    void ORBPoseSolver::FindHomography(BolVector &vbInliers, float &score, cv::Mat &H21)
    {
        const int N = mvMatches12.size();

        // Normalize coordinates
        vector<cv::Point2f> vPn1, vPn2;
        cv::Mat T1, T2;
        Normalize(mpInput->_query,vPn1, T1);
        Normalize(mpInput->_train,vPn2, T2);
        cv::Mat T2inv = T2.inv();

        // Best Results variables
        score = 0.0;
        vbInliers = BolVector(N,false);

        // Iteration variables
        vector<cv::Point2f> vPn1i(8);
        vector<cv::Point2f> vPn2i(8);
        cv::Mat H21i, H12i;
        BolVector vbCurrentInliers(N,false);
        float currentScore;

        // Perform all RANSAC iterations and save the solution with highest score
        for(int it=0; it<mMaxIterations; it++)
        {
            // Select a minimum set
            for(size_t j=0; j<8; j++)
            {
                int idx = mvSets[it][j];

                vPn1i[j] = vPn1[mvMatches12[idx].first];
                vPn2i[j] = vPn2[mvMatches12[idx].second];
            }

            cv::Mat Hn = ComputeH21(vPn1i,vPn2i);
            H21i = T2inv*Hn*T1;
            H12i = H21i.inv();

            currentScore = CheckHomography(H21i, H12i, vbCurrentInliers, mSigma);

            if(currentScore>score)
            {
                H21 = H21i.clone();
                vbInliers.swap(vbCurrentInliers);
                score = currentScore;
            }
        }
    }

    //计算基础矩阵
    void ORBPoseSolver::FindFundamental(BolVector &vbInliers, float &score, cv::Mat &F21)
    {
        // Number of putative matches
        const int N = vbInliers.size();

        // Normalize coordinates
        vector<cv::Point2f> vPn1, vPn2;
        cv::Mat T1, T2;
        
        Normalize(mpInput->_query,vPn1, T1);
        Normalize(mpInput->_train,vPn2, T2);
        cv::Mat T2t = T2.t();

        // Best Results variables
        score = 0.0;
        vbInliers = BolVector(N,false);

        // Iteration variables
        vector<cv::Point2f> vPn1i(8);
        vector<cv::Point2f> vPn2i(8);
        cv::Mat F21i;
        BolVector vbCurrentInliers(N,false);
        float currentScore;

        // Perform all RANSAC iterations and save the solution with highest score
        for(int it=0; it<mMaxIterations; it++)
        {
            // Select a minimum set
            for(int j=0; j<8; j++)
            {
                int idx = mvSets[it][j];

                vPn1i[j] = vPn1[mvMatches12[idx].first];
                vPn2i[j] = vPn2[mvMatches12[idx].second];
            }

            cv::Mat Fn = ComputeF21(vPn1i,vPn2i);

            F21i = T2t*Fn*T1;

            currentScore = CheckFundamental(F21i, vbCurrentInliers, mSigma);

            if(currentScore>score)
            {
                F21 = F21i.clone();
                vbInliers.swap(vbCurrentInliers);
                score = currentScore;
            }
        }
    }

    float ORBPoseSolver::CheckHomography(const cv::Mat &H21, const cv::Mat &H12, BolVector &vbMatchesInliers, float sigma)
    {
        const int N = mvMatches12.size();

        const MATTYPE h11 = H21.at<MATTYPE>(0,0);
        const MATTYPE h12 = H21.at<MATTYPE>(0,1);
        const MATTYPE h13 = H21.at<MATTYPE>(0,2);
        const MATTYPE h21 = H21.at<MATTYPE>(1,0);
        const MATTYPE h22 = H21.at<MATTYPE>(1,1);
        const MATTYPE h23 = H21.at<MATTYPE>(1,2);
        const MATTYPE h31 = H21.at<MATTYPE>(2,0);
        const MATTYPE h32 = H21.at<MATTYPE>(2,1);
        const MATTYPE h33 = H21.at<MATTYPE>(2,2);

        const MATTYPE h11inv = H12.at<MATTYPE>(0,0);
        const MATTYPE h12inv = H12.at<MATTYPE>(0,1);
        const MATTYPE h13inv = H12.at<MATTYPE>(0,2);
        const MATTYPE h21inv = H12.at<MATTYPE>(1,0);
        const MATTYPE h22inv = H12.at<MATTYPE>(1,1);
        const MATTYPE h23inv = H12.at<MATTYPE>(1,2);
        const MATTYPE h31inv = H12.at<MATTYPE>(2,0);
        const MATTYPE h32inv = H12.at<MATTYPE>(2,1);
        const MATTYPE h33inv = H12.at<MATTYPE>(2,2);

        vbMatchesInliers.resize(N);

        float score = 0;
         //卡方 阈值
        const float th = CHITH;

        const float invSigmaSquare = 1.0/(sigma*sigma);

        for(int i=0; i<N; i++)
        {
            bool bIn = true;

            const cv::KeyPoint &kp1 = mpInput->_query[mvMatches12[i].first];
            const cv::KeyPoint &kp2 = mpInput->_train[mvMatches12[i].second];

            const float u1 = kp1.pt.x;
            const float v1 = kp1.pt.y;
            const float u2 = kp2.pt.x;
            const float v2 = kp2.pt.y;

            // Reprojection error in first image
            // x2in1 = H12*x2

            const MATTYPE w2in1inv = 1.0/(h31inv*u2+h32inv*v2+h33inv);
            const MATTYPE u2in1 = (h11inv*u2+h12inv*v2+h13inv)*w2in1inv;
            const MATTYPE v2in1 = (h21inv*u2+h22inv*v2+h23inv)*w2in1inv;

            const MATTYPE squareDist1 = (u1-u2in1)*(u1-u2in1)+(v1-v2in1)*(v1-v2in1);

            const MATTYPE chiSquare1 = squareDist1*invSigmaSquare;

            if(chiSquare1>th)
                bIn = false;
            else
                score += th - chiSquare1;

            // Reprojection error in second image
            // x1in2 = H21*x1

            const MATTYPE w1in2inv = 1.0/(h31*u1+h32*v1+h33);
            const MATTYPE u1in2 = (h11*u1+h12*v1+h13)*w1in2inv;
            const MATTYPE v1in2 = (h21*u1+h22*v1+h23)*w1in2inv;

            const MATTYPE squareDist2 = (u2-u1in2)*(u2-u1in2)+(v2-v1in2)*(v2-v1in2);

            const MATTYPE chiSquare2 = squareDist2*invSigmaSquare;

            if(chiSquare2>th)
                bIn = false;
            else
                score += th - chiSquare2;

            if(bIn)
                vbMatchesInliers[i]=true;
            else
                vbMatchesInliers[i]=false;
        }

        return score;
    }

    float ORBPoseSolver::CheckFundamental(const cv::Mat &F21, BolVector &vbMatchesInliers, float sigma)
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

        const float th = 3.841;
        
        const float thScore = CHITH;

        const float invSigmaSquare = 1.0/(sigma*sigma);

        for(int i=0; i<N; i++)
        {
            bool bIn = true;

            const cv::KeyPoint &kp1 = mpInput->_query[mvMatches12[i].first];
            const cv::KeyPoint &kp2 = mpInput->_train[mvMatches12[i].second];

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

            if(chiSquare2>th)
                bIn = false;
            else
                score += thScore - chiSquare2;

            if(bIn)
                vbMatchesInliers[i]=true;
            else
                vbMatchesInliers[i]=false;
        }

        return score;
    }

    void ORBPoseSolver::Normalize(const KeyPtVector &vKeys, vector<cv::Point2f> &vNormalizedPoints, cv::Mat &T)
    {
        float meanX = 0;
        float meanY = 0;
        const int N = vKeys.size();

        vNormalizedPoints.resize(N);

        for(int i=0; i<N; i++)
        {
            meanX += vKeys[i].pt.x;
            meanY += vKeys[i].pt.y;
        }

        meanX = meanX/N;
        meanY = meanY/N;

        float meanDevX = 0;
        float meanDevY = 0;

        for(int i=0; i<N; i++)
        {
            vNormalizedPoints[i].x = vKeys[i].pt.x - meanX;
            vNormalizedPoints[i].y = vKeys[i].pt.y - meanY;

            meanDevX += fabs(vNormalizedPoints[i].x);
            meanDevY += fabs(vNormalizedPoints[i].y);
        }

        meanDevX = meanDevX/N;
        meanDevY = meanDevY/N;

        float sX = 1.0/meanDevX;
        float sY = 1.0/meanDevY;

        for(int i=0; i<N; i++)
        {
            vNormalizedPoints[i].x = vNormalizedPoints[i].x * sX;
            vNormalizedPoints[i].y = vNormalizedPoints[i].y * sY;
        }

        T = cv::Mat::eye(3,3,MATCVTYPE);
        T.at<MATTYPE>(0,0) = sX;
        T.at<MATTYPE>(1,1) = sY;
        T.at<MATTYPE>(0,2) = -meanX*sX;
        T.at<MATTYPE>(1,2) = -meanY*sY;
    }



    int ORBPoseSolver::CheckRT(const cv::Mat &R, const cv::Mat &t, const KeyPtVector &vKeys1, const KeyPtVector &vKeys2,
                                    const MatchPairs &vMatches12, BolVector &vbMatchesInliers,
                                    const cv::Mat &K, Pt3Vector &vP3D, float th2, BolVector &vbGood, float &parallax)
    {
         const float thparallax = 1;//0.99998;//大概1°  点到两帧光心夹角最小阈值
        // Calibration parameters
        const MATTYPE fx = K.at<MATTYPE>(0,0);
        const MATTYPE fy = K.at<MATTYPE>(1,1);
        const MATTYPE cx = K.at<MATTYPE>(0,2);
        const MATTYPE cy = K.at<MATTYPE>(1,2);

        vbGood = BolVector(vKeys1.size(),false);
        vP3D.resize(vKeys1.size());

        FloatVector vCosParallax;
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

        for(size_t i=0, iend=vMatches12.size();i<iend;i++)
        {
            if(!vbMatchesInliers[i])
                continue;

            const cv::KeyPoint &kp1 = vKeys1[vMatches12[i].first];
            const cv::KeyPoint &kp2 = vKeys2[vMatches12[i].second];
            cv::Mat p3dC1;

            PUtils::Triangulate(kp1.pt,kp2.pt,P1,P2,p3dC1);

            if(!isfinite(p3dC1.at<MATTYPE>(0)) || !isfinite(p3dC1.at<MATTYPE>(1)) || !isfinite(p3dC1.at<MATTYPE>(2)))
            {
                vbGood[vMatches12[i].first]=false;
                continue;
            }

            // Check parallax
            cv::Mat normal1 = p3dC1 - O1;
            MATTYPE dist1 = cv::norm(normal1);

            cv::Mat normal2 = p3dC1 - O2;
            MATTYPE dist2 = cv::norm(normal2);

            MATTYPE cosParallax = normal1.dot(normal2)/(dist1*dist2);

            // Check depth in front of first camera (only if enough parallax, as "infinite" points can easily go to negative depth)
            if(p3dC1.at<MATTYPE>(2)<=0 && cosParallax<thparallax)
                continue;

            // Check depth in front of second camera (only if enough parallax, as "infinite" points can easily go to negative depth)
            cv::Mat p3dC2 = R*p3dC1+t;

            if(p3dC2.at<MATTYPE>(2)<=0 && cosParallax<thparallax)
                continue;

            // Check reprojection error in first image
            MATTYPE im1x, im1y;
            MATTYPE invZ1 = 1.0/p3dC1.at<MATTYPE>(2);
            im1x = fx*p3dC1.at<MATTYPE>(0)*invZ1+cx;
            im1y = fy*p3dC1.at<MATTYPE>(1)*invZ1+cy;

            MATTYPE squareError1 = (im1x-kp1.pt.x)*(im1x-kp1.pt.x)+(im1y-kp1.pt.y)*(im1y-kp1.pt.y);

            if(squareError1>th2)
                continue;

            // Check reprojection error in second image
            MATTYPE im2x, im2y;
            MATTYPE invZ2 = 1.0/p3dC2.at<MATTYPE>(2);
            im2x = fx*p3dC2.at<MATTYPE>(0)*invZ2+cx;
            im2y = fy*p3dC2.at<MATTYPE>(1)*invZ2+cy;

            MATTYPE squareError2 = (im2x-kp2.pt.x)*(im2x-kp2.pt.x)+(im2y-kp2.pt.y)*(im2y-kp2.pt.y);

            if(squareError2>th2)
                continue;

            vCosParallax.push_back(cosParallax);
            vP3D[vMatches12[i].first] = cv::Point3f(p3dC1.at<MATTYPE>(0),p3dC1.at<MATTYPE>(1),p3dC1.at<MATTYPE>(2));
            nGood++;

            if(cosParallax<thparallax)
                vbGood[vMatches12[i].first]=true;
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

    //从基础矩阵恢复位姿
    bool ORBPoseSolver::ReconstructF(BolVector &vbMatchesInliers, cv::Mat &F21, cv::Mat &K,
                  cv::Mat &R21, cv::Mat &t21, Pt3Vector &vP3D, BolVector &vbTriangulated, float minParallax, int minTriangulated)
    {
        assert(mpInput);
        int N=0;
        for(size_t i=0, iend = vbMatchesInliers.size() ; i<iend; i++)
            if(vbMatchesInliers[i])
                N++;

        // Compute Essential Matrix from Fundamental Matrix
        cv::Mat E21 = K.t()*F21*K;

        cv::Mat R1, R2, t;

        // Recover the 4 motion hypotheses
        DecomposeE(E21,R1,R2,t);  

        cv::Mat t1=t;
        cv::Mat t2=-t;

        // Reconstruct with the 4 hyphoteses and check
        vector<cv::Point3f> vP3D1, vP3D2, vP3D3, vP3D4;
        BolVector vbTriangulated1,vbTriangulated2,vbTriangulated3, vbTriangulated4;
        float parallax1,parallax2, parallax3, parallax4;
        
        int nGood1 = CheckRT(R1,t1,mpInput->_query,mpInput->_train,mvMatches12,vbMatchesInliers,K, vP3D1, 4 * mSigma2, vbTriangulated1, parallax1);
        int nGood2 = CheckRT(R2,t1,mpInput->_query,mpInput->_train,mvMatches12,vbMatchesInliers,K, vP3D2, 4 * mSigma2, vbTriangulated2, parallax2);
        int nGood3 = CheckRT(R1,t2,mpInput->_query,mpInput->_train,mvMatches12,vbMatchesInliers,K, vP3D3, 4 * mSigma2, vbTriangulated3, parallax3);
        int nGood4 = CheckRT(R2,t2,mpInput->_query,mpInput->_train,mvMatches12,vbMatchesInliers,K, vP3D4, 4 * mSigma2, vbTriangulated4, parallax4);

        int maxGood = max(nGood1,max(nGood2,max(nGood3,nGood4)));

        R21 = cv::Mat();
        t21 = cv::Mat();

        //取内部匹配点的一个阈值和设置的最小三角化点位最小好点数量
        int nMinGood = max(static_cast<int>(0.8*N),minTriangulated);

        int nsimilar = 0;
        const float fsigma = 0.75;
        if(nGood1> fsigma * maxGood)
            nsimilar++;
        if(nGood2> fsigma * maxGood)
            nsimilar++;
        if(nGood3> fsigma * maxGood)
            nsimilar++;
        if(nGood4> fsigma * maxGood)
            nsimilar++;

        // If there is not a clear winner or not enough triangulated points reject initialization
        if(maxGood < nMinGood || nsimilar>1)
        {
            PROMTD_V("similar", nsimilar,"max < min :",maxGood, nMinGood);
            return false;
        }

        // If best reconstruction has enough parallax initialize
        if(maxGood==nGood1)
        {
            if(parallax1>=minParallax)
            {
                vP3D = vP3D1;
                vbTriangulated = vbTriangulated1;

                R1.copyTo(R21);
                t1.copyTo(t21);
                return true;
            }
        }
        else if(maxGood==nGood2)
        {
            if(parallax2>=minParallax)
            {
                vP3D = vP3D2;
                vbTriangulated = vbTriangulated2;

                R2.copyTo(R21);
                t1.copyTo(t21);
                return true;
            }
        }else if(maxGood==nGood3)
        {
            if(parallax3>=minParallax)
            {
                vP3D = vP3D3;
                vbTriangulated = vbTriangulated3;

                R1.copyTo(R21);
                t2.copyTo(t21);
                return true;
            }
        }
        else if(maxGood==nGood4)
        {
            if(parallax4>=minParallax)
            {
                vP3D = vP3D4;
                vbTriangulated = vbTriangulated4;

                R2.copyTo(R21);
                t2.copyTo(t21);
                return true;
            }
        }
        else
        {
            ;
        }  

        return false;
    }

    //从单应矩阵恢复位姿
    bool ORBPoseSolver::ReconstructH(BolVector &vbMatchesInliers, cv::Mat &H21, cv::Mat &K,
                  cv::Mat &R21, cv::Mat &t21, Pt3Vector &vP3D, BolVector &vbTriangulated, float minParallax, int minTriangulated)
    {
        assert(mpInput);
        int N=0;
        for(size_t i=0, iend = vbMatchesInliers.size() ; i<iend; i++)
            if(vbMatchesInliers[i])
                N++;

        // We recover 8 motion hypotheses using the method of Faugeras et al.
        // Motion and structure from motion in a piecewise planar environment.
        // International Journal of Pattern Recognition and Artificial Intelligence, 1988

        cv::Mat invK = K.inv();
        cv::Mat A = invK*H21*K;

        cv::Mat U,w,Vt,V;
        cv::SVD::compute(A,w,U,Vt,cv::SVD::FULL_UV);
        V=Vt.t();

        MATTYPE s = cv::determinant(U)*cv::determinant(Vt);

        MATTYPE d1 = w.at<MATTYPE>(0);
        MATTYPE d2 = w.at<MATTYPE>(1);
        MATTYPE d3 = w.at<MATTYPE>(2);

        if(d1/d2<1.00001 || d2/d3<1.00001)
        {
            return false;
        }

        vector<cv::Mat> vR, vt, vn;
        vR.reserve(8);
        vt.reserve(8);
        vn.reserve(8);

        //n'=[x1 0 x3] 4 posibilities e1=e3=1, e1=1 e3=-1, e1=-1 e3=1, e1=e3=-1
        MATTYPE aux1 = sqrt((d1*d1-d2*d2)/(d1*d1-d3*d3));
        MATTYPE aux3 = sqrt((d2*d2-d3*d3)/(d1*d1-d3*d3));
        MATTYPE x1[] = {aux1,aux1,-aux1,-aux1};
        MATTYPE x3[] = {aux3,-aux3,aux3,-aux3};

        //case d'=d2
        MATTYPE aux_stheta = sqrt((d1*d1-d2*d2)*(d2*d2-d3*d3))/((d1+d3)*d2);

        MATTYPE ctheta = (d2*d2+d1*d3)/((d1+d3)*d2);
        MATTYPE stheta[] = {aux_stheta, -aux_stheta, -aux_stheta, aux_stheta};

        for(int i=0; i<4; i++)
        {
            cv::Mat Rp=cv::Mat::eye(3,3,MATCVTYPE);
            Rp.at<MATTYPE>(0,0)=ctheta;
            Rp.at<MATTYPE>(0,2)=-stheta[i];
            Rp.at<MATTYPE>(2,0)=stheta[i];
            Rp.at<MATTYPE>(2,2)=ctheta;

            cv::Mat R = s*U*Rp*Vt;
            vR.push_back(R);

            cv::Mat tp(3,1,MATCVTYPE);
            tp.at<MATTYPE>(0)=x1[i];
            tp.at<MATTYPE>(1)=0;
            tp.at<MATTYPE>(2)=-x3[i];
            tp*=d1-d3;

            cv::Mat t = U*tp;
            vt.push_back(t/cv::norm(t));

            cv::Mat np(3,1,MATCVTYPE);
            np.at<MATTYPE>(0)=x1[i];
            np.at<MATTYPE>(1)=0;
            np.at<MATTYPE>(2)=x3[i];

            cv::Mat n = V*np;
            if(n.at<MATTYPE>(2)<0)
                n=-n;
            vn.push_back(n);
        }

        //case d'=-d2
        MATTYPE aux_sphi = sqrt((d1*d1-d2*d2)*(d2*d2-d3*d3))/((d1-d3)*d2);

        MATTYPE cphi = (d1*d3-d2*d2)/((d1-d3)*d2);
        MATTYPE sphi[] = {aux_sphi, -aux_sphi, -aux_sphi, aux_sphi};

        for(int i=0; i<4; i++)
        {
            cv::Mat Rp=cv::Mat::eye(3,3,MATCVTYPE);
            Rp.at<MATTYPE>(0,0)=cphi;
            Rp.at<MATTYPE>(0,2)=sphi[i];
            Rp.at<MATTYPE>(1,1)=-1;
            Rp.at<MATTYPE>(2,0)=sphi[i];
            Rp.at<MATTYPE>(2,2)=-cphi;

            cv::Mat R = s*U*Rp*Vt;
            vR.push_back(R);

            cv::Mat tp(3,1,MATCVTYPE);
            tp.at<MATTYPE>(0)=x1[i];
            tp.at<MATTYPE>(1)=0;
            tp.at<MATTYPE>(2)=x3[i];
            tp*=d1+d3;

            cv::Mat t = U*tp;
            vt.push_back(t/cv::norm(t));

            cv::Mat np(3,1,MATCVTYPE);
            np.at<MATTYPE>(0)=x1[i];
            np.at<MATTYPE>(1)=0;
            np.at<MATTYPE>(2)=x3[i];

            cv::Mat n = V*np;
            if(n.at<MATTYPE>(2)<0)
                n=-n;
            vn.push_back(n);
        }


        int bestGood = 0;
        int secondBestGood = 0;    
        int bestSolutionIdx = -1;
        float bestParallax = -1;
        vector<cv::Point3f> bestP3D;
        BolVector bestTriangulated;

        // Instead of applying the visibility constraints proposed in the Faugeras' paper (which could fail for points seen with low parallax)
        // We reconstruct all hypotheses and check in terms of triangulated points and parallax
        for(size_t i=0; i<8; i++)
        {
            float parallaxi;
            vector<cv::Point3f> vP3Di;
            BolVector vbTriangulatedi;

            int nGood = CheckRT(vR[i],vt[i],mpInput->_query,mpInput->_train,mvMatches12,vbMatchesInliers,K,vP3Di, mSigma2, vbTriangulatedi, parallaxi);

            if(nGood>bestGood)
            {
                secondBestGood = bestGood;
                bestGood = nGood;
                bestSolutionIdx = i;
                bestParallax = parallaxi;
                bestP3D = vP3Di;
                bestTriangulated = vbTriangulatedi;
            }
            else if(nGood>secondBestGood)
            {
                secondBestGood = nGood;
            }
        }


        if( /*secondBestGood<0.75*bestGood && */ bestParallax>=minParallax && bestGood>minTriangulated && bestGood>0.9*N)
        {
            vR[bestSolutionIdx].copyTo(R21);
            vt[bestSolutionIdx].copyTo(t21);
            vP3D = bestP3D;
            vbTriangulated = bestTriangulated;

            return true;
        }

        return false;
    }

    //单应矩阵计算
    cv::Mat ORBPoseSolver::ComputeH21(const PtVector&vP1, const PtVector &vP2)
    {
        const int N = vP1.size();

        cv::Mat A(2*N,9,MATCVTYPE);

        for(int i=0; i<N; i++)
        {
            const float u1 = vP1[i].x;
            const float v1 = vP1[i].y;
            const float u2 = vP2[i].x;
            const float v2 = vP2[i].y;

            A.at<MATTYPE>(2*i,0) = 0.0;
            A.at<MATTYPE>(2*i,1) = 0.0;
            A.at<MATTYPE>(2*i,2) = 0.0;
            A.at<MATTYPE>(2*i,3) = -u1;
            A.at<MATTYPE>(2*i,4) = -v1;
            A.at<MATTYPE>(2*i,5) = -1;
            A.at<MATTYPE>(2*i,6) = v2*u1;
            A.at<MATTYPE>(2*i,7) = v2*v1;
            A.at<MATTYPE>(2*i,8) = v2;

            A.at<MATTYPE>(2*i+1,0) = u1;
            A.at<MATTYPE>(2*i+1,1) = v1;
            A.at<MATTYPE>(2*i+1,2) = 1;
            A.at<MATTYPE>(2*i+1,3) = 0.0;
            A.at<MATTYPE>(2*i+1,4) = 0.0;
            A.at<MATTYPE>(2*i+1,5) = 0.0;
            A.at<MATTYPE>(2*i+1,6) = -u2*u1;
            A.at<MATTYPE>(2*i+1,7) = -u2*v1;
            A.at<MATTYPE>(2*i+1,8) = -u2;
        }

        cv::Mat u,w,vt;

        cv::SVDecomp(A,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

        return vt.row(8).reshape(0, 3);
    }

    //基础矩阵计算
    cv::Mat ORBPoseSolver::ComputeF21(const PtVector &vP1, const PtVector &vP2)
    {
        const int N = vP1.size();

        cv::Mat A(N,9,MATCVTYPE);

        for(int i=0; i<N; i++)
        {
            const float u1 = vP1[i].x;
            const float v1 = vP1[i].y;
            const float u2 = vP2[i].x;
            const float v2 = vP2[i].y;

            A.at<MATTYPE>(i,0) = u2*u1;
            A.at<MATTYPE>(i,1) = u2*v1;
            A.at<MATTYPE>(i,2) = u2;
            A.at<MATTYPE>(i,3) = v2*u1;
            A.at<MATTYPE>(i,4) = v2*v1;
            A.at<MATTYPE>(i,5) = v2;
            A.at<MATTYPE>(i,6) = u1;
            A.at<MATTYPE>(i,7) = v1;
            A.at<MATTYPE>(i,8) = 1;
        }

        cv::Mat u,w,vt;

        cv::SVDecomp(A,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

        cv::Mat Fpre = vt.row(8).reshape(0, 3);

        cv::SVDecomp(Fpre,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

        w.at<MATTYPE>(2)=0;

        return  u*cv::Mat::diag(w)*vt;
    }

    //分解
    void ORBPoseSolver::DecomposeE(const cv::Mat &E, cv::Mat &R1, cv::Mat &R2, cv::Mat &t)
    {
        cv::Mat u,w,vt;
        cv::SVD::compute(E,w,u,vt);
        u.col(2).copyTo(t);
        t=t/cv::norm(t);
        cv::Mat W(3,3,MATCVTYPE,cv::Scalar(0));
        W.at<MATTYPE>(0,1)=-1;
        W.at<MATTYPE>(1,0)=1;
        W.at<MATTYPE>(2,2)=1;
        R1 = u*W*vt;
        if(cv::determinant(R1)<0)
            R1=-R1;
        R2 = u*W.t()*vt;
        if(cv::determinant(R2)<0)
            R2=-R2;
    } 

    void ORBPoseSolver::initParams(const MatchVector &matches)
    {
        if(matches.empty())
            return;

        mvMatches12.clear();

        const int N = matches.size();
        mvMatches12.reserve(N);

        for(auto item : matches)
        {
            mvMatches12.push_back(std::make_pair(item.queryIdx,item.trainIdx));
        }

        // Indices for minimum set selection
        SzVector vAllIndices;
        vAllIndices.reserve(N);
        SzVector vAvailableIndices;      
        for(int i=0; i<N; i++)
        {
            vAllIndices.push_back(i);
        }  
        // Generate sets of 8 points for each RANSAC iteration
        mvSets = vector< SzVector >(mMaxIterations,SzVector(8,0));

        Random::Init();
        // DUtils::Random::SeedRandOnce(0);

        for(int it=0; it<mMaxIterations; it++)
        {
            vAvailableIndices = vAllIndices;

            // Select a minimum set
            for(size_t j=0; j<8; j++)
            {
                int randi = Random::RandomInt(0,vAvailableIndices.size()-1);
                int idx = vAvailableIndices[randi];

                mvSets[it][j] = idx;

                vAvailableIndices[randi] = vAvailableIndices.back();
                vAvailableIndices.pop_back();
            }
        }
    }

    //估计
    PoseResult ORBPoseSolver::estimate(const InputPair &input)
    {
        mpInput = &input;

        PoseResult result;
        if(input._match.size() < 8)
        {
            return result;
        }

        bool bol = false;
        const int maxInterator = 10;//最大迭代次数
        int interator = 0;
        while(!bol)
        {
            if(interator++ > maxInterator)
                break;
            initParams(input._match);
            // Launch threads to compute in parallel a fundamental matrix and a homography
            BolVector vbMatchesInliersH, vbMatchesInliersF;
            float SH, SF;
            cv::Mat H, F;

            // thread threadH(&ORBPoseSolver::FindHomography,this,ref(vbMatchesInliersH), ref(SH), ref(H));
            thread threadF(&ORBPoseSolver::FindFundamental,this,ref(vbMatchesInliersF), ref(SF), ref(F));

            // Wait until both threads have finished
            // threadH.join();
            threadF.join();

            // Compute ratio of scores
            float RH = SH/(SH+SF);

            const float minParallax = 0.00;//最小的时差角度
            const float minTriangle = 30; //最少需要多少个点 三角化


            BolVector bTriangle;
            // Try to reconstruct from homography or fundamental depending on the ratio (0.40-0.45)
            // if(RH > 0.45)
            //     bol = ReconstructH(vbMatchesInliersH,H,mCam.K,R,t,vPts,bTriangle,minParallax,minTriangle);
            // else //if(pF_HF>0.6)
                bol = ReconstructF(vbMatchesInliersF,F,mCam.K,result._R,result._t,result._vpts,bTriangle,minParallax,minTriangle);

            PROMTD_V("reconstruct >> ",RH, "Ret : ", bol);

            if(bol)
            {//剔除三角化失败的点
                result._match.reserve(input._match.size());
                MatchVector::const_iterator it = input._match.begin();
                for(;it !=  input._match.end();++it)
                {
                    if(!bTriangle[it->queryIdx])
                    {
                        result._match.emplace_back(*it);
                    }
                }
                
#if 0
                //save epline 
                Mat out = Position::PUtils::DrawKeyPoints( mCur->getData()->_img,mCur->getKeys());
                MATTYPE a,b,c;
                for(size_t i = 0; i < matches.size(); ++i)
                {
                    const Point2f prept = mPre->getKeys()[matches[i].queryIdx].pt;
                    const Point2f curpt = mCur->getKeys()[matches[i].trainIdx].pt;
              
                    Position::EpLine epline = Position::PUtils::ComputeEpLine(F,prept);
                    
                    Position::PUtils::DrawEpiLine(epline,curpt, out);
                }
                std::string outpath = GETCFGVALUE(GETGLOBALCONFIG(),OutPath,string);
                imwrite( outpath + "/orbepline.jpg",out);
                cout << "F" << endl;
#endif
            }
        }
        return result;
    }

#pragma endregion 

#pragma region CVPoseSolver

    //推算位姿
    PoseResult CVPoseSolver::estimate(const InputPair &input)
    {
        PoseResult result;
        
        const size_t len = input._match.size();
        if(len < 8)
        {
            return result;
        }

        PtVector prePts;
        PtVector curPts;
        prePts.reserve(len);
        curPts.reserve(len);
        mvMatches12.clear();
        for(size_t i = 0; i < len;++i)
        {
            prePts.emplace_back(input._query[input._match[i].queryIdx].pt);
            curPts.emplace_back(input._train[input._match[i].trainIdx].pt);
            mvMatches12.push_back(make_pair(input._match[i].queryIdx,input._match[i].trainIdx));
        }
        
        //三角化的点要与第一帧的特征数一致
        result._vpts.resize(input._query.size());
        Mat mask;
        Mat E = findEssentialMat(prePts, curPts, mCam.K, CV_FM_8POINT,
                             0.999, 1.0, mask);
        recoverPose(E, prePts, curPts, mCam.K, result._R, result._t, mask);

        Mat K1 = (Mat_<MATTYPE>(3,4) << 1,0,0,0,
                                        0,1,0,0,
                                        0,0,1,0);

        Mat K2 = (Mat_<MATTYPE>(3,4) <<
                                        result._R.at<MATTYPE>(0,0),result._R.at<MATTYPE>(0,1),result._R.at<MATTYPE>(0,2),result._t.at<MATTYPE>(0,0) ,
                                        result._R.at<MATTYPE>(1,0),result._R.at<MATTYPE>(1,1),result._R.at<MATTYPE>(1,2),result._t.at<MATTYPE>(1,0) ,
                                        result._R.at<MATTYPE>(2,0),result._R.at<MATTYPE>(2,1),result._R.at<MATTYPE>(2,2),result._t.at<MATTYPE>(2,0) );

        Mat out;
        vector<Point2d> pts_1, pts_2;
        pts_1.reserve(prePts.size());
        pts_2.reserve(curPts.size());
        for(int i = 0;i < prePts.size(); ++i)
        {
            pts_1.emplace_back(PUtils::Pixel2Cam(prePts[i],mCam.K));
            pts_2.emplace_back(PUtils::Pixel2Cam(curPts[i],mCam.K));
        }
        cv::triangulatePoints(K1,K2,pts_1,pts_2,out);
        
        BolVector bols;
        bols.resize(result._vpts.size());

        for(size_t i = 0; i < prePts.size(); ++i)
        {
            Mat x = out.col(i);
            x = x/x.at<MATTYPE>(3,0);
            if(x.at<MATTYPE>(2) < 0)
            {//剔除负点
                bols[ mvMatches12[i].first ] = true;
                continue;
            }
            result._vpts[ mvMatches12[i].first ] = (Point3f(x.at<MATTYPE>(0,0),x.at<MATTYPE>(1,0),x.at<MATTYPE>(2,0)));
        }

        MatchVector::const_iterator it = input._match.begin();
        result._match.reserve(input._match.size());
        for(;it != input._match.end();++it)
        {//剔除错误点

            if(!bols[it->queryIdx] )
            {
               result._match.emplace_back(*it);
            }
        }

        return result;
    }

#pragma endregion

}
