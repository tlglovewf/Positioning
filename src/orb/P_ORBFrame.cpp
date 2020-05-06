#include "P_ORBFrame.h"
#include "P_ORBMapPoint.h"
#include "P_Converter.h"
#include "P_ORBmatcher.h"
#include "P_SemanticGraph.h"

namespace Position
{
    u64 ORBFrame::nNextId=0;
    bool ORBFrame::mbInitialComputations=true;
    float ORBFrame::cx, ORBFrame::cy, ORBFrame::fx, ORBFrame::fy, ORBFrame::invfx, ORBFrame::invfy;
    float ORBFrame::mnMinX, ORBFrame::mnMinY, ORBFrame::mnMaxX, ORBFrame::mnMaxY;
    float ORBFrame::mfGridElementWidthInv, ORBFrame::mfGridElementHeightInv;

    ORBFrame::ORBFrame()
    {}

    //Copy Constructor
    ORBFrame::ORBFrame(const ORBFrame &frame)
        :mpORBvocabulary(frame.mpORBvocabulary), mpORBextractorLeft(frame.mpORBextractorLeft),
        mData(frame.mData), mK(frame.mK.clone()), mDistCoef(frame.mDistCoef.clone()),
        N(frame.N), mvKeys(frame.mvKeys),
        mvKeysUn(frame.mvKeysUn), mBowVec(frame.mBowVec), mFeatVec(frame.mFeatVec),
        mDescriptors(frame.mDescriptors.clone()),
        mvpMapPoints(frame.mvpMapPoints), mvbOutlier(frame.mvbOutlier), mnId(frame.mnId),
        mpReferenceKF(frame.mpReferenceKF), mnScaleLevels(frame.mnScaleLevels),
        mfScaleFactor(frame.mfScaleFactor), mfLogScaleFactor(frame.mfLogScaleFactor),
        mvScaleFactors(frame.mvScaleFactors), mvInvScaleFactors(frame.mvInvScaleFactors),
        mvLevelSigma2(frame.mvLevelSigma2), mvInvLevelSigma2(frame.mvInvLevelSigma2)
    {
        for(int i=0;i<FRAME_GRID_COLS;i++)
            for(int j=0; j<FRAME_GRID_ROWS; j++)
                mGrid[i][j]=frame.mGrid[i][j];

        if(!frame.mTcw.empty())
            setPose(frame.mTcw);
    }

    ORBFrame::ORBFrame(FrameData *data, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef)
        :mpORBvocabulary(voc),mpORBextractorLeft(extractor),
        mData(std::move(data)), mK(K.clone()),mDistCoef(distCoef.clone())
    {
        // Frame ID
        mnId=nNextId++;

        // Scale Level Info
        mnScaleLevels = mpORBextractorLeft->GetLevels();
        mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
        mfLogScaleFactor = log(mfScaleFactor);
        mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
        mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
        mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
        mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

        // ORB extraction
        ExtractORB(0,mData->_img);

        N = mvKeys.size();

        if(mvKeys.empty())
            return;

        UndistortKeyPoints();

        mvpMapPoints = MapPtVector(N,NULL);
        mvbOutlier = U8Vector(N,false);

        // This is done only for the first Frame (or after a change in the calibration)
        if(mbInitialComputations)
        {
            ComputeImageBounds(mData->_img);

            mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
            mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);

            fx = K.at<MATTYPE>(0,0);
            fy = K.at<MATTYPE>(1,1);
            cx = K.at<MATTYPE>(0,2);
            cy = K.at<MATTYPE>(1,2);
            invfx = 1.0f/fx;
            invfy = 1.0f/fy;

            mbInitialComputations=false;
        }

        AssignFeaturesToGrid();
    }

    void ORBFrame::AssignFeaturesToGrid()
    {
        int nReserve = 0.5f*N/(FRAME_GRID_COLS*FRAME_GRID_ROWS);
        for(unsigned int i=0; i<FRAME_GRID_COLS;i++)
            for (unsigned int j=0; j<FRAME_GRID_ROWS;j++)
                mGrid[i][j].reserve(nReserve);

        for(int i=0;i<N;i++)
        {
            const cv::KeyPoint &kp = mvKeysUn[i];

            int nGridPosX, nGridPosY;
            if(PosInGrid(kp,nGridPosX,nGridPosY))
                mGrid[nGridPosX][nGridPosY].push_back(i);
        }
    }

    void ORBFrame::ExtractORB(int flag, const cv::Mat &im)
    {
        (*mpORBextractorLeft)(im,cv::Mat(),mvKeys,mDescriptors);
    }

    void ORBFrame::setPose(const cv::Mat &pose)
    {
        mTcw = pose.clone();
        UpdatePoseMatrices();
    }

    void ORBFrame::UpdatePoseMatrices()
    { 
        mRcw = mTcw.rowRange(0,3).colRange(0,3);
        mRwc = mRcw.t();
        mtcw = mTcw.rowRange(0,3).col(3);
        mOw = -mRcw.t()*mtcw;
    }

    bool ORBFrame::isInFrustum(ORBMapPoint *pMP, float viewingCosLimit)
    {
        pMP->mbTrackInView = false;

        // 3D in absolute coordinates
        cv::Mat P = pMP->getWorldPos(); 

        // 3D in camera coordinates
        const cv::Mat Pc = mRcw*P+mtcw;
        const MATTYPE &PcX = Pc.at<MATTYPE>(0);
        const MATTYPE &PcY= Pc.at<MATTYPE>(1);
        const MATTYPE &PcZ = Pc.at<MATTYPE>(2);

        // Check positive depth
        if(PcZ<0.0f)
            return false;

        // Project in image and check it is not outside
        const MATTYPE invz = 1.0f/PcZ;
        const MATTYPE u=fx*PcX*invz+cx;
        const MATTYPE v=fy*PcY*invz+cy;

        if(u<mnMinX || u>mnMaxX)
            return false;
        if(v<mnMinY || v>mnMaxY)
            return false;

        // Check distance is in the scale invariance region of the ORBMapPoint
        const float maxDistance = pMP->GetMaxDistanceInvariance();
        const float minDistance = pMP->GetMinDistanceInvariance();
        //帧到地图点向量
        const cv::Mat PO = P-mOw;
        const float dist = cv::norm(PO);

        //判断点距帧的距离 在尺度不变性的范围内
        if(dist<minDistance || dist>maxDistance)
            return false;

    // Check viewing angle
        cv::Mat Pn = pMP->normal();
        //计算向量夹角
        const float viewCos = PO.dot(Pn)/dist;
        //夹角> 60度
        if(viewCos<viewingCosLimit)
            return false;

        // Predict scale in the image
        const int nPredictedLevel = pMP->PredictScale(dist,this);

        // Data used by the tracking
        pMP->mbTrackInView = true;
        pMP->mTrackProjX = u;
        pMP->mTrackProjY = v;
        pMP->mnTrackScaleLevel= nPredictedLevel;
        pMP->mTrackViewCos = viewCos;

        return true;
    }

    SzVector ORBFrame::GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel, const int maxLevel) const
    {
        SzVector vIndices;
        vIndices.reserve(N);

        const int nMinCellX = max(0,(int)floor((x-mnMinX-r)*mfGridElementWidthInv));
        if(nMinCellX>=FRAME_GRID_COLS)
            return vIndices;

        const int nMaxCellX = min((int)FRAME_GRID_COLS-1,(int)ceil((x-mnMinX+r)*mfGridElementWidthInv));
        if(nMaxCellX<0)
            return vIndices;

        const int nMinCellY = max(0,(int)floor((y-mnMinY-r)*mfGridElementHeightInv));
        if(nMinCellY>=FRAME_GRID_ROWS)
            return vIndices;

        const int nMaxCellY = min((int)FRAME_GRID_ROWS-1,(int)ceil((y-mnMinY+r)*mfGridElementHeightInv));
        if(nMaxCellY<0)
            return vIndices;

        const bool bCheckLevels = (minLevel>0) || (maxLevel>=0);

        for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
        {
            for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
            {
                const SzVector vCell = mGrid[ix][iy];
                if(vCell.empty())
                    continue;

                for(size_t j=0, jend=vCell.size(); j<jend; j++)
                {
                    const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];

                    if(SemanticGraph::Instance()->isDyobj(kpUn.pt,this->getData()->_name))
                        continue;

                    if(bCheckLevels)
                    {
                        if(kpUn.octave<minLevel)
                            continue;
                        if(maxLevel>=0)
                            if(kpUn.octave>maxLevel)
                                continue;
                    }

                    const float distx = kpUn.pt.x-x;
                    const float disty = kpUn.pt.y-y;

                    if(fabs(distx)<r && fabs(disty)<r)
                        vIndices.push_back(vCell[j]);
                }
            }
        }

        return vIndices;
    }

    bool ORBFrame::PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY)
    {
        posX = round((kp.pt.x-mnMinX)*mfGridElementWidthInv);
        posY = round((kp.pt.y-mnMinY)*mfGridElementHeightInv);

        //Keypoint's coordinates are undistorted, which could cause to go out of the image
        if(posX<0 || posX>=FRAME_GRID_COLS || posY<0 || posY>=FRAME_GRID_ROWS)
            return false;

        return true;
    }


    void ORBFrame::ComputeBoW()
    {
        if(mBowVec.empty())
        {
            vector<cv::Mat> vCurrentDesc = PConverter::toDescriptorVector(mDescriptors);
            mpORBvocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4);
        }
    }

    void ORBFrame::UndistortKeyPoints()
    {
        if( mDistCoef.empty() || mDistCoef.at<MATTYPE>(0)==0.0)
        {
            mvKeysUn=mvKeys;
            return;
        }

        // Fill matrix with points
        cv::Mat mat(N,2,MATCVTYPE);
        for(int i=0; i<N; i++)
        {
            mat.at<MATTYPE>(i,0)=mvKeys[i].pt.x;
            mat.at<MATTYPE>(i,1)=mvKeys[i].pt.y;
        }

        // Undistort points
        mat=mat.reshape(2);
        cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
        mat=mat.reshape(1);

        // Fill undistorted keypoint vector
        mvKeysUn.resize(N);
        for(int i=0; i<N; i++)
        {
            cv::KeyPoint kp = mvKeys[i];
            kp.pt.x=mat.at<MATTYPE>(i,0);
            kp.pt.y=mat.at<MATTYPE>(i,1);
            mvKeysUn[i]=kp;
        }
    }

    void ORBFrame::ComputeImageBounds(const cv::Mat &imLeft)
    {
        if(mDistCoef.empty() || mDistCoef.at<MATTYPE>(0)!=0.0)
        {
            cv::Mat mat(4,2,MATCVTYPE);
            mat.at<MATTYPE>(0,0)=0.0; mat.at<MATTYPE>(0,1)=0.0;
            mat.at<MATTYPE>(1,0)=imLeft.cols; mat.at<MATTYPE>(1,1)=0.0;
            mat.at<MATTYPE>(2,0)=0.0; mat.at<MATTYPE>(2,1)=imLeft.rows;
            mat.at<MATTYPE>(3,0)=imLeft.cols; mat.at<MATTYPE>(3,1)=imLeft.rows;

            // Undistort corners
            mat=mat.reshape(2);
            cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
            mat=mat.reshape(1);

            mnMinX = min(mat.at<MATTYPE>(0,0),mat.at<MATTYPE>(2,0));
            mnMaxX = max(mat.at<MATTYPE>(1,0),mat.at<MATTYPE>(3,0));
            mnMinY = min(mat.at<MATTYPE>(0,1),mat.at<MATTYPE>(1,1));
            mnMaxY = max(mat.at<MATTYPE>(2,1),mat.at<MATTYPE>(3,1));

        }
        else
        {
            mnMinX = 0.0f;
            mnMaxX = imLeft.cols;
            mnMinY = 0.0f;
            mnMaxY = imLeft.rows;
        }
    }
} 
