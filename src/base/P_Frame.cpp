#include "P_Frame.h"
#include "P_MapPoint.h"
namespace Position
{
#pragma region  FrameGrid
int   FrameGrid::FRAME_GRID_ROWS = 48;
int   FrameGrid::FRAME_GRID_COLS = 64;
float FrameGrid::mfGridElementWidthInv   = 0.0;
float FrameGrid::mfGridElementHeightInv  = 0.0;

static cv::Mat s_K;

    //设置静态变量
    void SetStaticParams(const CameraParam &cam)
    {
        s_K = cam.K;
    }



    //根据坐标 查询特征点序号
    SzVector FrameGrid::getFrameFeaturesInArea(IFrame *pframe,
                                               float x,float y,float r,
                                               int minLevel,int maxLevel)
    {
        SzVector vIndices;
        vIndices.reserve(pframe->getKeySize());

        const int nMinCellX = max(0, (int)floor((x - r) * mfGridElementWidthInv));
        if (nMinCellX >= FRAME_GRID_COLS)
            return vIndices;

        const int nMaxCellX = min((int)FRAME_GRID_COLS - 1, (int)ceil((x + r) * mfGridElementWidthInv));
        if (nMaxCellX < 0)
            return vIndices;

        const int nMinCellY = max(0, (int)floor((y - r) * mfGridElementHeightInv));
        if (nMinCellY >= FRAME_GRID_ROWS)
            return vIndices;

        const int nMaxCellY = min((int)FRAME_GRID_ROWS - 1, (int)ceil((y + r) * mfGridElementHeightInv));
        if (nMaxCellY < 0)
            return vIndices;

        const bool bCheckLevels = (minLevel > 0) || (maxLevel >= 0);

        for (int ix = nMinCellX; ix <= nMaxCellX; ix++)
        {
            for (int iy = nMinCellY; iy <= nMaxCellY; iy++)
            {
                const vector<size_t> &vCell = dynamic_cast<PFrame *>(pframe)->mGrid[ix][iy];
                if (vCell.empty())
                    continue;

                for (size_t j = 0, jend = vCell.size(); j < jend; j++)
                {
                    const cv::KeyPoint &kpUn = pframe->getKeys()[vCell[j]];
                    if (bCheckLevels)
                    {
                        if (kpUn.octave < minLevel)
                            continue;
                        if (maxLevel >= 0)
                            if (kpUn.octave > maxLevel)
                                continue;
                    }

                    const float distx = kpUn.pt.x - x;
                    const float disty = kpUn.pt.y - y;

                    // if(fabs(distx)<r && fabs(disty)<r)
                    //     vIndices.push_back(vCell[j]);

                    float d = sqrt(distx * distx + disty * disty);

                    if (d < r)
                    {
                        vIndices.push_back(vCell[j]);
                    }
                }
            }
        }

        return vIndices;
    }

    //初始化配置参数
    void FrameGrid::initParams(float width, float height)
    {
        assert(width > 1.0 && height > 1.0);
        mfGridElementWidthInv  = static_cast<float>(FRAME_GRID_COLS) / width;
        mfGridElementHeightInv = static_cast<float>(FRAME_GRID_ROWS) / height;
    }

    //将特征点分grid
    void FrameGrid::assignFeaturesToGrid(IFrame *pframe)
    {
        assert(pframe);
        PFrame &frame = *dynamic_cast<PFrame*>(pframe);
        
        size_t nReserve = 0.5f * frame.mN / (FRAME_GRID_COLS * FRAME_GRID_ROWS);

        frame.mGrid.resize(FRAME_GRID_COLS);
        for( int i = 0; i < FRAME_GRID_COLS; ++i)
        {
            frame.mGrid[i].resize(FRAME_GRID_ROWS);
            for( int j = 0; j < FRAME_GRID_ROWS; ++j)
            {
                frame.mGrid[i][j].reserve(nReserve);
            }
        }

        for(int i = 0;i < frame.mN; ++i)
        {
            const cv::KeyPoint &kp = frame.mKeypts[i];

            int nGridPosX, nGridPoxY;
            if( PosInGrid(kp, nGridPosX, nGridPoxY))
            {
                frame.mGrid[nGridPosX][nGridPoxY].emplace_back(i);
            }
        }
    }
#pragma endregion


u64 PFrame::s_nIndexCount = 0;


    //构造函数
    PFrame::PFrame(const FrameData &data,const std::shared_ptr<IFeature> &pFeature,bool retainimg /* = false */):
        mbBad(false),mData(data),mFeature(pFeature)
    {
        assert(pFeature.get());
        pFeature->detect(data,mKeypts,mDescript);
        mN = mKeypts.size();
        mPts.resize(mN);
        mIndex = s_nIndexCount++;
    }
    PFrame::~PFrame()
    {
        if(!mData._img.empty())
        {
            mData._img.release();
        }
    }

    //判断点在帧视锥体中
    bool PFrame::isInFrustum(IMapPoint* pMP, float viewingCosLimit)
    {
        assert(pMP);
        // pMP->mbTrackInView = false;

        //get world pose 
        const cv::Mat& P = pMP->getPose();

        const cv::Mat Pc = mPose.rowRange(0,3).colRange(0,3) * P + mPose.rowRange(0,3).col(3);
       
        MATTYPE pcZ = Pc.at<MATTYPE>(2);

        //check positive depth
        if(pcZ < 0)
            return false;

        cv::Mat pixel = s_K * Pc;
        const MATTYPE pixZ = pixel.at<MATTYPE>(2);
        const MATTYPE u = pixel.at<MATTYPE>(0) / pixZ;
        const MATTYPE v = pixel.at<MATTYPE>(1) / pixZ;

        if(u < 0 || u > mData._img.cols)
            return false;
        if(v < 0 || v > mData._img.rows)
            return false;
        
        const float maxDistance = pMP->minDistance();
        const float minDistance = pMP->maxDistance();

        const cv::Mat PO = P - mWd;
        const float dist = cv::norm(PO);

        if( dist < minDistance || dist > maxDistance)
            return false;
        
        cv::Mat Pn = pMP->normal();
        
        const double viewCos = PO.dot(Pn) / dist;

        if(viewCos < viewingCosLimit)
            return false;


        // Data used by the tracking
        // pMP->mbTrackInView = true;
        // pMP->mTrackProjX = u;
        // pMP->mTrackProjXR = u - mbf*invz;
        // pMP->mTrackProjY = v;
        // pMP->mnTrackScaleLevel= nPredictedLevel;
        // pMP->mTrackViewCos = viewCos;
        return true;
    }

    //析构
    PKeyFrame::~PKeyFrame()
    {
        if(NULL != mpFrame)
        {
            for(auto item : mpFrame->getPoints())
            {
                item->rmObservation(mpFrame);
            }
            mpFrame->release();
        }   
    }
}

