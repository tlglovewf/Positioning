#include "P_Frame.h"
#include "P_MapPoint.h"
#include "P_IOHelper.h"

namespace Position
{
#pragma region  FrameHelper
int    FrameHelper::FRAME_GRID_ROWS = 48;
int    FrameHelper::FRAME_GRID_COLS = 64;
float  FrameHelper::mfGridElementWidthInv   = 0.0;
float  FrameHelper::mfGridElementHeightInv  = 0.0;
bool   FrameHelper::mInit = false;

    //初始化配置参数
    void FrameHelper::initParams(float width, float height)
    {
        if(!mInit)
        {
            mfGridElementWidthInv  = static_cast<float>(FRAME_GRID_COLS) / width;
            mfGridElementHeightInv = static_cast<float>(FRAME_GRID_ROWS) / height;
            mInit = true;
        }
    }

    //根据坐标 查询特征点序号
    SzVector FrameHelper::getFrameFeaturesInArea(IFrame *pframe,
                                               float x,float y,float r,
                                               int minLevel,int maxLevel)
    {
        assert(mInit);
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
                const SzVector &vCell = dynamic_cast<PFrame *>(pframe)->mGrid[ix][iy];
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


    //将特征点分grid
    void FrameHelper::assignFeaturesToGrid(IFrame *pframe)
    {
        assert(mInit);
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

    //构造函数
    PFrame::PFrame(FrameData *data,const std::shared_ptr<IFeature> &pFeature,int index):mData(data),mFeature(pFeature)
    {
        assert(pFeature.get());
        if(data->_img.empty())
        {
            LOG_CRIT("Frame Image Is Empty!!!");
            exit(-1);
        }
        FeatureInfo info(data->_name);
        pFeature->detect(*data,info);
        mKeypts     = std::move(info._keys);
        mDescript   = std::move(info._des);
        Position::FrameHelper::initParams(data->_img.cols,data->_img.rows);
        mN = mKeypts.size();
        FrameHelper::assignFeaturesToGrid(this);
        mbOutlier = U8Vector(mN,false);
        mIndex = index;
        mPose = Mat::eye(4,4,MATCVTYPE);
        mOw = Mat::zeros(4,1,MATCVTYPE);
    }
    PFrame::~PFrame()
    {
        // if(!mData->_img.empty())
        // {
        //     mData->_img.release();
        // }
    }
    PKeyFrame::PKeyFrame(IFrame *pframe,IKeyFrame *prevFrame,PMap *pMap):
    mpFrame(pframe),mpMap(pMap),mbBad(false)
    {
        mPts.resize(mpFrame->getKeySize());
    }
    //析构
    PKeyFrame::~PKeyFrame()
    {
        if(NULL != mpFrame)
        {
            for(auto item : mPts)
            {
                if(NULL != item)
                    item->rmObservation(this);
            }
            mpFrame->release();
        }   
    }
}

