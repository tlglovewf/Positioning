
#include "P_ORBKeyFrame.h"
#include "P_Converter.h"
#include "P_ORBmatcher.h"
#include "P_ORBFeature.h"
#include "P_ORBFrame.h"
#include "P_ORBKeyFrameDatabase.h"


namespace Position
{

    u64 ORBKeyFrame::nNextId=0;

    ORBKeyFrame::ORBKeyFrame(ORBFrame &F, const std::shared_ptr<IMap>& pMap, ORBKeyFrameDatabase *pKFDB):
        mnFrameId(F.mnId),  mTimeStamp(F.mTimeStamp), mnGridCols(FRAME_GRID_COLS), mnGridRows(FRAME_GRID_ROWS),
        mfGridElementWidthInv(F.mfGridElementWidthInv), mfGridElementHeightInv(F.mfGridElementHeightInv),
        mnTrackReferenceForFrame(0), mnFuseTargetForKF(0), mnBALocalForKF(0), mnBAFixedForKF(0),
        mnLoopQuery(0), mnLoopWords(0), mnRelocQuery(0), mnRelocWords(0), mnBAGlobalForKF(0),
        fx(F.fx), fy(F.fy), cx(F.cx), cy(F.cy), invfx(F.invfx), invfy(F.invfy),
        N(F.N), mvKeys(F.mvKeys), mvKeysUn(F.mvKeysUn),
        mDescriptors(F.mDescriptors.clone()),
        mBowVec(F.mBowVec), mFeatVec(F.mFeatVec), mnScaleLevels(F.mnScaleLevels), mfScaleFactor(F.mfScaleFactor),
        mfLogScaleFactor(F.mfLogScaleFactor), mvScaleFactors(F.mvScaleFactors), mvLevelSigma2(F.mvLevelSigma2),
        mvInvLevelSigma2(F.mvInvLevelSigma2), mnMinX(F.mnMinX), mnMinY(F.mnMinY), mnMaxX(F.mnMaxX),
        mnMaxY(F.mnMaxY), mK(F.mK), mvpMapPoints(F.mvpMapPoints), mpKeyFrameDB(pKFDB),
        mpORBvocabulary(F.mpORBvocabulary), mbFirstConnection(true), mpParent(NULL),mpPrev(NULL),mpNext(NULL),mbNotErase(false),
        mbToBeErased(false), mbBad(false), mpMap(pMap)
    {
        mnId=nNextId++;

        mGrid.resize(mnGridCols);
        for(int i=0; i<mnGridCols;i++)
        {
            mGrid[i].resize(mnGridRows);
            for(int j=0; j<mnGridRows; j++)
                mGrid[i][j] = F.mGrid[i][j];
        }

        setPose(F.mTcw);    
    }

    void ORBKeyFrame::ComputeBoW()
    {
        if(mBowVec.empty() || mFeatVec.empty())
        {
            vector<cv::Mat> vCurrentDesc = PConverter::toDescriptorVector(mDescriptors);
            // Feature vector associate features with nodes in the 4th level (from leaves up)
            // We assume the vocabulary tree has 6 levels, change the 4 otherwise
            mpORBvocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4);
        }
    }

    void ORBKeyFrame::setPose(const cv::Mat &Tcw_)
    {
        unique_lock<mutex> lock(mMutexPose);
        Tcw_.copyTo(Tcw);
        cv::Mat Rcw = Tcw.rowRange(0,3).colRange(0,3);
        cv::Mat tcw = Tcw.rowRange(0,3).col(3);
        cv::Mat Rwc = Rcw.t();
        Ow = -Rwc*tcw;

        Twc = cv::Mat::eye(4,4,Tcw.type());
        Rwc.copyTo(Twc.rowRange(0,3).colRange(0,3));
        Ow.copyTo(Twc.rowRange(0,3).col(3));
    }

    cv::Mat ORBKeyFrame::getPose()
    {
        unique_lock<mutex> lock(mMutexPose);
        return Tcw.clone();
    }

    cv::Mat ORBKeyFrame::GetPoseInverse()
    {
        unique_lock<mutex> lock(mMutexPose);
        return Twc.clone();
    }

    cv::Mat ORBKeyFrame::GetCameraCenter()
    {
        unique_lock<mutex> lock(mMutexPose);
        return Ow.clone();
    }


    cv::Mat ORBKeyFrame::getRotation()
    {
        unique_lock<mutex> lock(mMutexPose);
        return Tcw.rowRange(0,3).colRange(0,3).clone();
    }
    cv::Mat ORBKeyFrame::getTranslation()
    {
        unique_lock<mutex> lock(mMutexPose);
        return Tcw.rowRange(0,3).col(3).clone();
    }
    //新增关联, weight共视点数量
    void ORBKeyFrame::AddConnection(ORBKeyFrame *pKF, const int &weight)
    {
        {
            unique_lock<mutex> lock(mMutexConnections);
            if(!mConnectedKeyFrameWeights.count(pKF))
                mConnectedKeyFrameWeights[pKF]=weight;
            else if(mConnectedKeyFrameWeights[pKF]!=weight)
                mConnectedKeyFrameWeights[pKF]=weight;
            else
                return;
        }

        UpdateBestCovisibles();
    }

    //更新最佳共视关系
    void ORBKeyFrame::UpdateBestCovisibles()
    {
        unique_lock<mutex> lock(mMutexConnections);
        vector<pair<int,ORBKeyFrame*> > vPairs;
        vPairs.reserve(mConnectedKeyFrameWeights.size());
        for(KeyFrameMap::iterator mit=mConnectedKeyFrameWeights.begin(), mend=mConnectedKeyFrameWeights.end(); mit!=mend; mit++)
        vPairs.push_back(make_pair(mit->second,ORBKEYFRAME(mit->first)));
        //根据权值排序
        sort(vPairs.begin(),vPairs.end());
        list<ORBKeyFrame*> lKFs;
        list<int> lWs;
        for(size_t i=0, iend=vPairs.size(); i<iend;i++)
        {
            lKFs.push_front(vPairs[i].second);
            lWs.push_front(vPairs[i].first);
        }

        mvpOrderedConnectedKeyFrames = vector<ORBKeyFrame*>(lKFs.begin(),lKFs.end());
        mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());    
    }

    set<ORBKeyFrame*> ORBKeyFrame::GetConnectedKeyFrames()
    {
        unique_lock<mutex> lock(mMutexConnections);
        set<ORBKeyFrame*> s;
        for(KeyFrameMap::iterator mit=mConnectedKeyFrameWeights.begin();mit!=mConnectedKeyFrameWeights.end();mit++)
            s.insert(ORBKEYFRAME(mit->first));
        return s;
    }

    vector<ORBKeyFrame*> ORBKeyFrame::GetVectorCovisibleKeyFrames()
    {
        unique_lock<mutex> lock(mMutexConnections);
        return mvpOrderedConnectedKeyFrames;
    }

    vector<ORBKeyFrame*> ORBKeyFrame::GetBestCovisibilityKeyFrames(const int &N)
    {
        unique_lock<mutex> lock(mMutexConnections);
        if((int)mvpOrderedConnectedKeyFrames.size()<N)
            return mvpOrderedConnectedKeyFrames;
        else
            return vector<ORBKeyFrame*>(mvpOrderedConnectedKeyFrames.begin(),mvpOrderedConnectedKeyFrames.begin()+N);

    }

    vector<ORBKeyFrame*> ORBKeyFrame::GetCovisiblesByWeight(const int &w)
    {
        unique_lock<mutex> lock(mMutexConnections);

        if(mvpOrderedConnectedKeyFrames.empty())
            return vector<ORBKeyFrame*>();

        vector<int>::iterator it = upper_bound(mvOrderedWeights.begin(),mvOrderedWeights.end(),w,ORBKeyFrame::weightComp);
        if(it==mvOrderedWeights.end())
            return vector<ORBKeyFrame*>();
        else
        {
            int n = it-mvOrderedWeights.begin();
            return vector<ORBKeyFrame*>(mvpOrderedConnectedKeyFrames.begin(), mvpOrderedConnectedKeyFrames.begin()+n);
        }
    }

    int ORBKeyFrame::GetWeight(ORBKeyFrame *pKF)
    {
        unique_lock<mutex> lock(mMutexConnections);
        if(mConnectedKeyFrameWeights.count(pKF))
            return mConnectedKeyFrameWeights[pKF];
        else
            return 0;
    }

    void ORBKeyFrame::addMapPoint(IMapPoint *pMP, int idx)
    {
        unique_lock<mutex> lock(mMutexFeatures);
        mvpMapPoints[idx]=pMP;
    }

    void ORBKeyFrame::rmMapPoint(int idx)
    {
        unique_lock<mutex> lock(mMutexFeatures);
        if(idx < 0)
            return;
        mvpMapPoints[idx]= NULL;
    }

    bool ORBKeyFrame::hasMapPoint(int index)
    {
        unique_lock<mutex> lock(mMutexFeatures);
        if(index < 0)
            return false;
        return  NULL != mvpMapPoints[index];
    }

    void ORBKeyFrame::rmMapPoint(IMapPoint* pMP)
    {
        unique_lock<mutex> lock(mMutexFeatures);
        int idx = pMP->getIndexInKeyFrame(this);
        if(idx>=0)
            mvpMapPoints[idx]= NULL;
    }


    void ORBKeyFrame::ReplaceMapPointMatch(const size_t &idx, ORBMapPoint* pMP)
    {
        mvpMapPoints[idx]=pMP;
    }

    MapPtSet ORBKeyFrame::GetMapPoints()
    {
        unique_lock<mutex> lock(mMutexFeatures);    
        MapPtSet s;
        for(size_t i=0, iend=mvpMapPoints.size(); i<iend; i++)
        {
            if(!mvpMapPoints[i])
                continue;
            IMapPoint* pMP = mvpMapPoints[i];
            if(!pMP->isBad())
                s.insert(pMP);
        }
        return s;
    }

    int ORBKeyFrame::TrackedMapPoints(const int &minObs)
    {
        unique_lock<mutex> lock(mMutexFeatures);

        int nPoints=0;
        const bool bCheckObs = minObs>0;
        for(int i=0; i<N; i++)
        {
            IMapPoint* pMP = mvpMapPoints[i];
            if(pMP)
            {
                if(!pMP->isBad())
                {
                    if(bCheckObs)
                    {
                        if(mvpMapPoints[i]->observations()>=minObs)
                            nPoints++;
                    }
                    else
                        nPoints++;
                }
            }
        }

        return nPoints;
    }

    const MapPtVector& ORBKeyFrame::getPoints()
    {
        unique_lock<mutex> lock(mMutexFeatures);
        return mvpMapPoints;
    }

    ORBMapPoint* ORBKeyFrame::GetMapPoint(const size_t &idx)
    {
        unique_lock<mutex> lock(mMutexFeatures);
        return ORBMAPPOINT(mvpMapPoints[idx]);
    }

    void ORBKeyFrame::UpdateConnections()
    {
        KeyFrameMap KFcounter;// <帧,与当前帧共视点数量>

        unique_lock<mutex> lockMPs(mMutexFeatures);
        const MapPtVector &vpMP = mvpMapPoints;

        //For all map points in keyframe check in which other keyframes are they seen
        //Increase counter for those keyframes
        for(MapPtVector::const_iterator vit=vpMP.begin(), vend=vpMP.end(); vit!=vend; vit++)
        {
            IMapPoint* pMP = *vit;

            if(!pMP)
                continue;

            if(pMP->isBad())
                continue;

            const KeyFrameMap& observations = pMP->getObservations();

            for(KeyFrameMap::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
            {
                if(mit->first->index()==mnId)
                    continue;
                KFcounter[mit->first]++;
            }
        }

        // This should not happen
        if(KFcounter.empty())
            return;

        //If the counter is greater than threshold add connection
        //In case no keyframe counter is over threshold add the one with maximum counter
        int nmax=0;
        ORBKeyFrame* pKFmax=NULL;
        int th = 15;//共视点数量 筛选阈值

        vector<pair<int,ORBKeyFrame*> > vPairs;
        vPairs.reserve(KFcounter.size());
        for(KeyFrameMap::iterator mit=KFcounter.begin(), mend=KFcounter.end(); mit!=mend; mit++)
        {
            ORBKeyFrame *orbkey = ORBKEYFRAME(mit->first);
            if(mit->second>nmax)
            {
                nmax=mit->second;
                pKFmax = orbkey;
            }
            if(mit->second>=th)
            {
                vPairs.push_back(make_pair(mit->second, orbkey));
                orbkey->AddConnection(this,mit->second);
            }
        }
        //共视帧 的共视点数量小于阈值,则加入共视点数量最多的一个帧建立关联
        if(vPairs.empty())
        {
            vPairs.push_back(make_pair(nmax,pKFmax));
            pKFmax->AddConnection(this,nmax);
        }
        //根据观测数量排序 默认 升序排列
        sort(vPairs.begin(),vPairs.end());
        list<ORBKeyFrame*> lKFs;
        list<int> lWs;
        for(size_t i=0; i<vPairs.size();i++)
        {
            lKFs.push_front(vPairs[i].second);
            lWs.push_front(vPairs[i].first);
        }

        {
            unique_lock<mutex> lockCon(mMutexConnections);

            mConnectedKeyFrameWeights = KFcounter;
            mvpOrderedConnectedKeyFrames = vector<ORBKeyFrame*>(lKFs.begin(),lKFs.end());
            mvOrderedWeights = vector<int>(lWs.begin(), lWs.end());

            if(mbFirstConnection && mnId!=0)
            {//首次建立关联,设置共视点数最多的一帧为父节点
                mpParent = mvpOrderedConnectedKeyFrames.front();
                mpParent->AddChild(this);
                mbFirstConnection = false;
            }

        }
    }

    void ORBKeyFrame::AddChild(ORBKeyFrame *pKF)
    {
        unique_lock<mutex> lockCon(mMutexConnections);
        mspChildrens.insert(pKF);
    }

    void ORBKeyFrame::EraseChild(ORBKeyFrame *pKF)
    {
        unique_lock<mutex> lockCon(mMutexConnections);
        mspChildrens.erase(pKF);
    }

    void ORBKeyFrame::ChangeParent(ORBKeyFrame *pKF)
    {
        unique_lock<mutex> lockCon(mMutexConnections);
        mpParent = pKF;
        pKF->AddChild(this);
    }

    const set<ORBKeyFrame*>& ORBKeyFrame::GetChilds()
    {
        unique_lock<mutex> lockCon(mMutexConnections);
        return mspChildrens;
    }

    ORBKeyFrame* ORBKeyFrame::GetParent()
    {
        unique_lock<mutex> lockCon(mMutexConnections);
        return mpParent;
    }

    //获取到下一帧
    IKeyFrame* ORBKeyFrame::getNext()
    {
        return mpNext;   
    }
    //获取上一帧
    IKeyFrame* ORBKeyFrame::getPrev()
    {
        return mpPrev;
    }

    bool ORBKeyFrame::hasChild(ORBKeyFrame *pKF)
    {
        unique_lock<mutex> lockCon(mMutexConnections);
        return mspChildrens.count(pKF);
    }

    void ORBKeyFrame::AddLoopEdge(ORBKeyFrame *pKF)
    {
        unique_lock<mutex> lockCon(mMutexConnections);
        mbNotErase = true;
        mspLoopEdges.insert(pKF);
    }

    set<ORBKeyFrame*> ORBKeyFrame::GetLoopEdges()
    {
        unique_lock<mutex> lockCon(mMutexConnections);
        return mspLoopEdges;
    }

    void ORBKeyFrame::SetNotErase()
    {
        unique_lock<mutex> lock(mMutexConnections);
        mbNotErase = true;
    }

    void ORBKeyFrame::SetErase()
    {
        {
            unique_lock<mutex> lock(mMutexConnections);
            if(mspLoopEdges.empty())
            {
                mbNotErase = false;
            }
        }

        if(mbToBeErased)
        {
            setBadFlag();
        }
    }

    void ORBKeyFrame::setBadFlag()
    {   
        unique_lock<mutex> lock(mMutexConnections);
        if(mnId==0)
            return;
        else if(mbNotErase)
        {
            mbToBeErased = true;
            return;
        }
        //抹除关联帧 关于次帧的连接
        for(KeyFrameMap::iterator mit = mConnectedKeyFrameWeights.begin(), mend=mConnectedKeyFrameWeights.end(); mit!=mend; mit++)
        ORBKEYFRAME(mit->first)->EraseConnection(this);

        for(size_t i=0; i<mvpMapPoints.size(); i++)
            if(mvpMapPoints[i])
                mvpMapPoints[i]->rmObservation(this);

        {
            unique_lock<mutex> lock(mMutexConnections);
            unique_lock<mutex> lock1(mMutexFeatures);

            mConnectedKeyFrameWeights.clear();
            mvpOrderedConnectedKeyFrames.clear();

            // Update Spanning Tree
            set<ORBKeyFrame*> sParentCandidates;
            sParentCandidates.insert(mpParent);

            // Assign at each iteration one children with a parent (the pair with highest covisibility weight)
            // Include that children as new parent candidate for the rest
            while(!mspChildrens.empty())
            {
                bool bContinue = false;

                int max = -1;
                ORBKeyFrame* pC;
                ORBKeyFrame* pP;

                for(set<ORBKeyFrame*>::iterator sit=mspChildrens.begin(), send=mspChildrens.end(); sit!=send; sit++)
                {
                    ORBKeyFrame* pKF = *sit;
                    if(pKF->isBad())
                        continue;
                    //获取子帧中 共视帧 与当前共父点的 更改父点(关联点数量最多的为父)
                    // Check if a parent candidate is connected to the keyframe
                    vector<ORBKeyFrame*> vpConnected = pKF->GetVectorCovisibleKeyFrames();
                    for(size_t i=0, iend=vpConnected.size(); i<iend; i++)
                    {
                        for(set<ORBKeyFrame*>::iterator spcit=sParentCandidates.begin(), spcend=sParentCandidates.end(); spcit!=spcend; spcit++)
                        {
                            if(vpConnected[i]->mnId == (*spcit)->mnId)
                            {
                                int w = pKF->GetWeight(vpConnected[i]);
                                if(w>max)
                                {
                                    pC = pKF;
                                    pP = vpConnected[i];
                                    max = w;
                                    bContinue = true;
                                }
                            }
                        }
                    }
                }

                if(bContinue)
                {
                    pC->ChangeParent(pP);
                    sParentCandidates.insert(pC);
                    mspChildrens.erase(pC);
                }
                else
                    break;
            }

            // If a children has no covisibility links with any parent candidate, assign to the original parent of this KF
            if(!mspChildrens.empty())
                for(set<ORBKeyFrame*>::iterator sit=mspChildrens.begin(); sit!=mspChildrens.end(); sit++)
                {
                    (*sit)->ChangeParent(mpParent);
                }

            mpParent->EraseChild(this);
            mTcp = Tcw*mpParent->GetPoseInverse();
            mbBad = true;
        }


        mpMap->rmKeyFrame(this);
        mpKeyFrameDB->erase(this);
    }

    bool ORBKeyFrame::isBad()
    {
        unique_lock<mutex> lock(mMutexConnections);
        return mbBad;
    }

    void ORBKeyFrame::EraseConnection(ORBKeyFrame* pKF)
    {
        bool bUpdate = false;
        {
            unique_lock<mutex> lock(mMutexConnections);
            if(mConnectedKeyFrameWeights.count(pKF))
            {
                mConnectedKeyFrameWeights.erase(pKF);
                bUpdate=true;
            }
        }

        if(bUpdate)
            UpdateBestCovisibles();
    }

    SzVector ORBKeyFrame::GetFeaturesInArea(const float &x, const float &y, const float &r) const
    {
        SzVector vIndices;
        vIndices.reserve(N);

        const int nMinCellX = max(0,(int)floor((x-mnMinX-r)*mfGridElementWidthInv));
        if(nMinCellX>=mnGridCols)
            return vIndices;

        const int nMaxCellX = min((int)mnGridCols-1,(int)ceil((x-mnMinX+r)*mfGridElementWidthInv));
        if(nMaxCellX<0)
            return vIndices;

        const int nMinCellY = max(0,(int)floor((y-mnMinY-r)*mfGridElementHeightInv));
        if(nMinCellY>=mnGridRows)
            return vIndices;

        const int nMaxCellY = min((int)mnGridRows-1,(int)ceil((y-mnMinY+r)*mfGridElementHeightInv));
        if(nMaxCellY<0)
            return vIndices;

        for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
        {
            for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
            {
                const SzVector vCell = mGrid[ix][iy];
                for(size_t j=0, jend=vCell.size(); j<jend; j++)
                {
                    const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
                    const float distx = kpUn.pt.x-x;
                    const float disty = kpUn.pt.y-y;

                    if(fabs(distx)<r && fabs(disty)<r)
                        vIndices.push_back(vCell[j]);
                }
            }
        }

        return vIndices;
    }

    bool ORBKeyFrame::IsInImage(const float &x, const float &y) const
    {
        return (x>=mnMinX && x<mnMaxX && y>=mnMinY && y<mnMaxY);
    }

    float ORBKeyFrame::ComputeSceneMedianDepth(const int q)
    {
        MapPtVector vpMapPoints;
        cv::Mat Tcw_;
        {
            unique_lock<mutex> lock(mMutexFeatures);
            unique_lock<mutex> lock2(mMutexPose);
            vpMapPoints = mvpMapPoints;
            Tcw_ = Tcw.clone();
        }

        FloatVector vDepths;
        vDepths.reserve(N);
        cv::Mat Rcw2 = Tcw_.row(2).colRange(0,3);
        Rcw2 = Rcw2.t();
        float zcw = Tcw_.at<MATTYPE>(2,3);
        for(int i=0; i<N; i++)
        {
            if(mvpMapPoints[i])
            {
                IMapPoint* pMP = mvpMapPoints[i];
                cv::Mat x3Dw = pMP->getWorldPos();
                float z = Rcw2.dot(x3Dw)+zcw;
                vDepths.push_back(z);
            }
        }

        sort(vDepths.begin(),vDepths.end());

        return vDepths[(vDepths.size()-1)/q];
    }

}
