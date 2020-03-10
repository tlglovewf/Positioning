#include "P_ORBMapPoint.h"
#include "P_ORBmatcher.h"
#include "P_ORBMap.h"
#include<mutex>

namespace Position
{

u64 ORBMapPoint::nNextId=0;
mutex ORBMapPoint::mGlobalMutex;

ORBMapPoint::ORBMapPoint(const cv::Mat &Pos, ORBKeyFrame *pRefKF, IMap* pMap):
    mnFirstKFid(pRefKF->mnId), mnFirstFrame(pRefKF->mnFrameId), nObs(0), mnTrackReferenceForFrame(0),
    mnLastFrameSeen(0), mnBALocalForKF(0), mnFuseCandidateForKF(0), mnLoopPointForKF(0), mnCorrectedByKF(0),
    mnCorrectedReference(0), mnBAGlobalForKF(0), mpRefKF(pRefKF), mnVisible(1), mnFound(1), mbBad(false),
    mpReplaced(static_cast<ORBMapPoint*>(NULL)), mfMinDistance(0), mfMaxDistance(0), mpMap(pMap)
{
    Pos.copyTo(mWorldPos);
    mNormalVector = cv::Mat::zeros(3,1,CV_32F);

    // MapPoints can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    unique_lock<mutex> lock( ORBMAP(mpMap)->mMutexPointCreation);
    mnId=nNextId++;
}

void ORBMapPoint::setWorldPos(const cv::Mat &Pos)
{
    unique_lock<mutex> lock2(mGlobalMutex);
    unique_lock<mutex> lock(mMutexPos);
    Pos.copyTo(mWorldPos);
}

cv::Mat ORBMapPoint::getWorldPos()
{
    unique_lock<mutex> lock(mMutexPos);
    return mWorldPos.clone();
}

cv::Mat ORBMapPoint::normal()
{
    unique_lock<mutex> lock(mMutexPos);
    return mNormalVector.clone();
}

ORBKeyFrame* ORBMapPoint::GetReferenceKeyFrame()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mpRefKF;
}

void ORBMapPoint::addObservation(IKeyFrame* pKF, int idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    if(mObservations.count(pKF))
        return;
    mObservations[pKF]=idx;
    nObs++;
}

void ORBMapPoint::rmObservation(IKeyFrame *pKF)
{
    bool bBad=false;
    {
        unique_lock<mutex> lock(mMutexFeatures);
        if(mObservations.count(pKF))
        {
            int idx = mObservations[pKF];
           
            nObs--;

            mObservations.erase(pKF);

            if(mpRefKF==pKF)
                mpRefKF = ORBKEYFRAME(mObservations.begin()->first);

            // If only 2 observations or less, discard point
            if(nObs<=2)
                bBad=true;
        }
    }

    if(bBad)
        setBadFlag();
}

KeyFrameMap ORBMapPoint::getObservations()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mObservations;
}

int ORBMapPoint::observations()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return nObs;
}

void ORBMapPoint::setBadFlag()
{
    KeyFrameMap obs;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        mbBad=true;
        obs = mObservations;
        mObservations.clear();
    }
    for(KeyFrameMap::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        IKeyFrame* pKF = mit->first;
        pKF->rmMapPoint(mit->second);
    }

    mpMap->rmMapPoint(this);
}

ORBMapPoint* ORBMapPoint::GetReplaced()
{
    unique_lock<mutex> lock1(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    return mpReplaced;
}

void ORBMapPoint::Replace(ORBMapPoint* pMP)
{
    if(pMP->mnId==this->mnId)
        return;

    int nvisible, nfound;
    KeyFrameMap obs;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        obs=mObservations;
        mObservations.clear();
        mbBad=true;
        nvisible = mnVisible;
        nfound = mnFound;
        mpReplaced = pMP;
    }

    for(KeyFrameMap::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        // Replace measurement in keyframe
        ORBKeyFrame* pKF =  ORBKEYFRAME(mit->first);

        if(!pMP->isInFrame(pKF))
        {
            pKF->ReplaceMapPointMatch(mit->second, pMP);
            pMP->addObservation(pKF,mit->second);
        }
        else
        {
            pKF->rmMapPoint(mit->second);
        }
    }
    pMP->IncreaseFound(nfound);
    pMP->IncreaseVisible(nvisible);
    pMP->ComputeDistinctiveDescriptors();

    mpMap->rmMapPoint(this);
}

bool ORBMapPoint::isBad()
{
    unique_lock<mutex> lock(mMutexFeatures);
    unique_lock<mutex> lock2(mMutexPos);
    return mbBad;
}

void ORBMapPoint::IncreaseVisible(int n)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mnVisible+=n;
}

void ORBMapPoint::IncreaseFound(int n)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mnFound+=n;
}

float ORBMapPoint::GetFoundRatio()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return static_cast<float>(mnFound)/mnVisible;
}

void ORBMapPoint::ComputeDistinctiveDescriptors()
{
    // Retrieve all observed descriptors
    vector<cv::Mat> vDescriptors;

    KeyFrameMap observations;

    {
        unique_lock<mutex> lock1(mMutexFeatures);
        if(mbBad)
            return;
        observations=mObservations;
    }

    if(observations.empty())
        return;

    vDescriptors.reserve(observations.size());

    for(KeyFrameMap::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        ORBKeyFrame* pKF = ORBKEYFRAME(mit->first);

        if(!pKF->isBad())
            vDescriptors.push_back(pKF->mDescriptors.row(mit->second));
    }

    if(vDescriptors.empty())
        return;

    // Compute distances between them
    const size_t N = vDescriptors.size();

    float Distances[N][N];
    for(size_t i=0;i<N;i++)
    {
        Distances[i][i]=0;
        for(size_t j=i+1;j<N;j++)
        {
            int distij = ORBmatcher::DescriptorDistance(vDescriptors[i],vDescriptors[j]);
            Distances[i][j]=distij;
            Distances[j][i]=distij;
        }
    }

    // Take the descriptor with least median distance to the rest
    int BestMedian = INT_MAX;
    int BestIdx = 0;
    for(size_t i=0;i<N;i++)
    {
        vector<int> vDists(Distances[i],Distances[i]+N);
        sort(vDists.begin(),vDists.end());
        int median = vDists[0.5*(N-1)];

        if(median<BestMedian)
        {
            BestMedian = median;
            BestIdx = i;
        }
    }

    {
        unique_lock<mutex> lock(mMutexFeatures);
        mDescriptor = vDescriptors[BestIdx].clone();
    }
}

cv::Mat ORBMapPoint::GetDescriptor()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mDescriptor.clone();
}

int ORBMapPoint::getIndexInKeyFrame(IKeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexFeatures);
    if(mObservations.count(pKF))
        return mObservations[pKF];
    else
        return -1;
}

bool ORBMapPoint::isInFrame(IKeyFrame *pKF) 
{
    unique_lock<mutex> lock(mMutexFeatures);
    return (mObservations.count(pKF));
}
//更新深度和法线
void ORBMapPoint::UpdateNormalAndDepth()
{
    KeyFrameMap observations;
    ORBKeyFrame* pRefKF;
    cv::Mat Pos;
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        if(mbBad)
            return;
        observations=mObservations;
        pRefKF=mpRefKF;
        Pos = mWorldPos.clone();
    }

    if(observations.empty())
        return;

    cv::Mat normal = cv::Mat::zeros(3,1,CV_32F);
    int n=0;
    for(KeyFrameMap::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
    {
        ORBKeyFrame* pKF = ORBKEYFRAME(mit->first);
        cv::Mat Owi = pKF->GetCameraCenter();
        cv::Mat normali = mWorldPos - Owi;
        normal = normal + normali/cv::norm(normali);
        n++;
    }

    cv::Mat PC = Pos - pRefKF->GetCameraCenter();
    const float dist = cv::norm(PC);
    const int level = pRefKF->mvKeysUn[observations[pRefKF]].octave;
    const float levelScaleFactor =  pRefKF->mvScaleFactors[level];
    const int nLevels = pRefKF->mnScaleLevels;

    {
        unique_lock<mutex> lock3(mMutexPos);
        mfMaxDistance = dist*levelScaleFactor;
        mfMinDistance = mfMaxDistance/pRefKF->mvScaleFactors[nLevels-1];
        mNormalVector = normal/n;
    }
}

float ORBMapPoint::GetMinDistanceInvariance()
{
    unique_lock<mutex> lock(mMutexPos);
    return 0.8f*mfMinDistance;
}

float ORBMapPoint::GetMaxDistanceInvariance()
{
    unique_lock<mutex> lock(mMutexPos);
    return 1.2f*mfMaxDistance;
}

int ORBMapPoint::PredictScale(const float &currentDist, ORBKeyFrame* pKF)
{
    float ratio;
    {
        unique_lock<mutex> lock(mMutexPos);
        ratio = mfMaxDistance/currentDist;
    }

    int nScale = ceil(log(ratio)/pKF->mfLogScaleFactor);
    if(nScale<0)
        nScale = 0;
    else if(nScale>=pKF->mnScaleLevels)
        nScale = pKF->mnScaleLevels-1;

    return nScale;
}

int ORBMapPoint::PredictScale(const float &currentDist, ORBFrame* pF)
{
    float ratio;
    {
        unique_lock<mutex> lock(mMutexPos);
        ratio = mfMaxDistance/currentDist;
    }

    int nScale = ceil(log(ratio)/pF->mfLogScaleFactor);
    if(nScale<0)
        nScale = 0;
    else if(nScale>=pF->mnScaleLevels)
        nScale = pF->mnScaleLevels-1;

    return nScale;
}



} 
