#include "P_FeatureMatcher.h"
#include "P_Frame.h"
namespace Position
{
#define HISTO_LENGTH    30
#define TH_LOW          60  //50
#define TH_HIGH         120 //100

#define SEARCHLEVEL     0    //匹配搜索层级

    //计算orb描述子的汉明距离
    static int DescriptorDistance(const cv::Mat &a, const cv::Mat &b)
    {
        const int *pa = a.ptr<int32_t>();
        const int *pb = b.ptr<int32_t>();

        int dist = 0;

        for (int i = 0; i < 8; i++, pa++, pb++)
        {
            unsigned int v = *pa ^ *pb;
            v = v - ((v >> 1) & 0x55555555);
            v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
            dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
        }
        return dist;
    }

    static void ComputeThreeMaxima(IntVector *histo, const int L, int &ind1, int &ind2, int &ind3)
    {
        int max1 = 0;
        int max2 = 0;
        int max3 = 0;

        for (int i = 0; i < L; ++i)
        {
            const int s = histo[i].size();
            if (s > max1)
            {
                max3 = max2;
                max2 = max1;
                max1 = s;
                ind3 = ind2;
                ind2 = ind1;
                ind1 = i;
            }
            else if (s > max2)
            {
                max3 = max2;
                max2 = s;
                ind3 = ind2;
                ind2 = i;
            }
            else if (s > max3)
            {
                max3 = s;
                ind3 = i;
            }
        }

        if (max2 < 0.1f * (float)max1)
        {
            ind2 = -1;
            ind3 = -1;
        }
        else if (max3 < 0.1f * (float)max1)
        {
            ind3 = -1;
        }
    }

    //匹配  返回匹配对
    MatchVector PFeatureMatcher::match(IFrame *preframe, IFrame *curframe, int windowsize)
    {
        assert(preframe && curframe);
        int nmatches = 0;

        IntVector vnMatches12 = IntVector(preframe->getKeySize(), -1);

        IntVector rotHist[HISTO_LENGTH];
        for (int i = 0; i < HISTO_LENGTH; i++)
            rotHist[i].reserve(500);
        const float factor = 1.0f / HISTO_LENGTH;

        IntVector vMatchedDistance(curframe->getKeySize(), INT_MAX);
        IntVector vnMatches21(curframe->getKeySize(), -1);

        for (size_t i1 = 0, iend1 = preframe->getKeySize(); i1 < iend1; i1++)
        {
            cv::KeyPoint kp1 = preframe->getKeys()[i1];
            int level1 = kp1.octave;
            //匹配搜索层数
            if (level1 > SEARCHLEVEL)
                continue;

            //根据范围在帧中搜索
            SzVector vIndices2 = FrameHelper::getFrameFeaturesInArea( curframe, preframe->getKeys()[i1].pt.x, preframe->getKeys()[i1].pt.y, windowsize, level1, level1);

            if (vIndices2.empty())
                continue;

            cv::Mat d1 = preframe->getDescript().row(i1);

            int bestDist = INT_MAX;
            int bestDist2 = INT_MAX;
            int bestIdx2 = -1;

            for (SzVector::iterator vit = vIndices2.begin(); vit != vIndices2.end(); vit++)
            {
                size_t i2 = *vit;

                cv::Mat d2 = curframe->getDescript().row(i2);

                int dist = DescriptorDistance(d1, d2);

                if (vMatchedDistance[i2] <= dist)
                    continue;

                if (dist < bestDist)
                {
                    bestDist2 = bestDist;
                    bestDist = dist;
                    bestIdx2 = i2;
                }
                else if (dist < bestDist2)
                {
                    bestDist2 = dist;
                }
            }

            if (bestDist <= TH_LOW)
            {
                if (bestDist < (float)bestDist2 * mfNNratio)
                {
                    if (vnMatches21[bestIdx2] >= 0)
                    {
                        vnMatches12[vnMatches21[bestIdx2]] = -1;
                        nmatches--;
                    }
                    vnMatches12[i1] = bestIdx2;
                    vnMatches21[bestIdx2] = i1;
                    vMatchedDistance[bestIdx2] = bestDist;
                    nmatches++;

                    if (mbCheckOrientation)
                    {
                        float rot = preframe->getKeys()[i1].angle - curframe->getKeys()[bestIdx2].angle;
                        if (rot < 0.0)
                            rot += 360.0f;
                        int bin = round(rot * factor);
                        if (bin == HISTO_LENGTH)
                            bin = 0;
                        assert(bin >= 0 && bin < HISTO_LENGTH);
                        rotHist[bin].push_back(i1);
                    }
                }
            }
        }

        if (mbCheckOrientation)
        {
            int ind1 = -1;
            int ind2 = -1;
            int ind3 = -1;

            ComputeThreeMaxima(rotHist, HISTO_LENGTH, ind1, ind2, ind3);

            for (int i = 0; i < HISTO_LENGTH; i++)
            {
                if (i == ind1 || i == ind2 || i == ind3)
                    continue;
                for (size_t j = 0, jend = rotHist[i].size(); j < jend; j++)
                {
                    int idx1 = rotHist[i][j];
                    if (vnMatches12[idx1] >= 0)
                    {
                        vnMatches12[idx1] = -1;
                        nmatches--;
                    }
                }
            }
        }

        MatchVector matches;

        for (int i = 0; i < vnMatches12.size(); ++i)
        {
            if (vnMatches12[i] >= 0)
            {
                matches.push_back(cv::DMatch(i,vnMatches12[i], 0.0) );
            }
        }

        return matches;
    }

    //knn匹配
    void knn_match(const Mat &descriptor1,const Mat &descriptor2, const  cv::Ptr<DescriptorMatcher> &match,MatchVector &matches)
    {
        const float minRatio = 0.45; //
        const int k = 2;
        
        std::vector<std::vector<DMatch> > knnMatches;
        match->knnMatch(descriptor1, descriptor2, knnMatches, k);
        
        for (size_t i = 0; i < knnMatches.size(); i++) {
            const DMatch& bestMatch = knnMatches[i][0];
            const DMatch& betterMatch = knnMatches[i][1];
            
            float  distanceRatio = bestMatch.distance / betterMatch.distance;
            if (distanceRatio < minRatio)
                matches.push_back(bestMatch);
                
        }   
    }


    PKnnMatcher::PKnnMatcher():mMatcher(new cv::FlannBasedMatcher())
    {

    }

    //匹配  返回匹配对
    MatchVector PKnnMatcher::match(IFrame *preframe, IFrame *curframe, int windowsize)
    {
    assert(preframe);
        assert(curframe);
        
        const Mat &descriptorLeft = preframe->getDescript();
        const Mat &descriptorRight= curframe->getDescript();
        
        const KeyPtVector &prekey = preframe->getKeys();
        const KeyPtVector &curkey = curframe->getKeys();

        MatchVector goods;

        knn_match(descriptorLeft, descriptorRight, mMatcher, goods);

        return goods;
    }
} // namespace Position