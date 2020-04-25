#include "P_MultiVisionTrajProcesser.h"
#include "P_Factory.h"
#include "P_Writer.h"
#include "P_Frame.h"

#include "FeatureQuadTree.h"
#include <thread>

#define SAVEMATCHIMG    0  //是否存储同名点匹配文件

namespace Position
{
     //构造
    PMultiVisionTrajProcesser::PMultiVisionTrajProcesser(const std::shared_ptr<IConfig> &pcfg,
                                                         const std::shared_ptr<IData> &pdata)
                   {
#if 1
                        mpFeature        = std::shared_ptr<IFeature>(new FeatureQuadTree(GETCFGVALUE(pcfg,nFeatures,int)));
                        mpFeatureMatcher = std::unique_ptr<IFeatureMatcher>(Position::PFactory::CreateFeatureMatcher(Position::eFMKnnMatch,GETCFGVALUE(pcfg,MatchRatio,float)));
#else
                        mpFeature        = std::shared_ptr<IFeature>(Position::PFactory::CreateFeature(Position::eFeatureOrb,pcfg));
                        mpFeatureMatcher = std::unique_ptr<IFeatureMatcher>(Position::PFactory::CreateFeatureMatcher(Position::eFMDefault,GETCFGVALUE(pcfg,MatchRatio,float)));
#endif
                        mpEst            = std::unique_ptr<IPoseEstimation>(Position::PFactory::CreatePoseEstimation(Position::ePoseEstCV));// ePoseEstOrb));
                        // mpEst            = std::unique_ptr<IPoseEstimation>(Position::PFactory::CreatePoseEstimation(Position::ePoseEstOrb));
                        mpOptimizer      = std::unique_ptr<IOptimizer>(Position::PFactory::CreateOptimizer(eOpG2o));

                        mCam = pdata->getCamera();

                        Position::FrameHelper::initParams(GETCFGVALUE(pcfg,ImgWd,int),GETCFGVALUE(pcfg,ImgHg,int),&mCam);
                        mFtSearchRadius = GETCFGVALUE(pcfg,SearchRadius,int);
                        mpEst->setCamera(mCam);
                        mpOptimizer->setCamera(mCam);
                   }

    //创建新关键帧
    IKeyFrame* PMultiVisionTrajProcesser::createNewKeyFrame()
    {
        assert(mpCurrent);
        return mpMap->createKeyFrame(mpCurrent);
    }

    bool PMultiVisionTrajProcesser::process(const FrameDataVector &framedatas)
    {
        if(framedatas.size() < 2)
        {
            return false;
        }
        else
        {
            for(size_t i = 0; i < framedatas.size(); ++i)
            {
                track(framedatas[i]);

                if(mpViewer)
                {
                    waitKey(1);
                }
            }

            // global optimization
            Position::KeyFrameVector keyframes(mpMap->getAllFrames());
            Position::MapPtVector    mappts(mpMap->getAllMapPts());
            bool pBstop = false;
            PROMT_V("Begin global optimization", keyframes.size());
            mpOptimizer->bundleAdjustment(keyframes,mappts,mpFeature->getSigma2(),5, &pBstop);
            PROMT_S("End Optimization.");

            return true;
        }
    }

    //跟踪
    cv::Mat PMultiVisionTrajProcesser::track(const FrameData &data)
    {
        assert(!data._img.empty());
        
        Mat grayimg ;

        if( !mCam.D.empty() || fabs(mCam.D.at<MATTYPE>(0)) > 1e-6 )
        {//有畸变参数存在
            cv::undistort(data._img,grayimg,mCam.K,mCam.D);
        }
        if(grayimg.channels() > 1)
        {//先只考虑rbg模式的
            cvtColor(grayimg,grayimg,CV_RGB2GRAY);
        }

        FrameData temp = data;
        temp._img = grayimg;
        mpCurrent = new PFrame(temp,mpFeature,mpMap->frameCount());
        Position::FrameHelper::assignFeaturesToGrid(mpCurrent);
        if(mStatus == eTrackNoImage)
        {
            mpLast    = mpCurrent;
            mpCurrentKeyFm = createNewKeyFrame();
            mpLastKeyFm    = mpCurrentKeyFm;
            Mat origin = Mat::eye(4,4,MATCVTYPE);
            mpCurrentKeyFm->setPose(origin);
            mStatus = eTrackNoReady;
            return origin;
        }
        else if(mStatus == eTrackNoReady)
        {
            mpCurrentKeyFm = createNewKeyFrame();
        }
        else
        {
            if(needCreateNewKeyFrame())
                mpCurrentKeyFm = createNewKeyFrame();
            else
            {
                //创建关键帧失败则释放
                mpCurrent->release();
                return Mat();
            }
        }

        assert(mpLast);
        assert(mpCurrent);
        assert(mpLastKeyFm);
        assert(mpCurrentKeyFm);

        Position::MatchVector matches = mpFeatureMatcher->match(IFRAME(mpLastKeyFm),IFRAME(mpCurrentKeyFm),mFtSearchRadius); 


        // if(matches.size() < 80)
        // {
        //     matches = pMatcher->match(IFRAME(mpLastKeyFm),IFRAME(mpCurrentKeyFm),searchradius * 2);
        // }
        // if(matches.size() < 80)
        // {
        //     mpLastKeyFm = mpCurrentKeyFm;
        //     PROMT_V("Match point not enough.",matches.size());
        //     return Mat();
        // }

        if(matches.empty())
        {
            mStatus = eTrackLost;
            PROMTD_V(data._name.c_str(),"can not find any match point with pre frame!");
            return Mat();
        }
        else
        {

#if SAVEMATCHIMG
            Mat oimg;
            cv::drawMatches(mpLastKeyFm->getData()._img,IFRAME(mpLastKeyFm)->getKeys(),mpCurrentKeyFm->getData()._img,IFRAME(mpCurrentKeyFm)->getKeys(),matches,oimg);
            const std::string text = string("Match:") + std::to_string(matches.size());
            putText(oimg, text, Point(150, 150), CV_FONT_HERSHEY_COMPLEX, 5, Scalar(0, 0, 255), 3, CV_AA);
            const string outname = outpath +  "match_"  + mpCurrentKeyFm->getData()._name;
            PROMTD_V("Save to",outname.c_str());
            imwrite(outname,oimg);
#endif
    
            mpEst->setFrames(IFRAME(mpLastKeyFm),IFRAME(mpCurrentKeyFm));
            Mat R,t;
            Position::Pt3Vector pts;
            PROMTD_V(data._name.c_str(),"origin matches number ",matches.size());
            if(mpEst->estimate(R,t, matches,pts))
            {//推算位姿
                PROMTD_V(data._name.c_str(),"estimate matches number ",matches.size());
        
                cv::Mat pose = cv::Mat::eye(4,4,MATCVTYPE);
                R.copyTo(pose.rowRange(0,3).colRange(0,3));
                t.copyTo(pose.rowRange(0,3).col(3));
                Mat wdpose = pose * mpLastKeyFm->getPose() ;
                mpCurrentKeyFm->setPose(wdpose);

                for(auto item : matches)
                {//遍历匹配信息,建立地图点与关键的关联关系
                    Position::IMapPoint *mppt = NULL;
                    if(!mpLastKeyFm->hasMapPoint(item.queryIdx))
                    {
                        const Point3f fpt = pts[item.queryIdx];
                        Mat mpt = (Mat_<MATTYPE>(4,1) << fpt.x,fpt.y,fpt.z,1.0);
                        mpt = mpLastKeyFm->getPose().inv() * mpt;
                        mpt = mpt / mpt.at<MATTYPE>(3);
                        mppt = mpMap->createMapPoint(mpt); 
                        mpLastKeyFm->addMapPoint(mppt,item.queryIdx);
                    }
                    else
                    {
                        mppt = mpLastKeyFm->getWorldPoints()[item.queryIdx];
                    }
                    mpCurrentKeyFm->addMapPoint(mppt,item.trainIdx);
                }

                auto temps = mpCurrentKeyFm->getWorldPoints();
                PROMTD_V("frame mppts size" ,std::count_if(temps.begin(),temps.end(),[](Position::IMapPoint* item)->bool
                {
                    return item != NULL;
                }));

                mpOptimizer->frameOptimization(mpCurrentKeyFm,mpFeature->getSigma2());
             
                for(size_t i = 0; i < temps.size(); ++i)
                {
                    if(temps[i])
                    {
                        if(IFRAME(mpCurrentKeyFm)->outlier(i))
                        {
                            IFRAME(mpCurrentKeyFm)->outlier(i) = false;
                            mpCurrentKeyFm->rmMapPoint(i);
                        }
                    }
                }

                // PROMTD_V("Pose", mpCurrentKeyFm->getPose());
            }
            else
            {
                //release data
                PROMT_V(mpCurrentKeyFm->getData()._name.c_str(),"estimate failed!");
            }
            mpLastKeyFm = mpCurrentKeyFm;
            mpLast      = mpCurrent;
            mpCurrent   = NULL;
            mpCurrentKeyFm = NULL;
        }
        mStatus = eTrackOk;
        return mpLastKeyFm->getPose();
    }
}