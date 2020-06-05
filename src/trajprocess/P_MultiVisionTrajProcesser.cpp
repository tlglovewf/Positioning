#include "P_MultiVisionTrajProcesser.h"
#include "P_Factory.h"
#include "P_IOHelper.h"
#include "P_Frame.h"

#include "P_UniformDistriFeature.h"
#include <thread>

#define SAVEMATCHIMG    0  //是否存储同名点匹配文件

namespace Position
{
     //构造
    PMultiVisionTrajProcesser::PMultiVisionTrajProcesser(const std::shared_ptr<IConfig> &pcfg,
                                                         const CameraParam &cam):mCam(cam)
                   {
#if 1
                        int featureCnt   = min(GETCFGVALUE(pcfg,FeatureCnt,int),300);
                        mpFeature        = std::shared_ptr<IFeature>(new PUniformDistriFeature(featureCnt));
                        mpFeatureMatcher = std::unique_ptr<IFeatureMatcher>(Position::PFactory::CreateFeatureMatcher(Position::eFMKnnMatch,0.5));
#else
                        mpFeature        = std::shared_ptr<IFeature>(Position::PFactory::CreateFeature(Position::eFeatureOrb,pcfg));
                        mpFeatureMatcher = std::unique_ptr<IFeatureMatcher>(Position::PFactory::CreateFeatureMatcher(Position::eFMDefault,0.8));
#endif
                        mpEst            = std::unique_ptr<IPoseSolver>(Position::PFactory::CreatePoseSolver(Position::ePSCv)); 
                        // mpEst            = std::unique_ptr<IPoseSolver>(Position::PFactory::CreatePoseSolver(Position::ePoseEstOrb));
                        mpOptimizer      = std::unique_ptr<IOptimizer>(Position::PFactory::CreateOptimizer(eOpG2o));
                       
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

    bool PMultiVisionTrajProcesser::process(const FrameDataPtrVector &framedatas)
    {
        LOG_INFO("Use MultiVision Process ...");
        if(framedatas.size() < 2)
        {
            return false;
        }
        else
        {
            for(size_t i = 0; i < framedatas.size(); ++i)
            {
                track(framedatas[i]);
#if !USE_VIEW
                framedatas[i]->_img.release();
#endif
                if(mpViewer)
                {
                    waitKey(1);
                }
            }

            // global optimization
            Position::KeyFrameVector keyframes(mpMap->getAllFrames());
            Position::MapPtVector    mappts(mpMap->getAllMapPts());
            bool pBstop = false;
            LOG_DEBUG_F("Begin Global Opt:%d-%d",keyframes.size(),mappts.size());
            mpOptimizer->bundleAdjustment(keyframes,mappts,mpFeature->getSigma2(),5, &pBstop);
            LOG_DEBUG("Global Opt Finished.");

            return true;
        }
    }

    //跟踪
    cv::Mat PMultiVisionTrajProcesser::track(FrameData *data)
    {
        LOG_INFO_F("Process:%s",data->_name.c_str());
        Mat grayimg ;

        if( !mCam.D.empty() && fabs(mCam.D.at<MATTYPE>(0)) > 1e-6 )
        {//有畸变参数存在
            cv::undistort(data->_img,grayimg,mCam.K,mCam.D);
        }
        else
        {
            grayimg = data->_img;
        }
        if(grayimg.channels() > 1)
        {//先只考虑rbg模式的
            cvtColor(grayimg,grayimg,CV_RGB2GRAY);
        }

        data->_img = grayimg;
        mpCurrent = new PFrame(data,mpFeature,mpMap->frameCount());
        Position::FrameHelper::assignFeaturesToGrid(mpCurrent);
        if(mStatus == eTrackNoImage)
        {
            mpLast          = mpCurrent;
            Mat origin      = Mat::eye(4,4,MATCVTYPE);
            mpCurrentKeyFm  = createNewKeyFrame();
            mpMap->addKeyFrame(mpCurrentKeyFm);
            mpLastKeyFm     = mpCurrentKeyFm;
            mpCurrent       = NULL;
            mpCurrentKeyFm  = NULL;
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
                mpCurrent = NULL;
                return Mat();
            }
        }

        assert(mpLast);
        assert(mpCurrent);
        assert(mpLastKeyFm);
        assert(mpCurrentKeyFm);

        Position::MatchVector matches = mpFeatureMatcher->match(IFRAME(mpLastKeyFm),IFRAME(mpCurrentKeyFm),mFtSearchRadius); 

        if(matches.size() < 8)
        {
            mpCurrentKeyFm->release();
            mpCurrentKeyFm  = NULL;
            mpCurrent       = NULL;
            LOG_WARNING_F("%s - %s Match Points Not Enough~~ %d",mpLastKeyFm->getData()->_name.c_str(),data->_name.c_str(), matches.size());
            return Mat();
        }
        else
        {

#if SAVEMATCHIMG
            Mat oimg;
            cv::drawMatches(mpLastKeyFm->getData()->_img,IFRAME(mpLastKeyFm)->getKeys(),mpCurrentKeyFm->getData()->_img,IFRAME(mpCurrentKeyFm)->getKeys(),matches,oimg);
            const std::string text = string("Match:") + std::to_string(matches.size());
            putText(oimg, text, Point(150, 150), CV_FONT_HERSHEY_COMPLEX, 5, Scalar(0, 0, 255), 3, CV_AA);
            const string outname = outpath +  "match_"  + mpCurrentKeyFm->getData()->_name;
            PROMTD_V("Save to",outname.c_str());
            imwrite(outname,oimg);
#endif
    
            mpEst->setFrames(IFRAME(mpLastKeyFm),IFRAME(mpCurrentKeyFm));
            Mat R,t;
            Position::Pt3Vector pts;

            if(mpEst->estimate(R,t, matches,pts))
            {//推算位姿
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
                mpMap->addKeyFrame(mpCurrentKeyFm);
                mpLastKeyFm = mpCurrentKeyFm;
                mpLast      = mpCurrent;
                mStatus = eTrackOk;
            }
            else
            {
                //release data
                PROMT_V(mpCurrentKeyFm->getData()->_name.c_str(),"estimate failed!");
                mpCurrentKeyFm->release();
            }
            mpCurrent   = NULL;
            mpCurrentKeyFm = NULL;
        }
        
        return mpLastKeyFm->getPose();
    }
}