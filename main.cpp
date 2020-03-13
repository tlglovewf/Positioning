#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/stitching.hpp>

#include <vector>

#include "P_Controller.h"

#include "P_Frame.h"
#include "P_MapPoint.h"
#include "P_Map.h"
#include "P_PangolinViewer.h"
#include "P_Factory.h"
#include "P_Config.h"
#include "P_Data.h"
#include "P_Detector.h"

using namespace std;
using namespace cv;

int main(void)
{  


    std::shared_ptr<Position::IConfig> pCfg = std::make_shared<Position::WeiyaConfig>("../config.yaml"); 
    std::shared_ptr<Position::IData> pData(new Position::WeiyaData(pCfg));
    std::shared_ptr<Position::IDetector> pdetecter = std::make_shared<Position::SSDDetector >();

#if 1
    std::unique_ptr<PositionController> system(new PositionController(pdetecter,pData,pCfg));

    system->run();

    return 0;
#endif
    pData->loadDatas();
    const string imgpath = GETCFGVALUE(pCfg,ImgPath ,string);//
    Mat img1 = imread(imgpath + "/20191107-072927356003-0000000300_L.jpg",IMREAD_UNCHANGED);
    Mat img2 = imread(imgpath + "/20191107-072928328188-0000000301_L.jpg",IMREAD_UNCHANGED);
    Mat img3 = imread(imgpath + "/20191107-072928863666-0000000302_L.jpg",IMREAD_UNCHANGED);
    Position::FrameData frame1;
    Position::FrameData frame2;
    Position::FrameData frame3;

    frame1._img = img1;
    frame2._img = img2;
    frame3._img = img3;

    std::shared_ptr<Position::IMap> map(new Position::PMap);

    

    std::shared_ptr<Position::IPositioning> position(Position::PFactory::CreatePositioning(Position::ePSingleImage, pData->getCamera()));

    std::shared_ptr<Position::IFeature> pFeature(Position::PFactory::CreateFeature(Position::eFeatureOrb,pCfg));

    Position::IFrame *pframe = new Position::PFrame(frame1,pFeature,map->frameCount());
    Position::IFrame *cframe = new Position::PFrame(frame2,pFeature,map->frameCount());
    Position::IFrame *oframe = new Position::PFrame(frame3,pFeature,map->frameCount());

    Position::CameraParam camparam = pData->getCamera();
    Position::FrameHelper::initParams(img1.cols,img1.rows,&camparam);
    Position::FrameHelper::assignFeaturesToGrid(pframe);
    Position::FrameHelper::assignFeaturesToGrid(cframe);
    Position::FrameHelper::assignFeaturesToGrid(oframe);
    
    //创建关键帧
    Position::IKeyFrame *preframe = map->createKeyFrame(pframe);
    Position::IKeyFrame *curframe = map->createKeyFrame(cframe);
    Position::IKeyFrame *othframe = map->createKeyFrame(oframe);

    Ptr<Position::IFeatureMatcher> pMatcher = Position::PFactory::CreateFeatureMatcher(Position::eFMDefault,0.9);

    Position::MatchVector matches = pMatcher->match(IFRAME(preframe),IFRAME(curframe),GETCFGVALUE(pCfg,SearchScale,int));

    Position::MatchVector others  = pMatcher->match(IFRAME(curframe),IFRAME(othframe),GETCFGVALUE(pCfg,SearchScale,int));

    if(matches.empty())
    {
        std::cout << "error ." << std::endl;
    }
    else
    {
        std::cout << "matches size " << matches.size() << std::endl;
        // Mat oimg;
        // cv::drawMatches(frame1._img,preframe->getKeys(),frame2._img,curframe->getKeys(),matches,oimg);

        // imwrite("/Users/TLG/Documents/Result/result.jpg",oimg);

        // cout << "write file successfully." << endl;

        std::unique_ptr<Position::IPoseEstimation> pPoseEst(Position::PFactory::CreatePoseEstimation(Position::ePoseEstOrb));

        pPoseEst->setCamera(pData->getCamera());

        pPoseEst->setFrames(IFRAME(preframe),IFRAME(curframe));
        Mat R,t;
        Position::Pt3Vector pts;
        if(pPoseEst->estimate(R,t, matches,pts))
        {
            cout << "R " << R << endl;
            cout << "t " << t << endl;
            cout << "matches number :" << matches.size() << endl;

            cv::Mat pose = cv::Mat::eye(4,4,MATCVTYPE);
            R.copyTo(pose.rowRange(0,3).colRange(0,3));
            t.copyTo(pose.rowRange(0,3).col(3));

            preframe->setPose(cv::Mat::eye(4,4,CV_64F));
            curframe->setPose(pose);
            
            for(auto item : matches)
            {
                Position::IMapPoint *mppt = map->createMapPoint(pts[item.queryIdx]); 
                preframe->addMapPoint(mppt,item.queryIdx);
                curframe->addMapPoint(mppt,item.trainIdx);
            }
            pPoseEst->setFrames(IFRAME(curframe),IFRAME(othframe));
        
           if(pPoseEst->estimate(R,t, others,pts))
           {
                for(auto item : others)
                {
                    if(curframe->hasMapPoint(item.queryIdx))
                    {
                        othframe->addMapPoint(curframe->getPoints()[item.queryIdx],item.trainIdx);
                    }
                    else
                    {
                        const Point3f &ppt = pts[item.queryIdx];
                        Mat temp = (Mat_<double>(4,1) << ppt.x , ppt.x,ppt.z,1);
                         
                        Position::IMapPoint *mppt = map->createMapPoint(pose.inv() * temp); 
                        preframe->addMapPoint(mppt,item.queryIdx);
                        curframe->addMapPoint(mppt,item.trainIdx);
                    }
                }
           }    

           R.copyTo(pose.rowRange(0,3).colRange(0,3));
           t.copyTo(pose.rowRange(0,3).col(3));
            
            Mat tpose = pose * curframe->getPose();
            othframe->setPose(tpose);

            Position::IOptimizer *pOp = Position::IOptimizer::getSingleton();
            pOp->setCamera(pData->getCamera());

            cout << "frame before optimize pose " << endl << pose << endl;
            pOp->frameOptimization(preframe,pFeature->getSigma2());
            pOp->frameOptimization(curframe,pFeature->getSigma2());
            cout << "frame after optimize pose " << endl << curframe->getPose() << endl;
            Position::KeyFrameVector keyframes(map->getAllFrames());
            Position::MapPtVector    mappts(map->getAllMapPts());
            bool pBstop = false;
            pOp->bundleAdjustment(keyframes,mappts,pFeature->getSigma2(),5, &pBstop);

            
            cout << "bundleadjust optimize pose " << endl << curframe->getPose() << endl;
        }
        else
        {
            cout << "pose estimate failed." << endl;
        }
      }
    Position::Pangolin_Viewer *pv = new Position::Pangolin_Viewer(pCfg,map);
    pv->renderLoop();

    return 0;
}