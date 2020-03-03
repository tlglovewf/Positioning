#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/stitching.hpp>

#include <vector>

#include "P_Controller.h"

#include "P_ORBFeature.h"
#include "P_Config.h"
#include "P_FeatureMatcher.h"
#include "P_Frame.h"
#include "P_PoseEstimation.h"
#include "P_Optimizer.h"
#include "P_Data.h"
#include "P_MapPoint.h"
#include "P_Map.h"
#include "Pangolin_Viewer.h"
#include "P_Positioning.h"

using namespace std;
using namespace cv;

int main(void)
{   
    // PositionController pcontroller("../config.yaml",PositionController::eWeiyaType);

    // //初始化信息
    // if(pcontroller.init())
    // {
    // }

    std::shared_ptr<Position::IConfig> pCfg = std::make_shared<Position::WeiyaConfig>(); 
    pCfg->load("../config.yaml");
    

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

    std::unique_ptr<Position::IData> pData(new Position::WeiyaData(pCfg));
    pData->loadDatas();

    std::shared_ptr<Position::IPositioning> position(new Position::Positioning(pData->getCamera()));

    std::shared_ptr<Position::IFeature> pFeature = std::make_shared<Position::ORBFeature>(pCfg);

    Position::IFrame *preframe = new Position::PFrame(frame1,pFeature);
    Position::IFrame *curframe = new Position::PFrame(frame2,pFeature);
    Position::IFrame *othframe = new Position::PFrame(frame3,pFeature);

    Position::FrameGrid::initParams(img1.cols,img1.rows);
    Position::FrameGrid::assignFeaturesToGrid(preframe);
    Position::FrameGrid::assignFeaturesToGrid(curframe);
    Position::FrameGrid::assignFeaturesToGrid(othframe);
    

    Ptr<Position::IFeatureMatcher> pMatcher = new Position::PFeatureMatcher(0.9);

    Position::MatchVector matches = pMatcher->match(preframe,curframe,GETCFGVALUE(pCfg,SearchScale,int));

    Position::MatchVector others  = pMatcher->match(curframe,othframe,GETCFGVALUE(pCfg,SearchScale,int));

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

        std::unique_ptr<Position::IPoseEstimation> pPoseEst(new Position::ORBPoseEstimation());

        pPoseEst->setCamera(pData->getCamera());

        pPoseEst->setFrames(preframe,curframe);
        Mat R,t;
        Position::BolVector bols;
        Position::Pt3Vector pts;
        if(pPoseEst->estimate(R,t, matches,pts, bols))
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
            pPoseEst->setFrames(curframe,othframe);
        
           if(pPoseEst->estimate(R,t, others,pts, bols))
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

            //创建关键帧
            map->createKeyFrame(preframe);
            map->createKeyFrame(curframe);
            map->createKeyFrame(othframe);

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
    Position::Pangolin_Viwer *pv = new Position::Pangolin_Viwer(pCfg,map,position);
    pv->render();

    return 0;
}