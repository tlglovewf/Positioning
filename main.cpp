#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

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
#include "P_Writer.h"

#include "project/hdproject.h"
#include "project/weiyaproject.h"

using namespace std;
using namespace cv;

#define SAVEMATCHIMG    0  //是否存储同名点匹配文件
#define WEIYA           1  //是否为weiya数据
#define USECONTROLLER   0  //是否启用定位框架

int main(void)
{  
#if WEIYA
    std::shared_ptr<Position::IConfig> pCfg = std::make_shared<WeiyaConfig>("../config_weiya.yaml"); 
    std::shared_ptr<Position::IData> pData(new WeiyaData(pCfg));
#else
    std::shared_ptr<Position::IConfig> pCfg = std::make_shared<HdConfig>("../config_hd.yaml"); 
    std::shared_ptr<Position::IData> pData(new HdData(pCfg));
#endif

    std::shared_ptr<Position::IDetector> pdetecter = std::make_shared<Position::SSDDetector >();

    const string imgpath = GETCFGVALUE(pCfg,ImgPath ,string) + "/";
    const string outpath = GETCFGVALUE(pCfg,OutPath ,string) + "/";
#if USECONTROLLER
    std::unique_ptr<PositionController> system(new PositionController(pdetecter,pData,pCfg));

    system->run();

    return 0;
#endif
    pData->loadDatas();

    std::shared_ptr<Position::IMap> map(new Position::PMap);
    std::shared_ptr<Position::IFeature> pFeature(Position::PFactory::CreateFeature(Position::eFeatureOrb,pCfg));
    Ptr<Position::IFeatureMatcher> pMatcher = Position::PFactory::CreateFeatureMatcher(Position::eFMDefault,0.9);
    //std::shared_ptr<Position::IPositioning> position(Position::PFactory::CreatePositioning(Position::ePSingleImage, pData->getCamera()));
    std::unique_ptr<Position::IPoseEstimation> pPoseEst(Position::PFactory::CreatePoseEstimation(Position::ePoseEstCV));// ePoseEstOrb));
    Position::IOptimizer *pOp = Position::IOptimizer::getSingleton();
    Position::CameraParam camparam = pData->getCamera();
    Position::FrameHelper::initParams(GETCFGVALUE(pCfg,ImgWd,int),GETCFGVALUE(pCfg,ImgHg,int),&camparam);
    pPoseEst->setCamera(camparam);
    pOp->setCamera(camparam);
  

    Position::FrameDataVIter iter = pData->begin();
    Position::FrameDataVIter ed   = pData->end();

    int index = 0;
    Position::IKeyFrame *curframe = NULL;
    Position::IKeyFrame *preframe = NULL;
    for(;iter != ed; ++iter)
    {
        Mat img = imread(imgpath + iter->_name ,IMREAD_UNCHANGED);
        Mat oimg = img;
        assert(!img.empty());
        // cv::undistort(img,oimg,camparam.K,camparam.D);
        // imgHistEqualized(img,oimg);
        if(oimg.channels() > 1)
        {
            cv::cvtColor(oimg,oimg,CV_RGB2GRAY);
        }
       
        iter->_img = oimg;
        Position::IFrame *pframe = new Position::PFrame(*iter,pFeature,map->frameCount());
        Position::FrameHelper::assignFeaturesToGrid(pframe);
        //needcreatenewkeyframe()
        curframe = map->createKeyFrame(pframe);

        if(0 == index++)
        {
            preframe = curframe;
            curframe->setPose(Mat::eye(4,4,MATCVTYPE));
        }
        else
        {
            assert(pframe);
            assert(curframe);
            int searchradius = GETCFGVALUE(pCfg,SearchScale,int);
            Position::MatchVector matches = pMatcher->match(IFRAME(preframe),IFRAME(curframe),searchradius); 

            if(matches.size() < 80)
            {
                matches = pMatcher->match(IFRAME(preframe),IFRAME(curframe),searchradius * 2);
            }

            if(matches.size() < 80)
            {
                preframe = curframe;
                PROMT_V("Match point not enough.",matches.size());
                continue;
            }

            if(matches.empty())
            {
                PROMTD_V(iter->_name.c_str(),"can not find any match point with pre frame!");
                continue;
            }
            else
            {

#if SAVEMATCHIMG
                Mat oimg;
                cv::drawMatches(preframe->getData()._img,IFRAME(preframe)->getKeys(),curframe->getData()._img,IFRAME(curframe)->getKeys(),matches,oimg);
                const std::string text = string("Match:") + std::to_string(matches.size());
                putText(oimg, text, Point(150, 150), CV_FONT_HERSHEY_COMPLEX, 5, Scalar(0, 0, 255), 3, CV_AA);
                const string outname = outpath +  "match_"  + curframe->getData()._name;
                PROMTD_V("Save to",outname.c_str());
                imwrite(outname,oimg);
#endif

                pPoseEst->setFrames(IFRAME(preframe),IFRAME(curframe));
                Mat R,t;
                Position::Pt3Vector pts;
                if(pPoseEst->estimate(R,t, matches,pts))
                {
                    // PROMTD_V(iter->_name.c_str(),"R\n",R);
                    // PROMTD_V("t\n",t);
                    PROMTD_V(iter->_name.c_str(),"matches number",matches.size());
        
                    cv::Mat pose = cv::Mat::eye(4,4,MATCVTYPE);
                    R.copyTo(pose.rowRange(0,3).colRange(0,3));
                    t.copyTo(pose.rowRange(0,3).col(3));
                    Mat wdpose = pose * preframe->getPose() ;
                    curframe->setPose(wdpose);
                    for(auto item : matches)
                    {
                        Position::IMapPoint *mppt = NULL;
                        if(!preframe->hasMapPoint(item.queryIdx))
                        {
                            const Point3f fpt = pts[item.queryIdx];
                            Mat mpt = (Mat_<MATTYPE>(4,1) << fpt.x,fpt.y,fpt.z,1.0);
                            mpt = preframe->getPose().inv() * mpt;
                            mpt = mpt / mpt.at<MATTYPE>(3);
                            mppt = map->createMapPoint(mpt); 
                            preframe->addMapPoint(mppt,item.queryIdx);
                            
                        }
                        else
                        {
                            mppt = preframe->getWorldPoints()[item.queryIdx];
                        }
                        curframe->addMapPoint(mppt,item.trainIdx);
                    }

                    cout << "frame before optimize pose " << endl << curframe->getPose() << endl;
                    auto temps = curframe->getWorldPoints();
                    cout << "frame mppts size: " << std::count_if(temps.begin(),temps.end(),[](Position::IMapPoint* item)->bool
                    {
                        return item != NULL;
                    }) << endl;
                    pOp->frameOptimization(curframe,pFeature->getSigma2());
                    cout << "frame after optimize pose " << endl << curframe->getPose() << endl;
                }
                else
                {
                    //release data
                    PROMT_V(curframe->getData()._name.c_str(),"estimate failed!");
                }
                preframe = curframe;
                curframe = NULL;
            }

        }
        
    }
    // global optimization
    Position::KeyFrameVector keyframes(map->getAllFrames());
    Position::MapPtVector    mappts(map->getAllMapPts());
    bool pBstop = false;
    pOp->bundleAdjustment(keyframes,mappts,pFeature->getSigma2(),5, &pBstop);

    Position::Pangolin_Viewer *pv = new Position::Pangolin_Viewer(pCfg);
    pv->setMap(map);
    pv->renderLoop();

    return 0;
}