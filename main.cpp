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

#define SAVEMATCHIMG 0  //是否存储同名点匹配文件
#define WEIYA 0         //是否为weiya数据


//图像像素 相似度计算
void checkPixelSimilarity(const Mat &img1, const Mat &img2)
{
    assert(img1.size() == img2.size());

    int total = 0;
    for(int i = 0;i < img1.rows; ++i)
    {
        const uchar *pimg1 = img1.ptr<uchar>(i);
        const uchar *pimg2 = img2.ptr<uchar>(i);
        for(int j = 0; j < img1.cols; ++j)
        {
            if(pimg1[j] != pimg2[j])
                ++total;
        }
    }
    PROMT_V("Image Similarity:",(1.0 - ((float)total / (img1.rows * img1.cols))) * 100 , "%");
}

//直方图 相似度计算
void checkHistSimilarity(const Mat &img1, const Mat &img2)
{
    assert(img1.size() == img2.size());
    Mat gimg1 = img1;
    Mat gimg2 = img2;
    if(img1.channels() > 1)
    {
        cvtColor(gimg1,gimg1,CV_RGB2GRAY);
    }
    if(img2.channels() > 1)
    {
        cvtColor(gimg2,gimg2,CV_RGB2GRAY);
    }

    int histsize = 256;
	float range[] = { 0,256 };
    const float*histRanges = { range };
    Mat hist1,hist2;
	calcHist(&gimg1, 1, 0, Mat(), hist1, 1, &histsize, &histRanges);
    calcHist(&gimg2, 1, 0, Mat(), hist2, 1, &histsize, &histRanges);
    assert(hist1.type() == hist2.type());
    float d = (1 - cv::compareHist(hist1,hist2,CV_COMP_BHATTACHARYYA)) * 100;
    PROMT_V("Hist Similarity ", d, "%" );
//  putText(img, std::to_string(basebase), Point(50, 50), CV_FONT_HERSHEY_COMPLEX, 1, Scalar(0, 0, 255), 2, CV_AA);
}



//直方图均衡
void imgHistEqualized(const Mat &img, Mat &outimg)
{
    if(img.empty())
        return;

    if(img.channels() > 1)
    {
        vector<Mat> channels;
        Mat blue,green,red;
        //拆分通道 默认rgb格式
        cv::split(img,channels);
        red   = channels.at(0);
        green = channels.at(1);
        blue  = channels.at(2);
        cv::equalizeHist(red  , red)  ;
        cv::equalizeHist(green, green);
        cv::equalizeHist(blue , blue) ;

        cv::merge(channels,outimg);
    }
    else
    {//单通道
        cv::equalizeHist(img,outimg);
    }
}

void histDraw(const Mat &img)
{
    if(img.empty())
    {
        return ;
    }

    if(img.channels() < 2)
    {
        //步骤二：计算直方图
	    int histsize = 256;
	    float range[] = { 0,256 };
        const float*histRanges = { range };
        Mat hist;
	    calcHist(&img, 1, 0, Mat(), hist, 1, &histsize, &histRanges);
        //归一化
        int hist_h = 400;//直方图的图像的高
        int hist_w = 512;////直方图的图像的宽
        int bin_w = hist_w / histsize;//直方图的等级
        Mat histImage(hist_w, hist_h, CV_8UC3, Scalar(0, 0, 0));//绘制直方图显示的图像
        normalize(hist, hist, 0, hist_h, NORM_MINMAX, -1, Mat());//归一化
        //步骤三：绘制直方图（render histogram chart）
        for (int i = 1; i < histsize; i++)
        {
            //绘制直方图
            line(histImage, Point((i - 1)*bin_w, hist_h - cvRound(hist.at<float>(i - 1))),
                Point((i)*bin_w, hist_h - cvRound(hist.at<float>(i))), Scalar(255, 0, 0), 2, CV_AA);
        }
        PROMTD_S("Hist image display.");
        imshow("hist",histImage);
        waitKey(0);
    }
    else
    {
        vector<Mat> channels;
        cv::split(img,channels);
         //步骤二：计算直方图
	    int histsize = 256;
	    float range[] = { 0,256 };
        const float*histRanges = { range };
        Mat rhist,ghist,bhist;
	    calcHist(&channels[0], 1, 0, Mat(), rhist, 1, &histsize, &histRanges);
        calcHist(&channels[1], 1, 0, Mat(), ghist, 1, &histsize, &histRanges);
        calcHist(&channels[2], 1, 0, Mat(), bhist, 1, &histsize, &histRanges);

        //归一化
        int hist_h = 400;//直方图的图像的高
        int hist_w = 512;////直方图的图像的宽
        int bin_w = hist_w / histsize;//直方图的等级
        Mat histImage(hist_w, hist_h, CV_8UC3, Scalar(0, 0, 0));//绘制直方图显示的图像
        normalize(rhist, rhist, 0, hist_h, NORM_MINMAX, -1, Mat());//归一化
        normalize(ghist, ghist, 0, hist_h, NORM_MINMAX, -1, Mat());//归一化
        normalize(bhist, bhist, 0, hist_h, NORM_MINMAX, -1, Mat());//归一化
        //步骤三：绘制直方图（render histogram chart）
        for (int i = 1; i < histsize; i++)
        {
            //绘制r直方图
            line(histImage, Point((i - 1 )* bin_w, hist_h - cvRound(rhist.at<float>(i - 1))),
                Point((i)*bin_w, hist_h - cvRound(rhist.at<float>(i))), CV_RGB(255,0,0), 2, CV_AA);
            //绘制g直方图
            line(histImage, Point((i - 1) * bin_w, hist_h - cvRound(ghist.at<float>(i - 1))),
                Point((i)*bin_w, hist_h - cvRound(ghist.at<float>(i))), CV_RGB(0,255,0), 2, CV_AA);
            //绘制b直方图
            line(histImage, Point((i - 1) * bin_w, hist_h - cvRound(bhist.at<float>(i - 1))),
                Point((i)*bin_w, hist_h - cvRound(bhist.at<float>(i))), CV_RGB(0,0,255), 2, CV_AA);
        }
        PROMTD_S("Hist image display.");
        imshow("hist",histImage);
        waitKey(0);
    }
}


void combineSave(const Mat &img1, const Mat &img2, const std::string &out)
{
    assert(img1.size() == img2.size());
    Mat outimg;
    cv::hconcat(img1, img2, outimg);
    PROMT_V("Save image to ", outimg);
    imwrite(out,outimg);
}


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
#if 0
    std::unique_ptr<PositionController> system(new PositionController(pdetecter,pData,pCfg));

    system->run();

    return 0;
#endif
    pData->loadDatas();

    std::shared_ptr<Position::IMap> map(new Position::PMap);
    std::shared_ptr<Position::IFeature> pFeature(Position::PFactory::CreateFeature(Position::eFeatureOrb,pCfg));
    Ptr<Position::IFeatureMatcher> pMatcher = Position::PFactory::CreateFeatureMatcher(Position::eFMDefault,0.7);
    //std::shared_ptr<Position::IPositioning> position(Position::PFactory::CreatePositioning(Position::ePSingleImage, pData->getCamera()));
    std::unique_ptr<Position::IPoseEstimation> pPoseEst(Position::PFactory::CreatePoseEstimation(Position::ePoseEstCV));//ePoseEstOrb));
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
            Position::MatchVector matches = pMatcher->match(IFRAME(preframe),IFRAME(curframe),GETCFGVALUE(pCfg,SearchScale,int)); 

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
                    PROMTD_V(iter->_name.c_str(),"R\n",R);
                    PROMTD_V("t\n",t);
                    PROMTD_V(iter->_name.c_str(),"matches number",matches.size());
        
                    cv::Mat pose = cv::Mat::eye(4,4,MATCVTYPE);
                    R.copyTo(pose.rowRange(0,3).colRange(0,3));
                    t.copyTo(pose.rowRange(0,3).col(3));
                    // for(auto item : matches)
                    // {
                    //     Position::IMapPoint *mppt = map->createMapPoint(pts[item.queryIdx]); 
                    //     preframe->addMapPoint(mppt,item.queryIdx);
                    //     curframe->addMapPoint(mppt,item.trainIdx);
                    // }
                    curframe->setPose( pose * preframe->getPose() );

                    // cout << "frame before optimize pose " << endl << pose << endl;
                    // pOp->frameOptimization(curframe,pFeature->getSigma2());
                    // cout << "frame after optimize pose " << endl << curframe->getPose() << endl;

                    preframe = curframe;
                    curframe = NULL;
                }
            }

        }
        
    }
    // // global optimization
    // Position::KeyFrameVector keyframes(map->getAllFrames());
    // Position::MapPtVector    mappts(map->getAllMapPts());
    // bool pBstop = false;
    // pOp->bundleAdjustment(keyframes,mappts,pFeature->getSigma2(),5, &pBstop);

    Position::Pangolin_Viewer *pv = new Position::Pangolin_Viewer(pCfg);
    pv->setMap(map);
    pv->renderLoop();

    return 0;
}