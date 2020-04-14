#include "P_Utils.h"
#include "P_SemanticGraph.h"
#include "P_Map.h"
#include "P_Factory.h"
#include "P_FeatureMatcher.h"
#include <opencv2/xfeatures2d.hpp>

#include <map>
#include <vector>

#include <fstream>

#include "ASiftDetector.h"

#include "project/hdproject.h"

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;


typedef vector<KeyPoint>        KeyPtVector;
typedef KeyPtVector::iterator   KeyPtVIter;

void detect(const Ptr<Feature2D> &feature,Mat &out, KeyPtVector &keypts)
{
    assert(!out.empty());

    Mat mask = Mat::zeros(out.size(),CV_8UC1);
    
    //提出图片中间部分的匹配点
    int w = out.cols * 0.35;
    int h = out.rows * 0.35;
    Rect roi(w,h, w, w);
    mask.setTo(255);
    mask(roi).setTo(0);

    feature->detect(out,keypts,mask);
    // Mat tt;

    // drawKeypoints(out,keypts,tt);
    // imwrite("/media/tlg/work/tlgfiles/HDData/result/mask.jpg",mask);
    waitKey(0);
}

Mat drawKeys(const Mat &img, const KeyPtVector &keys)
{
    Mat keypoint_img;
    drawKeypoints(img,keys,keypoint_img,CV_RGB(0,0,255),DrawMatchesFlags::DEFAULT);
    putText(keypoint_img,"key size:" + std::to_string(keys.size()),cv::Point2f(50,50),CV_FONT_HERSHEY_COMPLEX, 1, CV_RGB(255,0,0), 2, CV_AA);
    return keypoint_img;
}


#define FEATUREMATCH(X)         void feature##X##Match(const Mat &des1, const Mat &des2, vector<DMatch> &matches,vector<DMatch> &good_matches)
#define FEATUREMATCHFUNC(X)     case e##X##Type:\
                                return feature##X##Match(des1,des2,matches,good_matches);
#define FEATUREMATCHTYPE(X)  e##X##Type

enum eFeatureMType
{
    FEATUREMATCHTYPE(BF),
    FEATUREMATCHTYPE(Flann),
    FEATUREMATCHTYPE(Knn)
};

FEATUREMATCH(BF)
{
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::BRUTEFORCE);  
    matcher->match(des1,des2,matches);

    double min_dist = 10000,max_dist = 0;

    for(int i = 0; i < des1.rows; ++i)
    {
        double dist = matches[i].distance;
        if( dist < min_dist ) min_dist = dist;
        if( dist > max_dist ) max_dist = dist;
    }

    min_dist = min_element(matches.begin(), matches.end(),[](const DMatch &m1, const DMatch &m2){return m1.distance < m2.distance;})->distance;
    max_dist = max_element(matches.begin(), matches.end(),[](const DMatch &m1, const DMatch &m2){return m1.distance < m2.distance;})->distance;

    printf("-- min dist : %f \n",min_dist);
    printf("-- max dist : %f \n",max_dist);

    for(int i = 0; i < des1.rows; ++i)
    {
        if(matches[i].distance < max( 2 * min_dist, 30.0))
        {
            good_matches.emplace_back(matches[i]);
        }
    }
}

FEATUREMATCH(Flann)
{
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("FlannBased");
    matcher->match(des1,des2,matches);
     double maxDist = 10;
    for(int i =0;i<des1.rows;i++)
    {
        double dist = matches[i].distance;
        if(dist>maxDist)
            maxDist= dist;
    }

    for(int i =0;i<des1.rows;i++)
    {
        if(matches[i].distance < 0.3 * maxDist)             ////调参褚   0.1越小 越精确  官方推荐0.5 如果确定点 可改变
        {
            good_matches.push_back(matches[i]);
        }
    }
}

FEATUREMATCH(Knn)
{
     cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("FlannBased");
    // cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(DescriptorMatcher::BRUTEFORCE);
    vector< vector<DMatch> > knnMatches;
    matcher->knnMatch(des1,des2,knnMatches,2);
  
    const float minRatio = 0.45f;//0.4  0.5  0.6

    for(int i = 0;i < knnMatches.size();++i)
    {
        const DMatch &bestmatch   = knnMatches[i][0];
        const DMatch &bettermatch = knnMatches[i][1];

        float distanceRatio = bestmatch.distance / bettermatch.distance;
        if(distanceRatio < minRatio)
        {
            good_matches.push_back(bestmatch);
        }
    }
}

void featureMatch(eFeatureMType etype, const Mat &des1, const Mat &des2, vector<DMatch> &matches, vector<DMatch> &good_matches)
{
    switch (etype)
    {
        FEATUREMATCHFUNC(BF)
        FEATUREMATCHFUNC(Flann)
        FEATUREMATCHFUNC(Knn)   
    }
}

//特征接口
class SiftFeature : public Position::IFeature
{
public:
    //（1）nfeatures，保留的最佳特性的数量。特征按其得分进行排序(以SIFT算法作为局部对比度进行测量)；
    //（2）nOctavelLayers，高斯金字塔最小层级数，由图像自动计算出；
    //（3）constrastThreshold，对比度阈值用于过滤区域中的弱特征。阈值越大，检测器产生的特征越少。；
    //（4）edgeThreshold ，用于过滤掉类似边缘特征的阈值。 请注意，其含义与contrastThreshold不同，即edgeThreshold越大，滤出的特征越少；
    //（5）sigma，高斯输入层级， 如果图像分辨率较低，则可能需要减少数值。
    SiftFeature():mSift(SIFT::create(250,4,0.05,15,1.4))
    {
        Position::FloatVector  mvScaleFactor(4,0);
        Position::FloatVector  mvLevelSigma2(4,0);
        mSigmaVector.resize(4);
        mvLevelSigma2[0]= 1.0f;
        mSigmaVector[0] = 1.0f;

        for(int i=1; i < 4; i++)
        {
            mvScaleFactor[i]=mvScaleFactor[i-1]*1.4;
            mvLevelSigma2[i]= 1.0 /(mvScaleFactor[i]*mvScaleFactor[i]);
        }

    }

    //计算特征点
    virtual bool detect(const Position::FrameData &frame,Position::KeyPtVector &keys, Mat &descript) 
    {
        // Mat mask = Mat::zeros(out.size(),CV_8UC1);
    
        // //提出图片中间部分的匹配点
        // int w = out.cols * 0.35;
        // int h = out.rows * 0.35;
        // Rect roi(w,h, w, w);
        // mask.setTo(255);
        // mask(roi).setTo(0);

        mSift->detect(frame._img,keys);//,mask);
        mSift->compute(frame._img,keys,descript);

    }
    //返回sigma参数(主要用于优化 信息矩阵)
    virtual const Position::FloatVector& getSigma2() const
    {
        return mSigmaVector;
    }
protected:
    Position::FloatVector mSigmaVector;
    Ptr<SIFT> mSift;
};



float CheckFundamental(const Position::KeyPtVector &pt1s,const Position::KeyPtVector &pt2s,
                       const Position::MatchVector &mvMatches12, 
                       const cv::Mat &F21, Position::BolVector &vbMatchesInliers, float sigma)
    {
        const int N = mvMatches12.size();

        const MATTYPE f11 = F21.at<MATTYPE>(0,0);
        const MATTYPE f12 = F21.at<MATTYPE>(0,1);
        const MATTYPE f13 = F21.at<MATTYPE>(0,2);
        const MATTYPE f21 = F21.at<MATTYPE>(1,0);
        const MATTYPE f22 = F21.at<MATTYPE>(1,1);
        const MATTYPE f23 = F21.at<MATTYPE>(1,2);
        const MATTYPE f31 = F21.at<MATTYPE>(2,0);
        const MATTYPE f32 = F21.at<MATTYPE>(2,1);
        const MATTYPE f33 = F21.at<MATTYPE>(2,2);

        vbMatchesInliers.resize(N);

        float score = 0;

        const float th = 3.841;
        
        const float thScore = CHITH;

        const float invSigmaSquare = 1.0/(sigma*sigma);

        for(int i=0; i<N; i++)
        {
            bool bIn = true;

            const cv::KeyPoint &kp1 = pt1s[mvMatches12[i].queryIdx];
            const cv::KeyPoint &kp2 = pt2s[mvMatches12[i].trainIdx];

            const float u1 = kp1.pt.x;
            const float v1 = kp1.pt.y;
            const float u2 = kp2.pt.x;
            const float v2 = kp2.pt.y;

            // Reprojection error in second image
            // l2=F21x1=(a2,b2,c2)

            const MATTYPE a2 = f11*u1+f12*v1+f13;
            const MATTYPE b2 = f21*u1+f22*v1+f23;
            const MATTYPE c2 = f31*u1+f32*v1+f33;

            const MATTYPE num2 = a2*u2+b2*v2+c2;

            const MATTYPE squareDist1 = num2*num2/(a2*a2+b2*b2);

            const MATTYPE chiSquare1 = squareDist1*invSigmaSquare;

            if(chiSquare1>th)
                bIn = false;
            else
                score += thScore - chiSquare1;

            // Reprojection error in second image
            // l1 =x2tF21=(a1,b1,c1)

            const MATTYPE a1 = f11*u2+f21*v2+f31;
            const MATTYPE b1 = f12*u2+f22*v2+f32;
            const MATTYPE c1 = f13*u2+f23*v2+f33;

            const MATTYPE num1 = a1*u1+b1*v1+c1;

            const MATTYPE squareDist2 = num1*num1/(a1*a1+b1*b1);

            const MATTYPE chiSquare2 = squareDist2*invSigmaSquare;

            if(chiSquare2>th)
                bIn = false;
            else
                score += thScore - chiSquare2;

            if(bIn)
                vbMatchesInliers[i]=true;
            else
                vbMatchesInliers[i]=false;
        }

        return score / N;
    }





int main(void)
{
    SemanticGraph::Instance()->loadObjInfos("segraph.config");

    std::shared_ptr<Position::IConfig>  pCfg = std::make_shared<HdConfig>("../config_hd.yaml"); 
    std::shared_ptr<Position::IData>    pData(new HdData(pCfg));
    std::shared_ptr<Position::IFeature> pFeature(new SiftFeature);
    std::shared_ptr<Position::IMap>     pmap(new Position::PMap);
    std::shared_ptr<Position::IFeatureMatcher>  pmatcher(new Position::PKnnMatcher);
    std::shared_ptr<Position::IOptimizer>       pOptimizer(Position::PFactory::CreateOptimizer(Position::eOpG2o));
    std::shared_ptr<Position::IViewer>  pv(Position::PFactory::CreateViewer(Position::eVPangolin,pCfg));
    std::shared_ptr<Position::IPoseEstimation>  poseest(Position::PFactory::CreatePoseEstimation(Position::ePoseEstCV));
    pv->setMap(pmap);
    pData->loadDatas();
    pOptimizer->setCamera(pData->getCamera());
    const string imgpath = GETCFGVALUE(pCfg,ImgPath ,string) + "/";
    const string outpath = GETCFGVALUE(pCfg,OutPath ,string) + "/";

    Position::FrameData &predata = *pData->begin();
    Position::FrameData &curdata = *(pData->begin() + 1);

    std::string picname1 = predata._name;  //"0_14166";//"0_11370.jpg";
    std::string picname2 = curdata._name;  //"0_14167";//"0_11371.jpg";

    Position::CameraParam cam = pData->getCamera();

    const std::string seimgpath = "/media/tlg/work/tlgfiles/HDData/0326-1/Image-1/";

    const std::string pngfx  = "png";

    Mat aimg1 = imread(imgpath + picname1);
    Mat aimg2 = imread(imgpath + picname2);

    Position::PUtils::ReplaceFileSuffix(picname1,"jpg","png");
    Position::PUtils::ReplaceFileSuffix(picname2,"jpg","png");
    Mat seimg1 = imread(seimgpath + picname1);
    Mat seimg2 = imread(seimgpath + picname2);

    if(seimg1.empty())
        cout << picname1.c_str() << " semantic image is empty!!!" << endl;

    cv::undistort(aimg1,predata._img,cam.K,cam.D);
    cv::undistort(aimg2,curdata._img,cam.K,cam.D);

    Position::IFrame *preframe = new Position::PFrame(predata,pFeature, pmap->frameCount());
    Position::IFrame *curframe = new Position::PFrame(curdata,pFeature, pmap->frameCount());

    Position::MatchVector matches;
   
    Position::MatchVector good_matches =  pmatcher->match(preframe,curframe,5);
    
    vector<DMatch>::iterator it = good_matches.begin();
    vector<DMatch>::iterator ed = good_matches.end();

    matches.clear();
    matches.reserve(good_matches.size());
    for(; it != ed; ++it)
    {
        Point2f pt = preframe->getKeys()[it->queryIdx].pt;

        if(!SemanticGraph::Instance()->isDynamicObj(pt,seimg1))
        {
            matches.emplace_back(*it);
        }
    }
    good_matches.swap(matches);

    vector<Point2f> pt1s;
    vector<Point2f> pt2s;
 
    for(int i = 0; i < good_matches.size(); ++i)
    {
        pt1s.push_back( preframe->getKeys()[good_matches[i].queryIdx].pt);
        pt2s.push_back( curframe->getKeys()[good_matches[i].trainIdx].pt);
    }

    if(good_matches.size() < 4)
    {
        cout << "matches not enough~~~" << endl;
        return 0;
    }
    Position::U8Vector  stats;
    Position::BolVector bols;
    Mat F = cv::findFundamentalMat(pt1s,pt2s,stats,FM_RANSAC,4.0);

    cout << "score : " << CheckFundamental(preframe->getKeys(),
                            curframe->getKeys(),
                            good_matches,F,bols,2) << endl;

    cout << "bool size : " << std::count_if( bols.begin(), bols.end() ,[](bool bol)->bool{
        return !bol ;
    } ) << endl;

    pt1s.clear();
    pt2s.clear();
    for(int i = 0; i < good_matches.size(); ++i)
    {
        const Point2f &prept =  preframe->getKeys()[good_matches[i].queryIdx].pt;
        if(stats[i])
        {
            pt1s.push_back( prept );
            pt2s.push_back( curframe->getKeys()[good_matches[i].trainIdx].pt);
        }
    }

    poseest->setFrames(preframe,curframe);

    poseest->setCamera(cam);

    Mat R,t;
    Position::Pt3Vector pt3ds;
    Mat pose1 = Mat::eye(4,4,CV_64F);
    preframe->setPose(pose1);
    Position::IKeyFrame *preKeyFrame;
    Position::IKeyFrame *curKeyFrame;
    preKeyFrame = pmap->createKeyFrame(preframe);

    if(poseest->estimate(R,t,good_matches,pt3ds))
    {
        Mat pose2 = Mat::eye(4,4,CV_64F);
        R.copyTo(pose2.rowRange(0,3).colRange(0,3));
        t.copyTo(pose2.rowRange(0,3).col(3));
        curframe->setPose(pose2);
        curKeyFrame = pmap->createKeyFrame(curframe);

        for(auto item : good_matches)
        {
            const Point3f fpt = pt3ds[item.queryIdx];
            Mat mpt = (Mat_<MATTYPE>(4,1) << fpt.x,fpt.y,fpt.z,1.0);
            mpt = mpt / mpt.at<MATTYPE>(3);
            Position::IMapPoint *mppt = pmap->createMapPoint(mpt); 
            preKeyFrame->addMapPoint(mppt,item.queryIdx);
            curKeyFrame->addMapPoint(mppt,item.trainIdx);
        }
        cout << "before op " << good_matches.size() <<  endl;
        cout << curKeyFrame->getPose() << endl;
        pOptimizer->frameOptimization(curKeyFrame,pFeature->getSigma2());
        cout << "after op " << endl;
        cout << curKeyFrame->getPose() << endl;
    }

    pv->renderLoop();

    //save images
    Mat img_goodmatch;
   
    img_goodmatch = Position::PUtils::DrawFeatureMatch(preframe->getData()._img,
                                                       curframe->getData()._img,
                                                       preframe->getKeys(),
                                                       curframe->getKeys(),
                                                       good_matches,stats);

    imwrite("/media/tlg/work/tlgfiles/HDData/result/match.jpg",img_goodmatch);

    cout << "write successfully.." << endl;
    return 0;
}