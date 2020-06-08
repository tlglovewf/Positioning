#include "test.h"
#include "P_Utils.h"
#include "P_SemanticGraph.h"
#include "P_Map.h"
#include "P_Factory.h"
#include "P_FeatureMatcher.h"
#include "P_Factory.h"


#include <map>
#include <vector>

#include <fstream>

#include "project/hdproject.h"

#include "P_UniformDistriFeature.h"


using namespace std;
using namespace cv;


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

        float maxdist = 0;
        float mindist = 0;

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


            MATTYPE d = sqrt(squareDist1);

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

            d += sqrt(squareDist2);

            d = d / 2.0;
 
            if(d > maxdist)
                maxdist = d;

            if(i == 0)
                mindist = d;
            else if(mindist > d)
            {
                mindist = d;
            }
            

            if(chiSquare2>th)
                bIn = false;
            else
                score += thScore - chiSquare2;

            if(bIn)
                vbMatchesInliers[i]=true;
            else
                vbMatchesInliers[i]=false;
        }   

        cout << "max distance is : " << maxdist << endl;
        cout << "min distance is : " << mindist << endl;
        cout << "inlier percent : " << std::count_if(vbMatchesInliers.begin(),vbMatchesInliers.end(),[](bool bol)->bool{
            return bol;
        }) / (float)N << endl;
        return score / N;
    }



bool CheckUnique(const Position::KeyPtVector &pts, const cv::KeyPoint &keypt)
{
    int index = 0;
    for(size_t i = 0; i < pts.size() ; ++i)
    {
        
        if( pts[i].pt == keypt.pt)
        {
            if(index++ == 0)
                continue;
            cout << keypt.hash() << endl;
            cout << keypt.octave << endl;
            cout << keypt.class_id << endl;
            cout << keypt.response << endl;
            cout << keypt.size << endl;
            cout << "-----" << endl;

            cout << pts[i].hash() << endl;
            cout << pts[i].octave << endl;
            cout << pts[i].class_id << endl;
            cout << pts[i].response << endl;
            cout << pts[i].size << endl;

            cout << "||" << KeyPoint::overlap(keypt,pts[i]) << endl;

            return false;
        }
    }
}
Mat GernerateMask(const string &sempath,const string &segim, Rect rect)
{
    string str = segim;
    Position::PUtils::ReplaceFileSuffix(str,"jpg", Position::SemanticGraph::Instance()->defaultsuffix);
    cv::Mat seg = imread(sempath + str);
    if(seg.empty())
    {
        cout<<"Mask Gernerate err!"<<endl;
        return cv::Mat();
    }
    Mat maskim = cv::Mat::ones(seg.rows, seg.cols,CV_8UC1);
    for(size_t i = 0; i<seg.rows; i++)
        for(size_t j = 0; j<seg.cols; j++)
        {
            Point pt = Point(j,i);
            if(Position::SemanticGraph::Instance()->isDynamicObj(pt,seg)||rect.contains(pt))
            {
                maskim.at<uchar>(pt) = 0;
            }           
        }    
    return maskim;
}

void SelectFrameData(const std::shared_ptr<Position::IFrameData> &pdata, Position::FrameData &predata, Position::FrameData &curdata)
{
   predata = **pdata->begin();
   curdata = **(pdata->begin() + 2);
}



void writeKeyPoints(const std::string &path,const Position::KeyPtVector &keypts1,const Position::KeyPtVector &keypts2,const Position::KeyPtVector &keypts3)
{
    if(!keypts1.empty() &&
       !keypts2.empty() )
       //&& keypts1.size() == keypts2.size())
       {
           ofstream file;
           file.open(path);
           
            for(size_t i = 0; i < keypts1.size() ; ++i)
            {
                char fmt[255] = {0};

                sprintf(fmt,"%f,%f;%f,%f;%f,%f",keypts1[i].pt.x,keypts1[i].pt.y,keypts2[i].pt.x,keypts2[i].pt.y,keypts3[i].pt.x,keypts3[i].pt.y);

                file << fmt << endl;
            }
       

           file.close();
       }
}

void readKeyPoints(const std::string &path, Position::KeyPtVector &keypts1, Position::KeyPtVector &keypts2,Position::KeyPtVector &keypts3, Position::MatchVector &mts1,Position::MatchVector &mts2)
{
    if(path.empty())
    {
        return;
    }
    keypts1.clear();
    keypts2.clear();
    ifstream file;
    try
    {
        file.open(path);

        int index = 0;
        while(!file.eof())
        {
            string line;
            getline(file,line);
            if(line.empty())
                continue;
            
            Position::StringVector strs = Position::PUtils::SplitString(line,";");
            assert(strs.size() == 3);
            float x,y;
            sscanf(strs[0].c_str(),"%f,%f",&x,&y);
            keypts1.push_back(KeyPoint(Point2f(x,y),1));
            sscanf(strs[1].c_str(),"%f,%f",&x,&y);
            keypts2.push_back(KeyPoint(Point2f(x,y),1));
            sscanf(strs[2].c_str(),"%f,%f",&x,&y);
            keypts3.push_back(KeyPoint(Point2f(x,y),1));
            DMatch dm;
            dm.queryIdx = index;
            dm.trainIdx = index++;
            mts1.emplace_back(dm);
            mts2.emplace_back(dm);
        }
        file.close();
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
}

TESTBEGIN()
    Mat im1 = imread("/media/tu/Work/Datas/test/689_L.jpg");
    Mat im2 = imread("/media/tu/Work/Datas/test/691_L.jpg");
    Mat im3 = imread("/media/tu/Work/Datas/test/693_L.jpg");

    std::shared_ptr<Position::IConfig>  pcfg = std::make_shared<HdConfig>("../config/config_hd.yaml"); 
    SETGLOBALCONFIG(pcfg);
    std::shared_ptr<Position::IFeature> pfeature(GETFEATURE(Orb));   
    std::shared_ptr<Position::IFeatureMatcher> pmatch(GETFEATUREMATCHER(Knn));
    Position::FrameData fm1;
    Position::FrameData fm2;
    Position::FrameData fm3;
    fm1._img = im1;

    Position::KeyPtVector keypt1;
    Position::KeyPtVector keypt2;
    Position::KeyPtVector keypt3;
    Mat des1;
    Mat des2;
    Mat des3;
    // pfeature->detect(fm1,keypt1,des1);
    // pfeature->detect(fm1,keypt2,des2);

    

    //手动选点
    //1
    keypt1.emplace_back(KeyPoint(Point2f(1478,528),1));
    keypt2.emplace_back(KeyPoint(Point2f(1422,506),1));
    keypt3.emplace_back(KeyPoint(Point2f(1357,475),1));

    keypt1.emplace_back(KeyPoint(Point2f(776,346),1));
    keypt2.emplace_back(KeyPoint(Point2f(685,313),1));
    keypt3.emplace_back(KeyPoint(Point2f(584,279),1));

    keypt1.emplace_back(KeyPoint(Point2f(1604,574),1));
    keypt2.emplace_back(KeyPoint(Point2f(1565,562),1));
    keypt3.emplace_back(KeyPoint(Point2f(1521,546),1));

    keypt1.emplace_back(KeyPoint(Point2f(3425,478),1));
    keypt2.emplace_back(KeyPoint(Point2f(3561,433),1));
    keypt3.emplace_back(KeyPoint(Point2f(3738,365),1));

    keypt1.emplace_back(KeyPoint(Point2f(784,278),1));
    keypt2.emplace_back(KeyPoint(Point2f(690,243),1));
    keypt3.emplace_back(KeyPoint(Point2f(588,204),1));

    keypt1.emplace_back(KeyPoint(Point2f(1485,652),1));
    keypt2.emplace_back(KeyPoint(Point2f(1425,637),1));
    keypt3.emplace_back(KeyPoint(Point2f(1362,620),1));

    keypt1.emplace_back(KeyPoint(Point2f(1387,1122),1));
    keypt2.emplace_back(KeyPoint(Point2f(1289,1148),1));
    keypt3.emplace_back(KeyPoint(Point2f(1164,1180),1));

    keypt1.emplace_back(KeyPoint(Point2f(2482,1016),1));
    keypt2.emplace_back(KeyPoint(Point2f(2512,1033),1));
    keypt3.emplace_back(KeyPoint(Point2f(2558,1048),1));

    keypt1.emplace_back(KeyPoint(Point2f(2306,1020),1));
    keypt2.emplace_back(KeyPoint(Point2f(2298,1030),1));
    keypt3.emplace_back(KeyPoint(Point2f(2295,1036),1));

    keypt1.emplace_back(KeyPoint(Point2f(806,1152),1));
    keypt2.emplace_back(KeyPoint(Point2f(616,1186),1));
    keypt3.emplace_back(KeyPoint(Point2f(372,1229),1));

    //11
    keypt1.emplace_back(KeyPoint(Point2f(2287,318),1));
    keypt2.emplace_back(KeyPoint(Point2f(2291,250),1));
    keypt3.emplace_back(KeyPoint(Point2f(2300,156),1));

    keypt1.emplace_back(KeyPoint(Point2f(2109,302),1));
    keypt2.emplace_back(KeyPoint(Point2f(2094,230),1));
    keypt3.emplace_back(KeyPoint(Point2f(2076,135),1));

    keypt1.emplace_back(KeyPoint(Point2f(2583,330),1));
    keypt2.emplace_back(KeyPoint(Point2f(2624,263),1));
    keypt3.emplace_back(KeyPoint(Point2f(2677,170),1));

    keypt1.emplace_back(KeyPoint(Point2f(2200,564),1));
    keypt2.emplace_back(KeyPoint(Point2f(2183,554),1));
    keypt3.emplace_back(KeyPoint(Point2f(2167,537),1));

    keypt1.emplace_back(KeyPoint(Point2f(1451,695),1));
    keypt2.emplace_back(KeyPoint(Point2f(1422,696),1));
    keypt3.emplace_back(KeyPoint(Point2f(1396,690),1));

    keypt1.emplace_back(KeyPoint(Point2f(2965,941),1));
    keypt2.emplace_back(KeyPoint(Point2f(3168,962),1));
    keypt3.emplace_back(KeyPoint(Point2f(3526,975),1));

    keypt1.emplace_back(KeyPoint(Point2f(2405,1001),1));
    keypt2.emplace_back(KeyPoint(Point2f(2413,1013),1));
    keypt3.emplace_back(KeyPoint(Point2f(2430,1020),1));

    keypt1.emplace_back(KeyPoint(Point2f(1754,1004),1));
    keypt2.emplace_back(KeyPoint(Point2f(1718,1012),1));
    keypt3.emplace_back(KeyPoint(Point2f(1682,1015),1));

    keypt1.emplace_back(KeyPoint(Point2f(2736,1274),1));
    keypt2.emplace_back(KeyPoint(Point2f(2888,1374),1));
    keypt3.emplace_back(KeyPoint(Point2f(3154,1530),1));

    keypt1.emplace_back(KeyPoint(Point2f(3490,697),1));
    keypt2.emplace_back(KeyPoint(Point2f(3643,676),1));
    keypt3.emplace_back(KeyPoint(Point2f(3842,638),1));

    // Position::PFrame *pf1 = new Position::PFrame(&fm1,0);
    // Position::PFrame *pf2 = new Position::PFrame(&fm2,1);
    // Position::PFrame *pf3 = new Position::PFrame(&fm3,2);

    // Position::MatchVector  mts = pmatch->match(pf1,pf2,10);

    // Position::MatchVector mts;
    // for(size_t i = 0; i < keypt1.size(); ++i)
    // {
    //     cv::DMatch dm;
    //     dm.
    // }

    Position::MatchVector mts1,mts2;
    mts1.reserve(keypt1.size());
    mts2.reserve(keypt1.size());
    // for(size_t i = 0; i < keypt1.size(); ++i)
    // {
    //     cv::DMatch dm;
    //     dm.queryIdx = i;
    //     dm.trainIdx = i;
    //     mts.emplace_back(dm);
    // }


  

    writeKeyPoints("/media/tu/Work/Datas/test/test.txt",keypt1,keypt2,keypt3);

    readKeyPoints("/media/tu/Work/Datas/test/test.txt",keypt1,keypt2,keypt3,mts1,mts2);
    cout << keypt1.size() << endl;
    cout << keypt2.size() << endl;
    cout << mts1.size() << endl;
    Mat rstimg = Position::PUtils::DrawFeatureMatch(im1,im2,keypt1,keypt2,mts1);

    imwrite("/media/tu/Work/Datas/test/rst.jpg",rstimg);    


    return 0;
    Position::SemanticGraph::Instance()->loadObjInfos("segraph.config");

    std::shared_ptr<Position::IConfig>      pCfg = std::make_shared<HdConfig>("../config/config_hd.yaml"); 
    std::shared_ptr<Position::IFrameData>   pData(new HdData(pCfg));
    std::shared_ptr<Position::IFeature>     pFeature(GETFEATURE(Orb));

    //new UniformDistriFeature(GETCFGVALUE(pCfg,FeatureCnt,int)));// new SiftFeature);
    std::shared_ptr<Position::IMap>     pmap(new Position::PMap);
    std::shared_ptr<Position::IFeatureMatcher>  pmatcher(GETFEATUREMATCHER(HanMing));
    std::shared_ptr<Position::IOptimizer>       pOptimizer(Position::PFactory::CreateOptimizer(Position::eOpG2o));
#ifdef USE_VIEW
    std::shared_ptr<Position::IViewer>  pv(Position::PFactory::CreateViewer(Position::eVPangolin,pCfg));
#endif
    std::shared_ptr<Position::IPoseSolver>  poseest(GETPOSESOLVER(CVPoseSolver));
    

    std::string   sempath = GETCFGVALUE(pCfg,SemPath,string);
    if(!sempath.empty())
    {
        Position::SemanticGraph::Instance()->loadObjInfos("../config/semgraph.cfg");
        Position::SemanticGraph::Instance()->setSemanticPath(sempath);
    }

#ifdef USE_VIEW
    pv->setMap(pmap);
#endif
    pData->loadDatas();
    pOptimizer->setCamera(pcfg->getCamera());
    const string imgpath = GETCFGVALUE(pCfg,ImgPath ,string) + "/";
    const string outpath = GETCFGVALUE(pCfg,OutPath ,string) + "/";

    Position::FrameData predata;
    Position::FrameData curdata;

    SelectFrameData(pData,predata,curdata);

    std::string picname1 = predata._name;  //"0_14166";//"0_11370.jpg";
    std::string picname2 = curdata._name;  //"0_14167";//"0_11371.jpg";

    Position::CameraParam cam = pcfg->getCamera();
  
     //加mask
    int maskUse = GETCFGVALUE(pCfg,MaskEnable,int);
    Mat mask;
    if(maskUse == 1)
    {
        int roiLx   =   GETCFGVALUE(pCfg,Lx,int);
        int roiLy   =   GETCFGVALUE(pCfg,Ly,int);
        int roiW    =   GETCFGVALUE(pCfg,Wd,int);
        int roiH    =   GETCFGVALUE(pCfg,Ht,int);
        Rect rect(roiLx, roiLy, roiW, roiH);
        //生成mask 
        mask = GernerateMask(sempath, picname1,  rect);
    }
    //加mask
    const std::string pngfx  = "png";

    Mat aimg1 = imread(imgpath + picname1);
    Mat aimg2 = imread(imgpath + picname2);

    cv::undistort(aimg1,predata._img,cam.K,cam.D);
    cv::undistort(aimg2,curdata._img,cam.K,cam.D);

    Position::Time_Interval timer;
    timer.start();
    Position::IFrame *preframe = new Position::PFrame(&predata,pFeature, pmap->frameCount());
    Position::IFrame *curframe = new Position::PFrame(&curdata,pFeature, pmap->frameCount());
    timer.prompt("feature detect ",true);
    
    Position::MatchVector good_matches =  pmatcher->match(preframe,curframe,3);
    timer.prompt("feature match  ",true);
    vector<DMatch>::iterator it = good_matches.begin();
    vector<DMatch>::iterator ed = good_matches.end();

    Position::MatchVector matches;
    matches.reserve(good_matches.size());
    for(; it != ed; ++it)
    {
        Point2f pt = preframe->getKeys()[it->queryIdx].pt;
        //当mask未设置,或者设置量 有值的时候(权重问题 后续再考虑)
        if(mask.empty() || mask.at<uchar>(pt.x,pt.y))
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
    Mat F = cv::findFundamentalMat(pt1s,pt2s,stats,FM_RANSAC,1.0);

    cout << "score : " << CheckFundamental(preframe->getKeys(),
                            curframe->getKeys(),
                            good_matches,F,bols,2) << endl;

    // cout << "bool size : " << std::count_if( bols.begin(), bols.end() ,[](bool bol)->bool{
    //     return !bol ;
    // } ) << endl;

    Mat out = Position::PUtils::DrawKeyPoints(curdata._img,curframe->getKeys());

    MATTYPE a,b,c;
    for(size_t i = 0; i < good_matches.size(); ++i)
    {
        const Point2f prept = preframe->getKeys()[good_matches[i].queryIdx].pt;
        const Point2f curpt = curframe->getKeys()[good_matches[i].trainIdx].pt;

        Position::EpLine epline = Position::PUtils::ComputeEpLine(F,prept);
        
        Position::PUtils::DrawEpiLine(epline,curpt, out);
    }


    imwrite("/media/tlg/work/tlgfiles/HDData/result/epline.jpg",out);

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
    
     //save images
    Mat img_goodmatch = Position::PUtils::DrawFeatureMatch(preframe->getData()->_img,
                                                           curframe->getData()->_img,
                                                           preframe->getKeys(),
                                                           curframe->getKeys(),
                                                           good_matches,stats);

    imwrite("/media/tlg/work/tlgfiles/HDData/result/match.jpg",img_goodmatch);
    cout << "write successfully.." << endl;
    system("xdg-open /media/tlg/work/tlgfiles/HDData/result/match.jpg");

    poseest->setFrames(preframe,curframe);

    poseest->setCamera(cam);

    Mat R,t;
    Position::Pt3Vector pt3ds;
    Mat pose1 = Mat::eye(4,4,CV_64F);
    preframe->setPose(pose1);
    Position::IKeyFrame *preKeyFrame;
    Position::IKeyFrame *curKeyFrame;
    preKeyFrame = pmap->createKeyFrame(preframe);

    cout << "estimate" << endl;
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
        // cout << "before op " << good_matches.size() <<  endl;
        // cout << curKeyFrame->getPose() << endl;
        pOptimizer->frameOptimization(curKeyFrame,pFeature->getSigma2());
        // cout << "after op " << endl;
        // cout << curKeyFrame->getPose() << endl;

        cout << "estimate end" << endl;
#ifdef USE_VIEW
        pv->renderLoop();
#endif

    }
TESTEND()