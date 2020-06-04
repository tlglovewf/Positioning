//add sfm
//to do:
//1. 3d points merge
//2. mpOptimizer BA strategy
//3. gps fusion

#include "P_SfmVisonTrajProcesser.h"
#include "P_Factory.h"
#include "P_IOHelper.h"
#include "P_Frame.h"

#include "P_UniformDistriFeature.h"
#include <thread>

#define SAVEMATCHIMG    0  //是否存储同名点匹配文件

namespace Position
{
     //构造
    PSfmVisonTrajProcesser::PSfmVisonTrajProcesser(const std::shared_ptr<IConfig> &pcfg,
                                                         const CameraParam &cam):mCam(cam)
                   {
#if 1
                        mpFeature        = std::shared_ptr<IFeature>(new PUniformDistriFeature(GETCFGVALUE(pcfg,FeatureCnt,int)));
                        mpFeatureMatcher = std::unique_ptr<IFeatureMatcher>(Position::PFactory::CreateFeatureMatcher(Position::eFMKnnMatch,GETCFGVALUE(pcfg,MatchRatio,float)));
#else
                        mpFeature        = std::shared_ptr<IFeature>(Position::PFactory::CreateFeature(Position::eFeatureOrb,pcfg));
                        mpFeatureMatcher = std::unique_ptr<IFeatureMatcher>(Position::PFactory::CreateFeatureMatcher(Position::eFMDefault,GETCFGVALUE(pcfg,MatchRatio,float)));
#endif
                        mpEst            = std::unique_ptr<IPoseEstimation>(Position::PFactory::CreatePoseEstimation(Position::ePoseEstCV));// ePoseEstOrb));
                        // mpEst            = std::unique_ptr<IPoseEstimation>(Position::PFactory::CreatePoseEstimation(Position::ePoseEstOrb));
                        mpOptimizer      = std::unique_ptr<IOptimizer>(Position::PFactory::CreateOptimizer(eOpG2o));
                       
                        Position::FrameHelper::initParams(GETCFGVALUE(pcfg,ImgWd,int),GETCFGVALUE(pcfg,ImgHg,int),&mCam);
                        
                        mFtSearchRadius = GETCFGVALUE(pcfg,SearchRadius,int);
                        mpEst->setCamera(mCam);
                        mpOptimizer->setCamera(mCam);    
                   }
    
    PSfmVisonTrajProcesser::~PSfmVisonTrajProcesser()
    {

    }

    //创建新关键帧
    IKeyFrame* PSfmVisonTrajProcesser::createNewKeyFrame()
    {
        assert(mpCurrent);
        return mpMap->createKeyFrame(mpCurrent);
    }

    bool PSfmVisonTrajProcesser::process(const FrameDataPtrVector &framedatas)
    {
        LOG_INFO("Run SFM ...");
        LOG_INFO_F("FrameDatas Size:%d",framedatas.size());
        if(framedatas.size() < 2 || framedatas.size() > 5)
        {
            return false;
        }
        else
        {
            int worldindex;
            std::set<int>  mDoneImage;
            std::set<int>  mGoodImage;
            std::vector<cv::Mat>  mCameraPoses;
            mCameraPoses.resize(framedatas.size());
            //两两为一组
            ImagePairVector impairs;
            for (size_t i = 0; i < framedatas.size(); i++) 
            {
                for (size_t j = i + 1; j < framedatas.size(); j++)
                    impairs.push_back({ i, j });
            }

            //单张图提取特征点
            cout<<"run detect ..."<<endl;
            vector<IFrame*> mImageFrame;
            mImageFrame.resize(framedatas.size());
            for (size_t i = 0; i < framedatas.size(); i++) 
            {
                Mat grayimg ;

                if( !mCam.D.empty() && fabs(mCam.D.at<MATTYPE>(0)) > 1e-6 )
                {//有畸变参数存在
                    cv::undistort(framedatas[i]->_img,grayimg,mCam.K,mCam.D);
                }
                else
                {
                    grayimg = framedatas[i]->_img;
                }
                if(grayimg.channels() > 1)
                {//先只考虑rbg模式的
                    cvtColor(grayimg,grayimg,CV_RGB2GRAY);
                }

                framedatas[i]->_img = grayimg;
                mImageFrame[i] = new PFrame(framedatas[i],mpFeature,mpMap->frameCount());
                Position::FrameHelper::assignFeaturesToGrid(mImageFrame[i]);
            }

            //两两匹配
            LOG_INFO("Run Match ...");
            MatchMatrix mFeatureMatchMatrix;
            mFeatureMatchMatrix.resize(framedatas.size(), vector<MatchVector>(framedatas.size()));

            for (size_t i = 0; i < impairs.size(); i++) 
            {
                const ImagePair& pair = impairs[i];
                mpLast = mImageFrame[pair.pre];
                mpCurrent = mImageFrame[pair.cur];
                mFeatureMatchMatrix[pair.pre][pair.cur] = mpFeatureMatcher->match(mpLast,mpCurrent,mFtSearchRadius);
            }
            //用单应矩阵筛选合适的初始化帧
            LOG_INFO("Run Choose ...");
            map<float, ImagePair> matchesSizes;
            for (size_t i = 0; i < mImageFrame.size() - 1; i++) 
            {
                for (size_t j = i + 1; j < mImageFrame.size(); j++) 
                {
                    if (mFeatureMatchMatrix[i][j].size() < 100)
                    {
                        matchesSizes[1.0] = {i, j};
                        continue;
                    }

                    PtVector alignedLeftPt,alignedRightPt;
                    alignedLeftPt.clear();
                    alignedRightPt.clear();

                    for (size_t k=0; k<mFeatureMatchMatrix[i][j].size(); k++) 
                    {
                        alignedLeftPt.emplace_back(mImageFrame[i]->getKeys()[mFeatureMatchMatrix[i][j][k].queryIdx].pt);
                        alignedRightPt.emplace_back(mImageFrame[j]->getKeys()[mFeatureMatchMatrix[i][j][k].trainIdx].pt);
                    }
                    Mat inlierMask;
                    Mat homography;
                    if(mFeatureMatchMatrix[i][j].size() >= 4) {
                        homography = findHomography(alignedLeftPt, alignedRightPt,
                                                    cv::RANSAC, 3, inlierMask);
                    }

                    if(mFeatureMatchMatrix[i][j].size() < 4 || homography.empty()) {
                        return false;
                    }

                    const float inliersRatio = (float)countNonZero(inlierMask) / (float)(mFeatureMatchMatrix[i][j].size());
                    matchesSizes[inliersRatio] = {i, j};
                }
            }

            //找内点率满足要求的两帧初始化
            LOG_INFO("Looking For Best Two Frame ...")
            struct Point3DInMap 
            {
                cv::Point3f p;
                std::map<int, int> originatingViews;
            };
            vector<Point3DInMap> mPointCloud;

            for (auto& imagePair : matchesSizes) 
            {
                size_t i = imagePair.second.pre;
                size_t j = imagePair.second.cur;

                PtVector alignedLeftPt,alignedRightPt;
                alignedLeftPt.clear();
                alignedRightPt.clear();

                for (size_t k=0; k<mFeatureMatchMatrix[i][j].size(); k++) 
                {
                    alignedLeftPt.emplace_back(mImageFrame[i]->getKeys()[mFeatureMatchMatrix[i][j][k].queryIdx].pt);
                    alignedRightPt.emplace_back(mImageFrame[j]->getKeys()[mFeatureMatchMatrix[i][j][k].trainIdx].pt);
                }
               
                Mat E, R, t;
                Mat mask;
                E = findEssentialMat(alignedLeftPt, alignedRightPt, mCam.K, RANSAC, 0.999, 1.0, mask);

                float poseInliersRatio = (float)countNonZero(mask) / (float)mFeatureMatchMatrix[i][j].size();

                if (poseInliersRatio < 0.5)
                    continue;
                
                worldindex =i;
                mpCurrent = mImageFrame[i];
                mpLastKeyFm = createNewKeyFrame();
                mpCurrent = mImageFrame[j];
                mpCurrentKeyFm = createNewKeyFrame();
                mpEst->setFrames(IFRAME(mpLastKeyFm),IFRAME(mpCurrentKeyFm));
                
                // PROMTD_V(data._name.c_str()," matches ",matches.size());
                if(mFeatureMatchMatrix[i][j].size() < 4)
                {
                    mpLastKeyFm->release();
                    mpCurrentKeyFm->release();
                    return false;
                }
                Position::Pt3Vector pts;
                if(mpEst->estimate(R,t, mFeatureMatchMatrix[i][j],pts))
                {
                    mCameraPoses[i] = cv::Mat::eye(4,4,MATCVTYPE);
                    cv::Mat pose = cv::Mat::eye(4,4,MATCVTYPE);
                    R.copyTo(pose.rowRange(0,3).colRange(0,3));
                    t.copyTo(pose.rowRange(0,3).col(3));
                    mCameraPoses[j] = pose;
                    mpLastKeyFm->setPose(mCameraPoses[i]);
                    mpCurrentKeyFm->setPose(mCameraPoses[j]);

                    mpMap->addKeyFrame(mpLastKeyFm);
                    mpMap->addKeyFrame(mpCurrentKeyFm);

                    mDoneImage.insert(i);
                    mDoneImage.insert(j);
                    mGoodImage.insert(i);
                    mGoodImage.insert(j);
                    
                    for(auto item : mFeatureMatchMatrix[i][j])
                    {
                        Position::IMapPoint *mppt = NULL;
                        const Point3f fpt = pts[item.queryIdx];
                        Mat mpt = (Mat_<MATTYPE>(4,1) << fpt.x,fpt.y,fpt.z,1.0);
                        mpt = mpt / mpt.at<MATTYPE>(3);
                        mppt = mpMap->createMapPoint(mpt); 
                        mpLastKeyFm->addMapPoint(mppt,item.queryIdx); 
                        mpCurrentKeyFm->addMapPoint(mppt,item.trainIdx);

                        Point3DInMap p;
                        p.originatingViews[i] = item.queryIdx;
                        p.originatingViews[j] = item.trainIdx;
                        p.p = fpt;
                        mPointCloud.emplace_back(p);
                    }
                    
                    mpOptimizer->frameOptimization(mpCurrentKeyFm,mpFeature->getSigma2());
                    break;
                }
                else
                {
                    //release data
                    PROMT_V(mpCurrentKeyFm->getData()->_name.c_str(),"estimate failed!");

                    mpLastKeyFm->release();
                    mpCurrentKeyFm->release();
                    continue;
                }
            }

            //找3d-2d匹配最多的两个图进行匹配
            LOG_INFO("Run PnpSolver ...");
            struct Image2D3DMatch 
            {
                PtVector points2D;
                Pt3Vector points3D;
            };
            
            while (mDoneImage.size() != framedatas.size()) 
            {
                std::map<int, Image2D3DMatch> matches2D3D;
                for (size_t viewIdx = 0; viewIdx < framedatas.size(); viewIdx++) 
                {
                    if (mDoneImage.find(viewIdx) != mDoneImage.end()) 
                    {
                        continue; //skip done views
                    }
                    LOG_INFO_F("Process ViewId:%d", viewIdx);
                    Image2D3DMatch match2D3D;

                    //scan all cloud 3D points
                    for (const Point3DInMap& cloudPoint : mPointCloud) 
                    {
                        bool found2DPoint = false;

                        for (const auto& origViewAndPoint : cloudPoint.originatingViews) 
                        {                       
                            const int originatingViewIndex        = origViewAndPoint.first;
                            const int originatingViewFeatureIndex = origViewAndPoint.second;
                        
                            const int leftViewIdx  = (originatingViewIndex < viewIdx) ? originatingViewIndex : viewIdx;
                            const int rightViewIdx = (originatingViewIndex < viewIdx) ? viewIdx : originatingViewIndex;
                            
                            for (const DMatch& m : mFeatureMatchMatrix[leftViewIdx][rightViewIdx]) 
                            {
                                int matched2DPointInNewView = -1;
                                if (originatingViewIndex < viewIdx) 
                                { 
                                    if (m.queryIdx == originatingViewFeatureIndex) 
                                    {
                                        matched2DPointInNewView = m.trainIdx;
                                    }
                                } 
                                else 
                                {                             
                                    if (m.trainIdx == originatingViewFeatureIndex) 
                                    {
                                        matched2DPointInNewView = m.queryIdx;
                                    }
                                }
                                if (matched2DPointInNewView >= 0) 
                                {
                                    match2D3D.points2D.emplace_back(mImageFrame[viewIdx]->getKeys()[matched2DPointInNewView].pt);
                                    match2D3D.points3D.emplace_back(cloudPoint.p);
                                    found2DPoint = true;
                                    break;
                                }
                            }

                            if (found2DPoint) {
                                break;
                            }
                        }
                    }
                    matches2D3D[viewIdx] = match2D3D;
                }

                size_t bestView;
                size_t bestNumMatches = 0;
                for (const auto& m : matches2D3D) 
                {
                    const size_t numMatches = m.second.points2D.size();
                    if (numMatches > bestNumMatches) 
                    {
                        bestView       = m.first;
                        bestNumMatches = numMatches;
                    }
                }

                mDoneImage.insert(bestView);

                Mat rvec, tvec;
                Mat inliers;
                Mat distort;

                solvePnPRansac(matches2D3D[bestView].points3D, matches2D3D[bestView].points2D, mCam.K, distort, rvec, tvec, false, 100, 10, 0.99, inliers);

                if (((float)countNonZero(inliers) / (float)matches2D3D[bestView].points2D.size()) < 0.3) 
                {
                    LOG_ERROR_F("Inliers ratio is too small: %f // %d ",matches2D3D[bestView].points2D.size());
                    //continue;
                }

                Mat rotMat;
                Rodrigues(rvec, rotMat); 
                
                cv::Mat pose = cv::Mat::eye(4,4,MATCVTYPE);
                rotMat.copyTo(pose.rowRange(0,3).colRange(0,3));
                tvec.copyTo(pose.rowRange(0,3).col(3));
                mCameraPoses[bestView] = pose;

                mpCurrent = mImageFrame[bestView];
                mpCurrentKeyFm = createNewKeyFrame();
                mpCurrentKeyFm->setPose(mCameraPoses[bestView]);
                
                LOG_INFO("Run Merge ...");
                for (const int goodView : mGoodImage) 
                {
                    size_t leftIdx  = (goodView < bestView) ? goodView : bestView;
                    size_t rightIdx = (goodView < bestView) ? bestView : goodView;

                    PtVector alignedLeftPt,alignedRightPt;
                    alignedLeftPt.clear();
                    alignedRightPt.clear();

                    for (size_t k=0; k<mFeatureMatchMatrix[leftIdx][rightIdx].size(); k++) 
                    {
                        alignedLeftPt.emplace_back(mImageFrame[leftIdx]->getKeys()[mFeatureMatchMatrix[leftIdx][rightIdx][k].queryIdx].pt);
                        alignedRightPt.emplace_back(mImageFrame[rightIdx]->getKeys()[mFeatureMatchMatrix[leftIdx][rightIdx][k].trainIdx].pt);
                    }

                    Mat Etmp;
                    Mat masktmp;
                    Etmp = findEssentialMat(alignedLeftPt, alignedRightPt, mCam.K, RANSAC, 0.999, 1.0, masktmp);
                    MatchVector prunedMatches;
                    prunedMatches.clear();
                    for (size_t i = 0; i < masktmp.rows; i++) {
                        if (masktmp.at<uchar>(i)) {
                            prunedMatches.push_back(mFeatureMatchMatrix[leftIdx][rightIdx][i]);
                        }
                    }
                    
                    mFeatureMatchMatrix[leftIdx][rightIdx] = prunedMatches;
                    alignedLeftPt.clear();
                    alignedRightPt.clear();

                    for (size_t k=0; k<mFeatureMatchMatrix[leftIdx][rightIdx].size(); k++) 
                    {
                        alignedLeftPt.emplace_back(mImageFrame[leftIdx]->getKeys()[mFeatureMatchMatrix[leftIdx][rightIdx][k].queryIdx].pt);
                        alignedRightPt.emplace_back(mImageFrame[rightIdx]->getKeys()[mFeatureMatchMatrix[leftIdx][rightIdx][k].trainIdx].pt);
                    }

                    Mat points3d, points3dHomogeneous;
                    
                    triangulatePoints(mCameraPoses[leftIdx].rowRange(0,3), mCameraPoses[rightIdx].rowRange(0,3), alignedLeftPt, alignedRightPt, points3dHomogeneous);
                    convertPointsFromHomogeneous(points3dHomogeneous.t(), points3d);

                    Mat rvecLeft;
                    Rodrigues(mCameraPoses[leftIdx].rowRange(0,3).colRange(0,3), rvecLeft);
                    Mat tvecLeft(mCameraPoses[leftIdx].rowRange(0,3).col(3).t());

                    vector<Point2f> projectedOnLeft(alignedLeftPt.size());
                    projectPoints(points3d, rvecLeft, tvecLeft, mCam.K, Mat(), projectedOnLeft);

                    Mat rvecRight;
                    Rodrigues(mCameraPoses[rightIdx].rowRange(0,3).colRange(0,3), rvecRight);
                    Mat tvecRight(mCameraPoses[rightIdx].rowRange(0,3).col(3).t());

                    vector<Point2f> projectedOnRight(alignedRightPt.size());
                    projectPoints(points3d, rvecRight, tvecRight, mCam.K, Mat(), projectedOnRight);

                    vector<Point3DInMap> pointCloud;
                    for (size_t i = 0; i < points3d.rows; i++) 
                    {
                        if (norm(projectedOnLeft[i]  - alignedLeftPt[i])  > 10 ||//重投影误差太大
                            norm(projectedOnRight[i] - alignedRightPt[i]) > 10)
                        {
                            continue;
                        }
                        LOG_DEBUG("Calc 3d Points");
                        Position::IMapPoint *mppt = NULL;
                        const Point3f fpt = Point3f(points3d.at<float>(i, 0), points3d.at<float>(i, 1), points3d.at<float>(i, 2));
                        Mat mpt = (Mat_<MATTYPE>(4,1) << fpt.x,fpt.y,fpt.z,1.0);
                        mpt = mpt / mpt.at<MATTYPE>(3);
                        mppt = mpMap->createMapPoint(mpt); 
                        mpCurrentKeyFm->addMapPoint(mppt,mFeatureMatchMatrix[leftIdx][rightIdx][i].trainIdx);

                        Point3DInMap p;
                        p.p = fpt;
                        p.originatingViews[leftIdx] = mFeatureMatchMatrix[leftIdx][rightIdx][i].queryIdx;
                        p.originatingViews[rightIdx] = mFeatureMatchMatrix[leftIdx][rightIdx][i].trainIdx;
                        pointCloud.emplace_back(p);
                    }
 
                    //融合地图点
                    size_t newPoints = 0;
                    size_t mergedPoints = 0;

                    for (const Point3DInMap& p : pointCloud) {
                        const Point3f newPoint = p.p;

                        bool foundAnyMatchingExistingViews = false;
                        bool foundMatching3DPoint = false;
                        for (Point3DInMap& existingPoint : mPointCloud) {
                            if (norm(existingPoint.p - newPoint) < 0.5) 
                            {
                                foundMatching3DPoint = true;

                                for (const auto& newKv : p.originatingViews) {

                                    for (const auto& existingKv : existingPoint.originatingViews) {

                                        bool foundMatchingFeature = false;

                                        const bool newIsLeft = newKv.first < existingKv.first;
                                        const int leftViewIdx         = (newIsLeft) ? newKv.first  : existingKv.first;
                                        const int leftViewFeatureIdx  = (newIsLeft) ? newKv.second : existingKv.second;
                                        const int rightViewIdx        = (newIsLeft) ? existingKv.first  : newKv.first;
                                        const int rightViewFeatureIdx = (newIsLeft) ? existingKv.second : newKv.second;

                                        const MatchVector& matching = mFeatureMatchMatrix[leftViewIdx][rightViewIdx];
                                        for (const DMatch& match : matching) {
                                            if (   match.queryIdx == leftViewFeatureIdx
                                                && match.trainIdx == rightViewFeatureIdx
                                                && match.distance < 0.01) {

                                                //Found a 2D feature match for the two 3D points - merge
                                                foundMatchingFeature = true;
                                                break;
                                            }
                                        }

                                        if (foundMatchingFeature) {
                                            //Add the new originating view, and feature index
                                            existingPoint.originatingViews[newKv.first] = newKv.second;

                                            foundAnyMatchingExistingViews = true;

                                        }
                                    }
                                }
                            }
                            if (foundAnyMatchingExistingViews) {
                                mergedPoints++;
                                break; //Stop looking for more matching cloud points
                            }
                        }

                        if (!foundAnyMatchingExistingViews && !foundMatching3DPoint) {
                            // Position::IMapPoint *mppt = NULL;
                            // const Point3f fpt = p.p;
                            // Mat mpt = (Mat_<MATTYPE>(4,1) << fpt.x,fpt.y,fpt.z,1.0);
                            // mpt = mpt / mpt.at<MATTYPE>(3);
                            // mppt = mpMap->createMapPoint(mpt); 
                            mPointCloud.push_back(p);
                            newPoints++;
                        }
                    }
                    //cout<<"newPoints: "<<newPoints<<endl;
                }
                //cout<<"mPointCloud: "<<mPointCloud.size()<<endl;

                //ba
                // global optimization
                LOG_INFO("Run Ba ...");
                Position::KeyFrameVector keyframes(mpMap->getAllFrames());
                Position::MapPtVector    mappts(mpMap->getAllMapPts());
                bool pBstop = false;
                // LOG_DEBUG_F("Begin Global opt:%d",keyframes.size());
                mpOptimizer->bundleAdjustment(keyframes,mappts,mpFeature->getSigma2(),5, &pBstop);
                // LOG_DEBUG("Global opt Finished.");

                
                mGoodImage.insert(bestView);
            }

            // 暂时注释  add by tu. 
            // for (size_t i = 0; i < mDoneImage.size(); i++)
            // {
            //     cout<<"mpMap->getAllFrames()["<<i<<"]->getPose(): "<<mpMap->getAllFrames()[i]->getPose()<<endl;
            // }

            //位姿转换为相对时序的第一帧
            std::vector<cv::Mat> seqPoses;
            Mat transMapPointMat = cv::Mat::eye(4,4,MATCVTYPE);
            seqPoses.push_back(transMapPointMat.clone());
            // cout<<seqPoses[0]<<endl;
            //时序第一帧
            Mat tmpT; 
            tmpT = mpMap->getAllFrames()[0]->getPose().inv();
            
            // cout<<"transMapPointMat: "<<tmpT<<endl;
            // cout<<tmpT.size()<<endl;
            // tmpT = transMapPointMat.inv();
            // seqPoses.emplace_back(tmpT.clone());
            
            for (size_t i = 1; i < framedatas.size(); i++)
            {
                Mat tmpT1;
                if(i == worldindex)
                    tmpT1=tmpT;
                else
                {
                    tmpT1 = mpMap->getAllFrames()[i]->getPose()*tmpT;
                    // cout<<"tmpT1: "<<tmpT1<<endl;
                    // cout<<tmpT1.size()<<endl;
                }
                seqPoses.push_back(tmpT1.clone());    
            }
            
            // cout<<seqPoses.size()<<endl;
            //转换
            for (size_t i = 0; i < framedatas.size(); i++)
            {
                cout<<seqPoses[i]<<endl;
                mpMap->getAllFrames()[i]->setPose(seqPoses[i]);
            }

            // for (size_t i = 0; i < mDoneImage.size(); i++)
            // {
            //     cout<<"====after trans===="<<endl;
            //     cout<<"mpMap->getAllFrames()["<<i<<"]->getPose(): "<<mpMap->getAllFrames()[i]->getPose()<<endl;
            // }

            // cout<<"map trans ... "<<endl;
            //地图点转换为相对于第一帧
            // mpMap->clear();
            // for (const Point3DInMap& p : mPointCloud)
            // {
            //     Position::IMapPoint *mppt = NULL;
            //     const Point3f fpt = p.p;
            //     Mat mpt = (Mat_<MATTYPE>(4,1) << fpt.x,fpt.y,fpt.z,1.0);
            //     mpt = mpt / mpt.at<MATTYPE>(3);
            //     //cout<<"mpt: "<<mpt<<endl;
            //     mpt = transMapPointMat * mpt;
            //     mppt = mpMap->createMapPoint(mpt); 
            // }
            LOG_INFO("SFM Finished ...");
            return true;
        }
    }

    //跟踪
    cv::Mat PSfmVisonTrajProcesser::track(FrameData *data)
    {

    }
}