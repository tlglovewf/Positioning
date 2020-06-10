#include "P_Task.h"
#include "P_IOHelper.h"
#include "P_Frame.h"
#include "P_Factory.h"

namespace Position
{
    int FeatureTask::s_fmcount = 0;

    /*
        * 特征提取任务
        */

    FeatureTask::FeatureTask(const std::string &fname, const Position::CameraParam &cam) : mpFeature(GETFEATURE(fname)), mCamera(cam)
    {
        FrameHelper::initParams(GETCFGVALUE(GETGLOBALCONFIG(), ImgWd, int), GETCFGVALUE(GETGLOBALCONFIG(), ImgHg, int), &mCamera);
    }
    //! run one item
    IFrame *FeatureTask::run(Position::FrameData *item)
    {
        assert(item);
        const string imgpath = GETCFGVALUE(GETGLOBALCONFIG(), ImgPath, string) + "/";
        if (imgpath.size() < 2)
        {
            LOG_CRIT("Please Set Image Path.");
            exit(-1);
        }
        return runimpl(imgpath, item);
    }

    FrameVector FeatureTask::run(const Position::FrameDataPtrVector &items)
    {
        if (items.empty())
        {
            LOG_CRIT("No FrameData!!!");
            exit(-1);
        }
        const string imgpath = GETCFGVALUE(GETGLOBALCONFIG(), ImgPath, string) + "/";
        if (imgpath.size() < 2)
        {
            LOG_CRIT("Please Set Image Path.");
            exit(-1);
        }
        FrameVector framedatas;
        framedatas.reserve(items.size());
        for (size_t i = 0; i < items.size(); ++i)
        {
            framedatas.emplace_back(runimpl(imgpath, items[i]));
        }
        return std::move(framedatas);
    }

    IFrame *FeatureTask::runimpl(const std::string &path, Position::FrameData *item)
    {
        assert(item);

        if (item->_img.empty())
        {
            Mat img = imread(path + item->_name, CV_LOAD_IMAGE_UNCHANGED);

            if (!mCamera.D.empty())
            {
                undistort(img, item->_img, mCamera.K, mCamera.D);
            }
            else
            {
                item->_img = img;
            }
        }

        IFrame *pf = new Position::PFrame(item, mpFeature, s_fmcount++);
        FrameHelper::assignFeaturesToGrid(pf);
        return pf;
    }

    /*
        * 特征匹配任务
        */

    MatcherTask::MatcherTask(const std::string &name) : mpFeatureMatcher(GETFEATUREMATCHER(name)) {}

    //! 处理单个
    void MatcherTask::run(Item &item)
    {
        assert(item.queryItem && item.trainItem);
        Position::MatchVector matches = mpFeatureMatcher->match(item.queryItem, item.trainItem, GETCFGVALUE(GETGLOBALCONFIG(), SearchRadius, int));
        item.matches.swap(matches);
    }

    //! 批执行
    void MatcherTask::run(TaskItems &items)
    {
        for (Item &item : items)
        {
            run(item);
        }
    }

    /*
        * 位姿估计任务
        */

    PoseTask::PoseTask(const std::string &name, const Position::CameraParam &cam) : mpPoseEst(GETPOSESOLVER(name))
    {
        if (cam.K.empty())
        {
            LOG_CRIT("Camera Param Not Not Intialized!!!");
            exit(-1);
        }
        mpPoseEst->setCamera(cam);
    }

    //! 执行
    PoseResult PoseTask::run(const MatcherTask::Item &item)
    {
        InputPair input(item.queryItem->getKeys(),
                        item.trainItem->getKeys(),
                        item.matches);
        return std::move(mpPoseEst->estimate(input));
    }

    /*
        * 姿态估算
        */

    PoseFlowTask::PoseFlowTask(const Position::CameraParam &cam,
                            const std::string &feature /*= "SiftEx"*/,
                            const std::string &match   /*= "Knn"*/,
                            const std::string &est     /*= "CVPoseSolver"*/) : mpFeature(feature, cam), mpMatcher(match), mpPoser(est, cam)
    {
    }

    PoseResult PoseFlowTask::run(Item &item)
    {
        assert(item.queryF && item.trainF);
        std::unique_ptr<Position::IFrame> pf(mpFeature.run(item.queryF));
        std::unique_ptr<Position::IFrame> cr(mpFeature.run(item.trainF));
        MatcherTask::Item mi(pf.get(), cr.get());
        mpMatcher.run(mi);
        return mpPoser.run(mi);
    }
} // namespace Position