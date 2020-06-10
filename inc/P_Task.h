/**
 *   P_Task.h
 *   
 *   add by tu li gen   2020.6.10
 * 
 */
#ifndef _TASK_H_H_
#define _TASK_H_H_
#include "P_Interface.h"
namespace Position
{
    /*
    * 特征提取任务
    */
    class FeatureTask
    {
    public:
        FeatureTask(const std::string &fname, const Position::CameraParam &cam);

        //! run one item
        IFrame *run(FrameData *item);

        FrameVector run(const Position::FrameDataPtrVector &items);

    protected:
        //! run one item
        IFrame *runimpl(const std::string &path, Position::FrameData *item);

    protected:
        std::shared_ptr<Position::IFeature> mpFeature;
        CameraParam mCamera;
        static int s_fmcount;
    };

    /*
    * 特征匹配任务
    */
    class MatcherTask
    {
    public:
        struct Item
        {
            Position::IFrame *queryItem;
            Position::IFrame *trainItem;
            Position::MatchVector matches;
            Item(Position::IFrame *pf, Position::IFrame *cr) : queryItem(pf), trainItem(cr) {}
        };
        typedef std::vector<Item> TaskItems;
        MatcherTask(const std::string &name) ;

        //! 处理单个
        void run(Item &item);

        //! 批执行
        void run(TaskItems &items);

    protected:
        std::shared_ptr<Position::IFeatureMatcher> mpFeatureMatcher;
    };

    /*
    * 位姿估计任务
    */
    class PoseTask
    {
    public:
        PoseTask(const std::string &name, const Position::CameraParam &cam);
        //! 执行
        PoseResult run(const MatcherTask::Item &item);

    protected:
        std::shared_ptr<Position::IPoseSolver> mpPoseEst;
    };

    /*
    * 姿态估算
    */
    class PoseFlowTask
    {
    public:
        struct Item
        {
            Position::FrameData *queryF;
            Position::FrameData *trainF;
            Item(Position::FrameData *query, Position::FrameData *train) : queryF(query), trainF(train) {}
        };
        PoseFlowTask(const Position::CameraParam &cam,
                    const std::string &feature = "SiftEx",
                    const std::string &match = "Knn",
                    const std::string &est = "CVPoseSolver");
                    
        PoseResult run(Item &item);

    protected:
        FeatureTask mpFeature;
        MatcherTask mpMatcher;
        PoseTask mpPoser;
    };

} // namespace Position

#endif
