#include "P_PoseEstimator.h"
#include "P_Factory.h"

namespace Position
{
    PoseEstimator::PoseEstimator(const std::shared_ptr<IConfig> &pCfg, const CameraParam &cam, int eType /*= 1*/)
    {
        if(0 == eType)
        {
             mpUniformVTrajPro   = std::shared_ptr<ITrajProcesser>(PFactory::CreateTrajProcesser(eTjUniformSpeed, pCfg, cam));
        }
        else
        {
            /* code */
        }

        mpSimpleTrajPro     = std::shared_ptr<ITrajProcesser>(PFactory::CreateTrajProcesser(eTjMultiVision, pCfg, cam));
        mpCurrentTrajPro    = eType ? mpSimpleTrajPro : mpUniformVTrajPro;

        if(2 == eType)
        {
            mpCurrentTrajPro = std::shared_ptr<ITrajProcesser>(PFactory::CreateTrajProcesser(eTjSfmVision, pCfg, cam));
        }
    }

    bool PoseEstimator::handle(FrameData *fdata)
    {
        if (!mpCurrentTrajPro)
            mpCurrentTrajPro = mpUniformVTrajPro;

        mpCurrentTrajPro->track(fdata);
    }

    //处理帧数据
    bool PoseEstimator::process(const FrameDataPtrVector &datas, const std::string &imgpath /*=""*/)
    {

        if (!imgpath.empty())
        { //地址不为空 需要加载图片
            for (FrameData *data : datas)
            {
                data->_img = imread(imgpath + "/" + data->_name, IMREAD_UNCHANGED);
            }
        }

        return mpCurrentTrajPro->process(datas);
    }

    //获取地图
    const std::shared_ptr<IMap> &PoseEstimator::getMap()
    {
        assert(mpCurrentTrajPro);
        return mpCurrentTrajPro->getMap();
    }

    //设置可视接口
    void PoseEstimator::setViewer(const std::shared_ptr<IViewer> &pviewer)
    {
        mpViewer = pviewer;
        assert(mpViewer);
        mpViewer->setMap(mpCurrentTrajPro->getMap());
        mpCurrentTrajPro->setViewer(pviewer);
    }

    //重置状态
    void PoseEstimator::reset()
    {
        assert(mpCurrentTrajPro);
        mpCurrentTrajPro->reset();
    }

    //等待处理
    void PoseEstimator::waitingForHandle()
    {
        if (mpCurrentTrajPro)
        {
            mpCurrentTrajPro->wait();
        }
    }

} // namespace Position
