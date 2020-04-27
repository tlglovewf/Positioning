#include "P_TrajProSelector.h"
#include "P_Factory.h"

namespace Position
{
   TrajProSelector::TrajProSelector(const std::shared_ptr<IConfig> &pCfg, const std::shared_ptr<IData> &pdata)
   {
    //    mpUniformVTrajPro = std::shared_ptr<ITrajProcesser>(PFactory::CreateTrajProcesser(eTjUniformSpeed,pCfg,pdata));
       mpSimpleTrajPro   = std::shared_ptr<ITrajProcesser>(PFactory::CreateTrajProcesser(eTjMultiVision,pCfg,pdata));
       mpCurrentTrajPro = mpSimpleTrajPro; //mpUniformVTrajPro;
   }

   bool TrajProSelector::handle(const FrameData &fdata)
   {
       if(!mpCurrentTrajPro)
            mpCurrentTrajPro = mpUniformVTrajPro;
    
        mpCurrentTrajPro->track(fdata);
   }


     //处理帧数据
   bool TrajProSelector::process( FrameDataVector &datas,const std::string &imgpath /*=""*/)
   {
       
       if(!imgpath.empty())
       {//地址不为空 需要加载图片
            for(FrameData &data : datas)
            {
                data._img = imread(imgpath + "/" + data._name,IMREAD_UNCHANGED);
            }
       }

       return mpCurrentTrajPro->process(datas);
   }

   //获取地图
   const std::shared_ptr<IMap>& TrajProSelector::getMap()
   {
       assert(mpCurrentTrajPro);
       return mpCurrentTrajPro->getMap();
   }

   //设置可视接口
   void TrajProSelector::setViewer(const std::shared_ptr<IViewer> &pviewer)
   {
       mpViewer = pviewer;
       assert(mpViewer);
       mpViewer->setMap(mpCurrentTrajPro->getMap());
       mpCurrentTrajPro->setViewer(pviewer);
   }

   //重置状态
   void TrajProSelector::reset()
   {
       assert(mpCurrentTrajPro);
       mpCurrentTrajPro->reset();
   }

    //等待处理
    void TrajProSelector::waitingForHandle()
    {
        if(mpCurrentTrajPro)
        {
            mpCurrentTrajPro->wait();
        }
    }

} // namespace Position
