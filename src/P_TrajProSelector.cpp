#include "P_TrajProSelector.h"
#include "P_Factory.h"

namespace Position
{
   TrajProSelector::TrajProSelector(const std::shared_ptr<IConfig> &pCfg, const std::shared_ptr<IData> &pdata)
   {
       mpUniformVTrajPro = std::shared_ptr<ITrajProcesser>(PFactory::CreateTrajProcesser(eUniformSpeed,pCfg,pdata));
   }

     //处理帧数据
   bool TrajProSelector::handle(const FrameDataVector &datas)
   {
       size_t sz = datas.size();
       if(sz < 4)
       {//小于5帧 使用简单定位场景
            mpCurrentTrajPro = mpSimpleTrajPro;
       }
       else
       {//其他情况 使用匀速运动模型推算位姿
            mpCurrentTrajPro = mpUniformVTrajPro;
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
       mpViewer->init();
       mpViewer->setMap(mpCurrentTrajPro->getMap());
   }

   //重置状态
   void TrajProSelector::reset()
   {
       assert(mpCurrentTrajPro);
       mpCurrentTrajPro->reset();
   }
} // namespace Position
