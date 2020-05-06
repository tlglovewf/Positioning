/**
 *   P_ProjList.h
 *   
 *   add by tu li gen   2020.4.27
 * 
 */
#ifndef _PROJLIST_H_H_
#define _PROJLIST_H_H_

namespace Position
{
    class ProjList : public IProjList
    {
        public:

       //设置地图
       virtual void setMap(const std::shared_ptr<Position::IMap> &pmap)
       {
           mpMap = pmap;
       }
       //加载地图
       virtual void load(const std::string &path)
       {
           assert(NULL);
           //add more
       }
       //保存地图
       virtual void save(const std::string &path)
       {
           assert(NULL);
           //add more
       }
       //获取项目列表
       virtual Position::PrjBatchVector& getPrjList() 
       {
           return mBatches;
       }

       //设置生成器
       virtual void setBatcherGenerator(const std::shared_ptr<IBatchesGenerator> &pGtor)
       {
           mpGenerator = pGtor;
       }

       //目标信息
       virtual TrackerItemVector& trackInfos() 
       {
           return mTrackerInfos;
       }

    protected:
        //打开文件
        bool open(const std::string &path,std::ios::openmode type)
        {
            mfile.open(path, type);
            return mfile.is_open();
        }

    protected:
       std::fstream                        mfile;
       Position::PrjBatchVector            mBatches;
       std::shared_ptr<Position::IMap>     mpMap;
       std::shared_ptr<IBatchesGenerator>  mpGenerator;
       TrackerItemVector                   mTrackerInfos;
    };
}

#endif