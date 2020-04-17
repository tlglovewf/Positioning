#include "P_Controller.h"
#include "P_Factory.h"
#include "P_Detector.h"
#include "P_Writer.h"

#include "P_MultiVisionTrajProcesser.h"

#include "project/hdproject.h"
#include "project/weiyaproject.h"


#include "P_Map.h"
#include "P_Factory.h"
#include "P_Utils.h"

#include "P_FrameViewer.h"

#include "P_PangolinViewer.h"

#include "P_SemanticGraph.h"

#include "P_GpsFusion.h"

#include <thread>

using namespace std;
using namespace cv;

#define WEIYA           0  //是否为weiya数据
#define USECONTROLLER   1  //是否启用定位框架


void MapDisplay(const std::shared_ptr<Position::IConfig> &pCfg)
{
    string outpath = GETCFGVALUE(pCfg,OutPath,string) + "/";
#if 1
    std::shared_ptr<Position::IMap> baseMap(new Position::PMap());
    std::shared_ptr<Position::IMap> secMap(new Position::PMap());

    Position::MapSerManager::Instance()->setMap(baseMap);
    Position::MapSerManager::Instance()->tracSerPtr()->loadMap(outpath + "trac1.txt");
    Position::MapSerManager::Instance()->mpPtSerPtr()->loadMap(outpath + "mpts1.txt");

    Position::MapSerManager::Instance()->setMap(secMap);
    Position::MapSerManager::Instance()->tracSerPtr()->loadMap(outpath + "trac.txt");
    Position::MapSerManager::Instance()->mpPtSerPtr()->loadMap(outpath + "mpts.txt");

    Position::MapSerManager::Instance()->combineMap(baseMap,secMap);

    Position::IViewer *pv = Position::PFactory::CreateViewer(Position::eVPangolin,pCfg);
    pv->setMap(baseMap);
    pv->renderLoop();

#else

    Position::MapSerManager::Instance()->displayMap(pCfg,outpath + "trac.txt",outpath + "mpts.txt");

#endif
}



void BatchTraceDisplay(const std::shared_ptr<Position::IProjList> &prj,const std::shared_ptr<Position::IConfig> &pcfg)
{
    if(prj)
    {
        std::shared_ptr<Position::IViewer> pviewer(Position::PFactory::CreateViewer(Position::eVPangolin,pcfg));
        
        std::shared_ptr<Position::IMap> pmap(new Position::PMap);
        Position::PrjBatchVector &batchvector = prj->getPrjList();
        Position::PrjBatchVIter  iter = batchvector.begin();
        Position::PrjBatchVIter ed    = batchvector.end(); 

        //每个batch 间隔的空隙
        Mat spaceLen = Mat::zeros(4,4,MATCVTYPE);
        
        int index = 0;
        
        for(;iter != ed; ++iter)
        {
            spaceLen.at<MATTYPE>(0,3) = 10 * index++;
            if(iter->_poses.empty())
                break;
            PROMTD_V("display batch ", iter->_btname.c_str());
            cout << iter->_n << " " << iter->_poses.size() << endl;
            for(int i = 0;i < iter->_n; ++i)
            {
                if(iter->_poses[i].empty())
                    continue;
                Position::FrameData data;
                data._name = iter->_names[i];
                Position::IMap::CreateKeyFrame(pmap,data,spaceLen + iter->_poses[i]);
            }
            PROMTD_V("display end ", iter->_btname.c_str());
        }
        pviewer->setMap(pmap);
        pviewer->renderLoop();
    }
}

//加载hd batch
void LoadBatchList(const std::shared_ptr<Position::IConfig> &pCfg)
{
    const string bathpath = "/media/tlg/work/tlgfiles/hdoutformat/vslam.batch";
    const string outpath  = "/media/tlg/work/tlgfiles/hdoutformat/out.txt"; 
    
    const string imgpath  = GETCFGVALUE(pCfg,ImgPath,string);

    std::shared_ptr<Position::IProjList> prjlist(new HdPosePrj());
    prjlist->loadPrjList(bathpath);
    Position::PrjBatchVector &batches = prjlist->getPrjList();

    Position::PrjBatchVIter it = batches.begin();
    Position::PrjBatchVIter ed = batches.end();

    std::shared_ptr<Position::IData> pData(new HdData(pCfg));

    std::shared_ptr<Position::ITrajProcesser> pTraj(Position::PFactory::CreateTrajProcesser(Position::eUniformSpeed,pCfg,pData));
    std::shared_ptr<Position::IMap> map = pTraj->getMap();
    // std::shared_ptr<Position::IGpsFusion> gpsfusion(new Position::GpsFunsion());
    
    int index = 0;
    for(; it != ed; ++it)
    {
        //for test 
        // if(index++ > 10)
        //     break;
        cout << "Load Batch:" << it->_btname.c_str() << endl;

        Position::FrameDataVector framedatas;
        framedatas.reserve(it->_n);
        for(int i = 0; i < it->_n; ++i)
        {
            Position::FrameData fdata;
            fdata._name = it->_names[i];
            fdata._img  = imread(imgpath + "/" + it->_names[i] + ".jpg");
            if(fdata._img.channels() > 1)
            {
                cvtColor(fdata._img,fdata._img,CV_RGB2GRAY);
            }
            
            framedatas.emplace_back(fdata);
        }
        cout << "frame data size : " << framedatas.size() << endl;
        if(pTraj->process(framedatas))
        {
            //gps 融合
            // gpsfusion->fuse(pTraj->getMap(),pData->getCamera());
            cout << "Save Batch " << it->_btname.c_str() << "pose info" << endl;
            Position::KeyFrameVector frames = map->getAllFrames();

            for(size_t i = 0; i < it->_names.size(); ++i)
            {
                const std::string &nm = it->_names[i];

                Position::KeyFrameVter kit = std::find_if(frames.begin(),frames.end(),[nm](const Position::IKeyFrame *pframe)->bool
                {
                    return pframe->getData()._name == nm;
                });

                if(kit == frames.end())
                {
                    it->_poses.emplace_back(Mat());
                }
                else
                {
                    it->_poses.emplace_back((*kit)->getPose());
                }
                
            }
        }
        pTraj->reset();
    }
    BatchTraceDisplay(prjlist,pCfg);
    prjlist->saveMap(outpath);
}







int main(void)
{  
#if WEIYA
    std::shared_ptr<Position::IConfig> pCfg = std::make_shared<WeiyaConfig>("../config/config_weiya.yaml"); 
    std::shared_ptr<Position::IData> pData(new WeiyaData(pCfg));
#else
    std::shared_ptr<Position::IConfig> pCfg = std::make_shared<HdConfig>("../config/config_hd.yaml"); 
    std::shared_ptr<Position::IData> pData(new HdData(pCfg));
#endif

    std::string   sempath = GETCFGVALUE(pCfg,SemPath,string);
    if(!sempath.empty())
    {
        SemanticGraph::Instance()->loadObjInfos("../config/semgraph.cfg");
        SemanticGraph::Instance()->setSemanticPath(sempath);
    }

    std::shared_ptr<Position::IDetector> pdetecter = std::make_shared<Position::SSDDetector >();

    // MapDisplay(pCfg);

    LoadBatchList(pCfg);
    return 0;

#if USECONTROLLER
    std::unique_ptr<PositionController> system(new PositionController(pdetecter,pData,pCfg));

    Position::Time_Interval t;
    t.start();
    system->run();
    t.prompt("total cost : ");
    return 0;
#else
    //multi vision situation test 
    pData->loadDatas();
    const string imgpath = GETCFGVALUE(pCfg,ImgPath ,string) + "/";
    const string outpath = GETCFGVALUE(pCfg,OutPath ,string) + "/";
    std::shared_ptr<Position::ITrajProcesser> pTraj(Position::PFactory::CreateTrajProcesser(Position::eMultiVision,pCfg,pData));
    std::shared_ptr<Position::IMap> map = pTraj->getMap();
    Position::FrameDataVector datas;
    Position::FrameDataVIter iter = pData->begin();
    Position::FrameDataVIter ed   = pData->end();
     //可视化帧数据
    std::shared_ptr<Position::IViewer> pv(Position::PFactory::CreateViewer(Position::eVPangolin,pCfg));
    std::thread mainthread(&Position::IViewer::renderLoop,pv.get());
    //  pv->setMap(map);
     pTraj->setViewer(pv);
    //插入帧数据
    for(;iter != ed ; ++iter)
    {
        Mat img = imread(imgpath + iter->_name ,IMREAD_UNCHANGED);
        if(img.empty())
            continue;
        iter->_img = img;
        datas.push_back(*iter);
    }
    //处理帧数据
    pTraj->process(datas);

   
    mainthread.join();
#endif
    return 0;
}