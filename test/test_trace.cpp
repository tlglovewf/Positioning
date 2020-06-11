#include "P_MapDisplay.h"
#include "P_Factory.h"
#include "P_IOHelper.h"

#include "P_MultiVisionTrajProcesser.h"

#include "project/hdproject.h"
#include "project/weiyaproject.h"

#include "P_Map.h"
#include "P_Factory.h"
#include "P_MapSerializor.h"
#include "P_Utils.h"

#include "P_FrameViewer.h"

#include "P_PangolinViewer.h"

#include "P_SemanticGraph.h"

#include "P_GpsFusion.h"

#include <thread>

#include "Thirdparty/GeographicLib/include/LocalCartesian.hpp"
#include "P_CoorTrans.h"


using namespace std;
using namespace cv;

#define WEIYA           0  //是否为weiya数据
#define USECONTROLLER   1  //是否启用定位框架



//self define format file
void MapDisplay(const std::shared_ptr<Position::IConfig> &pCfg)
{
#ifdef USE_VIEW
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

    std::shared_ptr<Position::IViewer> pv(GETVIEWER());
    pv->setMap(baseMap);
    pv->renderLoop();

#else

    //Position::MapSerManager::Instance()->displayMap(pCfg,outpath + "trac.txt",outpath + "mpts.txt");

#endif

#endif
}

//hd batch trace dispaly
void BatchTraceDisplay(const std::shared_ptr<Position::IProjList> &prj,const std::shared_ptr<Position::IConfig> &pcfg)
{
#ifdef USE_VIEW
    if(prj)
    {
        std::shared_ptr<Position::IViewer> pviewer(GETVIEWER());
        
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
            if((*iter)->_poses.empty())
                break;
            PROMTD_V("display batch ", (*iter)->_btname.c_str());
            cout << (*iter)->_n << " " << (*iter)->_poses.size() << endl;
            for(int i = 0;i < (*iter)->_n; ++i)
            {
                if((*iter)->_poses[i].empty())
                    continue;
                Position::FrameData *data = (*iter)->_fmsdata[i]; 
                Position::IMap::CreateKeyFrame(pmap,data,spaceLen + (*iter)->_poses[i]);
            }
            PROMTD_V("display end ", (*iter)->_btname.c_str());
        }
        pviewer->setMap(pmap);
        pviewer->renderLoop();
    }
#endif
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

    std::shared_ptr<Position::IFrameData> pData(new HdData(pCfg));

    std::shared_ptr<Position::ITrajProcesser> pTraj(GETTRJPROCESSER("MViewsTraj")); 
    std::shared_ptr<Position::IMap> map = pTraj->getMap();
    // std::shared_ptr<Position::IGpsFusion> gpsfusion(new Position::GpsFunsion());
    
    //std::shared_ptr<Position::IViewer> mpViewer = std::shared_ptr<Position::IViewer>(GETVIEWER());
    //pTraj->setViewer(mpViewer);
    //std::unique_ptr<std::thread>  mptViewer = std::unique_ptr<std::thread>(new thread(&Position::IViewer::renderLoop,mpViewer));

    int index = 0;
    Position::Time_Interval timer;
    timer.start();
    for(; it != ed; ++it)
    {
        //for test 
        // if(index++ >= 10)
        //     break;
        cout << "Load Batch:" << (*it)->_btname.c_str() << endl;

        Position::FrameDataPtrVector framedatas;
        framedatas.reserve((*it)->_n);
        for(int i = 0; i < (*it)->_n; ++i)
        {
            Position::FrameData *fdata = (*it)->_fmsdata[i];
            fdata->_img  = imread(imgpath + "/" + fdata->_name + ".jpg");
            if(fdata->_img.channels() > 1)
            {
                cvtColor(fdata->_img,fdata->_img,CV_RGB2GRAY);
            }
            
            framedatas.emplace_back(fdata);
        }
        cout << "frame data size : " << framedatas.size() << endl;
        if(pTraj->process(framedatas))
        {
            //gps 融合
            // gpsfusion->fuse(pTraj->getMap(),pCfg->getCamera());
            cout << "Save Batch " << (*it)->_btname.c_str() << "pose info" << endl;
            Position::KeyFrameVector frames = map->getAllFrames();

            for(size_t i = 0; i < (*it)->_fmsdata.size(); ++i)
            {
                const std::string &nm = (*it)->_fmsdata[i]->_name;

                Position::KeyFrameVter kit = std::find_if(frames.begin(),frames.end(),[nm](const Position::IKeyFrame *pframe)->bool
                {
                    return pframe->getData()->_name == nm;
                });

                if(kit == frames.end())
                {
                    (*it)->_poses.emplace_back(Mat());
                }
                else
                {
                    Mat posefuse = Mat::eye(4,4,MATCVTYPE);
                    (*kit)->getRotation().copyTo(posefuse.rowRange(0,3).colRange(0,3));
                    (*kit)->getTranslation().copyTo(posefuse.rowRange(0,3).col(3));
                    (*it)->_poses.emplace_back(posefuse);
                }
                
            }
        }
        pTraj->reset();
    }
    BatchTraceDisplay(prjlist,pCfg);
    timer.prompt("process traj",true);
    prjlist->save(outpath);
    timer.prompt("save traj");
}

//display batch result

void DisplayBatchResult( const std::string &path,const std::shared_ptr<Position::IConfig> &pcfg)
{   
#ifdef USE_VIEW
    std::ifstream sfile;
    sfile.open(path);

    std::shared_ptr<Position::IViewer> pviewer(GETVIEWER());

    std::shared_ptr<Position::IMap> pmap(new Position::PMap);

    if(sfile.is_open())
    {
        std::string line;
        getline(sfile,line);//get header
         //每个batch 间隔的空隙
        Mat spaceLen = Mat::zeros(4,4,MATCVTYPE);
        
        int index = 0;
   
        while(!sfile.eof())
        {
            getline(sfile,line);
            if(line.empty())
                continue;
            //read batch name
            char nm[255] = {0};
            int len = 0;
            sscanf(line.c_str(),"%s %d",nm,&len);

            spaceLen.at<MATTYPE>(0,3) = 10 * index++;
            for(size_t i = 0; i < len; ++i)
            {
                getline(sfile,line);
                memset(nm,0,255);
                int v;
                MATTYPE R00,R01,R02,R10,R11,R12,R20,R21,R22;
                MATTYPE T0,T1,T2;
                sscanf(line.c_str(),"%s %d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf", 
                                     nm,&v,&R00,&R01,&R02,
                                           &R10,&R11,&R12,
                                           &R20,&R21,&R22,
                                           &T0 ,&T1 ,&T2);
                Mat pose = Mat::eye(4,4,MATCVTYPE);
                pose.at<MATTYPE>(0,0) = R00;
                pose.at<MATTYPE>(0,1) = R01;
                pose.at<MATTYPE>(0,2) = R02;
                pose.at<MATTYPE>(1,0) = R10;
                pose.at<MATTYPE>(1,1) = R11;
                pose.at<MATTYPE>(1,2) = R12;
                pose.at<MATTYPE>(2,0) = R20;
                pose.at<MATTYPE>(2,1) = R21;
                pose.at<MATTYPE>(2,2) = R22;
                pose.at<MATTYPE>(0,3) = T0;
                pose.at<MATTYPE>(1,3) = T1;
                pose.at<MATTYPE>(2,3) = T2;
                Position::FrameData *fdata = new Position::FrameData();
                fdata->_name = nm;
                cout << Position::IMap::CreateKeyFrame(pmap,fdata,spaceLen + pose)->index() << endl;
            }
        }
        sfile.close();
    }

    pviewer->setMap(pmap);
    pviewer->renderLoop();
#endif
}


int main(void)
{  

#if WEIYA
    std::shared_ptr<Position::IConfig> pCfg = std::make_shared<WeiyaConfig>("../config/config_weiya.yaml"); 
    std::shared_ptr<Position::IFrameData> pData(new WeiyaData(pCfg));
#else
    std::shared_ptr<Position::IConfig> pCfg = std::make_shared<HdConfig>("../config/config_hd.yaml"); 
    std::shared_ptr<Position::IFrameData> pData(new HdData(pCfg));
#endif

    std::string   sempath = GETCFGVALUE(pCfg,SemPath,string);
    if(!sempath.empty())
    {
        SETDYOSETTING("../config/semgraph.cfg");
        SETSEMANTICPATH(sempath);
    }

    // MapDisplay(pCfg);


    LoadBatchList(pCfg);
    return 0;

    // DisplayBatchResult("", pCfg);

#if USECONTROLLER
    std::unique_ptr<Position::PMapDisplay> system(new Position::PMapDisplay(pData,pCfg));

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
    std::shared_ptr<Position::ITrajProcesser> pTraj(Position::PFactory::CreateTrajProcesser(Position::eTjMultiVision,pCfg,pCfg->getCamera()));
    std::shared_ptr<Position::IMap> map = pTraj->getMap();
    Position::FrameDataPtrVector datas;
    Position::FrameDataPtrVIter iter = pData->begin();
    Position::FrameDataPtrVIter ed   = pData->end();
     //可视化帧数据
    std::shared_ptr<Position::IViewer> pv(GETVIEWER());

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

    pv->renderLoop();
#endif
    return 0;
}