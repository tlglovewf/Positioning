
#include "project/imgautoproject.h"
#include "P_Writer.h"
#include "P_Utils.h"
#include "P_Checker.h"
#include  "Thirdparty/rapidjson/document.h"

namespace Position
{

#define SPLITSTR ";"   //分割字符  

    ImgAutoConfig::ImgAutoConfig(const std::string &path):
          PrjPath(""),
          CamMatrixPath("")
    {
        PUSH_MAP(PrjPath);
        PUSH_MAP(CamMatrixPath);

        load(path);   
    }

    //其他信息加载
    void ImgAutoConfig::loadmore() 
    {
        READ_VALUE(PrjPath);
        READ_VALUE(CamMatrixPath);
    }

    void ImgAutoData::loadCameraParams(const std::string &path)
    {
        //read camera matrix
        FileStorage intr(path, FileStorage::READ);
        if (intr.isOpened())
	    {
	    	intr["P1"] >> mCamera.K;
	    	if (mCamera.K.cols > 3)
	    	{
	    		mCamera.K = mCamera.K.colRange(0, 3);
	    	}
            if(mCamera.K.type() != MATCVTYPE)
            {
                mCamera.K.convertTo(mCamera.K,MATCVTYPE);
                mCamera.RCam2Imu.convertTo(mCamera.RCam2Imu,MATCVTYPE);
                mCamera.TCam2Imu.convertTo(mCamera.TCam2Imu,MATCVTYPE);
            }
            mCamera.fps = GETCFGVALUE(mpCfg,ImgFps,int);
            mCamera.rgb = GETCFGVALUE(mpCfg,ImgRgb,int);
            PROMT_S("Camera params >>>>>>>>>>>>>>>>>>>>>>>>>");
            PROMT_V("K ",mCamera.K);
            PROMT_V("Fps ",(int)mCamera.fps);
            PROMT_V("Rgb ",(int)mCamera.rgb);
            PROMT_S("<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");
            //add
	    }
	    else
	    {
            LOG_ERROR("Read camera config failed!!!")
	    }

#if 0  //read camera ex paramers
        FileStorage bs  (bspath, FileStorage::READ);
	    if (bs.isOpened())
	    {
	    	bs["RotateMatrixCam2IMU"] >> mCamera.RCam2Imu;
	    	bs["TranslationVectorCam2IMU"] >> mCamera.TCam2Imu;
            PROMT_S("Camera params >>>>>>>>>>>>>>>>>>>>>>>>>");
            PROMT_V("Cam2ImuR",mCamera.RCam2Imu);
            PROMT_V("Cam2ImuT",mCamera.TCam2Imu);
            PROMT_S("<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");
	    }
	    else
	    {
            PROMT_S("Read config failed!!!")
	    }
#endif 
    }

     //加载track json
    bool ImgAutoData::loadTrackJson(const std::string &path,FrameDataVector &framedatas)
    {
        if(!PATHCHECK(path) || framedatas.empty())
        {
            LOG_ERROR_F("%s not found.",path.c_str());
            return false;
        }

        LOG_INFO_F("%s Load ...",path.c_str()); 

        ifstream json_file;
	    json_file.open(path.c_str());
	    string line, json;
	    if (!json_file.is_open())
	    {
            LOG_CRIT("Track json can not be open.");
	    	return 0;
	    }
	    while (!json_file.eof())
	    {
            getline(json_file, line);
	    	json.append(line);
	    }
	    json_file.close();

        rapidjson::Document doc;
        doc.Parse<0>(json.c_str());

        if (doc.HasMember("errmsg"))
        {
            rapidjson::Value &errmsgValue = doc["errmsg"];
            string errmsg = errmsgValue.GetString();
            if (errmsg != "OK")
            {
                LOG_CRIT("Json file error! check if the file is available!");
                return 0;
            }
            else
            {
                if (doc.HasMember("data"))
                {
                    rapidjson::Value &dataArray = doc["data"];
                    if (dataArray.IsArray())
                    {
                        rapidjson::Value& dataValue = dataArray[0];
                        
                        if (dataValue.HasMember("trackMark"))
                        {
                            rapidjson::Value &trmkArray = dataValue["trackMark"];
                            
                            if (trmkArray.IsArray())
                            {
                                LOG_INFO_F("trk size:%d",trmkArray.Size());
                                LOG_INFO_F("fmt size:%d",framedatas.size());

                                if(trmkArray.Size() != framedatas.size() )
                                {
                                    LOG_ERROR("Frame size and track array size not same!");

                                    return false;
                                }
                                for (int j = 0; j < trmkArray.Size(); j++)
                                {
                                    PoseData &item = framedatas[j]._pos;
                                    int mseq;
                                    rapidjson::Value& trmkValue = trmkArray[j];
                                    if (trmkValue.HasMember("latitude"))
                                    {
                                        item.pos.lat = trmkValue["latitude"].GetDouble();
                                    }
                                    if (trmkValue.HasMember("longitude"))
                                    {
                                        item.pos.lon = trmkValue["longitude"].GetDouble();
                                    }
                                    if (trmkValue.HasMember("seqNum"))
                                    {
                                         mseq = atoi(trmkValue["seqNum"].GetString()) - 1;
                                         if(mseq != j)
                                         {
                                             LOG_ERROR_F("%s seq numb error",framedatas[j]._name.c_str());
                                         }
                                    }
                                }
                            }
                        }
                        else
                        {
                            LOG_ERROR("Track file is empty.");
                            return false;
                        }	
                    }
                }
                else
                {
                    LOG_ERROR("Json File is emtpy.");
                    return false;
                }
                LOG_INFO("Json File finished.");
                return true;
            }
        }
        return false;
    }

    //读取trk文档
    static void ReadTrkTxt(const string &txtfile, StringVector &lines)
    {

        if(!PATHCHECK(txtfile))
        {
            LOG_ERROR_F("%s-%s",txtfile.c_str(),"not exsit.");
        }

    	ifstream txt_file;
    	txt_file.open(txtfile.c_str());
    	assert(txt_file.is_open());
    	string str;
    	while (!txt_file.eof())
    	{
            getline(txt_file, str);
            if(str.empty())
                continue;
    		lines.emplace_back(str);
    	}
    	txt_file.close();
    }


    //预处理数据
    bool ImgAutoData::loadDatas()
    {
        assert(mpCfg);
        //get path 
        const std::string expath        = GETCFGVALUE(mpCfg,PrjPath,string);

        if(expath.empty())
            return false;

        const std::string raw_data      = expath    + "raw_data";
        const std::string seqpath       = raw_data  + "/seq.txt";
        const std::string trkjson       = raw_data  + "/track.json";
        const std::string imgpath       = raw_data  + "/image/";
        const std::string trackerpath   = expath    + "tracker";
        const std::string trackrestpath = trackerpath + "/tracker_result/";
        const std::string campath       = expath    + "config/extrinsics.xml";

        //设置图片路径
        SETCFGVALUE(mpCfg,ImgPath,string(imgpath));

        //load camera paramers
        loadCameraParams(campath);

        try 
        {
            ifstream seqfile;
            seqfile.open(seqpath);
            if(!seqfile.is_open())
            {
                return false;
            }
            //区文件名
            mFrameDatas.clear();  
            while(!seqfile.eof())
            {
                std::string seqline;
                getline(seqfile,seqline);
                if(seqline.empty())
                    continue;
                FrameData fdata;
                size_t npos = seqline.find_last_of("/");
                fdata._name = seqline.substr(++npos);
                mFrameDatas.emplace_back(fdata);
            }
            seqfile.close();

            //加载track json 数据获取图片gps信号数据
            if(!loadTrackJson(trkjson,mFrameDatas))
                return false;

#if 0
            const std::string strori = "/media/tlg/work/tlgfiles/HDData/result/ori.txt";
            std::ofstream fori;
            fori.open(strori);
            for(int i = 0; i < framedatas.size(); ++i )
            {
                PStaticWriter::WriteRealTrace(fori, framedatas[i]._pos.pos ,framedatas[i]._name);
            }
            fori.close();
#endif
         
            //根据每帧加载对应图像识别信息
            //trk 文件每行对应一张图像
            for(size_t i = 0; i < mFrameDatas.size(); ++i)
            {
                std::string trkfile = mFrameDatas[i]._name;
                PUtils::ReplaceFileSuffix(trkfile,"jpg","trk");
                trkfile = trackrestpath  + trkfile;
                StringVector lines;
                ReadTrkTxt(trkfile,lines);
                if(!lines.empty())
                {//数据不为空,则表示当前帧没有识别的目标
                    for(const string &item : lines)
                    {//遍历目标
                       //根据拆分字符 拆分字符串
                       StringVector values = PUtils::SplitString(item,SPLITSTR);
                       
                       //获取包围盒-像素坐标
                       int xmin = atoi(values[0].c_str());   
                       int ymin = atoi(values[1].c_str());
                       int xmax = atoi(values[2].c_str());
                       int ymax = atoi(values[3].c_str());
                       int id   = atoi(values[10].c_str()); //获取id

                       TargetData target;
                       target._box  = Rect2f( xmin,ymin,(xmax - xmin),(ymax - ymin) );
                       target._type = id;
                       //插入目标信息
                       mFrameDatas[i]._targets.emplace_back(target);
                    }
                }
                else
                {
                    LOG_DEBUG_F("%s:no target!!!",mFrameDatas[i]._name.c_str());
                }
            }


        }catch(exception e)
        {
            LOG_ERROR_F("%s(%d) throw %s",__FILE__,__LINE__,e.what());
        }
      
        //load frame datas



        return true;
    }

    static inline InfoIndex parseIndex(const std::string &str)
    {
         static std::regex re(string("\\(\\d*, \\d*\\)"));
         bool ret = std::regex_match(str,re);
         if(ret)
         {
             int a,b;
             sscanf(str.c_str(),"(%d,%d)",&a,&b);
             return std::make_pair(a,b);
         }
         else
         {
             return std::make_pair(-1,-1);
         }
    }

    void ImgAutoPrjList::parseTracker(const std::string &line)
    {
        assert(!line.empty());
        StringVector values = PUtils::SplitString(line,SPLITSTR);
        if(!values.empty())
        {
            TrackerItem item;
            item.id         = atoi(values[10].c_str()); //目标id
            item.maxsize    = atoi(values[11].c_str()); // 在多少张图像中出现

            string startseq = values[12];               //(图片索引，探测框索引) 目标第一次出现图的索引
            string endseq   = values[values.size() - 1];//取最后一个 表示最后一帧的索引

            item.stno = parseIndex(startseq);
            item.edno = parseIndex(endseq);
            //add more information

            mTrackerInfos.emplace_back(item);
        }
    }


    //加载项目列表
    void ImgAutoPrjList::loadPrjList(const std::string &path)
    {
        if(!PATHCHECK(path))
        {
            LOG_ERROR_F("%s path error..",path.c_str());
            return;
        }
            
        if(!path.empty() && open(path,ios::in))
        {
            while(!mfile.eof())
            {
                std::string line;
                getline(mfile,line);
                if(line.empty())
                    continue;
                parseTracker(line);
                mTrkLines.emplace_back(line);
            }
            mfile.close();
        }

        assert(mpData);
        assert(mpGenerator);
        //genearte position batch 
        mBatches = std::move(mpGenerator->generate(mpData,mTrackerInfos));
    }
    //加载地图
    void ImgAutoPrjList::load(const std::string &path)
    {
       //add more
    }
    //保存地图
    void ImgAutoPrjList::save(const std::string &path)
    {
        if(!path.empty() && open(path,ios::out))
        {   
            assert(mTrkLines.size() == mTrackerInfos.size());
            assert(mTrackerInfos.size() == mBatches.size());
            for(size_t i = 0; i < mTrackerInfos.size(); ++i)
            {
                char gpsstr[50] = {0};
                const BLHCoordinate blh = mTrackerInfos[i].blh;
                if(BLHCoordinate::isValid(blh))
                {
                    sprintf(gpsstr,"%s(%.7f,%.7f)",SPLITSTR,blh.lat,blh.lon);
                }
                
                mTrkLines[i].append(string(gpsstr));
                mfile << mTrkLines[i].c_str() << endl;
            }
            mfile.close();
        }
    }
}
