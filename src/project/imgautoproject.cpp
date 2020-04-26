
#include "project/imgautoproject.h"
#include "P_Writer.h"
#include "P_Utils.h"

#include  "Thirdparty/rapidjson/document.h"

namespace Position
{
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
            PROMT_S("Read camera config failed!!!")
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
        if(path.empty() || framedatas.empty())
            return false;
        ifstream json_file;
	    json_file.open(path.c_str());
	    string line, json;
	    if (!json_file.is_open())
	    {
            PROMT_S("Error opening track json file");
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
                PROMT_S("Json File Error! Check if the file is available!")
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
                                cout << "trmk : " << trmkArray.Size()  << endl;
                                cout << "fmdt : " << framedatas.size() << endl;
                                if(trmkArray.Size() != framedatas.size() )
                                {
                                    PROMT_S("image seq size and trkarray size are not same!");
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
                                             PROMT_V(framedatas[j]._name,"seq numb error. ")
                                         }
                                    }
                                }
                            }
                        }
                        else
                        {
                            PROMT_S("Json File has no trackMark!");
                            return false;
                        }	
                    }
                }
                else
                {
                    PROMT_S("Json File has no data!");
                    return false;
                }
                return true;
            }
        }
        return false;
    }

//     bool ReadTracktrkTxt(const string &trkfile, const string &name, vector<LOCATIONINFO> &trs)
// {
// 	vector<string> strs;
// 	if (!ReadTxT(yu, strs))
// 	{
// 		cout << "Read Trk txt fail!" << endl;
// 		return 0;
// 	}
// 	for (auto s : strs)
// 	{
// 		LOCATIONINFO mtr;
// 		vector<string> ss = split(s, ';');

// 		auto pos1 = name.find_first_of('_');
// 		auto pos2 = name.find_last_of('_');
// 		int seq = atoi(name.substr(pos1 + 1, pos2).c_str());
// 		mtr.name = name;
// 		mtr.seqnum = seq;
// 		mtr.xmin = atoi(ss[0].c_str());
// 		mtr.ymin = atoi(ss[1].c_str());
// 		mtr.xmax = atoi(ss[2].c_str());
// 		mtr.ymax = atoi(ss[3].c_str());
// 		trs.push_back(mtr);
// 	}
// 	return 1;
// }

    //读取trk文档
    static void ReadTrkTxt(const string &txtfile, StringVector &lines)
    {
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

        const std::string raw_data      = expath    + "/raw_data";
        const std::string seqpath       = raw_data  + "/seq.txt";
        const std::string trkjson       = raw_data  + "/track.json";
        const std::string imgpath       = raw_data  + "/image/";
        const std::string trackerpath   = expath    + "tracker";
        const std::string trackrestpath = trackerpath + "/tracker_result/";
        const std::string campath       = expath    + "config/extrinsics.xml";
        const std::string splitchar     = ";";      //分割字符

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
                       StringVector values = PUtils::SplitString(item,splitchar);
                       
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
                    PROMTD_V(mFrameDatas[i]._name.c_str(),"no target!!!");
                }
            }


        }catch(exception e)
        {
            PROMT_V("error",e.what());
        }
      
        //load frame datas



        return true;
    }
}
