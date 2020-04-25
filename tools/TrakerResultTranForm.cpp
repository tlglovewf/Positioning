#include <rapidjson/document.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <iomanip>
#include <map>

#include <opencv2/opencv.hpp>//读取时用不到，测试时
using namespace std;
using namespace cv;

struct TRACKJSON//json中经纬度，和seq组成map
{
	double lat;
	double lon;
};

struct SIGNIMAGE//tracker_txt内容，包括有标牌的后两张图索引和trk_txt中矩形框索引
{
	int sum;
	int last1;
	int rec1;
	int last2;
	int rec2;
};

struct LOCATIONINFO//trk_txt矩形框信息
{
	string name;
	int seqnum;
	int xmin;
	int ymin; 
	int xmax;
	int ymax;
};

bool ReadTrackJson(const string &jsonfile, map<int, TRACKJSON> &trjson)
{
	ifstream json_file;
	json_file.open(jsonfile.c_str());
	string line, json;
	if (!json_file.is_open())
	{
		cout << "Error opening track json file" << endl;
		return 0;
	}
	while (getline(json_file, line))
	{
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
			cout << "Json File Error! Check if the file is available!" << endl;
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
							for (int j = 0; j < trmkArray.Size(); j++)
							{
								TRACKJSON mtj;
								int mseq;
								rapidjson::Value& trmkValue = trmkArray[j];
								if (trmkValue.HasMember("latitude"))
								{
									mtj.lat = trmkValue["latitude"].GetDouble();
								}
								if (trmkValue.HasMember("longitude"))
								{
									mtj.lon = trmkValue["longitude"].GetDouble();
								}
								if (trmkValue.HasMember("seqNum"))
								{
									mseq = atoi(trmkValue["seqNum"].GetString());
								}
								trjson.insert(make_pair(mseq, mtj));
							}
						}
					}
					else
					{
						cout << "Json File has no trackMark!" << endl;
						return 0;
					}	
				}
			}
			else
			{
				cout << "Json File has no data!" << endl;
				return 0;
			}
			return 1;
		}
	}
	return 0;
}

bool ReadTxT(const string &txtfile, vector<string> &strs)
{
	ifstream txt_file;
	txt_file.open(txtfile.c_str());
	assert(txt_file.is_open());
	string s;
	while (getline(txt_file, s))
	{
		strs.push_back(s);
	}
	txt_file.close();
	return 1;
}

vector<string> split(string res, const char reg)
{
	vector<string> v;
	signed int start;
	signed int end = res.find(reg, 0);
	v.push_back(res.substr(0, end));
	start = end + 1;
	while ((end = res.find(reg, start))>0)
	{
		v.push_back(res.substr(start, end - start));
		start = end + 1;
	}
	v.push_back(res.substr(start));//加上最后一个数
	return v;
}

bool ReadTrackerTxt(const string &trtxtfile, vector<string> &strs, vector<SIGNIMAGE> &trtxts)
{
	
	if (!ReadTxT(trtxtfile, strs))
	{
		cout << "Read Tracker txt fail!" << endl;
		return 0;
	}
	for (auto s : strs)
	{
		SIGNIMAGE mtrt;
		vector<string> ss = split(s, ';');

		auto pos1 = ss[14].find('(');
		auto pos2 = ss[14].find(',');
		auto pos3 = ss[14].find(')');

		auto pos4 = ss[15].find('(');
		auto pos5 = ss[15].find(',');
		auto pos6 = ss[15].find(')');

		mtrt.sum = atoi(ss[13].c_str());
		mtrt.last1 = atoi(ss[14].substr(pos1 + 1, pos2).c_str());//记录第14位及14位后两位数据，为了寻找对应图片和矩形框
		mtrt.rec1 = atoi(ss[14].substr(pos2 + 1, pos3).c_str());
		mtrt.last2 = atoi(ss[15].substr(pos4 + 1, pos5).c_str());
		mtrt.rec2 = atoi(ss[15].substr(pos5 + 1, pos6).c_str());
		trtxts.push_back(mtrt);
	}
	return 1;
}

bool ReadSeqTxt(const string &seqtxtfile, vector<string> &seqtxts)
{
	vector<string> strs;
	if (!ReadTxT(seqtxtfile, strs))
	{
		cout << "Read Seq txt fail!" << endl;
		return 0;
	}
	for (auto s : strs)
	{
		string imgname, name;
		auto pos1 = s.find_last_of('/');

		imgname = s.substr(pos1 + 1);
		auto pos2 = imgname.find_last_of('.');
		name = imgname.substr(0, pos2);

		seqtxts.push_back(name);
	}
	return 1;
}

bool ReadTracktrkTxt(const string &trkfile, const string &name, vector<LOCATIONINFO> &trs)
{
	vector<string> strs;
	if (!ReadTxT(trkfile, strs))
	{
		cout << "Read Trk txt fail!" << endl;
		return 0;
	}
	for (auto s : strs)
	{
		LOCATIONINFO mtr;
		vector<string> ss = split(s, ';');

		auto pos1 = name.find_first_of('_');
		auto pos2 = name.find_last_of('_');
		int seq = atoi(name.substr(pos1 + 1, pos2).c_str());
		mtr.name = name;
		mtr.seqnum = seq;
		mtr.xmin = atoi(ss[0].c_str());
		mtr.ymin = atoi(ss[1].c_str());
		mtr.xmax = atoi(ss[2].c_str());
		mtr.ymax = atoi(ss[3].c_str());
		trs.push_back(mtr);
	}
	return 1;
}

bool WriteTxT(const string &resultfile, vector<string> &result)
{
	ofstream outFile;
	outFile.open(resultfile);
	assert(outFile.is_open());
	for (auto s : result)
	{
		outFile << s << endl;
	}
	outFile.close();
	return 1;
}

int main()//测试代码
{
	string jsonPath = "./raw_data/track.json";//track.json
	map<int, TRACKJSON> mtrjson;
	ReadTrackJson(jsonPath, mtrjson);
	
	
	vector<SIGNIMAGE> trtxts;
	vector<string> trackerinfo;
	string trtxtPath = "./tracker/tracker.txt";//tracker.txt
	ReadTrackerTxt(trtxtPath, trackerinfo, trtxts);

	vector<string> seqtxts;
	string seqtxtPath = "./raw_data/seq.txt";//seq.txt
	ReadSeqTxt(seqtxtPath, seqtxts);


	string trkresultfolder = "./tracker/tracker_result";//tracker_result folder
	string imgPath = "./raw_data/image";//image floder

	//
	int tsize = 1;//trtxts.size()
	vector<string> result;
	cout.setf(ios::fixed);
	for (size_t i = 0; i < tsize; i++)//遍历tracker_txt
	{
		string trkfilePath1 = trkresultfolder + "/" + seqtxts[trtxts[i].last1] + ".trk";//倒数第一帧
		string trkfilePath2 = trkresultfolder + "/" + seqtxts[trtxts[i].last2] + ".trk";//倒数第二帧

		vector<LOCATIONINFO> vlast1, vlast2;
		ReadTracktrkTxt(trkfilePath1, seqtxts[trtxts[i].last1], vlast1);
		ReadTracktrkTxt(trkfilePath2, seqtxts[trtxts[i].last2], vlast2);

		if (vlast1.size() > 0 && vlast2.size() > 0)
		{
			Mat imgl1 = imread(imgPath + '/' + seqtxts[trtxts[i].last1] + ".jpg");
			Mat imgl2 = imread(imgPath + '/' + seqtxts[trtxts[i].last2] + ".jpg");

			Rect rect1(vlast1[trtxts[i].rec1].xmin, vlast1[trtxts[i].rec1].ymin, vlast1[trtxts[i].rec1].xmax - vlast1[trtxts[i].rec1].xmin, vlast1[trtxts[i].rec1].ymax - vlast1[trtxts[i].rec1].ymin);
			Rect rect2(vlast2[trtxts[i].rec2].xmin, vlast2[trtxts[i].rec2].ymin, vlast2[trtxts[i].rec2].xmax - vlast2[trtxts[i].rec2].xmin, vlast2[trtxts[i].rec2].ymax - vlast2[trtxts[i].rec2].ymin);
			rectangle(imgl1, rect1, Scalar(255, 0, 0), 1, LINE_8, 0);
			rectangle(imgl2, rect2, Scalar(255, 0, 0), 1, LINE_8, 0);
			cout << vlast1[trtxts[i].rec1].seqnum << endl;
			cout << setprecision(7) << mtrjson[vlast1[trtxts[i].rec1].seqnum].lat << endl;
			cout << setprecision(7) << mtrjson[vlast1[trtxts[i].rec1].seqnum].lon << endl;

			cout << setprecision(7) << mtrjson[vlast2[trtxts[i].rec2].seqnum].lat << endl;
			cout << setprecision(7) << mtrjson[vlast2[trtxts[i].rec2].seqnum].lon << endl;

			
			//此处调用定位模块
			double lon = 121.4223398;//量测结果假值
			double lat = 31.2321367;
			//此处调用定位模块

			//判断定位结果是否可信？
			std::stringstream sslon, sslat;
			sslon << setiosflags(ios::fixed) << std::setprecision(7) << lon;
			sslat << setiosflags(ios::fixed) << std::setprecision(7) << lat;

			string res = trackerinfo[i] + ";(" + sslat.str() + ", " + sslon.str() + ')';
			result.push_back(res);

			cv::waitKey(0);
		}
	}

	string resultfile = "./result.txt";
	WriteTxT(resultfile, result);
	return 0;
}
