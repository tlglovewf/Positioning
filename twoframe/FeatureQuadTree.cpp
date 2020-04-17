#include "FeatureQuadTree.h"

#define EDGE_THRESHOLD 16

const int nfeatures = 200;

const int PATCH_SIZE = 31;


FeatureQuadTree::FeatureQuadTree():mFeature(cv::xfeatures2d::SIFT::create(nfeatures,4,0.05,15,1.4))
{
    Position::FloatVector  mvScaleFactor(4,0);
    Position::FloatVector  mvLevelSigma2(4,0);
    mSigmaVector.resize(4);
    mvLevelSigma2[0]= 1.0f;
    mSigmaVector[0] = 1.0f;

    for(int i=1; i < 4; i++)
    {
        mvScaleFactor[i]=mvScaleFactor[i-1]*1.4;
        mvLevelSigma2[i]= 1.0 /(mvScaleFactor[i]*mvScaleFactor[i]);
    }
}

void drawKeyPts(const Mat &img,const KeyPtVector &keypts,Mat &mat)
{
    drawKeypoints(img,keypts,mat);
    const string text = "key size :" + std::to_string(keypts.size());
    putText(mat, text , Point(50, 50), CV_FONT_HERSHEY_COMPLEX, 2, Scalar(0, 0, 255), 3, CV_AA);
}

void FeatureQuadTree::detect(const Mat &img, KeyPtVector &keypts)
{
#if 0
    mFeature->detect(img,keypts);
    static int index = 0;
    Mat outimg;
    drawKeyPts(img,keypts,outimg);
    imwrite("/media/tlg/work/tlgfiles/HDData/result/normal_" + std::to_string(index++) + ".jpg",outimg);
#else
    //     allKeypoints.resize(nlevels);

    //grid 大小
    const float W = 300;
    //得到每一层图像进行特征检测区域上下两个坐标
    const int minBorderX = EDGE_THRESHOLD-3;
    const int minBorderY = minBorderX;
    const int maxBorderX = img.cols-EDGE_THRESHOLD+3;
    const int maxBorderY = img.rows-EDGE_THRESHOLD+3;
    //用于分配关键点
    KeyPtVector vToDistributeKeys;
    vToDistributeKeys.reserve(nfeatures * 1.5);

    const float width = (maxBorderX-minBorderX);
    const float height = (maxBorderY-minBorderY);

    const int nCols = width/W;
    const int nRows = height/W;
    const int wCell = ceil(width/nCols);
    const int hCell = ceil(height/nRows);
    Mat outimg = img.clone();


    mFeature = cv::xfeatures2d::SIFT::create(nfeatures / ((nRows-1) * (nCols-1)),4,0.05,15,1.4);

    //在每个格子内进行fast特征检测
    for(int i=0; i<nRows; i++)
    {
        const float iniY =minBorderY+i*hCell;
        float maxY = iniY+hCell+6;

        if(iniY>=maxBorderY-3)
            continue;
        if(maxY>maxBorderY)
            maxY = maxBorderY;

        for(int j=0; j<nCols; j++)
        {
            const float iniX =minBorderX+j*wCell;
            float maxX = iniX+wCell+6;
            if(iniX>=maxBorderX-6)
                continue;
            if(maxX>maxBorderX)
                maxX = maxBorderX;

            KeyPtVector vKeysCell;

            mFeature->detect(img.rowRange(iniY,maxY).colRange(iniX,maxX),vKeysCell);

            //如果检测到的fast特征为空,则降低阈值再进行检测
            if(vKeysCell.empty())
            {
                // cout << "缩小特征提取条件 " << iniX << " " << maxX << " " << iniY << " " << maxY << endl;
            }
            
            //计算特征点实际位置
            if(!vKeysCell.empty())
            {
                for(KeyPtVector::iterator vit=vKeysCell.begin(); vit!=vKeysCell.end();vit++)
                {
                    (*vit).pt.x+=j*wCell;
                    (*vit).pt.y+=i*hCell;
                    vToDistributeKeys.emplace_back(*vit);
                }
            }

        }
    }

    // KeyPtVector & keypoints = keypts;
    // keypoints.reserve(nfeatures);

    // float factor = 1.0 / 1.4;
    int n =  10;// nfeatures*(1 - factor)/(1 - (float)pow((double)factor, (double)0));

    // //将特征点进行八叉树划分
    // keypoints = distributeQuadTree(vToDistributeKeys, minBorderX, maxBorderX,
    //                               minBorderY, maxBorderY,n);

    KeyPtVector kkppts = distributeQuadTree(vToDistributeKeys, minBorderX, maxBorderX,
                                  minBorderY, maxBorderY,n);

    cout << "kkppts: " << kkppts.size() << endl;

    KeyPtVector &keypoints = vToDistributeKeys;

    // const int scaledPatchSize = PATCH_SIZE ;

    // Add border to coordinates and scale information
    const int nkps = keypoints.size();
    for(int i=0; i<nkps ; i++)
    {
        keypoints[i].pt.x+=minBorderX;
        keypoints[i].pt.y+=minBorderY;
    }
    vToDistributeKeys.swap(keypts);
    static int index = 0;
    drawKeyPts(img,keypts,outimg);
    imwrite("/media/tlg/work/tlgfiles/HDData/result/quad_" + std::to_string(index++) + ".jpg",outimg);
#endif
   
}


class ENode
{
public:
    ENode():bNoMore(false){}
    void DivideNode(ENode &n1, ENode &n2, ENode &n3, ENode &n4);
    KeyPtVector vKeys;
    cv::Point2i UL, UR, BL, BR;
    std::list<ENode>::iterator lit;
    bool bNoMore;
};


void ENode::DivideNode(ENode &n1, ENode &n2, ENode &n3, ENode &n4)
{
    const int halfX = ceil(static_cast<float>(UR.x-UL.x)/2);
    const int halfY = ceil(static_cast<float>(BR.y-UL.y)/2);

    //Define boundaries of childs
    n1.UL = UL;
    n1.UR = cv::Point2i(UL.x+halfX,UL.y);
    n1.BL = cv::Point2i(UL.x,UL.y+halfY);
    n1.BR = cv::Point2i(UL.x+halfX,UL.y+halfY);
    n1.vKeys.reserve(vKeys.size());

    n2.UL = n1.UR;
    n2.UR = UR;
    n2.BL = n1.BR;
    n2.BR = cv::Point2i(UR.x,UL.y+halfY);
    n2.vKeys.reserve(vKeys.size());

    n3.UL = n1.BL;
    n3.UR = n1.BR;
    n3.BL = BL;
    n3.BR = cv::Point2i(n1.BR.x,BL.y);
    n3.vKeys.reserve(vKeys.size());

    n4.UL = n3.UR;
    n4.UR = n2.BR;
    n4.BL = n3.BR;
    n4.BR = BR;
    n4.vKeys.reserve(vKeys.size());

    //Associate points to childs
    for(size_t i=0;i<vKeys.size();i++)
    {
        const cv::KeyPoint &kp = vKeys[i];
        if(kp.pt.x<n1.UR.x)
        {
            if(kp.pt.y<n1.BR.y)
                n1.vKeys.push_back(kp);
            else
                n3.vKeys.push_back(kp);
        }
        else if(kp.pt.y<n1.BR.y)
            n2.vKeys.push_back(kp);
        else
            n4.vKeys.push_back(kp);
    }

    if(n1.vKeys.size()==1)
        n1.bNoMore = true;
    if(n2.vKeys.size()==1)
        n2.bNoMore = true;
    if(n3.vKeys.size()==1)
        n3.bNoMore = true;
    if(n4.vKeys.size()==1)
        n4.bNoMore = true;

}


KeyPtVector FeatureQuadTree::distributeQuadTree(const KeyPtVector& vToDistributeKeys, const int &minX,
                                       const int &maxX, const int &minY, const int &maxY, const int &N)
{
    // Compute how many initial nodes   
    const int nIni = round(static_cast<float>(maxX-minX)/(maxY-minY));
    
    //获取节点间的间隔
    const float hX = static_cast<float>(maxX-minX)/nIni;
    cout << "inivalue " << nIni << " " << hX << endl;
    list<ENode> lNodes;

    vector<ENode*> vpIniNodes;
    vpIniNodes.resize(nIni);
    //创建跟节点
    for(int i=0; i<nIni; i++)
    {
        ENode ni;
        ni.UL = cv::Point2i(hX*static_cast<float>(i),0);
        ni.UR = cv::Point2i(hX*static_cast<float>(i+1),0);
        ni.BL = cv::Point2i(ni.UL.x,maxY-minY);
        ni.BR = cv::Point2i(ni.UR.x,maxY-minY);
        ni.vKeys.reserve(vToDistributeKeys.size());

        lNodes.push_back(ni);
        vpIniNodes[i] = &lNodes.back();
    }

    //Associate points to childs
    //根据特征点的坐标 分配子节点
    for(size_t i=0;i<vToDistributeKeys.size();i++)
    {
        const cv::KeyPoint &kp = vToDistributeKeys[i];
        vpIniNodes[kp.pt.x/hX]->vKeys.push_back(kp);
    }

    list<ENode>::iterator lit = lNodes.begin();

    while(lit!=lNodes.end())
    {   //如果只含一个特征点的时候 不再划分
        if(lit->vKeys.size()==1)
        {
            lit->bNoMore=true;
            lit++;
        }
        else if(lit->vKeys.empty())//没有特征点则删除
            lit = lNodes.erase(lit);
        else
            lit++;
    }

    bool bFinish = false;

    int iteration = 0;

    //节点 及其包含的特征点数
    vector<pair<int,ENode*> > vSizeAndPointerToNode;
    vSizeAndPointerToNode.reserve(lNodes.size()*4);

    while(!bFinish)
    {
        iteration++;

        int prevSize = lNodes.size();

        lit = lNodes.begin();
        //节点分解次数
        int nToExpand = 0;

        vSizeAndPointerToNode.clear();

        while(lit!=lNodes.end())
        {
            if(lit->bNoMore)
            {
                // If node only contains one point do not subdivide and continue
                lit++;
                continue;
            }
            else
            {
                // If more than one point, subdivide
                ENode n1,n2,n3,n4;
                cout << "divide four nodes"  << endl;
                //四叉树分裂
                lit->DivideNode(n1,n2,n3,n4);

                // Add childs if they contain points
                if(n1.vKeys.size()>0)
                {
                    lNodes.push_front(n1);                    
                    if(n1.vKeys.size()>1)
                    {
                        nToExpand++;
                        vSizeAndPointerToNode.push_back(make_pair(n1.vKeys.size(),&lNodes.front()));
                        lNodes.front().lit = lNodes.begin();
                    }
                }
                if(n2.vKeys.size()>0)
                {
                    lNodes.push_front(n2);
                    if(n2.vKeys.size()>1)
                    {
                        nToExpand++;
                        vSizeAndPointerToNode.push_back(make_pair(n2.vKeys.size(),&lNodes.front()));
                        lNodes.front().lit = lNodes.begin();
                    }
                }
                if(n3.vKeys.size()>0)
                {
                    lNodes.push_front(n3);
                    if(n3.vKeys.size()>1)
                    {
                        nToExpand++;
                        vSizeAndPointerToNode.push_back(make_pair(n3.vKeys.size(),&lNodes.front()));
                        lNodes.front().lit = lNodes.begin();
                    }
                }
                if(n4.vKeys.size()>0)
                {
                    lNodes.push_front(n4);
                    if(n4.vKeys.size()>1)
                    {
                        nToExpand++;
                        vSizeAndPointerToNode.push_back(make_pair(n4.vKeys.size(),&lNodes.front()));
                        lNodes.front().lit = lNodes.begin();
                    }
                }

                lit=lNodes.erase(lit);
                continue;
            }
        }       

        // Finish if there are more nodes than required features
        // or all nodes contain just one point
        if((int)lNodes.size()>=N || (int)lNodes.size()==prevSize)
        {
            cout << "finish ." << endl;
            bFinish = true;
        }
        else if(((int)lNodes.size()+nToExpand*3)>N)
        {//节点展开次数x3 大于需要包含的节点数量

            while(!bFinish)
            {

                prevSize = lNodes.size();

                vector<pair<int,ENode*> > vPrevSizeAndPointerToNode = vSizeAndPointerToNode;
                vSizeAndPointerToNode.clear();

                sort(vPrevSizeAndPointerToNode.begin(),vPrevSizeAndPointerToNode.end());
                for(int j=vPrevSizeAndPointerToNode.size()-1;j>=0;j--)
                {
                    ENode n1,n2,n3,n4;
                    vPrevSizeAndPointerToNode[j].second->DivideNode(n1,n2,n3,n4);

                    // Add childs if they contain points
                    if(n1.vKeys.size()>0)
                    {
                        lNodes.push_front(n1);
                        if(n1.vKeys.size()>1)
                        {
                            vSizeAndPointerToNode.push_back(make_pair(n1.vKeys.size(),&lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }
                    if(n2.vKeys.size()>0)
                    {
                        lNodes.push_front(n2);
                        if(n2.vKeys.size()>1)
                        {
                            vSizeAndPointerToNode.push_back(make_pair(n2.vKeys.size(),&lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }
                    if(n3.vKeys.size()>0)
                    {
                        lNodes.push_front(n3);
                        if(n3.vKeys.size()>1)
                        {
                            vSizeAndPointerToNode.push_back(make_pair(n3.vKeys.size(),&lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }
                    if(n4.vKeys.size()>0)
                    {
                        lNodes.push_front(n4);
                        if(n4.vKeys.size()>1)
                        {
                            vSizeAndPointerToNode.push_back(make_pair(n4.vKeys.size(),&lNodes.front()));
                            lNodes.front().lit = lNodes.begin();
                        }
                    }

                    lNodes.erase(vPrevSizeAndPointerToNode[j].second->lit);

                    if((int)lNodes.size()>=N)
                        break;
                }

                if((int)lNodes.size()>=N || (int)lNodes.size()==prevSize)
                    bFinish = true;

            }
        }
    }

    // Retain the best point in each node
    // 保留每个节点最好的特征点
    KeyPtVector vResultKeys;
    vResultKeys.reserve(nfeatures);
    cout << "lnodes : " << lNodes.size() << endl;
    for(list<ENode>::iterator lit=lNodes.begin(); lit!=lNodes.end(); lit++)
    {
        KeyPtVector &vNodeKeys = lit->vKeys;
        cv::KeyPoint* pKP = &vNodeKeys[0];
        float maxResponse = pKP->response;

        for(size_t k=1;k<vNodeKeys.size();k++)
        {
            if(vNodeKeys[k].response>maxResponse)
            {
                pKP = &vNodeKeys[k];
                maxResponse = vNodeKeys[k].response;
            }
        }

        vResultKeys.push_back(*pKP);
    }

    return vResultKeys;
}

bool FeatureQuadTree::detect(const FrameData &frame,KeyPtVector &keys, Mat &descript)
{
    // mFeature->detectAndCompute(frame._img,Mat(),keys,descript);
    detect(frame._img,keys);
    compute(frame._img,keys,descript);
    return true;
}

void FeatureQuadTree::compute(const Mat &img,KeyPtVector &keypts, Mat &des)
{
    mFeature->compute(img,keypts,des);
}


void FeatureQuadTree::createQuadTree(KeyPtVector &keypts)
{

}


