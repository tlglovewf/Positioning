#include "P_Interface.h"

template<typename T>
inline void display(T items)
{
    for(auto i : items)
    {
        cout << i << " ";
    }
    cout << endl;
}

int main()
{
    // vector<int> vts;
    // vts.emplace_back(1);
    // vts.emplace_back(2);
    // vts.emplace_back(3);

    // vector<int*> vt;
    // std::transform(vts.begin(),vts.end(),back_inserter(vt),[](int &a)->int*
    // {
    //     return &a;
    // });

    // display(vt);
    // *vt[0] = 10;
    // display(vts);

    Mat t = Mat::eye(3,3,CV_64F);
    cout << t << endl;
    cout << "---" << endl;
    Mat d = t;
    t.at<double>(1,0) = 10;
    cout << d << endl;
    cout << t << endl;
    // cout << distance(vts.begin(),vts.end()) << endl;

    // int x[] = {3,4,5,1,2};

    // std::copy(x,x+5,back_inserter(vts));
    
    // display(vts);
    // std::sort(vts.begin(),vts.end());
    // vts.erase(std::unique(vts.begin(),vts.end()),vts.end());
    // display(vts);

    return -1;
}