#include<iostream>
#include<thread>
#include<future>
#include<mutex>

using namespace std;










#pragma region     test future/promise

void thread_set_promise( std::promise<int> &item)
{
    std::cout << "generate data" << std::endl;
    std::this_thread::sleep_for(std::chrono::microseconds(1000));
    item.set_value(100);
    std::cout << "Finished . " << std::endl;
}

int test_future(void)
{
    std::promise<int> projitem;
    std::future<int>  ft = projitem.get_future();
    std::thread t(&thread_set_promise,std::ref(projitem));
    std::cout << ft.get() << std::endl;
    t.join();

    return 0;
}


bool is_prime(int x)
{
    for( int i = 1; i < x; ++i)
    {
        if( i % x == 0)
        {
            return false;
        }
    }
    return true;
} 


void test_async()
{
    std::future<bool> test = std::async(std::launch::async,is_prime,700020007);
    std::cout << "please wait." << std::endl;
    std::chrono::microseconds span(100);
    future_status st = test.wait_for(span);
    
    while( test.wait_for(span) == future_status::timeout )
    {
        cout << ".";
    }
    cout << endl;

    std::cout << "Result : " << test.get() << std::endl;
}

int countdown(int from, int to)
{
    for( int i = from; i != to; ++i)
    {
        std::cout << i << "\n";
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    std::cout << "life off" << std::endl;
    return from - to;
}

void  test_packagetask()
{
    //封装函数返回值
    std::packaged_task<int(int,int)> tsk(countdown);
    std::future<int> ret = tsk.get_future();

    std::thread th(std::move(tsk),10,30);

    //add more

    int value = ret.get();

    std::cout << "the last value " << value << std::endl;

    th.join();

}




#pragma endregion



#pragma region  condition_variable
//条件变量 简易生产 消费者
std::mutex mtx;
std::condition_variable cv;

int cargo = 0;
bool shipment_var()
{
    return cargo != 0;
}

void consume(int n)
{
    for(int i = 0; i < 10; ++i)
    {
        std::unique_lock <std::mutex> lck(mtx);
        cv.wait(lck,shipment_var);
        std::cout  << cargo << "\n";
        cargo = 0;
    }
}

void test_condition()
{
    std::thread consumer_thread(consume,10);

    for(int i = 0; i < 10; ++i)
    {
        while(shipment_var())
        //当前线程让出cpu时间片
            std::this_thread::yield();
        
        std::unique_lock<mutex> lck(mtx);
        cargo = i + 1;
        cv.notify_one();
    }

    consumer_thread.join();
}


#pragma endregion

int main()
{
    // //获取运行时 线程数量
    // int thread_num = thread::hardware_concurrency();
    // //将cpu轮片让出
    // this_thread::yield();
    
    // cout << thread_num << endl;
    // std::mutex mutex;

    // test_future();               //测试future变量
    // test_async();                //测试异步
    // test_packagetask();          //测试异步函数封装
    // test_condition();            //测试条件变量


    


    return 0;
}