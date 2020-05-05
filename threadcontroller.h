#ifndef THREADCONTROLLER_H
#define THREADCONTROLLER_H

#include <iostream>
//#include <pthread.h>
#include <queue>
#include <algorithm>
#include "unistd.h"
//#include <ctime>

#include <thread>
#include <mutex>


class BaseTask;
class AdditionTask;
class SubstractionTask;
class ThreadInterface;
class ThreadController;

void *workerThread( void *ptr );

class BaseTask
{
private:
    int m_taskType;
    int m_taskId;
    int m_taskStatus;

public:
    BaseTask();
    virtual ~BaseTask();

    virtual void process() = 0;

    typedef enum
    {
        WORK_TO_DO = 0,
        TASK_DONE,
        TASK_EMPTY
    } TaskStatus_t;

    int getTaskType() { return m_taskType; }
    void setTaskType(int taskType) { m_taskType = taskType; }
    int getTaskId() const;
    void setTaskId(int value);
    int getTaskStatus() const;
    void setTaskStatus(int value);
};

class ThreadInterface
{
private:
    ThreadController* m_ref_to_controller;
public:
    ThreadInterface(ThreadController* parent);

    void addOutput(BaseTask* task);
    std::thread thr;
    std::mutex mtx;

    // pthread_t id;
    // pthread_mutex_t mutex;
    std::deque<BaseTask*> TaskQueue;
    //    you can pass into the thread interface a pointer to the treadcontroller,
    //    add a container with a new mutex (to the threadcontroller), and collect all
    //    the finished task pointers from all threads in that container... than you
    //    just need to pick out the finished task and pass it to the next processing step...
    //    std::deque<BaseTask*> OutputQueue;
    int status;
    typedef enum{
        PROCESSING = 0,
        IDLE
    } ThreadStatus_t;




};

#define MAX_TASKS_IN_OUTPUT 20
#define THREAD_POOL_SIZE 7 // Using Max ( (2x Physical Core Count) - 1 ) looks like a good idea
#define ROUND_ROBIN 0

class ThreadController
{
private:
    //void *workerThread( void *ptr );
protected:
    std::vector<ThreadInterface*> m_thread_pool;
public:
    std::mutex output_mtx;
    std::deque<BaseTask*> OutputQueue;

    //    void addOutput(BaseTask* task)
    //    {
    //        OutputQueue.push_back(task);
    //    }

    virtual ~ThreadController();

    virtual void init();

    void addTask(BaseTask* task);
    BaseTask* getTask()
    {
        this->output_mtx.lock();
        auto tmp_task = OutputQueue.front();
        OutputQueue.pop_front();
        this->output_mtx.unlock();
        return tmp_task;
    }

    void isTaskReady( BaseTask* Task);

};



//#define RAND_MAX 64758 // exactly Trölf
//int main()
//{
//    std::srand(std::time(nullptr));
//    ThreadController tc;
//    tc.init();
//    while(1)
//    {
//        AdditionTask* at = new AdditionTask();
//        SubstractionTask* st = new SubstractionTask();
//        AdditionTask* at1 = new AdditionTask();
//        SubstractionTask* st1 = new SubstractionTask();
//        AdditionTask* at2 = new AdditionTask();
//        SubstractionTask* st2 = new SubstractionTask();
//        AdditionTask* at3 = new AdditionTask();
//        SubstractionTask* st3 = new SubstractionTask();
//        at->setTaskStatus(BaseTask::WORK_TO_DO); //...
//        // if you have large tasks which take long, just split them into two Tasks and pass them to the thread pool..

//        for (int i = 0; i < 100; i++)
//        {
//            std::pair<int32_t,int32_t> tp = { std::rand(), std::rand() };
//            at->in.push_back(tp);
//            at1->in.push_back(tp);
//            at2->in.push_back(tp);
//            at3->in.push_back(tp);
//            st->in.push_back(tp);
//            st1->in.push_back(tp);
//            st2->in.push_back(tp);
//            st3->in.push_back(tp);
//        }

//        std::cout<<"Adding 8 Jobs"<<std::endl;

//        at->setTaskId(std::rand());
//        st->setTaskId(std::rand());
//        at1->setTaskId(std::rand());
//        st1->setTaskId(std::rand());
//        at2->setTaskId(std::rand());
//        st2->setTaskId(std::rand());
//        at3->setTaskId(std::rand());
//        st3->setTaskId(std::rand());
//        tc.addTask(at);
//        tc.addTask(st);
//        tc.addTask(at1);
//        tc.addTask(st1);
//        tc.addTask(at2);
//        tc.addTask(st2);
//        tc.addTask(at3);
//        tc.addTask(st3);
//        nanosleep((const struct timespec[]){{0, 10L}}, NULL);

//    }
//    return 0;
//}


#endif // THREADCONTROLLER_H


