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
    int taskId;
    int taskStatus;

public:
    BaseTask()
    {
        this->taskStatus = TASK_EMPTY;
    }
    virtual ~BaseTask()
    {

    }

    virtual void process() = 0;

    typedef enum
    {
        WORK_TO_DO = 0,
        TASK_DONE,
        TASK_EMPTY
    } TaskStatus_t;

    int getTaskId() const { return taskId;  }
    void setTaskId(int value) { this->taskId = value;  }
    int getTaskStatus() const { return taskStatus; }
    void setTaskStatus(int value) { this->taskStatus = value; }
};


class AdditionTask : public BaseTask
{
public:
    AdditionTask()
    {

    }
    std::deque<std::pair<int32_t,int32_t>> in;
    std::vector<int> out;

    void process() override
    {
        std::cout<<"Process Addition"<<std::endl;

        int i = 0;
        out.resize(in.size());
        while (in.size())
        {
            std::pair<int32_t,int32_t> tmp = in.front();
            in.pop_front();
            out[i++] = tmp.first + tmp.second;
        }
        this->setTaskStatus(TASK_DONE);
    }
};

class SubstractionTask : public BaseTask
{
public:
    SubstractionTask()
    {

    }

    std::deque<std::pair<int32_t,int32_t>> in;
    std::vector<int> out;

    void process() override
    {
        std::cout<<"Process Substract"<<std::endl;
        int i = 0;
        out.resize(in.size());
        while (in.size())
        {
            std::pair<int32_t,int32_t> tmp = in.front();
            in.pop_front();
            out[i++] =tmp.first - tmp.second;
        }
        this->setTaskStatus(TASK_DONE);
    }
};


class ThreadInterface
{
public:
    std::thread thr;
    std::mutex mtx;

    // pthread_t id;
    // pthread_mutex_t mutex;
    std::deque<BaseTask*> TaskQueue;
    //    you can pass into the thread interface a pointer to the treadcontroller,
    //    add a container with a new mutex (to the threadcontroller), and collect all
    //    the finished task pointers from all threads in that container... than you
    //    just need to pick out the finished task and pass it to the next processing step...
    std::deque<BaseTask*> OutputQueue;
    int status;
    typedef enum{
        PROCESSING = 0,
        IDLE
    } ThreadStatus_t;
};

#define THREAD_POOL_SIZE 7 // Using Max ( (2x Physical Core Count) - 1 ) looks like a good idea
#define ROUND_ROBIN 1
class ThreadController
{
private:
    std::vector<ThreadInterface*> thread_pool;
public:

    void init()
    {
        for (int i = 0; i<THREAD_POOL_SIZE; i++)
        {
            thread_pool.push_back(new ThreadInterface());
            // interfaces.back()->mutex = PTHREAD_MUTEX_INITIALIZER;
            // pthread_create( &interfaces.back()->id, NULL, workerThread, (void*)interfaces.back() );
            thread_pool.back()->thr = std::thread(workerThread, static_cast<void*>(thread_pool.back()));
            // nanosleep((const struct timespec[]){{0, 1000000L}}, NULL);
            std::this_thread::sleep_for(std::chrono::nanoseconds(1000000));

        }
        std::cout<<"Let the Lions in!"<<std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    void addTask( BaseTask* Task)
    {
#if ROUND_ROBIN == 0
        int tIdx = 0;
        int tTasks = 0x7FFFFFFF;
        for(int i=0; i< this->interfaces.size() ; i++)
        {
            if (interfaces.at(i)->TaskQueue.size() < tTasks)
            {
                tTasks = interfaces.at(i)->TaskQueue.size();
                tIdx = i;
            }
        }
#else
        static int32_t tIdx = 0;
#endif
        thread_pool.at(tIdx)->mtx.lock();
        // pthread_mutex_lock( &interfaces.at(tIdx)->mutex );
        thread_pool.at(tIdx)->TaskQueue.push_back(Task);
        std::cout << "Choosing Thread -> " << thread_pool.at(tIdx)->thr.get_id()
                << " for Task: " << Task->getTaskId()<< std::endl;
        thread_pool.at(tIdx)->mtx.unlock();
        // pthread_mutex_unlock( &interfaces.at(tIdx)->mutex );
        if (++tIdx == THREAD_POOL_SIZE )
            tIdx = 0;
    }

    void isTaskReady( BaseTask* Task)
    {
        // iterate through, chec Task->getTaskId() ==
        // or just save the pointer to the task jo pass in, and check the status through that pointer..
    }

};

#define MAX_TASKS_IN_OUTPUT 10
void *workerThread( void *ptr )
{
    ThreadInterface* threadInterface = static_cast<ThreadInterface*>(ptr);
    std::cout << "Thread " << threadInterface->thr.get_id() << " starts"<<std::endl;
    while(1)
    {
        // pthread_mutex_lock(&threadInterface->mutex);
        threadInterface->mtx.lock();

        if (threadInterface->TaskQueue.size())
        {
            BaseTask* task;
            threadInterface->status = ThreadInterface::PROCESSING;
            task = threadInterface->TaskQueue.front();
            threadInterface->TaskQueue.pop_front();
            task->process(); // here happens some magic
            std::cout<<"Thread Id: "<<threadInterface->thr.get_id()<<" finished with Task " <<task->getTaskId()<<std::endl;
            threadInterface->OutputQueue.push_back(task);
        }
        else
            threadInterface->status = ThreadInterface::IDLE;

        // SAFETY xD -> throw away some, to prevent the cpu from melting
        if (threadInterface->OutputQueue.size() > MAX_TASKS_IN_OUTPUT)
        {
            threadInterface->OutputQueue.pop_front();
        }

        //  pthread_mutex_unlock(&threadInterface->mutex);
        threadInterface->mtx.unlock();
        // nanosleep((const struct timespec[]){{0, 100L}}, NULL);
        std::this_thread::sleep_for(std::chrono::nanoseconds(100));

    }
}


#endif // THREADCONTROLLER_H
