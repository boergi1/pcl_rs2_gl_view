#ifndef THREADCONTROLLER_H
#define THREADCONTROLLER_H

#include <iostream>
#include <queue>
#include <algorithm>
#include "unistd.h"
#include <thread>
#include <mutex>

#include "format.h"

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

#define MAX_TASKS_IN_OUTPUT 300
#define THREAD_POOL_SIZE 7 // Using Max ( (2x Physical Core Count) - 1 ) looks like a good idea
#define ROUND_ROBIN 1

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

#endif // THREADCONTROLLER_H


