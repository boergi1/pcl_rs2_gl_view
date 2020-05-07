#include "threadcontroller.h"

void *workerThread(void *ptr)
{
    ThreadInterface* threadInterface = static_cast<ThreadInterface*>(ptr);
    std::cout << "Thread " << threadInterface->thr.get_id() << " ready"<<std::endl;
    while(1)
    {
        // pthread_mutex_lock(&threadInterface->mutex);
        threadInterface->mtx.lock();

        if (threadInterface->TaskQueue.size())
        {
            auto start = std::chrono::high_resolution_clock::now();
            BaseTask* task;
            threadInterface->status = ThreadInterface::PROCESSING;
            task = threadInterface->TaskQueue.front();
            threadInterface->TaskQueue.pop_front();
            task->process();
#if (VERBOSE > 2)
            std::cout<<"(ThreadController) " << threadInterface->thr.get_id() << " finished with TASK (type:" << task->getTaskType()
                    << " id:" << task->getTaskId() << "), took " << std::chrono::duration_cast
                       <std::chrono::milliseconds>(std::chrono::high_resolution_clock::now()-start).count() << " ms" << std::endl;
#endif
            threadInterface->addOutput(task);
            // threadInterface->m_ref_to_controller->OutputQueue.push_back(task);
            // threadInterface->OutputQueue.push_back(task);
        }
        else
            threadInterface->status = ThreadInterface::IDLE;

        //  pthread_mutex_unlock(&threadInterface->mutex);
        threadInterface->mtx.unlock();
        // nanosleep((const struct timespec[]){{0, 100L}}, NULL);
        std::this_thread::sleep_for(std::chrono::nanoseconds(100));

    }
}

ThreadController::~ThreadController()
{

}

void ThreadController::init()
{
    std::cout << std::endl <<"(ThreadController) Initializing " << THREAD_POOL_SIZE << " threads:"<<std::endl;
    for (int i = 0; i<THREAD_POOL_SIZE; i++)
    {
        m_thread_pool.push_back(new ThreadInterface(this));
        // interfaces.back()->mutex = PTHREAD_MUTEX_INITIALIZER;
        // pthread_create( &interfaces.back()->id, NULL, workerThread, (void*)interfaces.back() );
        m_thread_pool.back()->thr = std::thread(workerThread, static_cast<void*>(m_thread_pool.back()));
        std::cout << "T #" << i << ": " << m_thread_pool.back()->thr.get_id() << std::endl;
        std::this_thread::sleep_for(std::chrono::nanoseconds(1000000));
    }
    std::cout << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));
}

void ThreadController::addTask(BaseTask *task)
{
#if ROUND_ROBIN == 0
    int tIdx = 0;
    int tTasks = 0x7FFFFFFF;
    for(int i=0; i< this->m_thread_pool.size() ; i++)
    {
        if (m_thread_pool.at(i)->TaskQueue.size() < tTasks)
        {
            tTasks = m_thread_pool.at(i)->TaskQueue.size();
            tIdx = i;
        }
    }
#else
    static int32_t tIdx = 0;
#endif
    m_thread_pool.at(tIdx)->mtx.lock();
    // pthread_mutex_lock( &interfaces.at(tIdx)->mutex );
    m_thread_pool.at(tIdx)->TaskQueue.push_back(task);
#if (VERBOSE > 2)
    std::cout << "(ThreadController) Choosing Thread -> " << m_thread_pool.at(tIdx)->thr.get_id()
              << " for TASK (type:" << task->getTaskType() << " id:" << task->getTaskId() << ")" << std::endl;
#endif
    m_thread_pool.at(tIdx)->mtx.unlock();
    // pthread_mutex_unlock( &interfaces.at(tIdx)->mutex );
    if (++tIdx == THREAD_POOL_SIZE )
        tIdx = 0;
}

void ThreadController::isTaskReady(BaseTask *Task)
{
    // iterate through, chec Task->getTaskId() ==
    // or just save the pointer to the task jo pass in, and check the status through that pointer..
}

ThreadInterface::ThreadInterface(ThreadController *parent)
{
    m_ref_to_controller = parent;

}

void ThreadInterface::addOutput(BaseTask *task)
{
    m_ref_to_controller->output_mtx.lock();
    m_ref_to_controller->OutputQueue.push_back(task);
    if (m_ref_to_controller->OutputQueue.size() > MAX_TASKS_IN_OUTPUT)
    {
        std::cerr << "(ThreadController) Too many tasks in output" << std::endl;
        auto tmpptr = m_ref_to_controller->OutputQueue.front();
        m_ref_to_controller->OutputQueue.pop_front();
      //  delete tmpptr;
    }
    m_ref_to_controller->output_mtx.unlock();
}

BaseTask::BaseTask()
{
    this->m_taskStatus = TASK_EMPTY;
}

BaseTask::~BaseTask()
{

}

int BaseTask::getTaskId() const { return m_taskId;  }

void BaseTask::setTaskId(int value) { this->m_taskId = value;  }

int BaseTask::getTaskStatus() const { return m_taskStatus; }

void BaseTask::setTaskStatus(int value) { this->m_taskStatus = value; }
