#ifndef THREADPOOL_H
#define THREADPOOL_H

#include <vector>
#include <deque>

#include <thread>
#include <mutex>
#include <condition_variable>

class ThreadPool
{
public:
	ThreadPool();
	ThreadPool(size_t threadCount);
	~ThreadPool();

	template<class T>
	void enqueue(T task);

private:
	friend class Worker;

	// List of workers
	std::vector<std::thread> m_Workers;
	// List of tasks
	std::deque<std::function<void(int, int)>> m_Tasks;
	// Synchronization
	std::mutex m_QueueMutex;
	std::condition_variable m_TaskAvailableCondition;
	bool m_bStop;
};

class Worker
{
public:
	Worker(ThreadPool& threadPoolInstance)
		: m_ThreadPool(threadPoolInstance) {}
	void operator()();

private:
	ThreadPool& m_ThreadPool;
};

// ----------------------------------------------------------------------------

template<class T>
void ThreadPool::enqueue(T task)
{
	// Acquire lock
	std::unique_lock<std::mutex> taskQueueLock(m_QueueMutex, std::defer_lock);

	// Lock the mutex
	taskQueueLock.lock();

	// Add the task
	m_Tasks.push_back(std::function<void(int, int)>(task));

	// Unlock mutex
	taskQueueLock.unlock();

	// Wake up one thread
	m_TaskAvailableCondition.notify_one();
}

#endif // THREADPOOL_H