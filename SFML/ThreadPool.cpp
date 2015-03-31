#include "ThreadPool.h"


// ----------------------------------------------------------------------------
// Thread pool ----------------------------------------------------------------
// ----------------------------------------------------------------------------

ThreadPool::ThreadPool()
{
	const unsigned int threadCount = std::thread::hardware_concurrency();

	for (size_t i = 0; i < threadCount; i++)
	{
		m_Workers.push_back(std::thread(Worker(*this)));
	}
}

// ----------------------------------------------------------------------------

ThreadPool::ThreadPool(size_t threadCount)
{
	for (size_t i = 0; i < threadCount; i++)
	{
		m_Workers.push_back(std::thread(Worker(*this)));
	}
}

// ----------------------------------------------------------------------------


ThreadPool::~ThreadPool()
{
	// Stop all threads
	std::unique_lock<std::mutex> lock(m_QueueMutex, std::defer_lock);
	lock.lock();
	m_bStop = true;
	lock.unlock();
	m_TaskAvailableCondition.notify_all();

	// Join the threads
	for (size_t i = 0; i < m_Workers.size(); i++)
	{
		m_Workers[i].join();
	}
}

// ----------------------------------------------------------------------------
// Worker ---------------------------------------------------------------------
// ----------------------------------------------------------------------------

void Worker::operator()()
{
	std::function<void(int, int)> task;

	while (true)
	{
		// Acquire lock
		std::unique_lock<std::mutex> stopConditionLock(m_ThreadPool.m_QueueMutex, std::defer_lock);

		stopConditionLock.lock();

		// Look for a work item
		m_ThreadPool.m_TaskAvailableCondition.wait(stopConditionLock,
			[this]{ return m_ThreadPool.m_bStop || !m_ThreadPool.m_Tasks.empty(); });

		// Exit if the pool has been set to stop
		if (m_ThreadPool.m_bStop)
		{
			return;
		}

		// Get a task from the queue
		task = std::move(m_ThreadPool.m_Tasks.front());
		m_ThreadPool.m_Tasks.pop_front();

		stopConditionLock.unlock();
		
		// Execute the task
		task(3, 4);
	}
}

// ----------------------------------------------------------------------------