#include "RobotOptimization.h"

void RobotOptimization::startThreads()
{
	//clean up old threads, necessary for changing sampling function
	if (!m_workerThreads.empty())
		stopThreads();
	m_isRunning = true;
	for (int i = 0; i < m_maxCoreCount; i++)
	{
		//make a new worker thread that uses contition variables to wait for new data
		m_workerThreads.emplace_back([this]()
		{
			auto samplingFunction = this->m_baseSamplingFunction->DuplicateSamplingFunction();
			std::unique_lock<std::mutex> threadLock(this->m_IOMutex);
			++m_runningThreads;
			while (this->m_isRunning)
			{
				if (this->m_sampleParameters.empty()) {
					this->m_ThreadController.wait(threadLock, [this]() {
						//check the condition of a stop command or queue being not empty
						return !(this->m_isRunning) || !(this->m_sampleParameters.empty());
					});

				}
				else {
					auto parameter = std::move(this->m_sampleParameters.front());
					this->m_sampleParameters.pop_front();
					threadLock.unlock();
					samplingFunction->Sample(parameter);
					threadLock.lock();
					--(this->m_outstandingJobs);
					if (this->m_outstandingJobs == 0)
					{
						//threadLock.unlock();
						m_queueNotification.notify_all();
					}

				}
			};
			--m_runningThreads;
			return m_runningThreads;
		});
	};
	//should not need to wait for threads
}

void RobotOptimization::stopThreads()
{
	std::unique_lock<std::mutex> waitLock(m_IOMutex);
	m_isRunning = false;
	m_outstandingJobs = 0;
	m_ThreadController.notify_all();
	waitLock.unlock();
	for (auto itr = m_workerThreads.begin(); itr != m_workerThreads.end(); ++itr)
	{
		itr->join();
	}
	waitLock.lock();
	m_sampleParameters.clear();
	m_workerThreads.clear();
}

void RobotOptimization::addSamplingJob(std::shared_ptr<SampleParameters> balanceJob)
{
	std::lock_guard<std::mutex> ioLock(m_IOMutex);
	m_sampleParameters.push_back(std::move(balanceJob));
	++m_outstandingJobs;
	m_ThreadController.notify_one();
}

void RobotOptimization::waitForQueuedSamples()
{
	std::unique_lock<std::mutex> waitLock(m_IOMutex);
	while (m_outstandingJobs > 0)
	{
		m_ThreadController.notify_one();
		m_queueNotification.wait(waitLock, [this]() {
			//check the condition of a stop command or queue being not empty
			return !this->m_isRunning || this->m_outstandingJobs == 0;
		});
	}

}

void RobotOptimization::initialSetup()
{
}

RobotOptimization::RobotOptimization() :
	m_maxCoreCount(std::thread::hardware_concurrency()),
	m_isRunning(false),
	m_outstandingJobs(0)
{
	m_workerThreads.reserve(m_maxCoreCount);
}

RobotOptimization::~RobotOptimization()
{
	stopThreads();
}

void RobotOptimization::setResourceUtilization(int maxNumCores, int maxMemory, int priority, size_t coreAffinity) {
	m_maxCoreCount = maxNumCores;
}

void RobotOptimization::setOptimizationParameters(std::shared_ptr<OptimizationParameters> newParameters)
{
	m_optimizationData = newParameters;
}

void RobotOptimization::runUntilThreshold(double threshold)
{
	//this->setupOutputFile();
	m_stepCount = 0;
	m_optimizationData->currentStepCount = 0;
	if (!m_isRunning)
		startThreads();
	initialSetup();
	while ((m_optimizationData->systemFitness > threshold) && (m_stepCount < m_optimizationData->maxStepCount))
	{
		this->step();
		m_optimizationData->currentStepCount += 1;
		m_stepCount++;
	}
	stopThreads();
}

void RobotOptimization::enableStandby()
{
	startThreads();
}

void RobotOptimization::setSamplingFunction(std::shared_ptr<SamplingFunction> newFunction)
{
	if (!m_isRunning) {
		m_baseSamplingFunction = std::move(newFunction);
	}
	else {
		stopThreads();
		m_baseSamplingFunction = std::move(newFunction);
		startThreads();
	}
}
