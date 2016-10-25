#pragma once

#pragma once
#ifndef ROBOTOPTIMIZATION_H
#define ROBOTOPTIMIZATION_H
#include <thread>
#include <atomic>
#include <deque>
#include <vector>
#include <functional>
#include <condition_variable>



class RobotOptimization
{
public:
	struct OptimizationParameters {
		//optimization control values
		size_t currentStepCount;
		size_t maxStepCount;
		double systemFitness;
		double bestFitness;
		double startTime;
		double endTime;
		bool minimizes;

		std::vector<double> bestVariables;
		std::vector<double> subSampleFitness;
		std::vector<std::string> variableName;
		std::vector<double> defaultValues;
		std::vector<double> domainMin;
		std::vector<double> domainMax;
		std::vector<bool> OptimizingVariableMask;


		virtual ~OptimizationParameters() = default;
	};
	struct SampleParameters {
		size_t sampleID;
		double currentFitness;
		std::vector<double> variableValues;
		std::vector<double> subSampleFitness;
		//sample properties
		virtual ~SampleParameters() = default;
	};
	class SamplingFunction {
	public:
		virtual void Sample(std::shared_ptr<RobotOptimization::SampleParameters>) {}; // runs the sampling code and returns the overall fitness
		virtual std::unique_ptr<RobotOptimization::SamplingFunction> DuplicateSamplingFunction() { return std::make_unique<RobotOptimization::SamplingFunction>(); };// { return std::make_unique<SamplingFunction>(*this); };
		virtual ~SamplingFunction() = default;
	};

protected:
	std::condition_variable m_ThreadController;
	std::mutex m_IOMutex;
	bool m_isRunning;
	//std::vector<std::unique_ptr<WorkerThread>> m_workerThreads;
	size_t m_stepCount; //number of samples divided by the sample partition size
	std::atomic<size_t> m_outstandingJobs;
	size_t m_maxCoreCount;
	size_t m_runningThreads;
	std::condition_variable m_queueNotification;
	std::vector<std::thread> m_workerThreads;
	std::deque<std::shared_ptr<SampleParameters>> m_sampleParameters;
	std::shared_ptr<OptimizationParameters> m_optimizationData;
	std::shared_ptr<SamplingFunction> m_baseSamplingFunction;

	void startThreads();
	void stopThreads();
	void addSamplingJob(std::shared_ptr<SampleParameters> balanceJob);
	void waitForQueuedSamples();
	virtual void initialSetup() = 0;
	virtual void stepInternal() = 0;
	virtual void exportBest() = 0;
public:
	RobotOptimization();
	virtual ~RobotOptimization();

	void setResourceUtilization(int maxNumCores, int maxMemory, int priority, size_t coreAffinity = 4294967295);
	void setOptimizationParameters(std::shared_ptr<OptimizationParameters> newParameters);
	void runUntilThreshold(double threshold);
	void enableStandby();
	virtual std::shared_ptr<SampleParameters> getBestValues() = 0;

	virtual void setOutputFile() {};
	virtual void step() = 0;
	virtual double getBestFitness() = 0;
	void setSamplingFunction(std::shared_ptr<SamplingFunction> newFunction);

};
#endif