#include "RobotGoldenSectionSearch.h"
#include <limits>

RobotGoldenSectionSearch::RobotGoldenSectionSearch():RobotUnimodalSearch()
{
	boundaryMin = std::make_shared<RobotGoldenSectionSearchSampleParameters>();
	boundaryMax = std::make_shared<RobotGoldenSectionSearchSampleParameters>();
	x1 = std::make_shared<RobotGoldenSectionSearchSampleParameters>();
	x2 = std::make_shared<RobotGoldenSectionSearchSampleParameters>();
};

void RobotGoldenSectionSearch::initialSetup() {
	boundaryMin->subSampleFitness = m_optimizationData->subSampleFitness;
	boundaryMin->variableValues = m_optimizationData->domainMin;
	boundaryMin->currentFitness = std::numeric_limits<double>::max();
	boundaryMin->sampleID = 0;
	boundaryMax->subSampleFitness = m_optimizationData->subSampleFitness;
	boundaryMax->variableValues = m_optimizationData->domainMax;
	boundaryMax->currentFitness = std::numeric_limits<double>::max();
	boundaryMax->sampleID = 1;
	x1->subSampleFitness = m_optimizationData->subSampleFitness;
	x1->variableValues = m_optimizationData->domainMin;
	x1->currentFitness = std::numeric_limits<double>::max();
	x1->sampleID = 3;
	x2->subSampleFitness = m_optimizationData->subSampleFitness;
	x2->variableValues = m_optimizationData->domainMin;
	x2->currentFitness = std::numeric_limits<double>::max();
	x2->sampleID = 4;
	bestData->subSampleFitness = m_optimizationData->subSampleFitness;
	bestData->variableValues = m_optimizationData->domainMin;
	bestData->currentFitness = std::numeric_limits<double>::max();
	bestData->sampleID = 5;

	x1->variableValues[0] = GoldenRatioX1(boundaryMin->variableValues[0], boundaryMax->variableValues[0]);
	x2->variableValues[0] = GoldenRatioX2(boundaryMin->variableValues[0], boundaryMax->variableValues[0]);
	addSamplingJob(x1);
	addSamplingJob(x2);

	waitForQueuedSamples();
};
void RobotGoldenSectionSearch::stepInternal()
{
	//calculate the point at the golden ratio between searchMin and searchMax

		if (x1->currentFitness < x2->currentFitness)
		{
			boundaryMax->currentFitness = x2->currentFitness;
			boundaryMax->variableValues[0] = x2->variableValues[0];
			x2->currentFitness = x1->currentFitness;
			x2->variableValues[0] = x1->variableValues[0];
			x1->variableValues[0] = GoldenRatioX1(boundaryMin->variableValues[0], boundaryMax->variableValues[0]);
			addSamplingJob(x1);
			waitForQueuedSamples();
		}
		else {
			boundaryMin->currentFitness = x1->currentFitness;
			boundaryMin->variableValues[0] = x1->variableValues[0];
			x1->currentFitness = x2->currentFitness;
			x1->variableValues[0] = x2->variableValues[0];
			x2->variableValues[0] = GoldenRatioX2(boundaryMin->variableValues[0], boundaryMax->variableValues[0]);
			addSamplingJob(x2);
			waitForQueuedSamples();
		}
	
	
	//cleanup
	if (x1->currentFitness < x2->currentFitness)
	{
		bestData->currentFitness = x1->currentFitness;
		bestData->variableValues[0] = x1->variableValues[0];

	}
	else {
		bestData->currentFitness = x2->currentFitness;
		bestData->variableValues[0] = x2->variableValues[0];

	}

};