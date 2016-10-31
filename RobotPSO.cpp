
#include "RobotPSO.h"
#include <random>
#include <limits>
#include <algorithm>

void RobotPSO::updateParticles()
{
	std::default_random_engine generator;
	//std::normal_distribution<double> distribution(0.0, 1.0);  //requires care to avoid issues with the boundaries
	std::uniform_real_distribution<double> distribution(0.0, 1.0);
	float currentParticleMass = m_psoData->startParticleWeight + std::static_pointer_cast<PSOParameters>(m_psoData)->startParticleWeight / m_psoData->maxStepCount;

	for (auto itr = m_particleData.begin(); itr != m_particleData.end(); ++itr)//update new position for next round, according to PSO methods
	{
		(*itr)->particleMass = currentParticleMass; // linear decreasing weight for weighted PSO, one of the best, if a bit slower than log weights
															// pull all variables toward the best one, add some friction to make settling faster, try 90% "inertia" to start and lower it as a best solution gets closer?
		auto optimizationFlagitr = (*itr)->OptimizingVariableMask.begin();
		for (int k = 0; k != (*itr)->variableValues.size(); ++k)
		{
			if ((*itr)->OptimizingVariableMask[k]) {
				// using vid=w*vid+c1r1(pid-xid)+c2r2(pgd-xid)   from 978-1-4577-1123-7/11 p640 (bansal - inertia weight strategies in PSO)
				(*itr)->velocity[k] *= (*itr)->particleMass;
				(*itr)->variableWeightSelf[k] = m_psoData->selfWeight*distribution(generator);
				(*itr)->variableWeightSocial[k] = m_psoData->socialWeight*distribution(generator);

				(*itr)->velocity[k] += (*itr)->variableWeightSelf[k] * ((*itr)->bestVariableValues[k] - (*itr)->variableValues[k]);  //(1.0-weightMax + ((float)i)*((weightMax - weightMin) / maxRunLength))*c1=.5, add "cognative scaling" (tendancy to pick what it knows will work)
				(*itr)->velocity[k] += (*itr)->variableWeightSocial[k] * ((*itr)->groupBest->bestVariableValues[k] - (*itr)->variableValues[k]);  //(1.0 - weightMax + ((float)i)*((weightMax - weightMin) / maxRunLength))*c2=.5, add "social scaling" (tendancy to pick what group found works better)
				(*itr)->variableValues[k] += (*itr)->velocity[k];
				switch (m_psoData->boundaryOption)
				{
				case PSOOptions::BoundaryBounce:
					if ((*itr)->variableValues[k] < m_psoData->domainMin[k]) {
						(*itr)->variableValues[k] = m_psoData->domainMin[k];
						(*itr)->velocity[k] = -(*itr)->velocity[k];
					};
					if ((*itr)->variableValues[k] > m_psoData->domainMax[k])
					{
						(*itr)->variableValues[k] = m_psoData->domainMax[k];
						(*itr)->velocity[k] = -(*itr)->velocity[k];
					};
					break;
				case PSOOptions::BoundaryAbsorb:
					if ((*itr)->variableValues[k] < m_psoData->domainMin[k]) {
						(*itr)->variableValues[k] = m_psoData->domainMin[k];
						(*itr)->velocity[k] = 0.f;
					};
					if ((*itr)->variableValues[k] > m_psoData->domainMax[k])
					{
						(*itr)->variableValues[k] = m_psoData->domainMax[k];
						(*itr)->velocity[k] = 0.f;
					};
					break;
				case PSOOptions::BoundaryRandom:
					if ((*itr)->variableValues[k] < m_psoData->domainMin[k]) {
						(*itr)->variableValues[k] = m_psoData->domainMin[k];
						(*itr)->velocity[k] = (distribution(generator) - 0.5f)*(m_psoData->domainMax[k] - m_psoData->domainMin[k]);
					};
					if ((*itr)->variableValues[k] > m_psoData->domainMax[k])
					{
						(*itr)->variableValues[k] = m_psoData->domainMax[k];
						(*itr)->velocity[k] = (distribution(generator) - 0.5)*(m_psoData->domainMax[k] - m_psoData->domainMin[k]);
					};
					break;

				}

			}

		}//for (int k = 0; k < (*itr)->numVariables; k++)
		addSamplingJob(*itr);
	}//for (int i = 0; i < maxParticles; i++)
}

void RobotPSO::updateParticleFitness()
{

	std::for_each( m_particleData.begin(), m_particleData.end(), [](std::shared_ptr<RobotPSO::PSOSampleParameters>& sample)
	{
		if (sample->currentFitness < sample->bestFitness) {
			sample->bestFitness = sample->currentFitness;
			sample->bestVariableValues = sample->variableValues;
		};
		if (sample->currentFitness < sample->groupBest->currentFitness) {
			sample->groupBest->groupBest = sample;
			sample->groupBest->variableValues = sample->variableValues;
			sample->groupBest->subSampleFitness = sample->subSampleFitness;
			sample->groupBest->currentFitness = sample->currentFitness;
		};
	});
	std::for_each(m_groupBestData.begin(), m_groupBestData.end(), [](std::shared_ptr<RobotPSO::PSOSampleParameters>& sample)
	{
		if (sample->currentFitness < sample->bestFitness) {
			sample->bestFitness = sample->currentFitness;
			sample->bestVariableValues = sample->variableValues;
		};
		if (sample->currentFitness < sample->groupBest->currentFitness) {
			sample->groupBest->groupBest = sample;
			sample->groupBest->variableValues = sample->variableValues;
			sample->groupBest->subSampleFitness = sample->subSampleFitness;
		};
	});


}

void RobotPSO::exportBest()
{
}

void RobotPSO::initialSetup()
{
	if (m_optimizationData)
		m_psoData = std::static_pointer_cast<PSOParameters>(m_optimizationData);
	m_groupBestData.reserve(m_psoData->maxNumberofGroups);
	for (int j = 0; j < m_psoData->maxNumberofGroups; j++)
	{
		m_groupBestData.push_back(std::make_unique<PSOSampleParameters>());
		m_groupBestData.back()->bestFitness = std::numeric_limits<double>::max();
		m_groupBestData.back()->particleMass = 1.0;
		m_groupBestData.back()->sampleID = j;
		m_groupBestData.back()->currentFitness = std::numeric_limits<double>::max();
		m_groupBestData.back()->OptimizingVariableMask = m_psoData->OptimizingVariableMask;
		m_groupBestData.back()->variableValues = m_psoData->defaultValues;
		m_groupBestData.back()->bestVariableValues = m_psoData->defaultValues;
		m_groupBestData.back()->velocity.resize(m_groupBestData.back()->variableValues.size());
		m_groupBestData.back()->variableWeightSocial.resize(m_groupBestData.back()->variableValues.size(), 0.0);
		m_groupBestData.back()->variableWeightSelf.resize(m_groupBestData.back()->variableValues.size(), 0.0);
		m_groupBestData.back()->variableWeightGroup.resize(m_groupBestData.back()->variableValues.size(), 0.0);
		m_groupBestData.back()->subSampleFitness.resize(m_groupBestData.back()->variableValues.size(), 0.0);
	}

	std::default_random_engine generator;
	std::uniform_real_distribution<double> unityDistribution(0.0, 1.0);
	std::uniform_real_distribution<double> biDistribution(-1.0, 1.0);

	m_particleData.reserve(m_psoData->maxParticles);

	for (int i = 0; i < m_psoData->maxParticles; ++i)
	{
		m_particleData.push_back(std::make_unique<PSOSampleParameters>());
		m_particleData.back()->subSampleFitness.resize(m_psoData->subSampleFitness.size(), 0.0);
		m_particleData.back()->bestFitness = std::numeric_limits<double>::max();
		m_particleData.back()->currentFitness = std::numeric_limits<double>::max();
		m_particleData.back()->OptimizingVariableMask = m_psoData->OptimizingVariableMask;
		m_particleData.back()->particleMass = m_psoData->startParticleWeight;
		m_particleData.back()->sampleID = i;
		m_particleData.back()->groupBest = m_groupBestData[i % (m_psoData->maxNumberofGroups)];
		m_particleData.back()->bestVariableValues = m_psoData->defaultValues;
		m_particleData.back()->variableValues.reserve(m_psoData->defaultValues.size());
		m_particleData.back()->velocity.reserve(m_psoData->defaultValues.size());
		for (int k = 0; k < m_psoData->defaultValues.size(); k++)
		{
			if (m_psoData->OptimizingVariableMask[k])
			{
				switch (m_psoData->initialPositionOption)
				{
				case PSOOptions::InitialPositionUniformRandom:
					m_particleData.back()->variableValues.push_back(m_psoData->domainMin[k] + (m_psoData->domainMax[k] - m_psoData->domainMin[k])* unityDistribution(generator));
					break;
				case PSOOptions::InitialPositionPoint:
					m_particleData.back()->variableValues.push_back(m_psoData->defaultValues[k]);
					break;
				case PSOOptions::InitialPositionDefaultCentered:
					if (biDistribution(generator) < 0.0f)
					{
						m_particleData.back()->variableValues.push_back(m_psoData->domainMin[k] + (m_psoData->defaultValues[k] - m_psoData->domainMin[k])*(unityDistribution(generator)));
					}
					else {
						m_particleData.back()->variableValues.push_back(m_psoData->defaultValues[k] + (m_psoData->domainMax[k] - m_psoData->defaultValues[k])*unityDistribution(generator));
					}
					break;
				default:
					m_particleData.back()->variableValues.push_back(m_psoData->domainMin[k] + (m_psoData->domainMax[k] - m_psoData->domainMin[k])* unityDistribution(generator));
					break;
				}
				m_particleData.back()->variableWeightSelf.push_back(m_psoData->selfWeight*unityDistribution(generator));
				m_particleData.back()->variableWeightSocial.push_back(m_psoData->socialWeight*unityDistribution(generator));
				m_particleData.back()->variableWeightGroup.push_back(m_psoData->groupWeight*unityDistribution(generator));

				switch (m_psoData->initialVelocityOption)
				{
				case PSOOptions::InitialVelocityUniformRandom:
					m_particleData.back()->velocity.push_back((m_psoData->domainMax[k] - m_psoData->domainMin[k])*(biDistribution(generator)));
					break;
				case PSOOptions::InitialVelocityZero:
					m_particleData.back()->velocity.push_back(0.0);
					break;
				case PSOOptions::InitialVelocityExploding:
					m_particleData.back()->velocity.push_back(m_particleData.back()->variableValues[k] - 0.5f*(m_psoData->domainMax[k] + m_psoData->domainMin[k]));
					break;
				default:
					m_particleData.back()->velocity.push_back(m_particleData.back()->variableValues[k] - 0.5f*(m_psoData->domainMax[k] + m_psoData->domainMin[k]));
					break;
				}
			}
			else {
				m_particleData.back()->variableValues.push_back(m_psoData->defaultValues[k]);
				m_particleData.back()->velocity.push_back(0.0f);
				m_particleData.back()->variableWeightSelf.push_back(0.0f);
				m_particleData.back()->variableWeightSocial.push_back(0.0f);
				m_particleData.back()->variableWeightGroup.push_back(0.0f);
				m_particleData.back()->bestVariableValues.push_back(m_psoData->defaultValues[k]);
			};
		};
	};
}
void RobotPSO::stepInternal()
{
	updateParticles();
	//check if sample is best for particle
	waitForQueuedSamples();
	updateParticleFitness();
};

RobotPSO::RobotPSO(std::shared_ptr<PSOParameters> setupParameters)
{
	m_psoData = std::move(setupParameters);
	m_optimizationData = m_psoData;
}

std::shared_ptr<RobotOptimization::SampleParameters> RobotPSO::getBestValues()
{
	return *std::max_element(m_groupBestData.begin(), m_groupBestData.end(), [](std::shared_ptr<PSOSampleParameters>& l, std::shared_ptr<PSOSampleParameters>& r)
	{
		return (l->bestFitness < r->bestFitness);
	});
}

void RobotPSO::step()
{
	stepInternal();
	exportBest();
	m_psoData->systemFitness = getBestFitness();
}

double RobotPSO::getBestFitness()
{
	return std::max_element(m_groupBestData.begin(), m_groupBestData.end(), [](std::shared_ptr<PSOSampleParameters>& l, std::shared_ptr<PSOSampleParameters>& r)
	{
		return (l->bestFitness < r->bestFitness);
	})->get()->bestFitness;
}
