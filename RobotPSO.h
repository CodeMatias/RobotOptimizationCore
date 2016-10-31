#pragma once
#pragma once
#ifndef ROBOTPSO_H
#define ROBOTPSO_H

#include "RobotControl\Optimization\RobotOptimization.h"

enum class PSOOptions {
		BoundaryFree,
		BoundaryAbsorb,
		BoundaryRandom,
		BoundaryBounce,
		InitialVelocityUniformRandom,
		InitialVelocityZero,
		InitialVelocityExploding,
		InitialPositionUniformRandom,
		InitialPositionDefaultCentered,
		InitialPositionPoint,
		GroupBoundaryOverlapping,
		GroupBoundaryExclusive,
		GroupBoundaryNone,
		GroupDistributionUniformRandom,
		GroupDistributionLine,
		GroupDistributionGrid,
		GroupDistributionVolumeGrid
	};
class RobotPSO : public RobotOptimization
{
public:
	struct PSOParameters : public OptimizationParameters {
		size_t maxParticles;
		size_t maxNumberofGroups;

		double groupWeight;
		double particleMass;
		double startParticleWeight;
		double endParticleWeight;
		double selfWeight;
		double socialWeight;

		PSOOptions boundaryOption;
		PSOOptions initialPositionOption;
		PSOOptions initialVelocityOption;
	};
	struct PSOSampleParameters :public SampleParameters {
		//optimization values
		double bestFitness;
		double particleMass;
		std::shared_ptr<PSOSampleParameters> groupBest;

		//data arrays
		std::vector<bool> OptimizingVariableMask;  //partial directional updates possible
		std::vector<double> velocity;
		std::vector<double> variableWeightSocial;
		std::vector<double> variableWeightSelf;
		std::vector<double> variableWeightGroup;
		std::vector<double> bestVariableValues;

	};

protected:
	std::vector<std::shared_ptr<PSOSampleParameters>> m_groupBestData;
	std::vector<std::shared_ptr<PSOSampleParameters>> m_particleData;
	std::shared_ptr<PSOParameters> m_psoData;

	virtual void updateParticles();
	virtual void updateParticleFitness();
	virtual void initialSetup();
	void stepInternal();
	void exportBest();

public:
	RobotPSO() = default;
	RobotPSO(std::shared_ptr<PSOParameters> setupParameters);
	RobotPSO(const RobotPSO&) = default;
	~RobotPSO() = default;
	std::shared_ptr<SampleParameters> getBestValues();
	void step();
	double getBestFitness();
};
#endif
