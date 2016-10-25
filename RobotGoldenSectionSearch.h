#pragma once
#ifndef RobotRobotGoldenSectionSearch_H
#define RobotRobotGoldenSectionSearch_H
#include "RobotUnimodalSearch.h"



class RobotGoldenSectionSearch : public RobotUnimodalSearch
{
public:
	enum RobotGoldenSectionSearchOptions {
		COL_RobotGoldenSectionSearch,
		COM_RobotGoldenSectionSearch
	};
	struct RobotGoldenSectionSearchParameters : public UnimodalSearchParameters {
		RobotGoldenSectionSearchOptions RobotGoldenSectionSearchGroupOptions;
	};
	struct RobotGoldenSectionSearchSampleParameters :public UnimodalSearchSampleParameters {

	};

private:
	static double GoldenRatioX1(double min, double max)
	{
			return max - (max - min)*0.61803398874989484820458683436564;

	}
	static double GoldenRatioX2(double min, double max)
	{

		return min + (max - min)*0.61803398874989484820458683436564;
	}
	void stepInternal();
	void initialSetup();
	RobotGoldenSectionSearchParameters* RobotGoldenSectionSearchData;
	std::shared_ptr<RobotGoldenSectionSearchSampleParameters> boundaryMin;
	std::shared_ptr<RobotGoldenSectionSearchSampleParameters> boundaryMax;
	std::shared_ptr<RobotGoldenSectionSearchSampleParameters> x1, x2;
public:
	RobotGoldenSectionSearch();

};
#endif