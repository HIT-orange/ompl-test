#pragma once
#include "vrep.h"
#include "2dpointplanningdemo.h"

using namespace std;
namespace vp = vrepspace;
vp::vrep m_vrep;

void Plane2DEnvironment::drawPath()
{
	float x, y;
	if (ss_->haveSolutionPath())
	{
		if (!useDeterministicSampling_)
			ss_->simplifySolution();

		og::PathGeometric& p = ss_->getSolutionPath();
		if (!useDeterministicSampling_)
		{
			ss_->getPathSimplifier()->simplifyMax(p);
			ss_->getPathSimplifier()->smoothBSpline(p);
		}

		size_t length = p.getStateCount();
		//p.interpolate(20);
		cout << "path length is : " << endl << length << endl;

		for (int i = 0; i < length; i++)
		{
			x = (p.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[0])/100;
			y = (p.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values[1])/100;
			cout << "path point : " << i << ".  x : " << x << ".y : " << y << endl;
			m_vrep.vrep_setpose2d(m_vrep.robot_handle, x, y);
			extApi_sleepMs(10);
		}

	}

}
