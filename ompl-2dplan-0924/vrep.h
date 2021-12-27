#pragma once
#include"extApi.h"
#include"simConst.h"
#include"extApiPlatform.h"
#include <math.h>
#include <iostream>
#include <vector>


namespace vrepspace
{
	class vrep
	{
	public:
		void vrep_connect(const char* ip);
		void vrep_start();
		void vrep_stop();
		void vrep_setpose2d(int handle, float x, float y);
		void getobjectpos(int handle, float* pos);
		int client_id;
		int robot_handle;
		int goal_handle;
	};



}


