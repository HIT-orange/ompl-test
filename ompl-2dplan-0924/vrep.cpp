
#include "vrep.h"

namespace vrepspace
{
    
    using namespace std;
	void vrep::vrep_connect(const char* ip)
	{
        

        int Port = 19997;

        client_id = simxStart((simxChar*)ip, Port, 1, 1, 1000, 5);
        vrep_stop();
        extApi_sleepMs(100);
        if (client_id != -1)
        {
            cout << "V-rep connected.";
            simxGetObjectHandle(client_id, "robot", &robot_handle, simx_opmode_blocking);
            simxGetObjectHandle(client_id, "goal", &goal_handle, simx_opmode_blocking);

        }
        else
        {
            cout << "V-rep can't be connected.";
        }
	}
    void vrep::vrep_start()
    {
        simxStartSimulation(client_id, simx_opmode_oneshot);
        cout << "simulation started ...... " << endl;
        extApi_sleepMs(100);
    }
    void vrep::vrep_stop()
    {
        simxStopSimulation(client_id, simx_opmode_oneshot);
    }
    void vrep::vrep_setpose2d(int handle, float x, float y)
    {
        float position[3] = { x, y, 0 };
        simxSetObjectPosition(client_id, (simxInt)handle, -1, position,simx_opmode_oneshot);
        
    }
    void vrep::getobjectpos(int handle, float* pos)
    {
        simxGetObjectPosition(client_id, handle, -1, pos, simx_opmode_blocking);
        extApi_sleepMs(50);
    }
}