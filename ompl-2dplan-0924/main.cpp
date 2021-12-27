/*********************************************************************
 * @ Harbin Institute of Technology
 *
 * @author : HI_FORREST
 * 
 * @date : 2021.12.27
 * 
 *
 *********************************************************************/

#include "main.h"



int main()
{
    m_vrep.vrep_connect("127.0.0.1");
    m_vrep.vrep_start();
    
    std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

    boost::filesystem::path path("E:/c++program/ompl/resources/resources/");
    bool useDeterministicSampling = true;
    Plane2DEnvironment env((path/"ppm/floor.ppm").string().c_str(), useDeterministicSampling);

    float pos[3] = { 0,0,0 };
    float target[3] = { 0,0,0 };

    m_vrep.getobjectpos(m_vrep.robot_handle, pos);
    m_vrep.getobjectpos(m_vrep.goal_handle, target);
    int x = (int)(pos[0] * 100);
    int y = (int)(pos[1] * 100);
    int tx = (int)(target[0] * 100);
    int ty = (int)(target[1] * 100);
    cout << "start position : x : " << x << " . y : " << y << endl;
    cout << "goal position : tx : " << tx << " . ty : " << ty << endl;

    if (env.plan(x, y, tx, ty))
    {
        env.recordSolution();
        
        cout << "saving path as pictur ... " << endl;
        env.save("result_demo.ppm");
        cout << "showing path in vrep ... " << endl;
        env.drawPath();
    }

    return 0;
}



