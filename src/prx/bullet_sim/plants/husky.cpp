#ifndef BULLET_NOT_BUILT
#include "prx/bullet_sim/plants/husky.hpp"

namespace prx
{
    husky_t::husky_t(const std::string& path) : bullet_t(path)
    {
        simulation_step = 0.01;
        sim->setTimeStep(simulation_step);

        std::cout << "Loading file " << robot_model_path << std::endl;
        b3RobotSimulatorLoadUrdfFileArgs* loadURDArgs = new b3RobotSimulatorLoadUrdfFileArgs();
        btVector3 basePosition;
        basePosition[0] = 0; basePosition[1] = 0; basePosition[2] = 0.1;
        loadURDArgs->m_startPosition = basePosition;
        uniqueId = sim->loadURDF(robot_model_path,*loadURDArgs);

        int numJoints = sim->getNumJoints(uniqueId);
        for (int i = 0; i < numJoints; i++)
        {
            b3RobotSimulatorJointMotorArgs controlArgs(CONTROL_MODE_VELOCITY);
            controlArgs.m_targetVelocity = 0;
            controlArgs.m_maxTorqueValue = 0;
            sim->setJointMotorControl(uniqueId,i,controlArgs);
        }

        for (int i =0; i < 100; i++)
        {
            sim->stepSimulation();
        }

        std::cout << "Finished stepping" << std::endl;

        btQuaternion baseOrientation;
        sim->getBasePositionAndOrientation(uniqueId,basePosition,baseOrientation);
        auto baseRotation = getEulerFromQuaternion(baseOrientation);
        sid = sim->saveStateToMemory();

        x = basePosition[0]; y = basePosition[1]; theta = baseRotation[2]; // start_z = basePosition[2];
        state_memory = {&x,&y,&theta,&sid};
        state_space = new space_t("EERD",state_memory,"XYThetaId");
        state_space->set_bounds({-15,-15,-PRX_PI,0},{15,15,PRX_PI,PRX_INFINITY});

        lf=rf=lr=rr=0;
        control_memory = {&lf,&rf,&lr,&rr};
        input_control_space = new space_t("EEEE",control_memory,"LfRfLrRr");
        input_control_space->set_bounds({-1,-1,-1,-1},{1,1,1,1});

        std::cout << "Escaping constructior" << std::endl;
    }

    husky_t::~husky_t()
    {}

    void husky_t::compute_control()
    {
        space_point_t sampled_control = input_control_space->make_point();
		input_control_space->copy_to_point(sampled_control);

        b3RobotSimulatorJointMotorArgs controlArgs(CONTROL_MODE_VELOCITY);
        controlArgs.m_maxTorqueValue = maxForce;

        for (int i = 0; i < wheelJoints.size(); i++)
        {
            controlArgs.m_targetVelocity = velocityMultiplier * sampled_control->at(i);
            sim->setJointMotorControl(uniqueId,wheelJoints[i],controlArgs);
        }
    }

    void husky_t::update_from_bullet(const space_point_t& point, const bool save_sim_state)
    {
        std::vector<double> current_state_vec;
        btVector3 basePosition, baseRotation;
		btQuaternion baseOrientation;

        sim->getBasePositionAndOrientation(uniqueId,basePosition,baseOrientation);
        current_state_vec.push_back(basePosition[0]);
        current_state_vec.push_back(basePosition[1]);
        current_state_vec.push_back(baseRotation[2]);

        int sid = state_space->at(3);
        if (save_sim_state) sid = sim->saveStateToMemory();
        current_state_vec.push_back(sid);

        state_space->copy_point_from_vector(point,current_state_vec);
		state_space->copy_from_point(point);
    }
}
#endif
