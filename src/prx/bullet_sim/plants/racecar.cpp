#ifndef BULLET_NOT_BUILT
#include "prx/bullet_sim/plants/racecar.hpp"

namespace prx
{


  racecar_t::racecar_t(const std::string& path): bullet_t(path){
    std::cout<<"in other constructor"<<std::endl;
    std::vector<double> start_state = {0,0,0.2,0,0,0};
      shared_constructor(path,start_state);
  }

  /*
  racecar_t::racecar_t(const std::string& path, std::vector<double> start_state) : bullet_t(path)
    {
      shared_constructor(path,start_state);
    }
    */


  void racecar_t::shared_constructor(const std::string& path, std::vector<double> start_state) {

        simulation_step = 0.01;
        sim->setTimeStep(simulation_step);

        std::cout << "Loading file " << robot_model_path << std::endl;
        b3RobotSimulatorLoadUrdfFileArgs* loadURDArgs = new b3RobotSimulatorLoadUrdfFileArgs();

        btVector3 basePosition, baseRotation;
        btQuaternion baseOrientation;
        basePosition[0] = start_state[0];
	basePosition[1] = start_state[1];
	basePosition[2] = start_state[2];
        baseRotation[0] = start_state[3];
	baseRotation[1] = start_state[4];
	baseRotation[2] = start_state[5];
        baseOrientation = getQuaternionFromEuler(baseRotation);
        loadURDArgs->m_startPosition = basePosition;
	// loadURDArgs->m_globalScaling = 1.0; //removed
        loadURDArgs->m_startOrientation = baseOrientation;
        uniqueId = sim->loadURDF(robot_model_path,*loadURDArgs);

        int numJoints = sim->getNumJoints(uniqueId);
        for (int i = 0; i < numJoints; i++)
        {
            b3RobotSimulatorJointMotorArgs controlArgs(CONTROL_MODE_VELOCITY);
            controlArgs.m_targetVelocity = 0;
            controlArgs.m_maxTorqueValue = 0;
            sim->setJointMotorControl(uniqueId,i,controlArgs);
        }


        b3JointInfo* jointInfo = new b3JointInfo;
        b3RobotUserConstraint* constraintInfo = new b3RobotUserConstraint;

        jointInfo->m_jointType = (int) JointType::eGearType;
        jointInfo->m_jointAxis[0] = 0; jointInfo->m_jointAxis[1] = 0; jointInfo->m_jointAxis[2] = 0;;
        for (int i = 0; i < 3; i++) jointInfo->m_parentFrame[i] = 0;
        for (int i = 0; i < 3; i++) jointInfo->m_childFrame[i] = 0;
        int constraintId = sim->createConstraint(uniqueId,9,uniqueId,11,jointInfo);
        constraintInfo->setGearRatio(1);
        constraintInfo->setMaxAppliedForce(10000);
        sim->changeConstraint(constraintId,constraintInfo);



        jointInfo->m_jointType = (int) JointType::eGearType;
        jointInfo->m_jointAxis[0] = 0; jointInfo->m_jointAxis[1] = 0; jointInfo->m_jointAxis[2] = 0;;
        for (int i = 0; i < 3; i++) jointInfo->m_parentFrame[i] = 0;
        for (int i = 0; i < 3; i++) jointInfo->m_childFrame[i] = 0;
        constraintId = sim->createConstraint(uniqueId,10,uniqueId,13,jointInfo);
        constraintInfo->setGearRatio(-1);
        constraintInfo->setMaxAppliedForce(10000);
        sim->changeConstraint(constraintId,constraintInfo);

        jointInfo->m_jointType = (int) JointType::eGearType;
        jointInfo->m_jointAxis[0] = 0; jointInfo->m_jointAxis[1] = 0; jointInfo->m_jointAxis[2] = 0;;
        for (int i = 0; i < 3; i++) jointInfo->m_parentFrame[i] = 0;
        for (int i = 0; i < 3; i++) jointInfo->m_childFrame[i] = 0;
        constraintId = sim->createConstraint(uniqueId,9,uniqueId,13,jointInfo);
        constraintInfo->setGearRatio(-1);
        constraintInfo->setMaxAppliedForce(10000);
        sim->changeConstraint(constraintId,constraintInfo);

        jointInfo->m_jointType = (int) JointType::eGearType;
        jointInfo->m_jointAxis[0] = 0; jointInfo->m_jointAxis[1] = 0; jointInfo->m_jointAxis[2] = 0;;
        for (int i = 0; i < 3; i++) jointInfo->m_parentFrame[i] = 0;
        for (int i = 0; i < 3; i++) jointInfo->m_childFrame[i] = 0;
        constraintId = sim->createConstraint(uniqueId,16,uniqueId,18,jointInfo);
        constraintInfo->setGearRatio(1);
        constraintInfo->setMaxAppliedForce(10000);
        sim->changeConstraint(constraintId,constraintInfo);

        jointInfo->m_jointType = (int) JointType::eGearType;
        jointInfo->m_jointAxis[0] = 0; jointInfo->m_jointAxis[1] = 0; jointInfo->m_jointAxis[2] = 0;;
        for (int i = 0; i < 3; i++) jointInfo->m_parentFrame[i] = 0;
        for (int i = 0; i < 3; i++) jointInfo->m_childFrame[i] = 0;
        constraintId = sim->createConstraint(uniqueId,16,uniqueId,19,jointInfo);
        constraintInfo->setGearRatio(-1);
        constraintInfo->setMaxAppliedForce(10000);
        sim->changeConstraint(constraintId,constraintInfo);

        jointInfo->m_jointType = (int) JointType::eGearType;
        jointInfo->m_jointAxis[0] = 0; jointInfo->m_jointAxis[1] = 0; jointInfo->m_jointAxis[2] = 0;;
        for (int i = 0; i < 3; i++) jointInfo->m_parentFrame[i] = 0;
        for (int i = 0; i < 3; i++) jointInfo->m_childFrame[i] = 0;
        constraintId = sim->createConstraint(uniqueId,17,uniqueId,19,jointInfo);
        constraintInfo->setGearRatio(-1);
        constraintInfo->setMaxAppliedForce(10000);
        sim->changeConstraint(constraintId,constraintInfo);

        jointInfo->m_jointType = (int) JointType::eGearType;
        jointInfo->m_jointAxis[0] = 0; jointInfo->m_jointAxis[1] = 0; jointInfo->m_jointAxis[2] = 0;;
        for (int i = 0; i < 3; i++) jointInfo->m_parentFrame[i] = 0;
        for (int i = 0; i < 3; i++) jointInfo->m_childFrame[i] = 0;
        constraintId = sim->createConstraint(uniqueId,1,uniqueId,18,jointInfo);
        constraintInfo->setGearRatio(-1);
        constraintInfo->setGearAuxLink(15);
        constraintInfo->setMaxAppliedForce(10000);
        sim->changeConstraint(constraintId,constraintInfo);

        jointInfo->m_jointType = (int) JointType::eGearType;
        jointInfo->m_jointAxis[0] = 0; jointInfo->m_jointAxis[1] = 0; jointInfo->m_jointAxis[2] = 0;;
        for (int i = 0; i < 3; i++) jointInfo->m_parentFrame[i] = 0;
        for (int i = 0; i < 3; i++) jointInfo->m_childFrame[i] = 0;
        constraintId = sim->createConstraint(uniqueId,3,uniqueId,19,jointInfo);
        constraintInfo->setGearRatio(-1);
        constraintInfo->setGearAuxLink(15);
        constraintInfo->setMaxAppliedForce(10000);
        sim->changeConstraint(constraintId,constraintInfo);

  /*
        for (int i =0; i < 100; i++)
        {
            sim->stepSimulation();
        }

        std::cout << "Finished stepping" << std::endl;

        sim->getBasePositionAndOrientation(uniqueId,basePosition,baseOrientation);
        baseRotation = getEulerFromQuaternion(baseOrientation);
        sid = sim->saveStateToMemory();
        lastSavedId = sid;

        x = basePosition[0]; y = basePosition[1]; theta = baseRotation[2];
        state_memory = {&x,&y,&theta,&sid};
        state_space = new space_t("EERD",state_memory,"XYThetaId");
        state_space->set_bounds({-15,-15,-3.15,0},{15,15,3.15,PRX_INFINITY});

        fwd=steer=0;
        control_memory = {&fwd,&steer};
        input_control_space = new space_t("ER",control_memory,"FwdSteer");
        input_control_space->set_bounds({-1,-1},{1,1});

        add_exclusion(0,-1,1,-1);  //exclude collisions with plane
        //add_exclusion(0,-1,2,-1);  //exclude collisions with plane
	*/
        std::cout << "Escaping constructor" << std::endl;
   }
    racecar_t::~racecar_t()
    {}


    void racecar_t::setup(){
      //set control to be -=0

      double targetVelocity = 0;
      double steeringAngle  = 0;

      b3RobotSimulatorJointMotorArgs controlArgs_speed(CONTROL_MODE_VELOCITY);
      controlArgs_speed.m_maxTorqueValue = maxForce;
      controlArgs_speed.m_targetVelocity = targetVelocity;

      for (int i = 0; i < motorizedWheels.size(); i++)
	sim->setJointMotorControl(uniqueId,motorizedWheels[i],controlArgs_speed);

      b3RobotSimulatorJointMotorArgs controlArgs_steering(CONTROL_MODE_POSITION_VELOCITY_PD);
      controlArgs_steering.m_targetPosition = steeringAngle;

      for (int i = 0; i < steeringLinks.size(); i++)
	sim->setJointMotorControl(uniqueId,steeringLinks[i],controlArgs_steering);


      //int id = sim->saveStateToMemory();
      //lastSavedId = std::max(lastSavedId, id);
      //std::cout<<"id = "<<id<<std::endl;
	//if(id!=0)
	//sim->restoreStateFromMemory(0);
	btVector3 basePosition;
        btQuaternion baseOrientation;
        for (int i =0; i < 100; i++)
        {

	  sim->getBasePositionAndOrientation(uniqueId,basePosition,baseOrientation);
	  std::cout<<basePosition[0]<<" "<<basePosition[1]<<" "<<basePosition[2]<<" "<<baseOrientation[0]<<" "<<baseOrientation[1]<<" "<<baseOrientation[0]<<" "<<baseOrientation[1]<<std::endl;
            sim->stepSimulation();
        }
        int id = sim->saveStateToMemory();

        //std::cout << "Finished stepping" << std::endl;



        sim->getBasePositionAndOrientation(uniqueId,basePosition,baseOrientation);
	std::cout<<basePosition[0]<<" "<<basePosition[1]<<" "<<basePosition[2]<<" "<<baseOrientation[0]<<" "<<baseOrientation[1]<<" "<<baseOrientation[0]<<" "<<baseOrientation[1]<<std::endl;
        double sid = sim->saveStateToMemory();
        lastSavedId = std::max((double)lastSavedId, sid);
	std::cout<<"saved state = "<<sid<<std::endl;
        x = basePosition[0]; y = basePosition[1];
	auto baseRotation = getEulerFromQuaternion(baseOrientation);
	double theta = baseRotation[2];
        state_memory = {&x,&y,&theta,&sid};
        state_space = new space_t("EERD",state_memory,"XYTId");
	std::vector<double> lower(4);
	lower[0]=-100;
	lower[1]=-100;
	lower[2]=-3.15;
	lower[3]=-0;
	std::vector<double> upper(4);
	upper[0]=100;
	upper[1]=100;
	upper[2]=3.15;
	upper[3]=PRX_INFINITY;
        state_space->set_bounds(lower,upper);

        fwd=steer=0;
        control_memory = {&fwd,&steer};
        input_control_space = new space_t("ER",control_memory,"FwdSteer");
        input_control_space->set_bounds({-1,-1},{1,1});
    }

    void racecar_t::compute_control()
    {
        space_point_t sampled_control = input_control_space->make_point();
		input_control_space->copy_to_point(sampled_control);

        double targetVelocity = sampled_control->at(0) * speedMultiplier;
        double steeringAngle  = sampled_control->at(1) * steeringMultiplier;

        b3RobotSimulatorJointMotorArgs controlArgs_speed(CONTROL_MODE_VELOCITY);
        controlArgs_speed.m_maxTorqueValue = maxForce;
        controlArgs_speed.m_targetVelocity = targetVelocity;

        for (int i = 0; i < motorizedWheels.size(); i++){
            sim->setJointMotorControl(uniqueId,motorizedWheels[i],controlArgs_speed);
	}
        b3RobotSimulatorJointMotorArgs controlArgs_steering(CONTROL_MODE_POSITION_VELOCITY_PD);
        controlArgs_steering.m_targetPosition = steeringAngle;

        for (int i = 0; i < steeringLinks.size(); i++){
            sim->setJointMotorControl(uniqueId,steeringLinks[i],controlArgs_steering);
	}
    }

    void racecar_t::update_from_bullet(const space_point_t& point, const bool save_sim_state)
    {
        std::vector<double> current_state_vec;
        btVector3 basePosition, baseRotation;
		btQuaternion baseOrientation;
		//std::cout<<"before get"<<std::endl;
        sim->getBasePositionAndOrientation(uniqueId,basePosition,baseOrientation);
	//std::cout<<"after get"<<std::endl;

        baseRotation = getEulerFromQuaternion(baseOrientation);
        current_state_vec.push_back(basePosition[0]);
        current_state_vec.push_back(basePosition[1]);
        current_state_vec.push_back(baseRotation[2]);
	//std::cout<<" current state vector in updae from bull"<<current_state_vec[0]<<" "<<current_state_vec[1]<<" "<<current_state_vec[2]<<std::endl;
	//std::cout<<"before 3"<<std::endl;
        int sid = state_space->at(3);
	//std::cout<<"before save state"<<std::endl;
        if (save_sim_state)
                sid = sim->saveStateToMemory();

        current_state_vec.push_back(sid);
        lastSavedId = std::max(lastSavedId, sid);
	//current_state_vec.pop_back();
        state_space->copy_point_from_vector(point,current_state_vec);
	state_space->copy_from_point(point);
	space_point_t result = state_space->make_point();
	state_space->copy_to_point(result);
    }
}

#endif

