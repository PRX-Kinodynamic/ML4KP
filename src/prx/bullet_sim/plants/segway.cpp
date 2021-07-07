#ifndef BULLET_NOT_BUILT
#include "prx/bullet_sim/plants/segway.hpp"

namespace prx
{
  segway_t::segway_t(const std::string& path) : bullet_t(path){
    std::vector<double> start_state = {0,0,0.2,0,0,0};
    shared_constructor(path,start_state);
  }

  segway_t::segway_t(const std::string& path, std::vector<double> start_state) : bullet_t(path)
    {
      shared_constructor(path,start_state);
    }

    

  void segway_t::shared_constructor(const std::string& path, std::vector<double> start_state) {
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

        uniqueId = sim->loadURDF(robot_model_path,*loadURDArgs);

        int numJoints = sim->getNumJoints(uniqueId);
        for (int i = 0; i < numJoints; i++)
        {
	  b3JointInfo jointInfo;
	  sim->getJointInfo(uniqueId,i,&jointInfo);
	  if (jointInfo.m_jointType == 0){
	    wheelJoints.push_back(i);
	  }

	  auto control_type = CONTROL_MODE_TORQUE;
	  if(first_order){
	    control_type = CONTROL_MODE_VELOCITY;
	  }
	  b3RobotSimulatorJointMotorArgs controlArgs(control_type);

	  controlArgs.m_targetVelocity = 0;
	  controlArgs.m_maxTorqueValue = 0;
	  sim->setJointMotorControl(uniqueId,i,controlArgs);
        }
       
        std::cout << "Escaping constructior" << std::endl;
    }

    segway_t::~segway_t()
    {}

   void segway_t::setup(){
	btVector3 basePosition;
        btQuaternion baseOrientation;
        for (int i =0; i < 100; i++)
        {

            sim->stepSimulation();
        }
       
        std::cout << "Finished stepping" << std::endl;
	simulation_step = 0.01;
        sim->getBasePositionAndOrientation(uniqueId,basePosition,baseOrientation);
        auto baseRotation = getEulerFromQuaternion(baseOrientation);
        btVector3 baseVel, baseAngVel;
				sim->getBaseVelocity(uniqueId, baseVel, baseAngVel);
				sid = sim->saveStateToMemory();
				
				x = basePosition[0];
				y = basePosition[1];
				z = basePosition[2];
				r = baseRotation[0];
				p = baseRotation[1];
				yaw = baseRotation[2];
				dx = baseVel[0];
				dy = baseVel[1];
				dz = baseVel[2];
				dr = baseAngVel[0];
				dp = baseAngVel[1];
				dyaw = baseAngVel[2];
	
	state_memory = {&x,&y,&z,&r,&p,&yaw,&dx,&dy,&dz,&dr,&dp,&dyaw,&sid};
	state_space = new space_t("EEERRREEEEEED",state_memory,"XYZRPYdxdydzdrdpdyaId");
	state_bounds_l.clear();//={-15,-15,,-PRX_PI,-10,-PRX_PI,0};
	state_bounds_u.clear();//={15,15,15,PRX_PI,100,PRX_PI,PRX_INFINITY};

	state_bounds_l.insert(state_bounds_l.end(),{-15,-15,-.1,-3.14,-3.14,-3.14,-20,-20,-20,-20,-20,-20,0});
	state_bounds_u.insert(state_bounds_u.end(),{15,15,1,3.14,3.14,3.14,20,20,20,20,20,20,PRX_INFINITY});
	
	state_space->set_bounds(state_bounds_l, state_bounds_u);
        lf=rf=lr=rr=0;
	if(sync_lr_wheels){
	  control_memory = {&lf,&rf};
	  input_control_space = new space_t("EE",control_memory,"LR");
	  control_bounds_l={-5,-5};
	  control_bounds_u={20,20};
	}else{
	  control_memory = {&lf,&rf,&lr,&rr};
	  input_control_space = new space_t("EEEE",control_memory,"LfRfLrRr");
	  control_bounds_l={-5,-5,-5,-5};
	  control_bounds_u={20,20,20,20};
	}
	input_control_space->set_bounds(control_bounds_l,control_bounds_u);	
		
    }

    void segway_t::compute_control()
    {
        space_point_t sampled_control = input_control_space->make_point();
    	input_control_space->copy_to_point(sampled_control);
	auto control_type = CONTROL_MODE_TORQUE;
	if(first_order){
	  control_type = CONTROL_MODE_VELOCITY;
	}
	b3RobotSimulatorJointMotorArgs controlArgs(control_type);
	
        controlArgs.m_maxTorqueValue = maxForce;

	if(sync_lr_wheels){
	  double control_0=controlMultiplier * sampled_control->at(0);
	  double control_1=controlMultiplier * sampled_control->at(1);
	  if(first_order){
	    controlArgs.m_targetVelocity = control_0;
	  }else{
	    controlArgs.m_maxTorqueValue = control_0; 
	  }
	  sim->setJointMotorControl(uniqueId,wheelJoints[0],controlArgs);
	  sim->setJointMotorControl(uniqueId,wheelJoints[2],controlArgs);
	  if(first_order){
	    controlArgs.m_targetVelocity = control_1;
	  }else{
	    controlArgs.m_maxTorqueValue = control_1; 
	  }
	  sim->setJointMotorControl(uniqueId,wheelJoints[1],controlArgs);
	  sim->setJointMotorControl(uniqueId,wheelJoints[3],controlArgs);

	}else{
	  for (int i = 0; i < wheelJoints.size(); i++)
	    {
	      if(first_order){
		controlArgs.m_targetVelocity = controlMultiplier * sampled_control->at(i);
	      }else{
		controlArgs.m_maxTorqueValue = controlMultiplier * sampled_control->at(i);
	      }
	      sim->setJointMotorControl(uniqueId,wheelJoints[i],controlArgs);
	    }
	}
    }


  void segway_t::set_control(std::vector<double> control){
    //input_control_space->copy_to_point(control);
	auto control_type = CONTROL_MODE_TORQUE;
	if(first_order){
	  control_type = CONTROL_MODE_VELOCITY;
	}
	b3RobotSimulatorJointMotorArgs controlArgs(control_type);
	
        controlArgs.m_maxTorqueValue = maxForce;

	if(sync_lr_wheels){
	  double control_0=controlMultiplier * control[0];
	  double control_1=controlMultiplier * control[1];
	  if(first_order){
	    controlArgs.m_targetVelocity = control_0;
	  }else{
	    controlArgs.m_maxTorqueValue = control_0; 
	  }
	  sim->setJointMotorControl(uniqueId,wheelJoints[0],controlArgs);
	  sim->setJointMotorControl(uniqueId,wheelJoints[2],controlArgs);
	  if(first_order){
	    controlArgs.m_targetVelocity = control_1;
	  }else{
	    controlArgs.m_maxTorqueValue = control_1; 
	  }
	  sim->setJointMotorControl(uniqueId,wheelJoints[1],controlArgs);
	  sim->setJointMotorControl(uniqueId,wheelJoints[3],controlArgs);

	}else{
	  for (int i = 0; i < wheelJoints.size(); i++)
	    {
	      if(first_order){
		controlArgs.m_targetVelocity = controlMultiplier * control[i];
	      }else{
		controlArgs.m_maxTorqueValue = controlMultiplier * control[i];
	      }
	      sim->setJointMotorControl(uniqueId,wheelJoints[i],controlArgs);
	    }
	}
    }
  
    
    void segway_t::update_from_bullet(const space_point_t& point, const bool save_sim_state)
    {
        std::vector<double> current_state_vec;
        btVector3 basePosition, baseRotation;
		btQuaternion baseOrientation;

        sim->getBasePositionAndOrientation(uniqueId,basePosition,baseOrientation);
		baseRotation = getEulerFromQuaternion(baseOrientation);

        current_state_vec.push_back(basePosition[0]);
        current_state_vec.push_back(basePosition[1]);
        current_state_vec.push_back(basePosition[2]);
		
		current_state_vec.push_back(baseRotation[0]);
        current_state_vec.push_back(baseRotation[1]);
        current_state_vec.push_back(baseRotation[2]);
		
		btVector3 baseVel, baseAngVel;
		sim->getBaseVelocity(uniqueId,baseVel,baseAngVel);
       current_state_vec.insert(current_state_vec.end(),{baseVel[0],baseVel[1],baseVel[2],baseAngVel[0],baseAngVel[1],baseAngVel[2]});	
	
        int sid = state_space->at(12);
       
        if (save_sim_state)
        {
        	sid = sim->saveStateToMemory();
	// std::cout << "sid: " << sid << std::endl;
        } 
        current_state_vec.push_back(sid);
        lastSavedId = std::max(lastSavedId, sid);
        state_space->copy_point_from_vector(point,current_state_vec);
		state_space->copy_from_point(point);
		space_point_t result = state_space->make_point();
		state_space->copy_to_point(result);
    }
}
#endif
