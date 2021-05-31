#include "prx/bullet_sim/plants/BulletMultibody.hpp"
#include <algorithm>

namespace prx
{


  bullet_multibody_t::bullet_multibody_t(const std::string& path): bullet_t(path){
    
  }



  
  void bullet_multibody_t::shared_constructor(const std::string& path, std::vector<double> start_state) {
    std::vector<double> upper;  //top do: make lower and upper params
    std::vector<double> lower;
    simulation_step = 0.05;
        sim->setTimeStep(simulation_step);
        //state_vec=start_state;
        std::cout << "Loading file " << robot_model_path << std::endl;
        b3RobotSimulatorLoadUrdfFileArgs* loadURDArgs = new b3RobotSimulatorLoadUrdfFileArgs();
	std::cout<<"done loading file"<<std::endl;
        btVector3 basePosition, baseRotation;
        btQuaternion baseOrientation;
        basePosition[0] = start_state[0];
	basePosition[1] = start_state[1];
	basePosition[2] = start_state[2];
        baseRotation[0] = start_state[3];
	baseRotation[1] = start_state[4];
	baseRotation[2] = start_state[5];
	//std::nout<<"here"<<std::endl;
        baseOrientation = getQuaternionFromEuler(baseRotation);
        loadURDArgs->m_startPosition = basePosition;
	// loadURDArgs->m_globalScaling = 1.0; //removed
        loadURDArgs->m_startOrientation = baseOrientation;
        uniqueId = sim->loadURDF(robot_model_path,*loadURDArgs);
	//sim->setBasePositionAndOrientation(uniqueId,basePosition,baseOrientation);

	std::vector<double> state_bounds_l_tmp, state_bounds_u_tmp, control_bounds_l_tmp, control_bounds_u_tmp;  //create bounds as we construct robot but overide them if bounds have already been defiend in descendant class
	control_topo = "";
	state_topo="";	
	state_topo.append("EEERRREEEEEE");
	state_bounds_l_tmp.insert(state_bounds_l_tmp.end(),{-10,-10,-.1,-3.14,-3.14,-3.14,-10,-10,-10,-10,-10,-10});
	state_bounds_u_tmp.insert(state_bounds_u_tmp.end(),{10,10,1,3.14,3.14,3.14,10,10,10,10,10,10});

	/*
	for(int i=0; i<12; i++){
	  if(i<lower.size()){
	    state_bounds_l.push_back(lower[i]);
	  }else if(i==3 || i==4 || i==5){  //rotational joints
	    state_bounds_l.push_back(-3.14);
	  }else{
	    state_bounds_l.push_back(-100);
	  }
	  if(i<upper.size()){
	    state_bounds_u.push_back(upper[i]);
	  }else if(i==3 || i==4 || i==5){  //rotational joints
	    state_bounds_u.push_back(3.14);
	  }else{
	    state_bounds_u.push_back(100);
	  }
	  }*/
	//std::cout<<"set topology"<<std::endl;
        int numJoints = sim->getNumJoints(uniqueId);
	control_mode = "torque";
	controllableJoints.clear();
        for (int i = 0; i < numJoints; i++)
        {

	    b3JointInfo jointInfo;
	    sim->getJointInfo(uniqueId,i,&jointInfo);
	    if (jointInfo.m_jointType == 0){		//0 = controllable joint
	      b3RobotSimulatorJointMotorArgs controlArgs(CONTROL_MODE_TORQUE);//CONTROL_MODE_VELOCITY);

	      controlArgs.m_targetVelocity = 0;
	      controlArgs.m_maxTorqueValue = 0;
	      sim->setJointMotorControl(uniqueId,i,controlArgs);  //set controls to be 0 for initial stepping
	      controllableJoints.push_back(i);
	      state_topo.append("RE");
	      state_bounds_l_tmp.push_back(std::max(jointInfo.m_jointLowerLimit*2,-10.0));
		  //state_bounds_u.push_back(3.15);
		  
		  if (jointInfo.m_jointLowerLimit < jointInfo.m_jointUpperLimit)	
		    state_bounds_u_tmp.push_back(std::min(jointInfo.m_jointUpperLimit*2,10.0));
		  else
		    state_bounds_u_tmp.push_back(std::min(jointInfo.m_jointLowerLimit*2,10.0));
		  
		  //state_bounds_l.push_back(-3.15);
		  //state_bounds_u.push_back(3.15);


		  state_bounds_l_tmp.push_back(-10);
		  //state_bounds_u.push_back(100);
		  state_bounds_u_tmp.push_back(std::min(jointInfo.m_jointMaxVelocity*2,10.0));
		  control_topo.append("E");

		  control_bounds_l_tmp.push_back(-10.0);
		  if (control_mode == "torque"){		    
		    //control_vec.push_back(-4.5);
		    //control_bounds_u.push_back(100);
		    
			if (jointInfo.m_jointMaxForce == 0.0)
				control_bounds_u_tmp.push_back(10.0);
			else
			  control_bounds_u_tmp.push_back(std::min(jointInfo.m_jointMaxForce,10.0));
		    
			
		}else if (control_mode == "velocity"){			
		    //control_vec.push_back(-10);
		    //control_bounds_u_tmp.push_back(100); 
		    
			if (jointInfo.m_jointMaxVelocity == 0.0)
				control_bounds_u_tmp.push_back(10.0); 
			else
			  control_bounds_u_tmp.push_back(std::min(jointInfo.m_jointMaxVelocity,10.0));
		    	
		  }else{
		    control_bounds_u_tmp.push_back(10);
		  }

		std::cout << "Found a controllable joint " << jointInfo.m_jointName << 
		  " with jPos limits [" << jointInfo.m_jointLowerLimit << "," << jointInfo.m_jointUpperLimit << "]" << 
		  " with jVel limits [0," << jointInfo.m_jointMaxVelocity << "]" <<
		  " maxForce " << jointInfo.m_jointMaxForce << std::endl;
            }
         }
	//std::cout<<"done joints"<<std::endl;
         //now set up state and control spaces
	state_vec.resize(12+2*controllableJoints.size()+1); 
	
         for (int i = 0; i < state_vec.size(); i++){
	   if(i<start_state.size()){
	     state_vec[i]=start_state[i];
	   }else{
	     state_vec[i]=0;
	   }
	   state_memory.push_back(&state_vec.at(i));
	  }
	 control_vec.resize(controllableJoints.size());
	  for (int i = 0; i < control_vec.size(); i++){
	    control_vec[i]=0;
	    control_memory.push_back(&control_vec.at(i));
	  }
	  state_topo.append("D");//for state ID
	  state_bounds_l_tmp.push_back(0);
	  state_bounds_u_tmp.push_back(PRX_INFINITY);
  	  state_space = new space_t(state_topo,state_memory,"PosVelsJointsState");
	  //std::cout<<"done state space"<<std::endl;
	  input_control_space = new space_t(control_topo, control_memory,"jTorques");
	  //std::cout<<"done control space"<<std::endl;
	  
	  for(int i=state_bounds_l.size(); i<state_bounds_l_tmp.size(); i++){
	    state_bounds_l.push_back(state_bounds_l_tmp[i]);
	  }
	  for(int i=state_bounds_u.size(); i<state_bounds_u_tmp.size(); i++){
	    state_bounds_u.push_back(state_bounds_u_tmp[i]);
	  }
	  for(int i=control_bounds_l.size(); i<control_bounds_l_tmp.size(); i++){
	    control_bounds_l.push_back(control_bounds_l_tmp[i]);
	  }
	  for(int i=control_bounds_u.size(); i<control_bounds_u_tmp.size(); i++){
	    control_bounds_u.push_back(control_bounds_u_tmp[i]);
	  }
	  //std::cout<<"control bounds "<<control_bounds_l.size()<<" "<<control_bounds_u.size()<<" "<<control_bounds_u_tmp.size()<<std::endl;
          state_space->set_bounds(state_bounds_l,state_bounds_u);
          input_control_space->set_bounds(control_bounds_l,control_bounds_u);
  	  std::cout << "Escaping constructor" << std::endl;
       }


    

    bullet_multibody_t::~bullet_multibody_t()
    {}
    
    
    void bullet_multibody_t::setup(){
      //set control to be -=0
      int numJoints = sim->getNumJoints(uniqueId);
      for (int i = 0; i < numJoints; i++){
	//b3RobotSimulatorJointMotorArgs controlArgs(CONTROL_MODE_VELOCITY);
	b3RobotSimulatorJointMotorArgs controlArgs(CONTROL_MODE_TORQUE);

            controlArgs.m_targetVelocity = 0;
            controlArgs.m_maxTorqueValue = 0;
            sim->setJointMotorControl(uniqueId,i,controlArgs);
      }
	btVector3 basePosition;
        btQuaternion baseOrientation;
	simulation_step = 0.001;
	/*
        for (int i =0; i < 100; i++)
        {

	  sim->getBasePositionAndOrientation(uniqueId,basePosition,baseOrientation);
	  std::cout<<basePosition[0]<<" "<<basePosition[1]<<" "<<basePosition[2]<<" "<<baseOrientation[0]<<" "<<baseOrientation[1]<<" "<<baseOrientation[2]<<" "<<baseOrientation[3]<<std::endl;
            sim->stepSimulation();
	    //if(basePosition[2]==0)
	    //break;
        }
	*/

        std::cout << "Finished stepping" << std::endl;

	simulation_step = 0.0002;

        sim->getBasePositionAndOrientation(uniqueId,basePosition,baseOrientation);
	std::cout<<basePosition[0]<<" "<<basePosition[1]<<" "<<basePosition[2]<<" "<<baseOrientation[0]<<" "<<baseOrientation[1]<<" "<<baseOrientation[2]<<" "<<baseOrientation[3]<<std::endl;
        double sid = sim->saveStateToMemory();	
        lastSavedId = std::max((double)lastSavedId, sid);
	std::cout<<"saved state = "<<sid<<std::endl;
	space_point_t result = state_space->make_point();
	update_from_bullet(result,false);
	state_space->copy_to_point(result);    
    }

    void bullet_multibody_t::compute_control()
    {
      //std::cout<<"in control ";
      space_point_t sampled_control = input_control_space->make_point();
      input_control_space->copy_to_point(sampled_control);      
      for (int i = 0; i < controllableJoints.size(); i++){		
	if (control_mode == "torque"){		    
	  b3RobotSimulatorJointMotorArgs controlArgs(CONTROL_MODE_TORQUE);
	  // std::cout << sampled_control->at(i) << ", ";
	  controlArgs.m_maxTorqueValue = sampled_control->at(i);		      
	  sim->setJointMotorControl(uniqueId,controllableJoints[i],controlArgs);		    
	}else if (control_mode == "velocity"){		    
	  b3RobotSimulatorJointMotorArgs controlArgs(CONTROL_MODE_VELOCITY);
	  controlArgs.m_maxTorqueValue = .25;
	  controlArgs.m_targetVelocity = sampled_control->at(i);
	  //std::cout<<controlArgs.m_targetVelocity<<" ";
	  sim->setJointMotorControl(uniqueId,controllableJoints[i],controlArgs);
	}	
      }
      //std::cout<<std::endl;
    }

    void bullet_multibody_t::update_from_bullet(const space_point_t& point, const bool save_sim_state)
    {     
      //std::vector<double> current_state_vec;
	state_vec.clear();
        btVector3 basePosition, baseRotation;
		btQuaternion baseOrientation;  
		//std::cout<<"before get"<<std::endl;
        sim->getBasePositionAndOrientation(uniqueId,basePosition,baseOrientation);
	//std::cout<<"after get"<<std::endl;

        baseRotation = getEulerFromQuaternion(baseOrientation);
        state_vec.insert(state_vec.end(),{basePosition[0],basePosition[1],basePosition[2],baseRotation[0],baseRotation[1],baseRotation[2]});
	  /*
	current_state_vec.push_back(basePosition[0]);
	current_state_vec.push_back(basePosition[1]);
	current_state_vec.push_back(basePosition[2]);
	current_state_vec.push_back(baseRotation[0]);
	current_state_vec.push_back(baseRotation[1]);
	current_state_vec.push_back(baseRotation[2]);
	  */
	btVector3 baseVel, baseAngVel;
	sim->getBaseVelocity(uniqueId,baseVel,baseAngVel);
        state_vec.insert(state_vec.end(),{baseVel[0],baseVel[1],baseVel[2],baseAngVel[0],baseAngVel[1],baseAngVel[2]});	
	//std::cout<<" current state vector in updae from bull"<<current_state_vec[0]<<" "<<current_state_vec[1]<<" "<<current_state_vec[2]<<std::endl;
	//std::cout<<"before 3"<<std::endl;
	for(int i=0;i<controllableJoints.size();i++){
	  b3JointSensorState jointState;
	  sim->getJointState(uniqueId,controllableJoints[i],&jointState);
	  state_vec.push_back(jointState.m_jointPosition);
	  state_vec.push_back(jointState.m_jointVelocity);
	}

	
        int sid = state_space->at(12+2*controllableJoints.size());
       
        if (save_sim_state){
                sid = sim->saveStateToMemory();
		//std::cout<<"saving state"<<sid<<std::endl;
	}
        state_vec.push_back(sid);
        lastSavedId = std::max(lastSavedId, sid);
	//current_state_vec.pop_back();
	//std::cout<<"state = ";
	//for(int i=0; i<state_vec.size(); i++){
	//std::cout<<state_vec[i]<<",";
	//}
	//std::cout<<std::endl;
        state_space->copy_point_from_vector(point,state_vec);
	state_space->copy_from_point(point);
	space_point_t result = state_space->make_point();
	state_space->copy_to_point(result);
	//std::cout<<"returning "<<std::endl;
    }

  void bullet_multibody_t::execute_plan(plan_t plan){
	sim->restoreStateFromMemory(0);
	std::cout<<"printing plan "<<plan.size()<<std::endl;
	for(unsigned i=0; i<plan.size();i++){
	  std::cout<<"plan step "<<i<<std::endl;
		space_point_t control = plan[i].control;
        	//for (int j = 0; j < controllableJoints.size(); j+=2){
		  for (int j = 0; j < controllableJoints.size(); j++){
		    std::cout<<control->at(j)<<" ";
		    if (control_mode == "torque"){
		      b3RobotSimulatorJointMotorArgs controlArgs(CONTROL_MODE_TORQUE);
		      controlArgs.m_maxTorqueValue = control->at(j);		      
		      sim->setJointMotorControl(uniqueId,controllableJoints[j],controlArgs);
		    }else if (control_mode == "velocity"){		    
		      b3RobotSimulatorJointMotorArgs controlArgs(CONTROL_MODE_VELOCITY);
		      controlArgs.m_maxTorqueValue = .25;
		      controlArgs.m_targetVelocity = control->at(j);
		      sim->setJointMotorControl(uniqueId,controllableJoints[j],controlArgs);
		    }		    
		  }
		  std::cout<<std::endl;
		  //}
		  //int inp;
		  //std::cin>>inp;
		  for(double d=0; d<plan[i].duration; d+=simulation_step){
		    std::cout<<"stepping "<<d<<std::endl;
		    sim->stepSimulation();
		  }
	}
  }
  
  /*
   void bullet_multibody_t::execute_traj(trajectory_t traj){
     sim->restoreStateFromMemory(0);
     for(int i=0; i<traj.size(); i++){
       space_point_t point = traj[(unsigned)i];
       sim->restoreStateFromMemory(point->at(state_bounds_l.size()-1));
	
     }
   }
  */ 
}
