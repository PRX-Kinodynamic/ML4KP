#ifndef BULLET_NOT_BUILT

#include "prx/bullet_sim/plants/bullet_plant.hpp"
#include <math.h>

namespace prx
{

  	bullet_t::bullet_t(const std::string& path) : plant_t(path)
	{
	  	count = 0;
		/*
		Initialize the state and control spaces for the robot.
		*/
		system_type = plant_type::BULLET;
		
		// Can also use other physics servers like DIRECT, SHARED_MEMORY, UDP etc.
		//sim->connect(eCONNECT_GUI);
		while(!sim->isConnected())
		{
		  	std::cout<<"waiting for connection"<<std::endl;
		  	sim->connect(eCONNECT_GUI);
		  	// sim->connect(eCONNECT_DIRECT);
		}
		// If connecting to an existing physics server, make sure to uncomment the following line.
		// sim->syncBodies();
		sim->configureDebugVisualizer(COV_ENABLE_GUI,0);
		sim->configureDebugVisualizer(COV_ENABLE_MOUSE_PICKING,0);
		sim->setTimeOut(10);
	       
		sim->setTimeStep(simulation_step);
		physicsArgs.m_deterministicOverlappingPairs = 1;
		sim->setPhysicsEngineParameter(physicsArgs);

		sim->setGravity(btVector3(0,0,-9.8));

		lineArgs->m_lineWidth = 2.0;
		lineArgs->m_colorRGB[1] = lineArgs->m_colorRGB[2] = 0;	

	  	current_state = state_space->make_point();
	}


	bullet_t::~bullet_t()
	{
		std::cout << "Disconnecting simulation..." << std::endl;
		//purge_saved_states();
		sim->disconnect();		
		std::cout << "Deleting simulation..." << std::endl;
		delete sim;
	}	
	
	
    void bullet_t::setup()
    {
	  prx_throw("Please implement setup method for your plant.")
	}

  	void bullet_t::print_trajectories(const std::vector<trajectory_t> trajs)
	{
		std::cout << "Number of trajectories = " << trajs.size() << std::endl;
		btVector3 targetPos;
		targetPos[0] = targetPos[1] = targetPos[2] = 0;

		int j=0;
		for (auto traj : trajs)
		{
		  	std::cout<<"Trajectory "<<j<<":"<<std::endl;
		  	j++;
  			for (int i = 0; i < traj.size()-1; i++)
			{
				unsigned idx = i;
				//to do - make this depend on dimension
				double* startLine = new double[3]{traj[idx]->at(0),traj[idx]->at(1),traj[idx]->at(2)};
				double* endLine = new double[3]{traj[idx+1]->at(0),traj[idx+1]->at(1),traj[idx]->at(2)};
				if(i==0)
				{
				  	std::cout<<startLine[0]<<", "<<startLine[1]<<", "<<startLine[2]<<std::endl;
				}
				std::cout<<endLine[0]<<", "<<endLine[1]<<", "<<endLine[2]<<std::endl;
			}
		}
	}
  
	void bullet_t::visualize_trajectories(const std::vector<trajectory_t> trajs)
	{
		std::cout << "Number of trajectories = " << trajs.size() << std::endl;
		btVector3 targetPos;
		targetPos[0] = targetPos[1] = targetPos[2] = 0;
		sim->resetDebugVisualizerCamera(4.0,-90.4,180.1,targetPos);		
		sim->restoreStateFromMemory(0);
		for (auto traj : trajs)
		{
  			for (int i = 1; i < traj.size()-1; i++)
			{
				unsigned idx = i;
				double* startLine = new double[3]{traj[idx]->at(0),traj[idx]->at(1),0.2};
				double* endLine = new double[3]{traj[idx+1]->at(0),traj[idx+1]->at(1),0.2};
				sim->addUserDebugLine(startLine,endLine,*lineArgs);
			}
		}
	}

	void bullet_t::visualize_goal(const space_point_t goal, const double radius)
	{
		btVector3 pos;
		pos[0] = goal->at(0); pos[1] = goal->at(1); pos[2] = 0.2;	
		b3RobotSimulatorAddUserDebugTextArgs* textArgs = new b3RobotSimulatorAddUserDebugTextArgs;
		textArgs->m_colorRGB[0] = textArgs->m_colorRGB[1] = textArgs->m_colorRGB[2] = 0;
		sim->addUserDebugText("GOAL",pos,*textArgs);
	}

	void bullet_t::propagate(const double simulation_step)
	{
		prx_assert(current_state != nullptr, "current_state not initialized!");
	  sim->setTimeStep(simulation_step);
	  if(!isCollision())
	  {
	    state_space->copy_to_point(current_state);
	    update_to_bullet(current_state);
	    if(!isCollision())
	    {
	      sim->stepSimulation();
	    }
	    this->update_from_bullet(current_state, true);
	  }
	}

	void bullet_t::propagate(const double simulation_step, const propagate_step step)
	{
    	        sim->setTimeStep(simulation_step);
		space_point_t current_state = state_space->make_point();
		state_space->copy_to_point(current_state);

		if (step == propagate_step::FIRST_STEP)
		{	
			update_to_bullet(current_state);
		}

		if(!isCollision())
		{
		  sim->stepSimulation();
		  bool save_sim_state = (step == propagate_step::FINAL_STEP);
		  this->update_from_bullet(current_state, save_sim_state);
		}
		// std::cout << "Exited Bullet propagate..." << std::endl;

	}

	void bullet_t::compute_control()
	{
		/*
		Needs to be implemented individually for each robot.
		*/
	}

  	void bullet_t::purge_saved_states()
	{
		std::cout << "Purging all non iniital states..." << std::endl;
		for (int i = 1; i < lastSavedId; i++)
		{
			sim->removeStateFromMemory(i);
		} 
		lastSavedId=0;
		std::cout << "Finished purging." << std::endl;
	}

	void bullet_t::update_from_bullet(const space_point_t& point, const bool save_sim_state)
	{
		prx_throw("Please implement a update_from_bullet method for your plant.")
	}

	void bullet_t::update_to_bullet(const space_point_t& point)
	{

		std::vector<double> current_state_vec;
		state_space->copy_vector_from_point(current_state_vec,point);
		sim->restoreStateFromMemory(current_state_vec.back());

	}

	void bullet_t::update_configuration()
	{
	    space_point_t current_state = state_space->make_point();
	    state_space->copy_to_point(current_state);
	    std::vector<double> current_state_vec;
	    state_space->copy_vector_from_point(current_state_vec,current_state);
	    sim->restoreStateFromMemory(current_state_vec.back());		
	}
	void bullet_t::compute_derivative()
	{

	}

	btVector3 bullet_t::getEulerFromQuaternion(btQuaternion quat)
	{
		btScalar roll, pitch, yaw;
		quat.getEulerZYX(yaw, pitch, roll);
		btVector3 rpy2 = btVector3(roll, pitch, yaw);
		return rpy2;
	}

	btQuaternion bullet_t::getQuaternionFromEuler(const btVector3& rollPitchYaw)
	{
		btQuaternion q;
		q.setEulerZYX(rollPitchYaw[2], rollPitchYaw[1], rollPitchYaw[0]);
		return q;
	}

  void bullet_t::add_exclusion(int bID1, int lID1, int bID2, int lID2)
	{
	  	std::pair<std::pair<int, int>, std::pair<int,int> > exclusion;
	  	exclusion.first.first = bID1;
	  	exclusion.first.second = lID1;
	  	exclusion.second.first = bID2;
	  	exclusion.second.second = lID2;
	  	m_CD_exclusion_list.push_back(exclusion);
  }
  
  bool bullet_t::b_exclude(std::pair<std::pair<int, int>, std::pair<int, int> > excluded_pair, const b3ContactPointData &contact)
	{
	  if(excluded_pair.first.first == contact.m_bodyUniqueIdA &&(excluded_pair.first.second == contact.m_linkIndexA || excluded_pair.first.second == -1))
		  {
		    if(excluded_pair.second.first == contact.m_bodyUniqueIdB &&(excluded_pair.second.second == contact.m_linkIndexB || excluded_pair.second.second == -1))
			{
	      		return true;
			}
	  	}
	  	if(excluded_pair.second.first == contact.m_bodyUniqueIdA &&(excluded_pair.second.second == contact.m_linkIndexA || excluded_pair.second.second == -1))
		  {
		    if(excluded_pair.first.first == contact.m_bodyUniqueIdB &&(excluded_pair.first.second == contact.m_linkIndexB || excluded_pair.first.second == -1))
		      {
	      		return true;
		      }
	  	}
	  	return false;
    }

   bool bullet_t::b_exclude_contact(const b3ContactPointData &contact)
	{
	  	for(int i=0; i<m_CD_exclusion_list.size(); i++)
		{		  
		  if(b_exclude(m_CD_exclusion_list[i], contact))
		    return true; 
	  	}  
	  	return false;
	}

  
  bool bullet_t::isCollision(bool b_include_bounding_box)
	{
	    const space_point_t s;
	  b3RobotSimulatorGetContactPointsArgs args;
	  b3ContactInformation *contactInfo = new b3ContactInformation();	  
	  sim->getContactPoints(args, contactInfo);
	  for(int i=0; i<contactInfo->m_numContactPoints; i++){	    
	    bool b_excluded = b_exclude_contact(contactInfo->m_contactPointData[i]);
	    if(!b_excluded){
	      return true;
	    }
	  }
	  return false;
	}
  
  void bullet_t::execute_traj(trajectory_t traj){
		btVector3 targetPos;
		targetPos[0] = targetPos[1] = targetPos[2] = 0;
		sim->resetDebugVisualizerCamera(15.0,-90.4,180.1,targetPos);	
    sim->restoreStateFromMemory(0);
    for(int i=0; i<traj.size(); i++){
      usleep(8000);
      space_point_t point = traj[(unsigned)i];
      int inpt;
      sim->restoreStateFromMemory(point->at(state_bounds_l.size()-1));
      
    }
  }


  void bullet_t::setBasePositionAndRotation(btVector3 basePosition, btVector3 baseRotation){
    btQuaternion baseOrientation = getQuaternionFromEuler(baseRotation);
    sim->resetBasePositionAndOrientation(uniqueId,basePosition,baseOrientation);
  }
}
#endif
