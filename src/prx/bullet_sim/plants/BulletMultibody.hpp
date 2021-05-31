#ifndef BULLET_NOT_BUILT
#pragma once

#include "prx/bullet_sim/plants/bullet_plant.hpp"
#include "prx/simulation/playback/plan.hpp"

namespace prx
{
    class bullet_multibody_t : public bullet_t
    {
        public:
        bullet_multibody_t(const std::string& path);
      
        virtual ~bullet_multibody_t();

        void setup();

        virtual void update_from_bullet(const space_point_t& point, const bool save_sim_state) override final;

	virtual void compute_control() override final;
      void execute_plan(plan_t plan);
      //void execute_traj(trajectory_t traj);

        protected:
      void shared_constructor(const std::string& path,std::vector<double> start_state);

      
      std::string robot_model_path;
      std::string control_mode;
      double x,y,theta,sid,fwd,steer;
      std::vector<int> controllableJoints;
      /*
      std::vector<int> steeringLinks = {0,2};
      double maxForce = 20;
      int nMotors = 2;
      std::vector<int> motorizedWheels = {8,15};
      double speedMultiplier = 20;
      double steeringMultiplier = 0.5;
      */
    };
}
#endif
