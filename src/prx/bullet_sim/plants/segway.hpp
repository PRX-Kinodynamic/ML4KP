#ifndef BULLET_NOT_BUILT
#include "prx/bullet_sim/plants/bullet_plant.hpp"

namespace prx
{
    class segway_t : public bullet_t
    {
        public:
      segway_t(const std::string& path);
      segway_t(const std::string& path,std::vector<double> start_state);
      
        virtual ~segway_t();

      void setup();

        virtual void update_from_bullet(const space_point_t& point, const bool save_sim_state) override final;

      virtual void compute_control() override final;
      void set_control(std::vector<double> control);

      bool  first_order = true;
      bool sync_lr_wheels = true;
      
        protected:
      const std::string robot_model_path = models_path + "/segway_440LE/segway_440LE.urdf";  //Note, you will need to change this path

      std::vector<int> wheelJoints;// = {2,3,4,5};

      double x,y,z,r,p,yaw,dx,dy,dz,dr,dp,dyaw,sid,lf,rf,lr,rr;
      double maxForce = 100;
      double controlMultiplier = 0.5;
 
      
    private:      
      void shared_constructor(const std::string& path,std::vector<double> start_state);
    };
}
#endif
