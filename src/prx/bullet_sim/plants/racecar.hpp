#ifndef BULLET_NOT_BUILT
#include "prx/bullet_sim/plants/bullet_plant.hpp"

namespace prx
{
    class racecar_t : public bullet_t
    {
        public:
        racecar_t(const std::string& path);
        racecar_t(const std::string& path, std::vector<double> start_state);

        virtual ~racecar_t();

        void setup();

        virtual void update_from_bullet(const space_point_t& point, const bool save_sim_state) override final;

		virtual void compute_control() override final;

        protected:
        void shared_constructor(const std::string& path, std::vector<double> start_state);

        const std::string robot_model_path = bullet_path + "/data/racecar/racecar_differential.urdf";

        double x,y,theta,sid,fwd,steer;

        std::vector<int> steeringLinks = {0,2};
        double maxForce = 20;
        int nMotors = 2;
        std::vector<int> motorizedWheels = {8,15};
        double speedMultiplier = 20;
        double steeringMultiplier = 0.5;
    };
}
#endif
