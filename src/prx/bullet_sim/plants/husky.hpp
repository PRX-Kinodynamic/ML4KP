#ifndef BULLET_NOT_BUILT
#include "prx/bullet_sim/plants/bullet_plant.hpp"

namespace prx
{
    class husky_t : public bullet_t
    {
        public:
        husky_t(const std::string& path);
        virtual ~husky_t();

        virtual void update_from_bullet(const space_point_t& point, const bool save_sim_state) override final;

		virtual void compute_control() override final;

        protected:
        const std::string robot_model_path = bullet_path + "/data/husky/husky.urdf";

        std::vector<int> wheelJoints = {2,3,4,5};

        double x,y,theta,sid,lf,rf,lr,rr;

        double maxForce = 100;
        double velocityMultiplier = 10;
    };
}
#endif
