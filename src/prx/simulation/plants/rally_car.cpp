
#include "prx/simulation/plants/rally_car.hpp"

namespace prx
{

    rally_car_t::rally_car_t(const std::string& path) : plant_t(path)
    {

        // x=y=theta=0;
        // state_memory = {&x, &y, &theta};
        _x=_y=_vx=_vy=_theta=_thetadot=_wf=_wr=0;
        state_memory = {&_x,&_y,&_vx,&_vy,&_theta,&_thetadot,&_wf,&_wr};

        state_space = new space_t("EEEEREEE",state_memory,"XYTheta");
        state_space->set_bounds({-26, -24, -9, -9, -3.14, -3.14, -40, -40},{0, 24, 9, 9, 3.14, 3.14, 40, 40});

        // v_linear=v_angular=0;
        // control_memory = {&v_linear, &v_angular};
        _sta=_tf=_tr=0;
        control_memory = {&_sta, &_tf, &_tr};   
        input_control_space = new space_t("REE",control_memory,"V_VTheta");
        input_control_space->set_bounds({-1.0472, -10, -10},{1.0472, 10, 10});

        // dx=dy=d_theta=0;
        // derivative_memory = {&dx, &dy, &d_theta};
        d_x = d_y = d_vx = d_vy = d_theta = d_thetadot = d_wf = d_wr = 0;
        derivative_memory = {&d_x,&d_y,&d_vx,&d_vy,&d_theta,&d_thetadot,&d_wf,&d_wr};
        derivative_space = new space_t("EEEERREE",derivative_memory,"XdotYdotThetadot");

/*      const std::string shape = "cylinder";
        if(shape=="sphere")
        {
            geometries["body"] = std::make_shared<geometry_t>(geometry_type_t::SPHERE);
            geometries["body"]->initialize_geometry({.05});
        }
        if(shape=="box")
        {
*/          geometries["body"] = std::make_shared<geometry_t>(geometry_type_t::BOX);
            geometries["body"]->initialize_geometry({1.5, 0.8, .5});
        // }
        // if(shape=="cylinder")
        // {
        //  geometries["body"] = std::make_shared<geometry_t>(geometry_type_t::CYLINDER);
        //  geometries["body"]->initialize_geometry({.5,1});
        // }
        // if(shape=="capsule")
        // {
        //  geometries["body"] = std::make_shared<geometry_t>(geometry_type_t::CAPSULE);
        //  geometries["body"]->initialize_geometry({.05,1});
        // }
        geometries["body"]->generate_collision_geometry();
        geometries["body"]->set_visualization_color("0x00ff00");
        configurations["body"]= std::make_shared<transform_t>();
        configurations["body"]->setIdentity();
        // printf("( %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f )\n" , _x, _y, _vx, _vy, _theta, _thetadot, _wf, _wr);
        set_integrator(integrator_t::kEULER);

    }

    rally_car_t::~rally_car_t()
    {

    }

    void rally_car_t::propagate(const double simulation_step)
    {
        integrator -> integrate(simulation_step);
    }

    void rally_car_t::update_configuration()
    {
        auto body = configurations["body"];
        // Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitZ());
        Eigen::AngleAxisd angle(_theta, Eigen::Vector3d::UnitZ());
        // Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitX());

        // Eigen::Quaternion<double> q = yawAngle;

        body->setIdentity();
        body->translation() = (vector_t(_x, _y, 0));
        body->linear() = angle.toRotationMatrix();
    }

    void rally_car_t::compute_derivative()
    {
        d_x = _vx;
        d_y = _vy;
        d_theta = _thetadot;

     //    printf("state: ( %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f )\n", _x, _y, _vx, _vy, _theta, _thetadot, _wf, _wr);
        // printf("deriv: ( %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f )\n", d_x, d_y, d_vx, d_vy, d_theta, d_thetadot, d_wf, d_wr);

        double V = sqrt(_vx*_vx+_vy*_vy);
        double beta = atan2(_vy,_vx) - _theta;
        double V_Fx = V*cos(beta-_sta) + _thetadot*LF*sin(_sta);
        double V_Fy = V*sin(beta-_sta) + _thetadot*LF*cos(_sta);
        double V_Rx = V*cos(beta);
        double V_Ry = V*sin(beta) - _thetadot*LR;

        double s_Fx = (V_Fx - _wf*R)/(_wf*R);
        // printf("s_Fx = (V_Fx - _wf*R)/(_wf*R)\t%.3f = (%.3f - %.3f*%.3f)/(%.3f*%.3f)\n", s_Fx, V_Fx, _wf, R, _wf, R);
        double s_Fy = V_Fy/(_wf*R);
        double s_Rx = (V_Rx - _wr*R)/(_wr*R);
        double s_Ry = V_Ry/(_wr*R);

        double s_F = sqrt(s_Fx*s_Fx+s_Fy*s_Fy);
        double s_R = sqrt(s_Rx*s_Rx+s_Ry*s_Ry);

        double mu_F = D*sin(C*atan(B*s_F));
        double mu_R = D*sin(C*atan(B*s_R));
        double mu_Fx;
        double mu_Fy;
        // printf("s's: ( %.3f, %.3f, %.3f, %.3f )\n", s_Fx, s_Fy, s_Rx, s_Ry);
        // printf("mu_F: %.3f\tmu_R %.3f\ts_F: %.3f\ts_R: %.3f\n", mu_F, mu_F, s_F, s_R);

        if(!std::isnan(s_Fx))
                mu_Fx = -1*(s_Fx/s_F)*mu_F;
        else
                mu_Fx = 0;
        if(!std::isnan(s_Fy))
                mu_Fy = -1*(s_Fy/s_F)*mu_F;
        else
                mu_Fy = 0;
        double mu_Rx;
        double mu_Ry;
        if(!std::isnan(s_Rx))
                mu_Rx = -1*(s_Rx/s_R)*mu_R;
        else
                mu_Rx = 0;
        if(!std::isnan(s_Ry))
                mu_Ry = -1*(s_Ry/s_R)*mu_R;
        else
                mu_Ry = 0;

        // printf("mu's: ( %.3f, %.3f, %.3f, %.3f )\n", mu_Fx, mu_Fy, mu_Rx, mu_Ry);
        double fFz = (LR*M*(9.8) - H*M*9.8*mu_Rx) / (LF+LR+H*(mu_Fx*cos(_sta)-mu_Fy*sin(_sta)-mu_Rx));
        double fRz = M*9.8 - fFz;

        double fFx = mu_Fx * fFz;
        double fFy = mu_Fy * fFz;
        double fRx = mu_Rx * fRz;
        double fRy = mu_Ry * fRz;;


        d_vx = (fFx*cos(_theta+_sta)-fFy*sin(_theta+_sta)+fRx*cos(_theta)-fRy*sin(_theta) )/M;
        d_vy = (fFx*sin(_theta+_sta)+fFy*cos(_theta+_sta)+fRx*sin(_theta)+fRy*cos(_theta) )/M;
        d_thetadot = ((fFy*cos(_sta)+fFx*sin(_sta))*LF - fRy*LR)/IZ;
        d_wf = (_tf-fFx*R)/IF;
        d_wr = (_tf-fRx*R)/IR;
    }
}
