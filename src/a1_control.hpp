#ifndef ROBOT_DART_CONTROL_A1_CONTROL
#define ROBOT_DART_CONTROL_A1_CONTROL

#include "a1_controller_simple.hpp"

#include <robot_dart/control/policy_control.hpp>

namespace robot_dart {
namespace control {

struct A1Policy {
public:
    void set_params(const Eigen::VectorXd& ctrl)
    {
        
        _controller.set_parameters(ctrl);
    }
    
    
    size_t output_size() const { return 12; }
    
    Eigen::VectorXd query(const std::shared_ptr<robot_dart::Robot>& robot, double t)
    {
        
        if (!_h_params_set) {
            _dt = robot->skeleton()->getTimeStep();
        }
        
        auto angles = _controller.pos(t);
        Eigen::VectorXd target_positions = Eigen::VectorXd::Zero(18);
        for (size_t i = 0; i < angles.size(); i++)
            target_positions(i+6) = angles[i];
        
        
        Eigen::VectorXd q = robot->skeleton()->getPositions();
        Eigen::VectorXd q_err = target_positions - q;
        
        double gain = 1.0 / (dart::math::constants<double>::pi() * _dt);
        Eigen::VectorXd vel = q_err * gain;
       
        return vel.tail(12);
    }
    
    void set_h_params(const Eigen::VectorXd& h_params)
    {
        _dt = h_params[0];
        _h_params_set = true;
    }
    
    Eigen::VectorXd h_params() const
    {
        return Eigen::VectorXd::Constant(1, _dt);
    }
    
protected:
    A1_controller::A1ControllerSimple _controller;
    double _dt;
    bool _h_params_set = false;
};

using A1Control = robot_dart::control::PolicyControl<A1Policy>;
} // namespace control
} // namespace robot_dart

#endif

