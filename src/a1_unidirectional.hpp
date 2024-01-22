#ifndef ___AIRL_TASKS_A1_UNIDIRECTIONAL_HPP__
#define ___AIRL_TASKS_A1_UNIDIRECTIONAL_HPP__

#ifdef EVO
#include <sferes/gen/evo_float.hpp>
#include <sferes/phen/parameters.hpp>
#include <sferes/fit/fit_qd.hpp>
#endif 

#include <robot_dart/robot_dart_simu.hpp>
#include <robot_dart/robot_pool.hpp>

#ifdef GRAPHIC
#include <robot_dart/gui/magnum/graphics.hpp>
#include <robot_dart/gui/magnum/sensor/camera.hpp>
#endif

#if defined(WINDOWLESS) || defined(VIDEO)
#include <robot_dart/gui/magnum/windowless_graphics.hpp>
#include <robot_dart/gui/magnum/sensor/camera.hpp>
#endif

#include <chrono>
#include <ctime>  

#include <dart/collision/bullet/BulletCollisionDetector.hpp>
#include <dart/constraint/ConstraintSolver.hpp>

#include "a1_control.hpp"

#if defined FOOT_CALF
#include "BD/desc_a1_foot_calf.hpp"
#elif defined FOOT_CALF_KNEE
#include "BD/desc_a1_foot_calf_knee.hpp"
#elif defined FOOT_CALF_THIGH
#include "BD/desc_a1_foot_calf_thigh.hpp"
#else
#include "BD/desc_a1_foot.hpp"
#endif

#ifdef ADAPT 
#include "params_adaptation.hpp"
#endif 

#include <unistd.h>
#include <ios>
#include <iostream>
#include <fstream>
#include <string>

#ifdef EVO
using namespace sferes::gen::evo_float;
#endif

namespace A1_unidirectional {
//initialising a global variable (but using a namespace) - this global variable is the robot object

namespace global{
    std::shared_ptr<robot_dart::Robot> global_robot; //initialize a shared pointer to the object Robot from robotdart and name it global_robot
    std::vector<std::string> dofs;
    std::vector<std::string> brokenLegs;
    std::vector<std::string> damageTypes;
    int ctrl_size; 
    int bd_size; 
    std::vector<robot_dart::RobotDamage> damages;
    int recovered_behaviour; 
    int number_of_iterations; 
    double fit_in_archive; 
}

void signal_handler(int signal)
{
      exit(signal);
}


void set_damages(const std::vector<robot_dart::RobotDamage>& damages)
{
    for (auto dmg : damages) {
        if (dmg.type == "blocked_joint") {
            global::global_robot->set_actuator_type("locked", dmg.data);
        }
        else if (dmg.type == "free_joint") {
          global::global_robot->set_actuator_type("passive", dmg.data);
          // global::global_robot->skeleton()->getJoint(dmg.data)->setActuatorType(dart::dynamics::Joint::PASSIVE);
        }
    }
}

namespace pool {
   std::shared_ptr<robot_dart::Robot> robot_creator()
   {
  
     auto robot = std::make_shared<robot_dart::Robot>("exp/fyp-a-1-unidirectional-adaptation/singularity/resources/unitree_a1/a1.urdf");
     robot->set_position_enforced(true);
     robot->set_actuator_types("servo");
     robot->set_color_mode("material");
     //robot->skeleton()->enableSelfCollisionCheck(); //enable SelfCollision , can take longer to simulate! (see https://dartsim.github.io/tutorials_biped.html)
     return robot;
   }
#if defined(GRAPHIC) || defined(VIDEO) || defined(WINDOWLESS) || defined(ADAPT)
   robot_dart::RobotPool robot_pool(robot_creator, 1);
#else
   robot_dart::RobotPool robot_pool(robot_creator, NUM_THREADS);
#endif
}


// ##################################################################################
// ##################################  EVO BLOCK ####################################
// ##################################################################################
#ifdef EVO
FIT_QD(A1_unidirectional)
{
private:
    std::vector<Eigen::VectorXd> _traj;
    
public:
    A1_unidirectional(){  }
    typedef Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic, Eigen::RowMajor > Mat;

    template<typename Indiv>

    void eval(Indiv& ind)
    {
        float value; 
        std::vector<double> bd; 
        bool isDead = false; 


        std::tie(value, bd, isDead) = simulate(ind.data());

        if (bd.back() > 0){
            isDead = true;
        }

        int bd_dim = 4; 
        #if defined D_3_8
                bd_dim = 8;
        #endif
        std::vector<double> results2(bd.begin(), bd.begin() + bd_dim);
        
        this->_value = value; 
        this->_dead = isDead;
        this->set_desc(results2);
        
    }
#endif 
// ##################################################################################
// ##################################  ADAPTATION BLOCK #############################
// ##################################################################################
#ifdef ADAPT
struct Eval {
#if defined(D_8) 
    BO_PARAM(size_t, dim_in, 8);
#else 
    BO_PARAM(size_t, dim_in, 4);
#endif

    BO_PARAM(size_t, dim_out, 1);
    // the function to be optimized
    Eigen::VectorXd operator()(const Eigen::VectorXd& x) const
    {

        std::vector<double> key(x.size(), 0);
        Eigen::VectorXd::Map(key.data(), key.size()) = x; 
        
        global::recovered_behaviour = Params::archiveparams::archive.at(key).index;
        std::vector<float> ctrl = Params::archiveparams::archive.at(key).controller;   
        
        global::fit_in_archive = Params::archiveparams::archive.at(key).fit;
        std::cout<< "Index: " << global::recovered_behaviour << " fit in archive: "<< global::fit_in_archive <<std::endl;
        global::number_of_iterations += 1;
        return tools::make_vector(eval(ctrl));
    }
    
    float eval(const std::vector<float>& ctrl_vec, bool ask_measure = true, std::string video_name = "video") const
    {
        bool ok_flag=false;
        float distance = 0;
        std::vector<double> bd; 
        bool isDead = false;  
        // while(!ok_flag){
          std::tie(distance, bd, isDead) = simulate(ctrl_vec, video_name, global::damages);
          // int input = 1;

          // // while (input != 1 && input != 0 )
          // // {
          // //   std::cin.clear();
          // //   std::cout << "eval ok (1 or 0)? ";
          // //   std::cin >> input;
          // // }
          
          // if(input)
        //   ok_flag=true;
          // else
          //   ok_flag=false;
          // std::cout<<" You said: "<<ok_flag<<std::endl;
        //   }
      return distance;
  }


#endif

// ##################################################################################
// ##################################  COMMON BLOCK #################################
// ##################################################################################
    std::tuple<float, std::vector<double>, bool > simulate (const std::vector<float>& indiv, 
                                                            const std::string& name_video = "/git/sferes2/exp/robotdart-example/singularity/video.mp4",
                                                            const std::vector<robot_dart::RobotDamage>& damages = {}, 
                                                            bool record=true
                                                            ) const
    {
        #ifdef WINDOWLESS
            // Windowless requires to set the contexts for magnum before
            Corrade::Utility::Error magnum_silence_error{nullptr};
            get_gl_context_with_sleep(gl_context, 20);
        #endif
                Eigen::VectorXd ctrl(48);
                for(size_t i=0;i<indiv.size();i++)
                {
                    ctrl[i] = round( std::min(1.0f, std::max(0.0f,indiv[i])) * 1000.0 ) / 1000.0;// limite numerical issues
                }
                auto g_robot = pool::robot_pool.get_robot();
                //auto g_robot = global::global_robot->clone(); 

                auto names = g_robot->dof_names(true, true, true);
                names = std::vector<std::string>(names.begin() + 6, names.end());
                g_robot->set_positions(robot_dart::make_vector({0.0, 0.67, -1.3, -0.0, 0.67, -1.3, 0.0, 0.67, -1.3, -0.0, 0.67, -1.3}), names);
                
                g_robot->set_base_pose(robot_dart::make_vector({0., 0., 0., 0., 0., 0.5}));

                for (auto dmg : damages) {
                    if (dmg.type == "blocked_joint") {
                        g_robot->set_actuator_type("locked", dmg.data);
                    }
                    else if (dmg.type == "free_joint") {
                        g_robot->set_actuator_type("passive", dmg.data);
                    }
                }

                double ctrl_dt = 0.02;
                double gain = 0.05;
                g_robot->add_controller(std::make_shared<robot_dart::control::A1Control>(ctrl_dt, ctrl, names));
                std::static_pointer_cast<robot_dart::control::A1Control>(g_robot->controllers()[0])->set_h_params(Eigen::VectorXd::Constant(1, gain));
                std::static_pointer_cast<robot_dart::control::A1Control>(g_robot->controllers()[0])->activate(true);
                robot_dart::RobotDARTSimu simu(ctrl_dt);
                simu.set_collision_detector("fcl");
        #ifdef GRAPHIC
                auto graphics = std::make_shared<robot_dart::gui::magnum::Graphics>();
                simu.set_graphics(graphics);
        #endif
        #ifdef WINDOWLESS

            robot_dart::gui::magnum::GraphicsConfiguration graphics_configuration = robot_dart::gui::magnum::WindowlessGraphics::default_configuration();
            // WARNING: the rendering may use more than 1 CPU,
            // so if you increase the width and the height, you may need to reduce your total number of threads
            graphics_configuration.width = 128;
            graphics_configuration.height = 128;

            auto graphics = std::make_shared<robot_dart::gui::magnum::WindowlessGraphics>(graphics_configuration);
            simu.set_graphics(graphics);
            graphics->look_at({0.5, 3., 0.75}, {0.5, 0., 0.2});

            // WARNING: If you can, avoid enabling the windowless graphics during the simulation to avoid computationally expensive operations!
            graphics->set_enable(false);
        #endif
        #ifdef VIDEO
            if (record){
                robot_dart::gui::magnum::GraphicsConfiguration graphics_configuration = robot_dart::gui::magnum::WindowlessGraphics::default_configuration();
                graphics_configuration.bg_color = Eigen::Vector4d{0.52941, 0.8078433, 0.9215686, 1.}; // using sky blue as background color
                auto graphics = std::make_shared<robot_dart::gui::magnum::WindowlessGraphics>(graphics_configuration);
                simu.set_graphics(graphics);
                constexpr int c_fps = 30;
                graphics->record_video(name_video + ".mp4", c_fps);
                graphics->look_at({0.5, 5., 0.75}, {1.5, 0., 0.2});
            }
        #endif


            simu.world()->getConstraintSolver()->setCollisionDetector(dart::collision::BulletCollisionDetector::create());
            simu.add_floor(40, 0.1);
            simu.add_robot(g_robot);
            
            //stabilising the robot for 0.5 second at the 0 pos
            simu.run(0.2);
            simu.world()->setTime(0);
            //run the controller to go to the t=0 pos and stabilise for 0.5 second
            std::static_pointer_cast<robot_dart::control::A1Control>(g_robot->controllers()[0])->activate(false);
            simu.run(0.5);
            
            simu.add_descriptor(std::make_shared<robot_dart::descriptor::DutyCycle>(robot_dart::descriptor::DutyCycle()));
            
            std::static_pointer_cast<robot_dart::control::A1Control>(g_robot->controllers()[0])->set_h_params(Eigen::VectorXd::Constant(1, gain));
            // run the controller for 5 seconds
            simu.world()->setTime(0);
            double time = 10.0;
            std::static_pointer_cast<robot_dart::control::A1Control>(g_robot->controllers()[0])->activate(true);
            simu.run(time);

        #ifdef WINDOWLESS
            // Example of Image saving using WindowlessGraphics
            graphics->set_enable(true);
            graphics->refresh();
            auto image = graphics->image();

            // Clearing magnum contexts
            release_gl_context(gl_context);
        #endif
        #ifdef VIDEO
            // Only save one video and exit the process
            std::cout << "Saved video: " << name_video << ".mp4" << std::endl;
        #endif


                std::vector<double> results;
                
                
                // stops the controller and stabilize for 0.5 seconds.
                std::static_pointer_cast<robot_dart::control::A1Control>(g_robot->controllers()[0])->activate(false);
                simu.world()->setTime(0);
                simu.run(0.5);

                std::static_pointer_cast<robot_dart::descriptor::DutyCycle>(simu.descriptor(0))->get(results);
                // To-DO Make sure the BD has the right dimension 


                auto pos=simu.robots().back()->skeleton()->getPositions().head(6).tail(3);
                bool isDead = false; 
                if ( abs(pos[1]) > 1){
                    isDead = true; 
                }

                // g_robot.reset(); 

                g_robot->set_actuator_types("servo");
                simu.remove_robot(g_robot);
                //global::global_robot.reset(); 
                // double vm, rss;
                // mem_usage(vm, rss);
                // std::cout << "Virtual Memory: " << vm << "\nResident set size: " << rss << std::endl;

                pool::robot_pool.free_robot(g_robot);
                
                return {pos[0], results, isDead};
}


};

#ifdef ADAPT
void lecture(const std::vector<float>& ctrl, std::string video_name = "video")
{
  Eval eval;
  eval.eval(ctrl, false, video_name);
  return;
}
#endif


#ifdef EVO
// Definition of the Genotype, Phenotype and Fitness object according to a provided Param Object.
template<typename Params>
using fit_t = A1_unidirectional<Params>;
template<typename Params>
using gen_t = sferes::gen::EvoFloat<48, Params>;
template<typename Params>
using phen_t = sferes::phen::Parameters<gen_t<Params>, fit_t<Params>, Params>;
#endif

}

#endif
