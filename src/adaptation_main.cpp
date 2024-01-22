#include <csignal>

#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <map>
#include <assert.h>
#include <stdio.h>      /* printf */
#include <math.h>  

#include <limbo/limbo.hpp>
#include <limbo/stop/max_predicted_value.hpp>

#include <exhaustive_search_archive.hpp>
#include <binary_map.hpp>
#include <mean_archive.hpp>

#include "params_adaptation.hpp"

#include "a1_unidirectional.hpp"

#include "tools.hpp"

#include "global_params.hpp"

#include <boost/program_options.hpp>

using namespace limbo;
using namespace A1_unidirectional; 

Params::archiveparams::archive_t Params::archiveparams::archive;
BO_DECLARE_DYN_PARAM(int, Params::stop_maxiterations, iterations);
// BO_DECLARE_DYN_PARAM(double, Params::stop_maxpredictedvalue, ratio);

struct Arguments {
        std::string path_load_file;
        std::vector <size_t> indexes;
        std::string name_video;
        std::vector<std::string> damages;
        // std::vector<std::string> damages_type; 
    };

void get_arguments(const boost::program_options::options_description &desc, Arguments &arg, int argc, char **argv) {
  // For the moment, only returning number of threads
  boost::program_options::variables_map vm;
  boost::program_options::parsed_options parsed = boost::program_options::command_line_parser(argc, argv).options(
          desc).allow_unregistered().run();

  boost::program_options::store(parsed, vm);
  boost::program_options::notify(vm);
  arg.path_load_file = vm["load"].as<std::string>();
//   arg.indexes = vm["index"].as<std::vector<size_t>>();
  arg.name_video = vm["video"].as<std::string>();
  arg.damages = vm["damages"].as<std::vector<std::string>>();
//   arg.damages_type = vm["types"].as<std::vector<std::string>>();
}


void write_results(const std::string& filename, std::vector<std::string> damages, int number_of_iterations, int recovered_behaviour, double fit_in_archive, double val, Eigen::VectorXd bd, float best_indiv_val){
  std::ofstream MyFile;

  // Write to the file
  MyFile.open(filename.c_str(), std::ios_base::app);
  
  for (int i=0; i<damages.size(); i++){
    MyFile << damages[i] << " "; 
  } 
  MyFile << "\t" << number_of_iterations << "\t" 
                 << global_params::best_fit_index << "\t"
                 << global_params::best_fit << "\t"
                 << best_indiv_val << "\t"
                 << recovered_behaviour << "\t"
                 << fit_in_archive << "\t" 
                 << val << "\t" 
                 << bd.transpose(); 
  MyFile << std::endl;
  
  // Close the file
  MyFile.close();
}

int main(int argc, char** argv)
{
  // Install a signal handler
    std::signal(SIGINT, signal_handler);
    std::signal(SIGSEGV, signal_handler);


    boost::program_options::options_description desc;
    desc.add_options()("load", boost::program_options::value<std::string>(), "Set path of file to load");
    // desc.add_options()("index", boost::program_options::value<std::vector<size_t>>()->multitoken(), "List of Indexes of behaviour to save in that file");
    desc.add_options()("video", boost::program_options::value<std::string>()->default_value("video"), "Name video to save without the mp4 extension");
    desc.add_options()("damages", boost::program_options::value<std::vector<std::string>>()->multitoken()->default_value(std::vector<std::string>{""}, ""), "Indicate which joints are damages");
    // desc.add_options()("types", boost::program_options::value<std::vector<std::string>>()->multitoken()->default_value(std::vector<std::string>{"blocked_joint"}, "blocked_joint"), "Indicate how the joints are damaged");


    Arguments arg{};
    get_arguments(desc, arg, argc, argv);
    global::number_of_iterations = 0;

    global::fit_in_archive = -1.0;
    global::recovered_behaviour = -1;
    global_params::ctrl_size = 48;
    global_params::best_fit = -1; 
    global_params::best_fit_index = -1;
#if defined(D_8) 
    global_params::bd_size = 8; // Change also in the experiment Struct Eval the in dimensions 
#else 
    global_params::bd_size = 4;
#endif
    global::brokenLegs = {}; 
    global::damageTypes = {};
    if (arg.damages.size() > 1){
      for (int i=0; i<arg.damages.size(); i+=2){
          global::brokenLegs.push_back(arg.damages[i]); 
          global::damageTypes.push_back(arg.damages[i+1]); 
      }
    } 
    global::damages = {}; 
    if (global::brokenLegs.size() > 0){
      for (size_t i = 0; i < global::brokenLegs.size(); ++i){
          global::damages.push_back(robot_dart::RobotDamage(global::damageTypes[i], global::brokenLegs[i]));
      }
    }
    // init_simu(global::brokenLegs, global::damageTypes);

    Params::archiveparams::archive = load_archive(arg.path_load_file);

    Params::stop_maxiterations::set_iterations(100);

    typedef kernel::MaternFiveHalves<Params> Kernel_t;
    typedef opt::ExhaustiveSearchArchive<Params> InnerOpt_t;

    // typedef boost::fusion::vector<stop::MaxIterations<Params>> Stop_t;
    // typedef boost::fusion::vector<stop::MaxIterations<Params>, Params::stop_maxpredictedvalue > Stop_t;
    using Stop_t = boost::fusion::vector<stop::MaxPredictedValue<Params>>;

    typedef mean::MeanArchive<Params> Mean_t;
    typedef boost::fusion::vector<stat::Samples<Params>, stat::BestObservations<Params>, stat::ConsoleSummary<Params>> Stat_t;
    
    typedef init::NoInit<Params> Init_t;
    typedef model::GP<Params, Kernel_t, Mean_t> GP_t;
    typedef acqui::UCB<Params, GP_t> Acqui_t;


    bayes_opt::BOptimizer<Params, modelfun<GP_t>, initfun<Init_t>, acquifun<Acqui_t>, stopcrit<Stop_t>, acquiopt<InnerOpt_t>, statsfun<Stat_t>> opt;
    // bayes_opt::BOptimizer<Params, modelfun<GP_t>, initfun<Init_t>, acquifun<Acqui_t>, acquiopt<InnerOpt_t>, statsfun<Stat_t>> opt;

    opt.optimize(Eval());


    auto val = opt.best_observation();
    Eigen::VectorXd result = opt.best_sample().transpose();



    std::cout << "Fitness: " << val << "\t Recovered Behaviour BD:  " << result.transpose() << std::endl;

    float best_perfoming_indiv_val = Eval().eval(global_params::best_fit_controller);
    std::cout << "Fitness of unrecovered Solution: " << best_perfoming_indiv_val << std::endl;  
    write_results("Results", arg.damages, global::number_of_iterations, global::recovered_behaviour, global::fit_in_archive, val[0], result, best_perfoming_indiv_val);

    return global::recovered_behaviour;
}
