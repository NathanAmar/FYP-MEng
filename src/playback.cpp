#include <csignal>

#include <vector>
#include <map>
#include <assert.h>
#include <math.h>       /* round, floor, ceil, trunc */

#include "params_adaptation.hpp"
#include "tools.hpp"
#include "a1_unidirectional.hpp"

#include <boost/program_options.hpp>


#include "global_params.hpp"

using namespace A1_unidirectional;

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
  arg.indexes = vm["index"].as<std::vector<size_t>>();
  arg.name_video = vm["video"].as<std::string>();
  arg.damages = vm["damages"].as<std::vector<std::string>>();
  // arg.damages_type = vm["types"].as<std::vector<std::string>>();
}

int main(int argc, char** argv)
{
  // Install a signal handler
  std::signal(SIGINT, signal_handler);
  std::signal(SIGSEGV, signal_handler);

  global_params::ctrl_size = 48;
#if defined(D_8) 
    global_params::bd_size = 8; // Change also in the experiment Struct Eval the in dimensions 
#else 
    global_params::bd_size = 4;
#endif

  boost::program_options::options_description desc;
  desc.add_options()("load", boost::program_options::value<std::string>(), "Set path of file to load");
  desc.add_options()("index", boost::program_options::value<std::vector<size_t>>()->multitoken(), "List of Indexes of behaviour to save in that file");
  desc.add_options()("video", boost::program_options::value<std::string>()->default_value("video"), "Name video to save without the mp4 extension");
  desc.add_options()("damages", boost::program_options::value<std::vector<std::string>>()->multitoken()->default_value(std::vector<std::string>{""}, ""), "Indicate which joints are damages");
  // desc.add_options()("types", boost::program_options::value<std::vector<std::string>>()->multitoken()->default_value(std::vector<std::string>{"blocked_joint"}, ""), "Indicate how the joints are damaged");

  Arguments arg{};
  get_arguments(desc, arg, argc, argv);

  std::vector< std::vector<float> > ctrls;
  ctrls = load_ctrl_file(arg.path_load_file);
  for (int i=0; i<arg.indexes.size(); i++){
    if (arg.indexes[i] < 0 || arg.indexes[i] > ctrls.size()){
      std::cout << "Wrong index" << std::endl; 
      return 0; 
    } 
  }

  
  
  std::vector< std::vector<float> > ctrls_indexes;
  for (int i=0; i<arg.indexes.size(); i++) {
    ctrls_indexes.push_back(ctrls[arg.indexes[i]]); 
  }
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

  for (const auto & ctrl: ctrls_indexes)
  {
    assert(ctrl.size() == 48);
    lecture(ctrl, arg.name_video);
  }

  std::cout<<"finished"<<std::endl;
  
  return 0;
}


