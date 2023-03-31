#include <sbmpo/sbmpo.hpp>
#include <my_project/MyCustomModel.hpp>
#include <iostream>

using namespace my_namespace;

int main(int argc, char ** argv) {

  MyCustomModel model;

  sbmpo::SBMPOParameters params;
  /* Add in parameters here */
  
  sbmpo::SBMPO planner(model, params);
  planner.run();
  
  std::cout << "---- Planner Results ----" << std::endl;
  std::cout << "Iterations: " << planner.iterations() << std::endl;
  std::cout << "Exit code: " << planner.exit_code() << std::endl;
  std::cout << "Computation Time: " << planner.time_us() << "us" << std::endl;
  std::cout << "Path cost: " << planner.cost() << std::endl;
  std::cout << "Number of nodes: " << planner.size() << std::endl;
  
  std::cout << "-- State Path --" << std::endl;
  for (sbmpo::State state : planner.state_path()) {
    std::cout << "  - [ ";
    for (float s : state) {
      std::cout << s << " ";
    }
    std::cout << "]" << std::endl;
  }
  
  std::cout << "-- Control Path --" << std::endl;
  for (sbmpo::Control control : planner.control_path()) {
    std::cout << "  - [ ";
    for (float c : control) {
      std::cout << c << " ";
    }
    std::cout << "]" << std::endl;
  }
  
  return 0;
}