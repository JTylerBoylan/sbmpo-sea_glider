# SBMPO Getting Started

CMake project template for [Sample Based Model Predictive Optimization (SBMPO) for robot trajectory planning](https://github.com/JTylerBoylan/sbmpo).

## Dependencies
- [CMake 3.5](https://cmake.org/install/) or later
- [GCC 6.1](https://gcc.gnu.org/) or later

## Installation
To begin, clone this workspace to your directory of choice and pull down the sbmpo submodule using the following commands:
```
git clone https://github.com/JTylerBoylan/sbmpo-getting-started my_project_ws
cd my_project_ws
git submodule update --init
```

## Creating a custom model
To plan trajectories using the SBMPO algorithm, you must first define a model for your robot.  To do this, you can use the template defined in [`my_project/include/my_project/MyCustomModel.hpp`](https://github.com/JTylerBoylan/sbmpo-getting-started/blob/main/my_project/include/my_project/MyCustomModel.hpp).

The model class comprises of 5 main functions that you'll need to edit:
- `next_state`: Determines a new state given the previous state, the control, and the time span using your robot kinematics.

- `cost`: Determines the cost of a point given its state and control, for which SBMPO will try to minimize.

- `heuristic`: Determines a heuristic of a point given its state and the goal state, which SBMPO will use to lead its search.

- `is_valid`: Determines if a point is valid given its state, which defines the bounds of the search space.

- `is_goal`: Determines if a point can be considered the goal, which will end the search when found true.

You can also define a constructor for your model that can be used to initialize parameters values, and setter functions to change the parameters.

Enums for States and Controls are included that can be used to track the indices of your State and Control vectors.

## Creating an executable
Once your model is defined, you can start using SBMPO to plan trajectories. To create an executable that uses SBMPO, you can use the template defined in [`my_project/src/main.cpp`](https://github.com/JTylerBoylan/sbmpo-getting-started/blob/main/my_project/src/main.cpp).

In your executable, you will have to set the SBMPO parameters for the run:
| Name | Description | Type |
| ---- | ----------- | ---- |
| `max_iterations` | Maximum branchout iterations | `int` |
| `max_generations` | Maximum branchout generations | `int` |
| `sample_time` | Time period per branchout | `float` |
| `grid_resolution` | Grid resolutions | `std::vector<float>` |
| `start_state` | Initial state of plan | `sbmpo::State` |
| `goal_state` | Goal state of plan | `sbmpo::State` |
| `samples` | List of controls to be sampled in a branchout | `std::vector<sbmpo::Control>` |


The template will then find the optimal path to the goal for your model and print out the results to the terminal.

## Building the project
The project comes with custom CMakeLists.txt files that will generate the CMake files for the project.

*Note that if you rename your project files, you will have to manually edit the CMakeLists.txt files in both the workspace and your project folder to match the changes.*

In the project workspace (*Default:* `my_project_ws`):
```
mkdir build && cd build
cmake .. && make
```

After building once, you only have to run the command `make` in the `build` folder to compile changes in the source code. However, if you make changes to the CMakeLists.txt files or make changes to the file structure, you will have to delete, then re-make the build folder.

## Running the executable
After the project has successfully been compiled, you can then run your executable using the following commands:
```
cd build/my_project
./my_executable
```
If you get segmentation faults during your run, you might want to check that your parameters are set correctly and that your start and goal states are the correct size.

## Using SBMPO Models

If you want to use one of the models defined in the `sbmpo_models` package as a template for your model, you can do that simply by setting it as the base class for your custom model. The template models are set up so the functions can be overriden by a child class. This is useful if you only need to make minor changes to a template model.

For example, if you wanted to make a custom double integrator model that has bounds on the maximum velocity, it could be done as follows:
```
#ifndef MY_CUSTOM_DOUBLE_INTEGRATOR_HPP_
#define MY_CUSTOM_DOUBLE_INTEGRATOR_HPP_

#include <sbmpo_models/DoubleIntegrator.hpp>

namespace my_namespace {

using namespace sbmpo_models;
using namespace sbmpo;

class MyCustomDoubleIntegratorModel : public DoubleIntegratorModel {

  public:
  
  // Custom constructor
  MyCustomDoubleIntegratorModel() {
    v_lim_ = 10.0f; // Limit velocity to 10
  }
  
  // Overriding the is_valid function
  bool is_valid(const State& state) override {
    return std::abs(state[V]) < v_lim_;
  }
  
  // Setter method
  void set_velocity_limit(float v_lim) {
    v_lim_ = v_lim;
  }
  
  private:
  
  float v_lim_;

};

}

#endif
```

## Using SBMPO Benchmarking

You can use the `sbmpo_benchmarking` package to compare different SBMPO parameters on your model. This is useful in determining optimal parameters, such as grid resolution, sample time, and/or branchout factors. 

You can access this package by switching sbmpo to the `benchmarking` branch, using the following commands:
```
cd my_project_ws/sbmpo
git checkout benchmarking
```

You can create benchmarking configuration files for your model using the sbmpo_config MATLAB function found in [`sbmpo/sbmpo_benchmarking/matlab`](https://github.com/JTylerBoylan/sbmpo/tree/benchmarking/sbmpo_benchmarking/matlab) by passing in a csv folder, SBMPO params, and number of runs for the parameter set.

To run the benchmarker, include the basic benchmarking class in your source, create a benchmarking object with the path to the folder containing your csv config file, and then run the benchmarker. This will look something like this:
```
#include <my_project/MyCustomModel.hpp>
#include <sbmpo_benchmarking/benchmark.hpp>

int main (int argc, char ** argv) {

    // Path to csv workspace
    std::string csv_folder = "/path/to/my_project_ws/my_project/csv/";

    // Create instance of your model
    my_namespace::MyCustomModel myModel;

    // Create new benchmarker
    sbmpo_benchmarking::Benchmark benchmarker(csv_folder);

    // Run benchmark on the model (saves results to csv_folder)
    benchmarker.benchmark(myModel);

    return 0;
}
```

Other benchmarkers available in the `sbmpo_benchmarking` package, such as the [`Obstacles2D`](https://github.com/JTylerBoylan/sbmpo/blob/benchmarking/sbmpo_benchmarking/include/sbmpo_benchmarking/benchmarks/Obstacles2D.hpp) benchmarker that can be used to compare plans around various sets of obstacles in 2D space.

More information can be found in the [SBMPO benchmarking branch](https://github.com/JTylerBoylan/sbmpo/tree/benchmarking).
