#ifndef SBMPO_SEA_GLIDER_HPP_
#define SBMPO_SEA_GLIDER_HPP_

#include <sbmpo/model.hpp>

namespace sea_glider {

using namespace sbmpo;

class SeaGliderModel : public Model {

    public:

    // States of the Model
    enum States {X, Y, Q, Vx, Vy, dQdt, SoC, SG, PCMT, t};

    // Controls of the Model
    enum Controls {dSGdt};

    // Constructor
    SeaGliderModel() {

    }

    // Evaluate a node with a control
    virtual State next_state(const State &state, const Control& control, const float time_span) {
        
        State next_state = state;

        /*
            Dynamics of the system
            How does each state change with respect to the controls?
        */

       return next_state;

    }

    // Get the cost of a control
    virtual float cost(const State& state, const Control& control, const float time_span) {

        // Maximize time at desired depth

        float diffY = std::abs(state[Y] - desired_depth_);

        return diffY * time_span;
    }

    // Get the heuristic of a state
    virtual float heuristic(const State& state, const State& goal) {

        /*
            Heuristic of a state with respect to the goal
            Leads the planner to the goal
            What is the lowest cost possible from this state to the goal?
        */

        float diffY = state[Y] - desired_depth_;

        return 0.0f;
    }

    // Determine if node is valid
    virtual bool is_valid(const State& state) {

        if (state[SoC] < min_energy_)
            return false;

        if (state[Y] < min_depth_)
            return false;
        
        if (state[Y] > max_depth_)
            return false;


        return true;
    }

    // Determine if state is goal
    virtual bool is_goal(const State& state, const State& goal) {

        // Plan ends after resurface time reached and the glider is at the surface

        if (state[t] > resurface_time_ && state[Y] > surface_threshold_)
            return true;

        return false;
    }

    // Deconstructor
    virtual ~SeaGliderModel() {}

    protected:

    float gravity_;
    float mass_;
    float moment_;
    float rotational_damping_;
    float body_area_;
    float wing_area_;
    float desired_depth_;
    float resurface_time_;
    float surface_threshold_;
    float min_energy_;
    float min_depth_;
    float max_depth_;


    float power_output_(const float pcm_temp, const float depth) {

        float water_temp = water_temperature_(depth);

        float diffT = pcm_temp - water_temp;

        // Power output

        return 0.0f;
    }

    float water_temperature_(const float depth) {

        // Water temp

        return 0.0f;
    }


};

}

#endif