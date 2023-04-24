#ifndef SBMPO_SEA_GLIDER_HPP_
#define SBMPO_SEA_GLIDER_HPP_

#include <sbmpo/model.hpp>

namespace sea_glider {

using namespace sbmpo;

class SeaGliderModel : public Model {

    public:

    // States of the Model
    enum States {X, Y, Q, Vx, Vy, dQdt, SG, SoC, PCMT, t};

    // Controls of the Model
    enum Controls {dSGdt};

    // Constructor
    SeaGliderModel() {
        integration_size_ = 5;

        gravity_ = -9.81f;
        mass_ = 10.0f;
        moment_ = 5.0f;
        rotational_damping_ = 10.0f;
        body_area_ = 0.25f;
        wing_area_ = 1.0f;
        water_density_ = 1000.0f;
        cop_length_ = -0.1f;

        dive_speed_ = -1.0;
        float_speed_ = 1.0;

        desired_depth_ = -5.0f;
        resurface_time_ = 10.0f;
        surface_threshold_ = 0.1f;
        min_energy_ = 0.0f;
        min_depth_ = -15.0f;
        max_depth_ = 0.0f;
        min_specific_gravity_ = 0.45;
        max_specific_gravity_ = 1.55;
    }

    // Evaluate a node with a control
    virtual State next_state(const State &state, const Control& control, const float time_span) {

        /*
            Dynamics of the system
            How does each state change with respect to the controls?
        */

        State next_state = state;

        const float integration_time = time_span / integration_size_;
        for (size_t i = 0; i < integration_size_; i++) {

            next_state[SG] += control[dSGdt] * integration_time;

            const float vx = next_state[Vx];
            const float vy = next_state[Vy];
            const float theta = next_state[Q];

            const float v2 = vx*vx + vy*vy;
            const float theta_v = atan2f(vy, vx);
            const float phi = theta - theta_v;

            const float F_drag_wx = -0.5f*water_density_*body_area_*v2*drag_coefficient_(phi);
            const float F_lift_sy = 0.5f*water_density_*wing_area_*v2*lift_coefficient_(phi);
            const float F_bouy_ny = mass_*gravity_*(next_state[SG] - 1);

            const float F_drag_nx = F_drag_wx * cosf(theta_v);
            const float F_drag_ny = F_drag_wx * sinf(theta_v);

            const float F_lift_nx = F_lift_sy * -sinf(theta);
            const float F_lift_ny = F_lift_sy * cosf(theta);

            const float cop_nx = cop_length_ * cosf(theta);
            const float cop_ny = cop_length_ * sinf(theta);

            const float M_pres = cop_nx*F_drag_ny - cop_ny*F_drag_nx;

            const float Ax = (F_drag_nx + F_lift_nx) / mass_;
            const float Ay = (F_drag_ny + F_lift_ny + F_bouy_ny) / mass_;
            const float Mz = (M_pres - rotational_damping_*next_state[dQdt]) / moment_;

            next_state[X] += next_state[Vx] * integration_time;
            next_state[Y] += next_state[Vy] * integration_time;
            next_state[Q] += next_state[dQdt] * integration_time;
            next_state[Vx] += Ax * integration_time;
            next_state[Vy] += Ay * integration_time;
            next_state[dQdt] += Mz * integration_time;
            // TODO: State of Charge, PCM Temp
            next_state[t] += integration_time;
        }

        return next_state;

    }

    // Get the cost of a control
    virtual float cost(const State& state, const Control& control, const float time_span) {

        // Maximize time at desired depth

        float diffY = std::abs(desired_depth_ - state[Y]);

        return diffY * time_span;
    }

    // Get the heuristic of a state
    virtual float heuristic(const State& state, const State& goal) {

        /*
            Heuristic of a state with respect to the goal
            Leads the planner to the goal
            What is the lowest cost possible from this state to the goal?
        */

        const float Y0 = state[Y] - desired_depth_;
        const float YS = -desired_depth_;
        const float T = resurface_time_ - state[t];

        if (T < 0)
            return Y0; // TODO

        const float t_intersect = ((YS - Y0) - float_speed_*T) / (dive_speed_ - float_speed_);
        const float t_dive = -Y0 / dive_speed_;
        const float t_float = -YS / float_speed_ + T;
        
        const float t_dive_int = t_dive < t_intersect ? t_dive : t_intersect;
        const float t_float_int = t_float > t_intersect ? t_float : t_intersect;

        const float A_dive = 0.5f*dive_speed_*t_dive_int*t_dive_int + Y0*t_dive_int;
        const float A_float = -0.5f*float_speed_*T*T - 0.5f*float_speed_*t_float_int*t_float_int + float_speed_*T*t_float_int + YS*T - YS*t_float_int;

        const float A = A_dive + A_float;

        return A;
    }

    // Determine if node is valid
    virtual bool is_valid(const State& state) {

        if (state[SoC] < min_energy_)
            return false;

        if (state[Y] < min_depth_)
            return false;
        
        if (state[Y] > max_depth_)
            return false;

        if (state[SG] < min_specific_gravity_)
            return false;
        
        if (state[SG] > max_specific_gravity_)
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

    size_t integration_size_;

    // glider params
    float gravity_;
    float mass_;
    float moment_;
    float rotational_damping_;
    float body_area_;
    float wing_area_;
    float water_density_;
    float cop_length_;
    float dive_speed_;
    float float_speed_;

    // plan params
    float desired_depth_;
    float resurface_time_;

    // constraints
    float surface_threshold_;
    float min_energy_;
    float min_depth_;
    float max_depth_;
    float min_specific_gravity_;
    float max_specific_gravity_;

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

    float drag_coefficient_(const float phi) {
        const float CD0 = 0.01;
        const float KD = 0.05;
        return CD0 + std::abs(KD*sinf(phi));
    }

    float lift_coefficient_(const float phi) {
        const float KL = 0.25;
        return KL*sinf(2*phi);
    }


};

}

#endif