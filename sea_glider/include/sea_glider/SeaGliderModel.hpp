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

        integration_size_ = 100;

        gravity_ = 9.81f;
        mass_ = 1.26f;
        moment_ = 0.1f;
        rotational_damping_ = 0.25f;
        body_area_ = 0.5f;
        wing_area_ = 1.0f;
        water_density_ = 1000.0f;
        cop_length_ = -0.1f;

        dive_speed_ = -0.155; //-0.03;
        surface_speed_ = 0.145; // 0.03;

        desired_depth_ = -8.0; // -1.0f;
        surface_threshold_ = 0.1f;
        min_energy_ = -std::numeric_limits<float>::infinity();
        min_depth_ = -10.0f; // -3.0f;
        max_depth_ = 0.0f;
        min_specific_gravity_ = 0.952; // 0.988;
        max_specific_gravity_ = 1.048; // 1.012;

        max_time_ = 250;

        P_electronics_ = 0.265f;
        P_friction_ = 2.28f;
        p_control_ = 0.06481f;

    }

    // Evaluate a node with a control
    virtual State next_state(const State &state, const Control& control, const float time_span) {

        // Glider dynamics

        State next_state = state;

        const float integration_time = time_span / integration_size_;
        for (size_t i = 0; i < integration_size_; i++) {

            next_state[SG] += control[dSGdt] * integration_time;

            const float vx = next_state[Vx];
            const float vy = next_state[Vy];
            const float theta = next_state[Q];

            const float v2 = vx*vx + vy*vy;
            const float theta_v = atan2f(vy, vx);
            const float alpha = theta - theta_v;

            const float F_drag_wx = -0.5f*water_density_*body_area_*v2*drag_coefficient_(alpha);
            const float F_lift_wy = 0.5f*water_density_*wing_area_*v2*lift_coefficient_(alpha);
            const float F_bouy_ny = mass_*gravity_*(1.0f - next_state[SG]);

            const float F_drag_nx = F_drag_wx * cosf(theta_v);
            const float F_drag_ny = F_drag_wx * sinf(theta_v);

            const float F_lift_nx = F_lift_wy * -sinf(theta_v);
            const float F_lift_ny = F_lift_wy * cosf(theta_v);

            const float cop_nx = cop_length_ * cosf(theta);
            const float cop_ny = cop_length_ * sinf(theta);

            const float M_pres = cop_nx*(F_drag_ny + F_lift_ny) - cop_ny*(F_drag_nx + F_lift_nx);

            const float Ax = (F_drag_nx + F_lift_nx) / mass_;
            const float Ay = (F_drag_ny + F_lift_ny + F_bouy_ny) / mass_;
            const float Mz = (M_pres - rotational_damping_*next_state[dQdt]) / moment_;

            const float power_depth_ = control[dSGdt] < 0.0f ? p_control_ : 0.0f;
            const float power_nom_ = control[dSGdt] == 0.0f ? P_friction_ + P_electronics_ : P_electronics_;

            next_state[X] += next_state[Vx] * integration_time;
            next_state[Y] += next_state[Vy] * integration_time;
            next_state[Q] += next_state[dQdt] * integration_time;
            next_state[Vx] += Ax * integration_time;
            next_state[Vy] += Ay * integration_time;
            next_state[dQdt] += Mz * integration_time;
            next_state[SoC] += (power_depth_*-state[Y] + power_nom_)*integration_time;
            // TODO: PCM Temp

        }

        next_state[t] += time_span;

        return next_state;

    }

    // Get the cost of a control
    virtual float cost(const State& state, const Control& control, const float time_span) {

        // Maximize time at desired depth
        float diffY = std::abs(desired_depth_ - state[Y]);

        return diffY*time_span; // + 5.0f*control[0]*control[0];
    }

    // Get the heuristic of a state
    virtual float heuristic(const State& state, const State& goal) {

        // Transforms
        const float Y0 = state[Y] - desired_depth_;
        const float YS = -desired_depth_;
        const float T = goal[t] - state[t];

        // Cases
        if (YS - Y0 > surface_speed_*T) {
            if (Y0 < 0) {
                // CASE 5
                return (YS*YS + Y0*Y0) / surface_speed_;
            } else {
                // CASE 4
                return (YS*YS - Y0*Y0) / surface_speed_;
            }
        } else {
            if (Y0 < 0) {
                // CASE 3
                return 0.5f * (YS*YS + Y0*Y0) / surface_speed_;
            } else {
                if (YS/surface_speed_ - Y0/dive_speed_ > T) {
                    // CASE 2
                    return 0.5f * ((T + 2.0f*surface_speed_*Y0 - 2.0f*dive_speed_*YS)*T + (YS-Y0)*(YS-Y0)) / (surface_speed_ - dive_speed_);
                } else {
                    // CASE 1
                    return 0.5f * (YS*YS/surface_speed_ - Y0*Y0/dive_speed_);
                }
            }
        }

    }

    // Determine if node is valid
    virtual bool is_valid(const State& state) {

        if (state[Y] < min_depth_)
            return false;
        
        if (state[Y] > max_depth_)
            return false;

        if (state[SoC] < min_energy_)
            return false;

        if (state[SG] < min_specific_gravity_)
            return false;
        
        if (state[SG] > max_specific_gravity_)
            return false;

        if (state[Q] > M_PI_2f || state[Q] < -M_PI_2f)
            return false;

        if (state[t] > max_time_)
            return false;

        const float Y0 = state[Y] - desired_depth_;
        const float YS = -desired_depth_;
        const float T = 240.0f - state[t];
        if (YS - Y0 > surface_speed_*T)
            return false;
        

        return true;
    }

    // Determine if state is goal
    virtual bool is_goal(const State& state, const State& goal) {

        // Plan ends after resurface time reached and the glider is at the surface
        if (state[t] + 1.5 >= goal[t] && state[Y] >= -surface_threshold_)
            return true;

        return false;
    }

    // Deconstructor
    virtual ~SeaGliderModel() {}

    protected:

    // Parameters

    size_t integration_size_;

    float gravity_;
    float mass_;
    float moment_;
    float rotational_damping_;

    float body_area_;
    float wing_area_;

    float water_density_;
    float cop_length_;

    float dive_speed_;
    float surface_speed_;

    float min_depth_;
    float max_depth_;
    float desired_depth_;
    float surface_threshold_;
    float max_time_;

    float min_energy_;

    float min_specific_gravity_;
    float max_specific_gravity_;

    float P_friction_;
    float P_electronics_;
    float p_control_;

    // Private Functions

    float drag_coefficient_(const float alpha) {
        const float CD0 = 0.1185f;
        const float CD2 = 0.001614f;
        return CD0 + CD2*alpha*alpha;
    }

    float lift_coefficient_(const float alpha) {
        const float CL1 = 0.1066f;
        return CL1*alpha;
    }

    float power_output_(const float pcm_temp, const float depth) {

        float water_temp = water_temperature_(depth);

        float diffT = pcm_temp - water_temp;

        // Power output
        // TODO

        return diffT;
    }

    float water_temperature_(const float depth) {

        // Water temp
        // TODO

        return 0.0f;
    }


};

}

#endif