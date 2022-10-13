#ifndef FLOCK_MANAGER_H_
#define FLOCK_MANAGER_H_

#include <vector>

#include <omp.h>
#include <glm/glm.hpp>
#include <glm/geometric.hpp>    // Geometric functions

#include "Common.hpp"
#include "RandGenerator.hpp"
#include "AccelerationOperators.hpp"

#if ENABLE_LOGGER
#include <iostream>
#endif

class FlockManager
{
private: 
    RandGenerator rand_generator;

    std::vector<Boid*> boids_;
    glm::vec2 global_target_;

    unsigned int num_boid_;
    unsigned int num_leading_boid_;
    float avoidance_scalar_;
    float centering_scalar_;
    float velocity_matching_scalar_;

public:
    FlockManager() {};

    void reset(unsigned int num_boid, unsigned int num_leading_boid, float avoidance_scalar, float centering_scalar, float velocity_matching_scalar)
    {
        num_boid_ = num_boid;
        num_leading_boid_ = num_leading_boid;
        avoidance_scalar_ = avoidance_scalar;
        centering_scalar_ = centering_scalar;
        velocity_matching_scalar_ = velocity_matching_scalar;

        boids_.clear();
        for(int i = 0; i < num_boid_; i++)
        {
            boids_.push_back(new Boid());
            boids_[i]->position[0] = rand_generator.generate_random_uniform_vec3(0, EDGE_SIZE);
            boids_[i]->velocity[0] = rand_generator.generate_random_gaussian_vec3(
                                        rand_generator.generate_uniform(5, 15), 
                                        rand_generator.generate_uniform(7, 13));
        }
    }

    void compute_acceleration()
    {
        glm::vec3 acc_steering;
        glm::vec3 acc_repeller;
        glm::vec3 acc_flocking;

        glm::vec3 vel_vortex;
        glm::vec3 vel_wind;

        int i;

        for(i = 0; i < num_boid_; i++)
        {
            /**
             * TODO: detemine and save total force on i due to the env, e.g.
             * 1. LEAD_BOID
             * 2. POTENTIAL FIELDS
             * 3. ATTRACTORS / REPELLORS
            */ 
            if(i < num_leading_boid_)
            {
                glm::vec3 lead_pos = boids_[i]->position[0] + rand_generator.generate_random_direction_vec(EDGE_SIZE);
                if (glm::all(glm::lessThan(lead_pos, glm::vec3({EDGE_SIZE, EDGE_SIZE, EDGE_SIZE}))))
                {
                    boids_[i]->position[1] = lead_pos;
                }
                continue;
            }

            acc_flocking = compute_flocking_acc(boids_, i, num_boid_, avoidance_scalar_, centering_scalar_, velocity_matching_scalar_);
            acc_repeller = compute_bb_repel_acc(boids_[i]->position[0]);

            vel_vortex = compute_vortex_velocity(boids_[i]);
            vel_wind = compute_wind_acc(boids_[i]);

            /**
             * TODO: integrate all of the forces on i: prioritize accelerations
            */

            boids_[i]->velocity[1] = boids_[i]->velocity[0] + (float)TIMESTEP * (acc_repeller + acc_flocking);
            boids_[i]->velocity[1] *= (k_cruising_velocity_limit / glm::length(boids_[i]->velocity[1]));
            
            boids_[i]->position[1] = boids_[i]->position[0] + (float)TIMESTEP * boids_[i]->velocity[0];
        }

        #pragma omp parallel for\
            shared(boids_)\
            private(i)
        for(i = 0; i < num_boid_; i++)
        {
            boids_[i]->velocity[0] = boids_[i]->velocity[1];
            boids_[i]->position[0] = boids_[i]->position[1];
        }
        
    }

    std::vector<Boid*> &get_boids()
    {
        return boids_;
    }
    
    void get_cursor_position(float cursor_pos_x, float cursor_pos_y)
    {
        global_target_[0] = cursor_pos_x;
        global_target_[1] = cursor_pos_y;
    };

    ~FlockManager() {};
};

#endif // FLOCK_MANAGER_H_