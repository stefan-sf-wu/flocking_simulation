#ifndef FLOCK_MANAGER_H_
#define FLOCK_MANAGER_H_

#include <vector>

#include <omp.h>

#include <glm/glm.hpp>
#include <glm/geometric.hpp>    // Geometric functions

#include "Common.hpp"

#if ENABLE_LOGGER
#include <iostream>
#endif

float d_a = 3, d_b = 5;
float theta_a = M_PI * (90 / 360), theta_b = M_PI * (220 / 360);

struct Boid
{   
    std::vector<glm::vec3> position;    // For timestep T and timestep T+1
    std::vector<glm::vec3> velocity;    // For timestep T and timestep T+1
    // glm::vec3 position;
    // glm::vec3 velocity;
    float birthday;

    Boid()
    : position({{0, 0, 0}, {0, 0, 0}})
    , velocity({{0, 0, 0}, {0, 0, 0}})
    , birthday(0) {}
};

class FlockManager
{
private: 
    std::vector<Boid*> boids_;
    glm::vec2 global_target_;

private:
    inline glm::vec3 compute_aviodance(float dist, glm::vec3 &unit_vec, float scalar=AVOIDANCE_SCALAR)
    {
        return -(scalar) * (1 / dist) * unit_vec;
    }

    inline glm::vec3 compute_centering(float dist, glm::vec3 &unit_vec, float scalar=CENTERING_SCALAR)
    {
        return scalar * dist * unit_vec;
    }

    inline glm::vec3 compute_velocity_matching(glm::vec3 &vec, float scalar=VELOCITY_MATCHING_SCALAR)
    {
        return scalar * vec;
    }

    inline float compute_distance_weight(float d)
    {
        if(d < d_a) return 1;
        if(d > d_b) return 0;
        return (d_b - d) / (d_b - d_a);
    }

    inline float compute_direction_weight(glm::vec3 &velocity, glm::vec3 &unit_vec_ij)
    {
        glm::vec3 unit_velocity = velocity;
        glm::normalize(unit_velocity);

        float cos_theta = glm::dot(unit_velocity, unit_vec_ij);
        float sin_theta = std::pow((1 - std::pow(cos_theta, 2)), 1/2);
        float theta = std::atan(sin_theta / cos_theta);
        theta = std::abs(theta);

        if(theta >= (-theta_a / 2) && theta <= (theta_a / 2)) return 1;
        if(theta > (theta_b / 2)) return 0;
        return (theta_b - theta) / (theta_b - theta_a);
    }

public:
    FlockManager()
    {

    };

    void reset()
    {
        boids_.clear();
        for(int i = 0; i < BOID_NUMBER; i++)
        {
            boids_.push_back(new Boid());
        }
    }

    void compute_flocking_acceleration(unsigned int idx_i, unsigned int idx_j)
    {
        float dist_ij;
        glm::vec3 vec_ij;
        glm::vec3 unit_vec_ij;

        float dist_weight;
        float dir_weight;

        glm::vec3 acc_steeing;

        glm::vec3 acc_avoidance;
        glm::vec3 acc_velocity_matching;
        glm::vec3 acc_centering;


        int i, j;
        for(i = 0; i < BOID_NUMBER; i++)
        {
            /**
             * TODO: detemine and save total force on i due to the env, e.g.
             * 1. LEAD_BOID
             * 2. POTENTIAL FIELDS
             * 3. ATTRACTORS / REPELLORS
            */ 
            acc_avoidance = acc_velocity_matching = acc_centering = {0, 0, 0};

            #pragma omp parallel for\
                shared(i, acc_avoidance, acc_velocity_matching, acc_centering)\
                private(j, dist_ij, vec_ij, unit_vec_ij, dist_weight, dir_weight)
            {
                for(j = 0; j < BOID_NUMBER; j++)
                {
                    /**
                     * TODO: add contribution of particle j to i's total force
                    */
                    dist_ij     = glm::distance(boids_[i]->position[0], boids_[j]->position[0]);
                    vec_ij      = (boids_[j]->position[0] - boids_[i]->position[0]);
                    unit_vec_ij = vec_ij / dist_ij;

                    dir_weight  = compute_direction_weight(boids_[i]->velocity[0], unit_vec_ij);
                    dist_weight = compute_distance_weight(dist_ij);

                    #pragma omp atomic update
                    acc_avoidance += dist_weight * dir_weight * compute_aviodance(dist_ij, unit_vec_ij);
                    
                    #pragma omp atomic update
                    acc_centering += dist_weight * dir_weight * compute_centering(dist_ij, unit_vec_ij);

                    #pragma omp atomic update
                    acc_velocity_matching += dist_weight * dir_weight * compute_velocity_matching(vec_ij);
                }
            }   // omp parallel for
            
            /**
             * TODO: integrate all of the forces on i
            */



        }


    }
    
    void get_cursor_position(float cursor_pos_x, float cursor_pos_y)
    {
        global_target_[0] = cursor_pos_x;
        global_target_[1] = cursor_pos_y;
    };

    ~FlockManager() {};
};

#endif // FLOCK_MANAGER_H_