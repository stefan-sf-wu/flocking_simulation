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

float d_a = (EDGE_SIZE * 1 / 6);
float d_b = (EDGE_SIZE * 1 / 4);
float theta_a = (M_PI * 90 / 360);
float theta_b = (M_PI * 240 / 360);

struct Boid
{   
    std::vector<glm::vec3> position;    // For timestep T and timestep T+1
    std::vector<glm::vec3> velocity;    // For timestep T and timestep T+1
    // glm::vec3 position;
    // glm::vec3 velocity;

    Boid()
    : position({{0, 0, 0}, {0, 0, 0}})
    , velocity({{0, 0, 0}, {0, 0, 0}}) {}
    Boid(std::vector<glm::vec3> pos, std::vector<glm::vec3> vel)
    : position(pos)
    , velocity(vel) {}
};

class FlockManager
{
private: 
    RandGenerator rand_generator;

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

    inline glm::vec3 compute_velocity_matching(glm::vec3 &velocity_i, glm::vec3 &velocity_j, float scalar=VELOCITY_MATCHING_SCALAR)
    {
        return scalar * (velocity_j - velocity_i);
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
            boids_[i]->position[0] = rand_generator.generate_random_uniform_vec(0, EDGE_SIZE);
            boids_[i]->velocity[0] = rand_generator.generate_random_gaussian_vec(10, 10);
        }
    }

    void compute_acceleration()
    {
        float dist_ij;
        glm::vec3 vec_ij;
        glm::vec3 unit_vec_ij;

        float dist_dir_weight;

        glm::vec3 acc_steering;
        glm::vec3 acc_repeller;

        glm::vec3 acc_avoidance;
        glm::vec3 acc_velocity_matching;
        glm::vec3 acc_centering;

        glm::vec3 my_acc_avoidance;
        glm::vec3 my_acc_velocity_matching;
        glm::vec3 my_acc_centering;


        int i, j;
        for(i = 0; i < BOID_NUMBER; i++)
        {
            /**
             * TODO: detemine and save total force on i due to the env, e.g.
             * 1. LEAD_BOID
             * 2. POTENTIAL FIELDS
             * 3. ATTRACTORS / REPELLORS
            */ 

            acc_repeller = compute_bb_repel(boids_[i]->position[0]);

            acc_avoidance = acc_velocity_matching = acc_centering = {0, 0, 0};

            // #pragma omp parallel for\
            //     default(none)\
            //     shared(i, boids_, acc_avoidance, acc_velocity_matching, acc_centering)\
            //     private(j, dist_ij, vec_ij, unit_vec_ij, dist_dir_weight, my_acc_avoidance, my_acc_velocity_matching, my_acc_centering)
            for(j = 0; j < BOID_NUMBER; j++)
            {
                if(i == j) continue;

                dist_ij     = glm::distance(boids_[i]->position[0], boids_[j]->position[0]);
                vec_ij      = (boids_[j]->position[0] - boids_[i]->position[0]);
                unit_vec_ij = vec_ij / dist_ij;

                dist_dir_weight = compute_direction_weight(boids_[i]->velocity[0], unit_vec_ij) * compute_distance_weight(dist_ij);

                my_acc_avoidance = dist_dir_weight * compute_aviodance(dist_ij, unit_vec_ij);
                my_acc_centering = dist_dir_weight * compute_centering(dist_ij, unit_vec_ij);
                my_acc_velocity_matching = dist_dir_weight * compute_velocity_matching(boids_[i]->velocity[0], boids_[j]->velocity[0]);

                for(int k = 0; k < 3; k++)
                {
                    #pragma omp atomic update
                    acc_centering[k] += my_acc_centering[k];                  
                    #pragma omp atomic update
                    acc_avoidance[k] += my_acc_avoidance[k];
                    #pragma omp atomic update
                    acc_velocity_matching[k] += my_acc_velocity_matching[k]; 
                }
            }
            
            /**
             * TODO: integrate all of the forces on i: prioritize accelerations
            */
        //    std::cout << "acc_aoivdance";
        //    print_vec(acc_avoidance);
        //    std::cout << "acc_centering";
        //    print_vec(acc_centering);
        //    std::cout << "acc_v_matching";
        //    print_vec(acc_velocity_matching);
        //    std::cout << "acc_flocking_all";
        //    print_vec(acc_avoidance+acc_centering + acc_velocity_matching);
        //    std::cout << std::endl;

            // boids_[i]->velocity[1] = boids_[i]->velocity[0] + (float)TIMESTEP * (acc_repeller);
            boids_[i]->velocity[1] = boids_[i]->velocity[0] + (float)TIMESTEP * (acc_repeller + acc_avoidance + acc_centering + acc_velocity_matching);
            boids_[i]->velocity[1] *= (k_cruising_velocity_limit / glm::length(boids_[i]->velocity[1]));
            
            boids_[i]->position[1] = boids_[i]->position[0] + (float)TIMESTEP * boids_[i]->velocity[0];
        }

        #pragma omp parallel for\
            shared(boids_)\
            private(i)
        for(i = 0; i < BOID_NUMBER; i++)
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