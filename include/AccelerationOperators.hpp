#ifndef ACCELERATION_OPERATORS_H_
#define ACCELERATION_OPERATORS_H_

#include <cmath>
#include <vector>

#include <omp.h>
#include <glm/glm.hpp>
#include <glm/geometric.hpp>

#include "Common.hpp"

const float k_cruising_velocity_limit = 80;

// WIND ----------------------------------------------------------------------------------------//

glm::vec3 compute_wind_acc(Boid *boid, float scalar = 1e-6)
{
    if(boid->position[0][2] > EDGE_SIZE * (4/6) && boid->position[0][2] < EDGE_SIZE * (5/6))
    {
        return scalar * glm::normalize(glm::vec3({-1, 2, 1}));
    }
    if(boid->position[0][2] > EDGE_SIZE * (1/6) && boid->position[0][2] < EDGE_SIZE * (2/6))
    {
        return scalar * glm::normalize(glm::vec3({1, 3, -1}));
    }
    return {0, 0, 0};
}

// VORTEX --------------------------------------------------------------------------------------//
const glm::vec3 k_vortex_center = {EDGE_SIZE/3, EDGE_SIZE/3, EDGE_SIZE/3};
const glm::vec3 k_vortex_unit_normal = {1/3, 1/3, 1/3};
const float k_vortex_radius = 3;
const float k_vortex_height = 30;
const float k_vortex_fq = 20;    // Hz
const float k_fq_max = 40;      // Hz

glm::vec3 compute_vortex_velocity(Boid *boid, float tao = 4)
{
    glm::vec3   vec_obj_c   = boid->position[0] - k_vortex_center;
    float       l           = glm::dot(k_vortex_unit_normal, vec_obj_c);
    glm::vec3   vec_r       = vec_obj_c - l * k_vortex_unit_normal;
    float       r           = glm::length(vec_r);
    glm::vec3   vec_acc     = -vec_r;

    float       fq          = std::pow((k_vortex_radius / r), tao) * k_vortex_fq;
    fq                      = std::min(k_fq_max, fq);
    float omega = 2 * M_PI * fq;

    glm::vec3 u_x = glm::normalize(boid->velocity[0]); 
    glm::vec3 u_y = glm::normalize(glm::cross(boid->velocity[0], vec_acc));
    glm::vec3 u_z = glm::cross(u_x, u_y);

    glm::mat4 R = {
        {u_x[0], u_x[1], u_x[2], 0},
        {u_y[0], u_y[1], u_y[2], 0},
        {u_z[0], u_z[1], u_z[2], 0},
        {0     ,0      , 0     , 1},
    };

    glm::mat4 R_theta = {
        {1, 0, 0, 0},
        {0, std::cos(omega), -std::sin(omega), 0},
        {0, std::sin(omega), std::cos(omega), 0},
        {0, 0, 0, 1}
    };

    glm::mat4 T = {
        {1, 0, 0, -boid->position[0][0]},
        {0, 1, 0, -boid->position[0][1]},
        {0, 0, 1, -boid->position[0][2]},
        {0, 0, 0, 1}
    };

    glm::mat4 ret = glm::inverse(T) * glm::inverse(R) * R_theta * R * T;

    return {ret[0][3], ret[1][3], ret[2][3]};
}

// FLOCKING ------------------------------------------------------------------------------------//
const float d_a = (EDGE_SIZE * 1 / 6);
const float d_b = (EDGE_SIZE * 1 / 4);
const float theta_a = (M_PI * 90 / 360);
const float theta_b = (M_PI * 240 / 360);

inline glm::vec3 compute_aviodance(float dist, glm::vec3 &unit_vec, float scalar)
{
    return -(scalar) * (1 / dist) * unit_vec;
}

inline glm::vec3 compute_centering(float dist, glm::vec3 &unit_vec, float scalar)
{
    return scalar * dist * unit_vec;
}

inline glm::vec3 compute_velocity_matching(glm::vec3 &velocity_i, glm::vec3 &velocity_j, float scalar)
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

glm::vec3 compute_flocking_acc(
    std::vector<Boid*> &boids_, 
    int i, 
    unsigned int num_boid,
    float avoidance_scalar, 
    float centering_scalar,
    float velocity_matching_scalar)
{
    glm::vec3 acc_avoidance;
    glm::vec3 acc_velocity_matching;
    glm::vec3 acc_centering;
    acc_avoidance = acc_velocity_matching = acc_centering = {0.0, 0.0, 0.0};

    glm::vec3 my_acc_avoidance;
    glm::vec3 my_acc_velocity_matching;
    glm::vec3 my_acc_centering;

    float dist_ij;
    glm::vec3 vec_ij;
    glm::vec3 unit_vec_ij;

    float dist_dir_weight;
    
    int j;

    #pragma omp parallel for\
        default(none)\
        shared(i, boids_, acc_avoidance, acc_velocity_matching, acc_centering, num_boid, avoidance_scalar, centering_scalar, velocity_matching_scalar)\
        private(j, dist_ij, vec_ij, unit_vec_ij, dist_dir_weight, my_acc_avoidance, my_acc_velocity_matching, my_acc_centering)
    for(j = 0; j < num_boid; j++)
    {
        if(i == j) continue;
        
        dist_ij     = glm::distance(boids_[i]->position[0], boids_[j]->position[0]);
        vec_ij      = (boids_[j]->position[0] - boids_[i]->position[0]);
        unit_vec_ij = vec_ij / dist_ij;

        dist_dir_weight = compute_direction_weight(boids_[i]->velocity[0], unit_vec_ij) * compute_distance_weight(dist_ij);

        my_acc_avoidance = dist_dir_weight * compute_aviodance(dist_ij, unit_vec_ij, avoidance_scalar);
        my_acc_centering = dist_dir_weight * compute_centering(dist_ij, unit_vec_ij, centering_scalar);
        my_acc_velocity_matching = dist_dir_weight * compute_velocity_matching(boids_[i]->velocity[0], boids_[j]->velocity[0], velocity_matching_scalar);

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

    return acc_avoidance + acc_centering + acc_velocity_matching;
}

// BOUNDING BOX --------------------------------------------------------------------------------//

std::vector<glm::vec4> box_plain_list = 
{
    {1, 0, 0, 0},           // BACK
    {0, 1, 0, 0},           // LEFT
    {0, 0, 1, 0},           // BOTTOM
    {-1, 0, 0, EDGE_SIZE},  // FRONT
    {0, -1, 0, EDGE_SIZE},  // RIGHT
    {0, 0, -1, EDGE_SIZE},  // TOP
};

std::vector<glm::vec3> point_on_plain_list =
{
    {0, 0, 0},
    {0, 0, 0},
    {0, 0, 0},
    {EDGE_SIZE, EDGE_SIZE, EDGE_SIZE},
    {EDGE_SIZE, EDGE_SIZE, EDGE_SIZE},
    {EDGE_SIZE, EDGE_SIZE, EDGE_SIZE}
};

inline float calculate_signed_distance(int i, glm::vec3 &position, glm::vec3 &plain_normal_vec) {
    glm::vec3 v = 
    {
        position[0] - point_on_plain_list[i][0],
        position[1] - point_on_plain_list[i][1],
        position[2] - point_on_plain_list[i][2]
    };
    glm::normalize(plain_normal_vec);

    return glm::dot(plain_normal_vec, v);
}


glm::vec3 compute_bb_repel_acc(glm::vec3 &obj_pos)
{
    glm::vec3 acc_all = {0, 0, 0};

    glm::vec3 my_acc;
    glm::vec3 plain_unit_normal_vec;
    float dist;
    int i;

    #pragma omp parallel for\
        shared(box_plain_list, point_on_plain_list, acc_all, obj_pos)\
        private(i, my_acc, dist, plain_unit_normal_vec)
    for(i = 0; i < 6; i++)
    {
        plain_unit_normal_vec = glm::vec3(box_plain_list[i][0], box_plain_list[i][1], box_plain_list[i][2]);

        dist = (calculate_signed_distance(i, obj_pos, plain_unit_normal_vec));

        my_acc = (float)(std::pow(EDGE_SIZE, REPELLER_DISTANCE_SCALAR) 
                 / std::pow(std::abs(dist), REPELLER_DISTANCE_SCALAR)) 
                 * plain_unit_normal_vec;

        #pragma omp atomic update
        acc_all[0] += my_acc[0]; 
        #pragma omp atomic update
        acc_all[1] += my_acc[1]; 
        #pragma omp atomic update
        acc_all[2] += my_acc[2]; 
    }

    return acc_all;
}



#endif // ACCELERATION_OPERATORS_H_