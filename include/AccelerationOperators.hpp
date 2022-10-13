#ifndef ACCELERATION_OPERATORS_H_
#define ACCELERATION_OPERATORS_H_

#include <cmath>
#include <vector>

#include <omp.h>
#include <glm/glm.hpp>
#include <glm/geometric.hpp>

#include "Common.hpp"

// const float gravitional_gravity = 6.67 * 10e-11;    // N * m^2 * kg^-2
const float k_cruising_velocity_limit = 50;

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


inline glm::vec3 compute_bb_repel(glm::vec3 &obj_pos)
{
    glm::vec3 acc_all = {0, 0, 0};

    glm::vec3 my_acc;
    glm::vec3 plain_unit_normal_vec;
    float dist;
    int i;

    // #pragma omp parallel for\
    //     shared(box_plain_list, point_on_plain_list, acc_all, obj_pos)\
    //     private(i, my_acc, dist, plain_unit_normal_vec)
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

/*
glm::vec3 gravitional_attracter(glm::vec3 obj_pos, glm::vec3 attracter_pos, float scalar = 2, bool isAttracter = true)
{
    float distance = glm::distance(obj_pos, attracter_pos);
    int sign = (isAttracter) ? -1 : 1;

    return sign * gravitional_gravity / std::pow(distance, scalar) * (attracter_pos - obj_pos);
}
*/


#endif // ACCELERATION_OPERATORS_H_