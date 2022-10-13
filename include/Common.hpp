#ifndef COMMON_H_
#define COMMON_H_

#define PROJ_NAME "PARTICLE SYSTEM"
#define ENABLE_LOGGER true

/**
 * OBJECT PARAMS
 */ 
#define EDGE_SIZE 512
#define PARTICLE_MASS 1.0   
#define BOID_NUMBER 12

/**
 * PARTICLE GENERATOR PARAMS
 */ 
#define RAND_VEC_LEN 9973
#define INIT_SPEED_MEAN 10.0
#define INIT_SPEED_STD 5.0

/**
 * FLOCKING 
*/
#define AVOIDANCE_SCALAR 1e1
#define VELOCITY_MATCHING_SCALAR 1
#define CENTERING_SCALAR 1e-2
#define LEADING_BOIDS_PERCENTAGE 0.1

/**
 * REPELLER 
*/
// #define REPELLER_STRENGTH_SCALAR 
#define REPELLER_DISTANCE_SCALAR 1.2

/**
 * CRONO
 */
#define TIMESTEP 0.01            // sec
#define MAX_DISPLAY_TIME 3000     // sec

/**
 * VIEW PARAMS
 */ 
#define SCR_WIDTH 1600
#define SCR_HEIGHT 1000
#define DISPLAY_REFRESH_INTERVAL (1.0 / 30.0) //  sec (HZ)

#include <vector>
#include <iostream>

#include <glm/glm.hpp>

inline glm::vec3 transform_phy2gl(glm::vec3 v) {
    return {
        (v[0] * 2 / EDGE_SIZE) - 1,
        (v[1] * 2 / EDGE_SIZE) - 1,
        (v[2] * 2 / EDGE_SIZE) - 1,
    };
}

inline glm::vec3 transform_gl2phy(glm::vec3 v) {
    return {
        (v[0] + 1) * (EDGE_SIZE / 2),
        (v[1] + 1) * (EDGE_SIZE / 2),
        (v[2] + 1) * (EDGE_SIZE / 2),
    };
}

#if ENABLE_LOGGER
void print_vec(glm::vec3 v)
{
    std::cout << "(" << v[0] << ", "<<v[1]<<", "<<v[2]<<")\n";
}
#endif


#endif // COMMON_H_