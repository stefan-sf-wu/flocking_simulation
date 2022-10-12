#ifndef COMMON_H_
#define COMMON_H_

#define PROJ_NAME "PARTICLE SYSTEM"
#define ENABLE_LOGGER true

/**
 * OBJECT PARAMS
 */ 
#define EDGE_SIZE 256         
#define PARTICLE_MASS 1.0   
#define BOID_NUMBER 5

/**
 * PARTICLE GENERATOR PARAMS
 */ 
#define RAND_VEC_LEN 9973
#define INIT_SPEED_MEAN 10.0
#define INIT_SPEED_STD 5.0

/**
 * FLOCKING ACCELERATION
*/
#define AVOIDANCE_SCALAR 0.2
#define VELOCITY_MATCHING_SCALAR 0.2
#define CENTERING_SCALAR 0.2


/**
 * CRONO
 */
#define TIMESTEP 0.02            // sec
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


inline void print_vec(glm::vec3 v)
{
    std::cout << "(" << v[0] << ", "<<v[1]<<", "<<v[2]<<")\n";
}

inline glm::vec3 transform_phy2gl(glm::vec3 v) {
    return {
        (v[0] * 2 / EDGE_SIZE) - 1,
        (v[1] * 2 / EDGE_SIZE) - 1,
        (v[2] * 2 / EDGE_SIZE) - 1,
    };
}

const double k_airres_coef = 0.05;
const double k_friction_coef = 0.20;
const double k_simplified_friction_coef = 0.2;
const double k_restitution_coef = 0.6;
const glm::vec3 k_gravity = {0, 0, -15};

// const struct Vec k_wind_velocity = {0.0, 0.0, 0.0};
const glm::vec3 k_wind_velocity = {0.2, -0.2, 0};



#endif // COMMON_H_