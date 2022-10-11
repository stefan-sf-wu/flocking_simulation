#ifndef COMMON_H_
#define COMMON_H_

#define PROJ_NAME "PARTICLE SYSTEM"
#define ENABLE_LOGGER true

/**
 * OBJECT PARAMS
 */ 
#define EDGE_SIZE 100         
#define PARTICLE_MASS 1.0   
#define PARTICLE_NUMBER 150000

/**
 * PARTICLE GENERATOR PARAMS
 */ 
#define RAND_VEC_LEN 9973
#define INIT_SPEED_MEAN 10.0
#define INIT_SPEED_STD 5.0
#define GENERATION_RATE 2500.0    // per sec
#define LIFE_SPAN (PARTICLE_NUMBER / GENERATION_RATE + 1)         // sec

enum struct GENERATOR_TYPE 
{
    OMNI, DIRECTED, DISK, TRIANGLE, SPHERE
};

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

#include "Math.h"

/**
 * FORCE
 */ 

enum struct ACCELERATION 
{
    GRAVITY, AIR_RESISTANCE
};


const double k_airres_coef = 0.05;
const double k_friction_coef = 0.20;
const double k_simplified_friction_coef = 0.2;
const double k_restitution_coef = 0.6;
const struct Vec k_gravity = {0, 0, -15};

// const struct Vec k_wind_velocity = {0.0, 0.0, 0.0};
const struct Vec k_wind_velocity = {0.2, -0.2, 0};



#endif // COMMON_H_