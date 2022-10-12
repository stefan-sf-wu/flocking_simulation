#ifndef ACCELERATION_OPERATORS_H_
#define ACCELERATION_OPERATORS_H_

#include <cmath>

#include <glm/glm.hpp>
#include <glm/geometric.hpp>

const float gravitional_gravity = 6.67 * 10e-11;    // N * m^2 * kg^-2
const float k_airres_coef = 0.05;
const float k_friction_coef = 0.20;
const float k_simplified_friction_coef = 0.2;
const float k_restitution_coef = 0.6;
const glm::vec3 k_gravity = {0, 0, -15};
const glm::vec3 k_wind_velocity = {0.2, -0.2, 0};



glm::vec3 gravitional_attracter(glm::vec3 obj_pos, glm::vec3 attracter_pos, float scalar = 2, bool isAttracter = true)
{
    float distance = glm::distance(obj_pos, attracter_pos);
    int sign = (isAttracter) ? -1 : 1;

    return sign * gravitional_gravity / std::pow(distance, scalar) * (attracter_pos - obj_pos);
}

#endif // ACCELERATION_OPERATORS_H_