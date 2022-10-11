#ifndef MATH_H_
#define MATH_H_

#include <cmath>

#include "Common.h"
#include "Math.h"

struct Vec {
    float x;
    float y;
    float z;

    inline float get_sqrt() 
    { 
        return std::sqrt(std::pow(x, 2) + std::pow(y, 2) + std::pow(z, 2)); 
    }

    inline Vec get_unit_vec() 
    {    
        if(x == 0 && y == 0 && z == 0) { return {0, 0, 0}; }
        float dist = get_sqrt();
        return {
            x / dist,
            y / dist,
            z / dist
        };
    }
};


inline Vec vec_add(Vec v1, Vec v2) {
    return {
        v1.x + v2.x,
        v1.y + v2.y,
        v1.z + v2.z
    };
};

inline Vec vec_add(Vec v1, float n) {
    return {
        v1.x + n,
        v1.y + n,
        v1.z + n
    };
}

inline void vec_add(Vec *v1, Vec *v2) {
        v1->x += v2->x;
        v1->y += v2->y;
        v1->z += v2->z;
};

inline Vec vec_substract(Vec v1, Vec v2) {
    return {
        v1.x - v2.x,
        v1.y - v2.y,
        v1.z - v2.z
    };
};

inline Vec vec_substract(Vec v1, float n) {
    return {
        v1.x - n,
        v1.y - n,
        v1.z - n
    };
};

inline float vec_dot_product(Vec v1, Vec v2) {
    return ((v1.x * v2.x) + (v1.y * v2.y) + (v1.z * v2.z));
}

inline Vec vec_multiply(Vec v1, float n) {
    return {
        v1.x *= n,
        v1.y *= n,
        v1.z *= n
    };
};

void vec_multiply(Vec *v1, float n) {
    v1->x *= n;
    v1->y *= n;
    v1->z *= n;
};

inline Vec vec_cross_product(Vec v1, Vec v2) {
    return {
        (v1.y * v2.z) - (v2.y * v1.z),
        -((v1.x * v2.z) - (v2.x * v1.z)),
        (v1.x * v2.y) - (v2.x * v1.y)
    };
};

inline Vec transform_phy2gl(Vec v1) {
    return vec_substract(
               vec_multiply(v1, (2 / EDGE_SIZE)),
               1.0
           );
}

inline Vec transform_gl2phy(Vec v1) {
    return vec_multiply(
                vec_add(v1, 1),
                (EDGE_SIZE / 2)
    );
}

struct TrianglePlain {
    Vec p1;
    Vec p2;
    Vec p3;

    Vec get_unit_normal_vec() {
        Vec cross_product = vec_cross_product(
                                vec_substract(p1, p2),
                                vec_substract(p3, p2)
                            );
        return cross_product.get_unit_vec();
    }
};


struct Plain {
    // a*x + b*y + c*z + d = 0
    float a;
    float b;
    float c;
    float d;
        
    Vec get_unit_normal_vec() {
        return {a, b, c};
    }
};

struct Triangle {
    float v1;
    float v2;
    float v3;
};


#ifdef ENABLE_LOGGER
void print_vec(Vec v1)
{
    std::cout << "[" << v1.x << ", " << v1.y << ", " << v1.z << "]" << std::endl;
}
#endif

#endif // MATH_H_