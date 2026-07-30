#ifndef PROPERTIES_H_
#define PROPERTIES_H_

#include "raylib_all.h"

typedef struct motionprops_t
{
    Vector2 pos;
    Vector2 vel;
    Vector2 acc;
} MotionProperties;

typedef struct phyprops_t
{
    float restitution;
} PhysicalProperties;

MotionProperties newMotionProperties(float px, float py, float vx, float vy, float ax, float ay);
MotionProperties newMotionPropertiesV(Vector2 pos, Vector2 vel, Vector2 acc);

PhysicalProperties newPhysicalProperties(float restitution);

#endif
