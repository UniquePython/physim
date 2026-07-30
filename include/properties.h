#ifndef PROPERTIES_H_
#define PROPERTIES_H_

#include "raylib_all.h"
#include "shape.h"

#include <stdbool.h>

typedef struct motionprops_t
{
    struct linear_t
    {
        Vector2 pos;
        Vector2 vel;
        Vector2 acc;
    } linear;

    struct angular_t
    {
        float angle;
        float vel;
        float acc;
    } angular;
} MotionProperties;

typedef struct phyprops_t
{
    float mass;
    float invMass;

    float inertia;
    float invInertia;

    bool isStatic;

    float restitution;
} PhysicalProperties;

struct linear_t newLinearMotion(float px, float py, float vx, float vy, float ax, float ay);
struct linear_t newLinearMotionV(Vector2 pos, Vector2 vel, Vector2 acc);

struct angular_t newAngularMotion(float angle, float vel, float acc);

MotionProperties newMotionProperties(struct linear_t linear, struct angular_t angular);

PhysicalProperties newPhysicalProperties(const Shape *shape, float mass, float restitution);

#endif
