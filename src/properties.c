#include "properties.h"

#include <math.h>

struct linear_t newLinearMotion(float px, float py, float vx, float vy, float ax, float ay)
{
    return (struct linear_t){
        .pos.x = px,
        .pos.y = py,
        .vel.x = vx,
        .vel.y = vy,
        .acc.x = ax,
        .acc.y = ay,
    };
}

struct linear_t newLinearMotionV(Vector2 pos, Vector2 vel, Vector2 acc)
{
    return (struct linear_t){
        .pos = pos,
        .vel = vel,
        .acc = acc,
    };
}

struct angular_t newAngularMotion(float angle, float vel, float acc)
{
    return (struct angular_t){
        .angle = angle,
        .vel = vel,
        .acc = acc,
    };
}

MotionProperties newMotionProperties(struct linear_t linear, struct angular_t angular)
{
    return (MotionProperties){
        .linear = linear,
        .angular = angular,
    };
}

PhysicalProperties newPhysicalProperties(float mass, float restitution)
{
    if (mass == INFINITY)
    {
        return (PhysicalProperties){
            .mass = mass,
            .invMass = 0.0f,
            .isStatic = true,
            .restitution = restitution,
        };
    }

    return (PhysicalProperties){
        .mass = mass,
        .invMass = 1.0f / mass,
        .isStatic = false,
        .restitution = restitution,
    };
}
