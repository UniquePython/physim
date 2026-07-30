#include "properties.h"

MotionProperties newMotionProperties(float px, float py, float vx, float vy, float ax, float ay)
{
    return (MotionProperties){
        .pos.x = px,
        .pos.y = py,
        .vel.x = vx,
        .vel.y = vy,
        .acc.x = ax,
        .acc.y = ay,
    };
}

MotionProperties newMotionPropertiesV(Vector2 pos, Vector2 vel, Vector2 acc)
{
    return (MotionProperties){
        .pos = pos,
        .vel = vel,
        .acc = acc,
    };
}

PhysicalProperties newPhysicalProperties(float restitution)
{
    return (PhysicalProperties){
        .restitution = restitution,
    };
}
