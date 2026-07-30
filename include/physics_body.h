#ifndef PHYSICS_BODY_H_
#define PHYSICS_BODY_H_

#include "raylib_all.h"
#include "shape.h"
#include "properties.h"
#include "world.h"

typedef struct phybody_t
{
    Shape shape;
    MotionProperties motion;
    PhysicalProperties physical;
} PhysicsBody;

PhysicsBody newPhysicsBody(Shape shape, MotionProperties motion, PhysicalProperties physical);

void drawPhysicsBody(const PhysicsBody *body);

void updatePhysicsBody(const World *world, PhysicsBody *body, float dt);

#endif
