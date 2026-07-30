#ifndef PHYSICS_BODY_H_
#define PHYSICS_BODY_H_

#include "shape.h"
#include "properties.h"

typedef struct phybody_t
{
    Shape shape;
    MotionProperties motion;
    PhysicalProperties physical;
} PhysicsBody;

PhysicsBody newPhysicsBody(Shape shape, MotionProperties motion, PhysicalProperties physical);

void drawPhysicsBody(const PhysicsBody *body);

void updatePhysicsBody(PhysicsBody *body, float dt);

#endif
