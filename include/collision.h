#ifndef COLLISION_H_
#define COLLISION_H_

#include "raylib_all.h"
#include "shape.h"
#include "physics_body.h"

#include <stdbool.h>

typedef struct collision_t
{
    Vector2 normal; // From B -> A
    float penetration;
    Vector2 contactPoint;
} Collision;

void resolveOverlap(PhysicsBody *a, PhysicsBody *b, const Collision *collision);
void resolveVelocity(PhysicsBody *a, PhysicsBody *b, const Collision *collision);

bool bodiesColliding(const PhysicsBody *a, const PhysicsBody *b, Collision *collision);

#endif