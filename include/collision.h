#ifndef COLLISION_H_
#define COLLISION_H_

#include "raylib_all.h"
#include "shape.h"
#include "physics_body.h"

#include <stdbool.h>

bool bodiesColliding(const PhysicsBody *a, const PhysicsBody *b);

#endif