#ifndef WORLD_H_
#define WORLD_H_

#include <raymath.h>

typedef struct world_t
{
    float width;
    float height;

    Vector2 gravity;
} World;

World newWorld(float width, float height, float gx, float gy);
World newWorldV(float width, float height, Vector2 gravity);

#endif
