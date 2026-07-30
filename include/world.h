#ifndef WORLD_H_
#define WORLD_H_

typedef struct world_t
{
    float width;
    float height;

    float gravity;
} World;

World newWorld(float width, float height, float gravity);

#endif
