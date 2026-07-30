#include "world.h"

World newWorld(float width, float height, float gx, float gy)
{
    return (World){
        .width = width,
        .height = height,
        .gravity = (Vector2){gx, gy},
    };
}

World newWorldV(float width, float height, Vector2 gravity)
{
    return (World){
        .width = width,
        .height = height,
        .gravity = gravity,
    };
}
