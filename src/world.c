#include "world.h"

World newWorld(float width, float height, float gravity)
{
    return (World){
        .width = width,
        .height = height,
        .gravity = gravity,
    };
}
