#include "world.h"

World newWorld(float width, float height)
{
    return (World){
        .width = width,
        .height = height,
    };
}
