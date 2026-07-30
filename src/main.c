#include <stdbool.h>
#include <stddef.h>

#include "raylib_all.h"
#include "shape.h"
#include "properties.h"
#include "physics_body.h"
#include "world.h"

#define WIDTH 900
#define HEIGHT 600
#define TITLE "Physim - Simple Physics Simulator"

#define GRAVITY 0, 500
#define NBODIES 5

int main(void)
{
    InitWindow(WIDTH, HEIGHT, TITLE);
    SetTargetFPS(GetMonitorRefreshRate(GetCurrentMonitor()));

    World world = newWorld(WIDTH, HEIGHT, GRAVITY);

    PhysicsBody bodies[NBODIES] = {0};

    for (size_t i = 0; i < NBODIES; i++)
    {
        Shape shape = newShapeCircle(10, RAYWHITE);

        MotionProperties motion = newMotionProperties(
            (float)i, (float)i,
            (float)(20 * i), (float)(20 * i),
            0.0f, 0.0f);
        PhysicalProperties physical = newPhysicalProperties(0.8f);

        bodies[i] = newPhysicsBody(shape, motion, physical);
    }

    while (!WindowShouldClose())
    {
        float dt = GetFrameTime();

        BeginDrawing();

        ClearBackground(BLACK);

        for (size_t i = 0; i < NBODIES; i++)
        {
            updatePhysicsBody(&world, &bodies[i], dt);
            drawPhysicsBody(&bodies[i]);
        }

        EndDrawing();
    }

    CloseWindow();

    return 0;
}