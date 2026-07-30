#include <raylib.h>
#include <raymath.h>

#include <stdbool.h>

#include "shape.h"
#include "properties.h"
#include "physics_body.h"

#define WIDTH 900
#define HEIGHT 600
#define TITLE "Physim - Simple Physics Simulator"

#define INIT_POS WIDTH / 2, HEIGHT / 2
#define INIT_VEL 200, -100
#define GRAVITY 0, 500
#define RESTITUTION 0.8

int main(void)
{
    InitWindow(WIDTH, HEIGHT, TITLE);
    SetTargetFPS(GetMonitorRefreshRate(GetCurrentMonitor()));

    Shape shape = newShapeCircle(10, RAYWHITE);
    MotionProperties motion = newMotionProperties(INIT_POS, INIT_VEL, GRAVITY);
    PhysicalProperties physical = newPhysicalProperties(RESTITUTION);
    PhysicsBody circle = newPhysicsBody(shape, motion, physical);

    while (!WindowShouldClose())
    {
        float dt = GetFrameTime();

        BeginDrawing();

        ClearBackground(BLACK);

        updatePhysicsBody(&circle, dt);
        drawPhysicsBody(&circle);

        EndDrawing();
    }

    CloseWindow();

    return 0;
}