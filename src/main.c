#include <stdbool.h>
#include <stddef.h>
#include <math.h>

#include "raylib_all.h"
#include "shape.h"
#include "properties.h"
#include "physics_body.h"
#include "world.h"
#include "collision.h"

#define WIDTH 900
#define HEIGHT 600
#define TITLE "Physim - Simple Physics Simulator"

#define GRAVITY 0, 500

#define NWALLS 4
#define NCIRCLES 6
#define NBODIES (NCIRCLES + NWALLS)

#define WALL_THICKNESS 25

PhysicsBody newWall(Vector2 size, Vector2 pos)
{
    return newPhysicsBody(
        newShapeBoxV(size, DARKGRAY),
        newMotionPropertiesV(pos, Vector2Zero(), Vector2Zero()),
        newPhysicalProperties(INFINITY, 1.0f));
}

void initWalls(const World *world, PhysicsBody bodies[NBODIES])
{
    bodies[NCIRCLES + 0] = newWall(
        (Vector2){world->width, WALL_THICKNESS},
        (Vector2){world->width * 0.5f, WALL_THICKNESS * 0.5f});

    bodies[NCIRCLES + 1] = newWall(
        (Vector2){world->width, WALL_THICKNESS},
        (Vector2){world->width * 0.5f, world->height - WALL_THICKNESS * 0.5f});

    bodies[NCIRCLES + 2] = newWall(
        (Vector2){WALL_THICKNESS, world->height},
        (Vector2){WALL_THICKNESS * 0.5f, world->height * 0.5f});

    bodies[NCIRCLES + 3] = newWall(
        (Vector2){WALL_THICKNESS, world->height},
        (Vector2){world->width - WALL_THICKNESS * 0.5f, world->height * 0.5f});
}

void initBalls(PhysicsBody bodies[NBODIES])
{
    for (size_t i = 0; i < NCIRCLES; i++)
    {
        Shape shape = newShapeCircle(10, RAYWHITE);

        MotionProperties motion = newMotionProperties(
            (float)(i + WALL_THICKNESS), (float)(i + WALL_THICKNESS),
            (float)(20 * i), (float)(20 * i),
            0.0f, 0.0f);
        PhysicalProperties physical = newPhysicalProperties((float)(10 * (i + 1)), 0.8f);

        bodies[i] = newPhysicsBody(shape, motion, physical);
    }
}

void updatePhysicsBodies(const World *world, PhysicsBody bodies[NBODIES], float dt)
{
    for (size_t i = 0; i < NBODIES; i++)
        if (!bodies[i].physical.isStatic)
            updatePhysicsBody(world, &bodies[i], dt);
}

void handleCollisions(PhysicsBody bodies[NBODIES])
{
    for (size_t i = 0; i < NBODIES; i++)
    {
        for (size_t j = i + 1; j < NBODIES; j++)
        {
            Collision collision;

            PhysicsBody *a = &bodies[i];
            PhysicsBody *b = &bodies[j];

            if (bodiesColliding(a, b, &collision))
            {
                resolveOverlap(a, b, &collision);
                resolveVelocity(a, b, &collision);
            }
        }
    }
}

void drawPhysicsBodies(const PhysicsBody bodies[NBODIES])
{
    for (size_t i = 0; i < NBODIES; i++)
        drawPhysicsBody(&bodies[i]);
}

int main(void)
{
    InitWindow(WIDTH, HEIGHT, TITLE);
    SetTargetFPS(GetMonitorRefreshRate(GetCurrentMonitor()));

    World world = newWorld(WIDTH, HEIGHT, GRAVITY);

    PhysicsBody bodies[NBODIES] = {0};

    initWalls(&world, bodies);
    initBalls(bodies);

    while (!WindowShouldClose())
    {
        float dt = GetFrameTime();

        BeginDrawing();

        ClearBackground(BLACK);

        updatePhysicsBodies(&world, bodies, dt);
        handleCollisions(bodies);
        drawPhysicsBodies(bodies);

        EndDrawing();
    }

    CloseWindow();

    return 0;
}