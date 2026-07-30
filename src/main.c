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
#define NCIRCLES 3
#define NBOXES 3
#define NSHAPES (NBOXES + NCIRCLES)
#define NBODIES (NSHAPES + NWALLS)

#define WALL_THICKNESS 25

PhysicsBody newWall(Vector2 size, Vector2 pos)
{
    Shape shape = newShapeBoxV(size, DARKGRAY);
    struct linear_t linear = newLinearMotionV(pos, Vector2Zero(), Vector2Zero());
    struct angular_t angular = newAngularMotion(0.0f, 0.0f, 0.0f);
    PhysicalProperties physical = newPhysicalProperties(&shape, INFINITY, 1.0f);
    return newPhysicsBody(shape, newMotionProperties(linear, angular), physical);
}

void initWalls(const World *world, PhysicsBody bodies[NBODIES])
{
    bodies[NSHAPES + 0] = newWall(
        (Vector2){world->width, WALL_THICKNESS},
        (Vector2){world->width * 0.5f, WALL_THICKNESS * 0.5f});

    bodies[NSHAPES + 1] = newWall(
        (Vector2){world->width, WALL_THICKNESS},
        (Vector2){world->width * 0.5f, world->height - WALL_THICKNESS * 0.5f});

    bodies[NSHAPES + 2] = newWall(
        (Vector2){WALL_THICKNESS, world->height},
        (Vector2){WALL_THICKNESS * 0.5f, world->height * 0.5f});

    bodies[NSHAPES + 3] = newWall(
        (Vector2){WALL_THICKNESS, world->height},
        (Vector2){world->width - WALL_THICKNESS * 0.5f, world->height * 0.5f});
}

void initBalls(PhysicsBody bodies[NBODIES])
{
    for (size_t i = NBOXES; i < NSHAPES; i++)
    {
        Shape shape = newShapeCircle(10, RAYWHITE);

        struct linear_t linear = newLinearMotion(WIDTH / 2, HEIGHT / 2, (float)(10 * i), (float)(10 * i), 0.0f, 0.0f);
        struct angular_t angular = newAngularMotion(0.0f, 0.0f, 0.0f);

        MotionProperties motion = newMotionProperties(linear, angular);
        PhysicalProperties physical = newPhysicalProperties(&shape, (float)(10 * (i + 1)), 0.8f);

        bodies[i] = newPhysicsBody(shape, motion, physical);
    }
}

void initBoxes(PhysicsBody bodies[NBODIES])
{
    for (size_t i = 0; i < NBOXES; i++)
    {
        Shape shape = newShapeBox(20, 20, RAYWHITE);

        struct linear_t linear = newLinearMotion(WIDTH / 2, HEIGHT / 2, (float)(20 * i), (float)(20 * i), 0.0f, 0.0f);
        struct angular_t angular = newAngularMotion((float)(5 * i), (float)(5 * i), 0.0f);

        MotionProperties motion = newMotionProperties(linear, angular);
        PhysicalProperties physical = newPhysicalProperties(&shape, (float)(10 * (i + 1)), 0.8f);

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
    initBoxes(bodies);

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
