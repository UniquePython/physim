#include <raylib.h>
#include <raymath.h>

#include <stdbool.h>

#define WIDTH 900
#define HEIGHT 600
#define TITLE "Physim - Simple Physics Simulator"

typedef enum sk_t
{
    SK_CIRCLE,
} ShapeKind;

typedef struct shape_t
{
    ShapeKind sk;

    // For circle
    float radius;

    // For all shapes
    Color color;
} Shape;

Shape newShapeBase(ShapeKind sk, Color color)
{
    return (Shape){
        .sk = sk,
        .color = color,
    };
}

Shape newShapeCircle(float radius, Color color)
{
    Shape circle = newShapeBase(SK_CIRCLE, color);
    circle.radius = radius;
    return circle;
}

typedef struct motionprops_t
{
    Vector2 pos;
    Vector2 vel;
    Vector2 acc;
} MotionProperties;

MotionProperties newMotionProperties(float px, float py, float vx, float vy, float ax, float ay)
{
    return (MotionProperties){
        .pos.x = px,
        .pos.y = py,
        .vel.x = vx,
        .vel.y = vy,
        .acc.x = ax,
        .acc.y = ay,
    };
}

MotionProperties newMotionPropertiesV(Vector2 pos, Vector2 vel, Vector2 acc)
{
    return (MotionProperties){
        .pos = pos,
        .vel = vel,
        .acc = acc,
    };
}

typedef struct phyprops_t
{
    float restitution;
} PhysicalProperties;

PhysicalProperties newPhysicalProperties(float restitution)
{
    return (PhysicalProperties){
        .restitution = restitution,
    };
}

typedef struct phybody_t
{
    Shape shape;
    MotionProperties motion;
    PhysicalProperties physical;
} PhysicsBody;

PhysicsBody newPhysicsBody(Shape shape, MotionProperties motion, PhysicalProperties physical)
{
    return (PhysicsBody){
        .shape = shape,
        .motion = motion,
        .physical = physical,
    };
}

void drawPhysicsBody(const PhysicsBody *body)
{
    const Shape *s = &body->shape;
    const MotionProperties *m = &body->motion;

    switch (s->sk)
    {
    case SK_CIRCLE:
        DrawCircleV(m->pos, s->radius, s->color);
        break;

    default:
        break;
    }

    return;
}

void resolveWindowCollisions(PhysicsBody *body)
{
    const Shape *s = &body->shape;
    MotionProperties *m = &body->motion;
    const PhysicalProperties *p = &body->physical;

    switch (s->sk)
    {
    case SK_CIRCLE:
    {
        const float left = m->pos.x - s->radius;
        const float right = m->pos.x + s->radius;
        const float top = m->pos.y - s->radius;
        const float bottom = m->pos.y + s->radius;

        if (left < 0)
        {
            m->pos.x = s->radius;
            m->vel.x *= -p->restitution;
        }

        if (right > WIDTH)
        {
            m->pos.x = WIDTH - s->radius;
            m->vel.x *= -p->restitution;
        }

        if (top < 0)
        {
            m->pos.y = s->radius;
            m->vel.y *= -p->restitution;
        }

        if (bottom > HEIGHT)
        {
            m->pos.y = HEIGHT - s->radius;
            m->vel.y *= -p->restitution;
        }

        break;
    }

    default:
        break;
    }

    return;
}

void updatePhysicsBody(PhysicsBody *body, float dt)
{
    MotionProperties *m = &body->motion;

    m->vel = Vector2Add(m->vel, Vector2Scale(m->acc, dt));
    m->pos = Vector2Add(m->pos, Vector2Scale(m->vel, dt));

    resolveWindowCollisions(body);
}

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