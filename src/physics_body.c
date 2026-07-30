#include "physics_body.h"

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
    {
        const struct circle_t *c = &s->data.circle;
        DrawCircleV(m->pos, c->radius, s->color);
        break;
    }

    case SK_BOX:
    {
        const struct box_t *b = &s->data.box;
        Vector2 half = Vector2Scale(b->size, 0.5f);
        DrawRectangleV(Vector2Subtract(body->motion.pos, half), b->size, body->shape.color);
        break;
    }

    default:
        break;
    }

    return;
}

void updatePhysicsBody(const World *world, PhysicsBody *body, float dt)
{
    MotionProperties *m = &body->motion;

    m->acc = (Vector2){0, 0};
    m->acc = Vector2Add(m->acc, world->gravity);

    m->vel = Vector2Add(m->vel, Vector2Scale(m->acc, dt));
    m->pos = Vector2Add(m->pos, Vector2Scale(m->vel, dt));
}
