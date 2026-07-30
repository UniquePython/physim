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

    default:
        break;
    }

    return;
}

static void resolveWindowCollisions(const World *world, PhysicsBody *body)
{
    const Shape *s = &body->shape;
    MotionProperties *m = &body->motion;
    const PhysicalProperties *p = &body->physical;

    switch (s->sk)
    {
    case SK_CIRCLE:
    {
        const struct circle_t *c = &s->data.circle;

        const float left = m->pos.x - c->radius;
        const float right = m->pos.x + c->radius;
        const float top = m->pos.y - c->radius;
        const float bottom = m->pos.y + c->radius;

        if (left < 0)
        {
            m->pos.x = c->radius;
            m->vel.x *= -p->restitution;
        }

        if (right > world->width)
        {
            m->pos.x = world->width - c->radius;
            m->vel.x *= -p->restitution;
        }

        if (top < 0)
        {
            m->pos.y = c->radius;
            m->vel.y *= -p->restitution;
        }

        if (bottom > world->height)
        {
            m->pos.y = world->height - c->radius;
            m->vel.y *= -p->restitution;
        }

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

    resolveWindowCollisions(world, body);
}
