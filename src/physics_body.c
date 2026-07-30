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
    const struct linear_t *l = &body->motion.linear;
    const struct angular_t *a = &body->motion.angular;

    switch (s->sk)
    {
    case SK_CIRCLE:
    {
        const struct circle_t *c = &s->data.circle;
        DrawCircleV(l->pos, c->radius, s->color);
        break;
    }

    case SK_BOX:
    {
        const struct box_t *b = &s->data.box;

        DrawRectanglePro(
            (Rectangle){
                l->pos.x,
                l->pos.y,
                b->width,
                b->height,
            },
            Vector2Scale(b->size, 0.5f),
            a->angle * RAD2DEG,
            s->color);
        break;
    }

    default:
        break;
    }
}

void updatePhysicsBody(const World *world, PhysicsBody *body, float dt)
{
    struct linear_t *l = &body->motion.linear;
    struct angular_t *a = &body->motion.angular;

    // Linear
    l->acc = (Vector2){0, 0};
    l->acc = Vector2Add(l->acc, world->gravity);

    l->vel = Vector2Add(l->vel, Vector2Scale(l->acc, dt));
    l->pos = Vector2Add(l->pos, Vector2Scale(l->vel, dt));

    // Angular
    a->acc = 0.0f;

    a->vel += a->acc * dt;
    a->angle += a->vel * dt;
}
