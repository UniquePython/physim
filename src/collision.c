#include "collision.h"

bool circlesColliding(const PhysicsBody *a, const PhysicsBody *b)
{
    const MotionProperties *ma = &a->motion;
    const MotionProperties *mb = &b->motion;

    const Shape *sa = &a->shape;
    const Shape *sb = &b->shape;

    Vector2 delta = Vector2Subtract(ma->pos, mb->pos);
    float radiusSum = sa->radius + sb->radius;

    return Vector2LengthSqr(delta) <= radiusSum * radiusSum;
}

bool bodiesColliding(const PhysicsBody *a, const PhysicsBody *b)
{
    const Shape *sa = &a->shape;
    const Shape *sb = &b->shape;

    switch (sa->sk)
    {
    case SK_CIRCLE:
        switch (sb->sk)
        {
        case SK_CIRCLE:
            return circlesColliding(a, b);

        default:
            return false;
        }

    default:
        return false;
    }
}
