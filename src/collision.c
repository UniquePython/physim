#include "collision.h"

#include <math.h>

void resolveOverlap(PhysicsBody *a, PhysicsBody *b, const Collision *collision)
{
    const float invMassA = a->physical.invMass;
    const float invMassB = b->physical.invMass;

    const float totalInvMass = invMassA + invMassB;

    if (totalInvMass == 0.0f)
        return;

    Vector2 correctionA = Vector2Scale(collision->normal, collision->penetration * (invMassA / totalInvMass));
    Vector2 correctionB = Vector2Scale(collision->normal, -collision->penetration * (invMassB / totalInvMass));

    a->motion.pos = Vector2Add(a->motion.pos, correctionA);
    b->motion.pos = Vector2Add(b->motion.pos, correctionB);
}

bool circlesColliding(const PhysicsBody *a, const PhysicsBody *b, Collision *collision)
{
    const MotionProperties *ma = &a->motion;
    const MotionProperties *mb = &b->motion;

    const Shape *sa = &a->shape;
    const Shape *sb = &b->shape;

    Vector2 delta = Vector2Subtract(ma->pos, mb->pos);
    float distSq = Vector2LengthSqr(delta);
    float radiusSum = sa->radius + sb->radius;

    if (distSq > radiusSum * radiusSum)
        return false;

    float distance = sqrtf(distSq);

    Vector2 normal;
    if (distance == 0.0f)
        normal = (Vector2){0.0f, -1.0f};
    else
        normal = Vector2Scale(delta, 1.0f / distance);

    *collision = (Collision){
        .normal = normal,
        .penetration = radiusSum - distance,
    };

    return true;
}

bool bodiesColliding(const PhysicsBody *a, const PhysicsBody *b, Collision *collision)
{
    const Shape *sa = &a->shape;
    const Shape *sb = &b->shape;

    switch (sa->sk)
    {
    case SK_CIRCLE:
        switch (sb->sk)
        {
        case SK_CIRCLE:
            return circlesColliding(a, b, collision);

        default:
            return false;
        }

    default:
        return false;
    }
}
