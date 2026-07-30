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

void resolveVelocity(PhysicsBody *a, PhysicsBody *b, const Collision *collision)
{
    const float invMassA = a->physical.invMass;
    const float invMassB = b->physical.invMass;

    const float totalInvMass = invMassA + invMassB;
    if (totalInvMass == 0.0f)
        return;

    Vector2 relativeVelocity = Vector2Subtract(a->motion.vel, b->motion.vel);
    float relativeVelocityAlongNormal = Vector2DotProduct(relativeVelocity, collision->normal);

    // Already separating.
    if (relativeVelocityAlongNormal > 0.0f)
        return;

    float restitution = fminf(a->physical.restitution, b->physical.restitution);

    float impulseScalar = -(1.0f + restitution) * relativeVelocityAlongNormal / totalInvMass;
    Vector2 impulse = Vector2Scale(collision->normal, impulseScalar);

    a->motion.vel = Vector2Add(a->motion.vel, Vector2Scale(impulse, invMassA));
    b->motion.vel = Vector2Subtract(b->motion.vel, Vector2Scale(impulse, invMassB));
}

bool circlesColliding(const PhysicsBody *a, const PhysicsBody *b, Collision *collision)
{
    const MotionProperties *ma = &a->motion;
    const MotionProperties *mb = &b->motion;

    const struct circle_t *sa = &a->shape.data.circle;
    const struct circle_t *sb = &b->shape.data.circle;

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

bool boxesColliding(const PhysicsBody *a, const PhysicsBody *b, Collision *collision)
{
    const MotionProperties *ma = &a->motion;
    const MotionProperties *mb = &b->motion;

    const struct box_t *ba = &a->shape.data.box;
    const struct box_t *bb = &b->shape.data.box;

    Vector2 halfA = Vector2Scale(ba->size, 0.5f);
    Vector2 halfB = Vector2Scale(bb->size, 0.5f);

    Vector2 delta = Vector2Subtract(ma->pos, mb->pos);

    float overlapX = halfA.x + halfB.x - fabsf(delta.x);
    if (overlapX <= 0.0f)
        return false;

    float overlapY = halfA.y + halfB.y - fabsf(delta.y);
    if (overlapY <= 0.0f)
        return false;

    if (overlapX < overlapY)
    {
        *collision = (Collision){
            .normal = (delta.x >= 0.0f) ? (Vector2){1.0f, 0.0f} : (Vector2){-1.0f, 0.0f},
            .penetration = overlapX,
        };
    }
    else
    {
        *collision = (Collision){
            .normal = (delta.y >= 0.0f) ? (Vector2){0.0f, 1.0f} : (Vector2){0.0f, -1.0f},
            .penetration = overlapY,
        };
    }

    return true;
}

bool circleBoxColliding(const PhysicsBody *circle, const PhysicsBody *box, Collision *collision)
{
    const MotionProperties *mc = &circle->motion;
    const MotionProperties *mb = &box->motion;

    const struct circle_t *c = &circle->shape.data.circle;
    const struct box_t *b = &box->shape.data.box;

    Vector2 halfSize = Vector2Scale(b->size, 0.5f);

    float left = mb->pos.x - halfSize.x;
    float right = mb->pos.x + halfSize.x;
    float top = mb->pos.y - halfSize.y;
    float bottom = mb->pos.y + halfSize.y;

    Vector2 closest = {Clamp(mc->pos.x, left, right), Clamp(mc->pos.y, top, bottom)};

    Vector2 delta = Vector2Subtract(mc->pos, closest);
    float distSq = Vector2LengthSqr(delta);

    if (distSq > c->radius * c->radius)
        return false;

    // Circle center is outside the box.
    if (distSq > 0.0f)
    {
        float distance = sqrtf(distSq);

        *collision = (Collision){
            .normal = Vector2Scale(delta, 1.0f / distance),
            .penetration = c->radius - distance,
        };

        return true;
    }

    // Circle center is inside the box.
    float leftDist = mc->pos.x - left;
    float rightDist = right - mc->pos.x;
    float topDist = mc->pos.y - top;
    float bottomDist = bottom - mc->pos.y;

    float minDist = leftDist;
    Vector2 normal = {-1.0f, 0.0f};

    if (rightDist < minDist)
    {
        minDist = rightDist;
        normal = (Vector2){1.0f, 0.0f};
    }

    if (topDist < minDist)
    {
        minDist = topDist;
        normal = (Vector2){0.0f, -1.0f};
    }

    if (bottomDist < minDist)
    {
        minDist = bottomDist;
        normal = (Vector2){0.0f, 1.0f};
    }

    *collision = (Collision){
        .normal = normal,
        .penetration = c->radius + minDist,
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

        case SK_BOX:
            return circleBoxColliding(a, b, collision);

        default:
            return false;
        }

    case SK_BOX:
        switch (sb->sk)
        {
        case SK_CIRCLE:
        {
            bool result = circleBoxColliding(b, a, collision);
            collision->normal = Vector2Negate(collision->normal);
            return result;
        }

        case SK_BOX:
            return boxesColliding(a, b, collision);

        default:
            return false;
        }

    default:
        return false;
    }
}
