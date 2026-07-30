#include "collision.h"

#include <math.h>

void resolveOverlap(PhysicsBody *a, PhysicsBody *b, const Collision *collision)
{
    struct linear_t *la = &a->motion.linear;
    struct linear_t *lb = &b->motion.linear;

    const float invMassA = a->physical.invMass;
    const float invMassB = b->physical.invMass;

    const float totalInvMass = invMassA + invMassB;

    if (totalInvMass == 0.0f)
        return;

    Vector2 correctionA = Vector2Scale(collision->normal, collision->penetration * (invMassA / totalInvMass));
    Vector2 correctionB = Vector2Scale(collision->normal, -collision->penetration * (invMassB / totalInvMass));

    la->pos = Vector2Add(la->pos, correctionA);
    lb->pos = Vector2Add(lb->pos, correctionB);
}

void resolveVelocity(PhysicsBody *a, PhysicsBody *b, const Collision *collision)
{
    struct linear_t *la = &a->motion.linear;
    struct linear_t *lb = &b->motion.linear;

    const float invMassA = a->physical.invMass;
    const float invMassB = b->physical.invMass;

    const float totalInvMass = invMassA + invMassB;
    if (totalInvMass == 0.0f)
        return;

    Vector2 relativeVelocity = Vector2Subtract(la->vel, lb->vel);
    float relativeVelocityAlongNormal = Vector2DotProduct(relativeVelocity, collision->normal);

    // Already separating.
    if (relativeVelocityAlongNormal > 0.0f)
        return;

    float restitution = fminf(a->physical.restitution, b->physical.restitution);

    float impulseScalar = -(1.0f + restitution) * relativeVelocityAlongNormal / totalInvMass;
    Vector2 impulse = Vector2Scale(collision->normal, impulseScalar);

    la->vel = Vector2Add(la->vel, Vector2Scale(impulse, invMassA));
    lb->vel = Vector2Subtract(lb->vel, Vector2Scale(impulse, invMassB));
}

bool circlesColliding(const PhysicsBody *a, const PhysicsBody *b, Collision *collision)
{
    const struct linear_t *la = &a->motion.linear;
    const struct linear_t *lb = &b->motion.linear;

    const struct circle_t *sa = &a->shape.data.circle;
    const struct circle_t *sb = &b->shape.data.circle;

    Vector2 delta = Vector2Subtract(la->pos, lb->pos);
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
        .contactPoint = Vector2Subtract(la->pos, Vector2Scale(normal, sa->radius)),
    };

    return true;
}

bool boxesColliding(const PhysicsBody *a, const PhysicsBody *b, Collision *collision)
{
    const struct linear_t *la = &a->motion.linear;
    const struct linear_t *lb = &b->motion.linear;

    const struct box_t *ba = &a->shape.data.box;
    const struct box_t *bb = &b->shape.data.box;

    Vector2 halfA = Vector2Scale(ba->size, 0.5f);
    Vector2 halfB = Vector2Scale(bb->size, 0.5f);

    Vector2 delta = Vector2Subtract(la->pos, lb->pos);

    float overlapX = halfA.x + halfB.x - fabsf(delta.x);
    if (overlapX <= 0.0f)
        return false;

    float overlapY = halfA.y + halfB.y - fabsf(delta.y);
    if (overlapY <= 0.0f)
        return false;

    if (overlapX < overlapY)
    {
        float x = (delta.x >= 0.0f) ? la->pos.x - halfA.x : la->pos.x + halfA.x;

        float y = Clamp(
            0.5f * (la->pos.y + lb->pos.y),
            fmaxf(la->pos.y - halfA.y, lb->pos.y - halfB.y),
            fminf(la->pos.y + halfA.y, lb->pos.y + halfB.y));

        *collision = (Collision){
            .normal = (delta.x >= 0.0f) ? (Vector2){1.0f, 0.0f} : (Vector2){-1.0f, 0.0f},
            .penetration = overlapX,
            .contactPoint = (Vector2){x, y},
        };
    }
    else
    {
        float y = (delta.y >= 0.0f) ? la->pos.y - halfA.y : la->pos.y + halfA.y;

        float x = Clamp(
            0.5f * (la->pos.x + lb->pos.x),
            fmaxf(la->pos.x - halfA.x, lb->pos.x - halfB.x),
            fminf(la->pos.x + halfA.x, lb->pos.x + halfB.x));

        *collision = (Collision){
            .normal = (delta.y >= 0.0f) ? (Vector2){0.0f, 1.0f} : (Vector2){0.0f, -1.0f},
            .penetration = overlapY,
            .contactPoint = (Vector2){x, y},
        };
    }

    return true;
}

bool circleBoxColliding(const PhysicsBody *circle, const PhysicsBody *box, Collision *collision)
{
    const struct linear_t *lc = &circle->motion.linear;
    const struct linear_t *lb = &box->motion.linear;

    const struct circle_t *c = &circle->shape.data.circle;
    const struct box_t *b = &box->shape.data.box;

    Vector2 halfSize = Vector2Scale(b->size, 0.5f);

    float left = lb->pos.x - halfSize.x;
    float right = lb->pos.x + halfSize.x;
    float top = lb->pos.y - halfSize.y;
    float bottom = lb->pos.y + halfSize.y;

    Vector2 closest = {Clamp(lc->pos.x, left, right), Clamp(lc->pos.y, top, bottom)};

    Vector2 delta = Vector2Subtract(lc->pos, closest);
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
            .contactPoint = closest,
        };

        return true;
    }

    // Circle center is inside the box.
    float leftDist = lc->pos.x - left;
    float rightDist = right - lc->pos.x;
    float topDist = lc->pos.y - top;
    float bottomDist = bottom - lc->pos.y;

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

    Vector2 contact = lc->pos;

    if (normal.x < 0.0f)
        contact.x = left;
    else if (normal.x > 0.0f)
        contact.x = right;
    else if (normal.y < 0.0f)
        contact.y = top;
    else
        contact.y = bottom;

    *collision = (Collision){
        .normal = normal,
        .penetration = c->radius + minDist,
        .contactPoint = contact,
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
