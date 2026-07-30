#include "collision.h"

#include <math.h>

typedef struct boxcorners_t
{
    Vector2 points[4];
} BoxCorners;

BoxCorners getBoxCorners(const PhysicsBody *body)
{
    const struct linear_t *l = &body->motion.linear;
    const struct angular_t *a = &body->motion.angular;
    const struct box_t *b = &body->shape.data.box;

    float hx = b->width * 0.5f;
    float hy = b->height * 0.5f;

    float c = cosf(a->angle);
    float s = sinf(a->angle);

    Vector2 local[4] = {
        {-hx, -hy},
        {hx, -hy},
        {hx, hy},
        {-hx, hy},
    };

    BoxCorners corners;

    for (int i = 0; i < 4; i++)
    {
        float x = local[i].x;
        float y = local[i].y;

        corners.points[i] = (Vector2){
            .x = l->pos.x + x * c - y * s,
            .y = l->pos.y + x * s + y * c,
        };
    }

    return corners;
}

typedef struct projection_t
{
    float min;
    float max;
} Projection;

Projection projectBox(BoxCorners corners, Vector2 axis)
{
    float p = Vector2DotProduct(corners.points[0], axis);

    Projection proj = {
        .min = p,
        .max = p,
    };

    for (int i = 1; i < 4; i++)
    {
        p = Vector2DotProduct(corners.points[i], axis);

        if (p < proj.min)
            proj.min = p;

        if (p > proj.max)
            proj.max = p;
    }

    return proj;
}

float projectionOverlap(Projection a, Projection b)
{
    return fminf(a.max, b.max) - fmaxf(a.min, b.min);
}

static inline Vector2 ScalarCrossVector(float s, Vector2 v)
{
    // Positive angular velocity = clockwise in screen-space (y-down).
    return (Vector2){-s * v.y, s * v.x};
}

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

    struct angular_t *aa = &a->motion.angular;
    struct angular_t *ab = &b->motion.angular;

    const PhysicalProperties *pa = &a->physical;
    const PhysicalProperties *pb = &b->physical;

    Vector2 rA = Vector2Subtract(collision->contactPoint, la->pos);
    Vector2 rB = Vector2Subtract(collision->contactPoint, lb->pos);

    Vector2 vA = Vector2Add(la->vel, ScalarCrossVector(aa->vel, rA));
    Vector2 vB = Vector2Add(lb->vel, ScalarCrossVector(ab->vel, rB));

    Vector2 relativeVelocity = Vector2Subtract(vA, vB);
    float relativeVelocityAlongNormal = Vector2DotProduct(relativeVelocity, collision->normal);

    if (relativeVelocityAlongNormal > 0.0f)
        return;

    float e = fminf(pa->restitution, pb->restitution);

    float rAcrossN = Vector2CrossProduct(rA, collision->normal);
    float rBcrossN = Vector2CrossProduct(rB, collision->normal);

    float denominator = pa->invMass + pb->invMass +
                        pa->invInertia * rAcrossN * rAcrossN +
                        pb->invInertia * rBcrossN * rBcrossN;

    if (denominator == 0.0f)
        return;

    float j = -(1.0f + e) * relativeVelocityAlongNormal / denominator;

    Vector2 impulse = Vector2Scale(collision->normal, j);

    la->vel = Vector2Add(la->vel, Vector2Scale(impulse, pa->invMass));
    lb->vel = Vector2Subtract(lb->vel, Vector2Scale(impulse, pb->invMass));

    aa->vel += pa->invInertia * Vector2CrossProduct(rA, impulse);
    ab->vel -= pb->invInertia * Vector2CrossProduct(rB, impulse);
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

static Vector2 supportPoint(BoxCorners corners, Vector2 direction)
{
    Vector2 best = corners.points[0];
    float bestDot = Vector2DotProduct(best, direction);

    for (int i = 1; i < 4; i++)
    {
        float d = Vector2DotProduct(corners.points[i], direction);

        if (d > bestDot)
        {
            bestDot = d;
            best = corners.points[i];
        }
    }

    return best;
}

bool boxesColliding(const PhysicsBody *a, const PhysicsBody *b, Collision *collision)
{
    const struct linear_t *la = &a->motion.linear;
    const struct linear_t *lb = &b->motion.linear;

    const struct angular_t *aa = &a->motion.angular;
    const struct angular_t *ab = &b->motion.angular;

    BoxCorners cornersA = getBoxCorners(a);
    BoxCorners cornersB = getBoxCorners(b);

    float ca = cosf(aa->angle);
    float sa = sinf(aa->angle);

    float cb = cosf(ab->angle);
    float sb = sinf(ab->angle);

    Vector2 axes[4] = {
        {ca, sa},
        {-sa, ca},
        {cb, sb},
        {-sb, cb},
    };

    float bestOverlap = INFINITY;
    Vector2 bestAxis = Vector2Zero();

    for (int i = 0; i < 4; i++)
    {
        Projection pa = projectBox(cornersA, axes[i]);
        Projection pb = projectBox(cornersB, axes[i]);

        float overlap = projectionOverlap(pa, pb);

        if (overlap <= 0.0f)
            return false;

        if (overlap < bestOverlap)
        {
            bestOverlap = overlap;
            bestAxis = axes[i];
        }
    }

    Vector2 delta = Vector2Subtract(la->pos, lb->pos);

    if (Vector2DotProduct(delta, bestAxis) < 0.0f)
        bestAxis = Vector2Negate(bestAxis);

    Vector2 pointA = supportPoint(cornersA, Vector2Negate(bestAxis));
    Vector2 pointB = supportPoint(cornersB, bestAxis);

    *collision = (Collision){
        .normal = bestAxis,
        .penetration = bestOverlap,
        .contactPoint = Vector2Scale(Vector2Add(pointA, pointB), 0.5f),
    };

    return true;
}

// bool boxesColliding(const PhysicsBody *a, const PhysicsBody *b, Collision *collision)
// {
//     const struct linear_t *la = &a->motion.linear;
//     const struct linear_t *lb = &b->motion.linear;

//     const struct box_t *ba = &a->shape.data.box;
//     const struct box_t *bb = &b->shape.data.box;

//     Vector2 halfA = Vector2Scale(ba->size, 0.5f);
//     Vector2 halfB = Vector2Scale(bb->size, 0.5f);

//     Vector2 delta = Vector2Subtract(la->pos, lb->pos);

//     float overlapX = halfA.x + halfB.x - fabsf(delta.x);
//     if (overlapX <= 0.0f)
//         return false;

//     float overlapY = halfA.y + halfB.y - fabsf(delta.y);
//     if (overlapY <= 0.0f)
//         return false;

//     if (overlapX < overlapY)
//     {
//         float x = (delta.x >= 0.0f) ? la->pos.x - halfA.x : la->pos.x + halfA.x;

//         float y = Clamp(
//             0.5f * (la->pos.y + lb->pos.y),
//             fmaxf(la->pos.y - halfA.y, lb->pos.y - halfB.y),
//             fminf(la->pos.y + halfA.y, lb->pos.y + halfB.y));

//         *collision = (Collision){
//             .normal = (delta.x >= 0.0f) ? (Vector2){1.0f, 0.0f} : (Vector2){-1.0f, 0.0f},
//             .penetration = overlapX,
//             .contactPoint = (Vector2){x, y},
//         };
//     }
//     else
//     {
//         float y = (delta.y >= 0.0f) ? la->pos.y - halfA.y : la->pos.y + halfA.y;

//         float x = Clamp(
//             0.5f * (la->pos.x + lb->pos.x),
//             fmaxf(la->pos.x - halfA.x, lb->pos.x - halfB.x),
//             fminf(la->pos.x + halfA.x, lb->pos.x + halfB.x));

//         *collision = (Collision){
//             .normal = (delta.y >= 0.0f) ? (Vector2){0.0f, 1.0f} : (Vector2){0.0f, -1.0f},
//             .penetration = overlapY,
//             .contactPoint = (Vector2){x, y},
//         };
//     }

//     return true;
// }

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
