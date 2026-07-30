#ifndef SHAPE_H_
#define SHAPE_H_

#include <raylib.h>

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

Shape newShapeBase(ShapeKind sk, Color color);

Shape newShapeCircle(float radius, Color color);

#endif
