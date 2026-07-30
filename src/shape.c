#include "shape.h"

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
