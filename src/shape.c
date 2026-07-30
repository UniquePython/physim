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
    Shape shape = newShapeBase(SK_CIRCLE, color);
    shape.data.circle.radius = radius;
    return shape;
}

Shape newShapeBox(float width, float height, Color color)
{
    Shape shape = newShapeBase(SK_BOX, color);
    shape.data.box.width = width;
    shape.data.box.height = height;
    return shape;
}

Shape newShapeBoxV(Vector2 dimensions, Color color)
{
    Shape shape = newShapeBase(SK_BOX, color);
    shape.data.box.dimensions = dimensions;
    return shape;
}
