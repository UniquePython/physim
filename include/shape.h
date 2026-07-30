#ifndef SHAPE_H_
#define SHAPE_H_

#include "raylib_all.h"

typedef enum sk_t
{
    SK_CIRCLE,
    SK_BOX,
} ShapeKind;

typedef struct shape_t
{
    ShapeKind sk;

    union data_t
    {
        struct circle_t
        {
            float radius;
        } circle;

        struct box_t
        {
            union
            {
                struct
                {
                    float width;
                    float height;
                };
                Vector2 size;
            };
        } box;

    } data;

    // For all shapes
    Color color;
} Shape;

Shape newShapeBase(ShapeKind sk, Color color);

Shape newShapeCircle(float radius, Color color);

#endif
