#include "Geometry.h"

void Point::translate(const Vector &v)
{
    x += v.x;
    y += v.y;
    z += v.z;
}

Vector::Vector(Point p1, Point p2)
{
    x = p2.x - p1.x;
    y = p2.y - p1.y;
    z = p2.z - p1.z;
}

reel Vector::norm()
{
    reel norm;

    norm = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));

    return norm;
}

Vector Vector::integral(reel delta_t)
{
    Vector res;

    res.x = delta_t * x;
    res.y = delta_t * y;
    res.z = delta_t * z;

    return res;
}

void Vector::operator+=(const Vector &v)
{
    x += v.x;
    y += v.y;
    z += v.z;
}

reel distance(Point p1, Point p2)
{
    Vector vect(p1, p2);

    return vect.norm();
}


Vector operator+(const Vector &v1, const Vector &v2)
{
    Vector res = v1;

    res.x += v2.x;
    res.y += v2.y;
    res.z += v2.z;

    return res;
}

Vector operator-(const Vector &v)
{
    Vector res;

    res.x = -v.x;
    res.y = -v.y;
    res.z = -v.z;

    return res;
}

Vector operator-(const Vector &v1, const Vector &v2)
{
    Vector res = v1;

    res.x -= v2.x;
    res.y -= v2.y;
    res.z -= v2.z;

    return res;
}

Vector operator*(const reel &k, const Vector &v)
{
    Vector res = v;

    res.x *= k;
    res.y *= k;
    res.z *= k;

    return res;
}

Point operator*(const reel &k, const Point &p)
{
    Point res = p;

    res.x *= k;
    res.y *= k;
    res.z *= k;

    return res;
}


// Scalar product
reel operator*(const Vector &v1, const Vector &v2)
{
    reel res;

    res = v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;

    return res;
}

// Vector product
Vector operator^(const Vector &v1, const Vector &v2)
{
    Vector res;

    res.x = v1.y * v2.z - v1.z * v2.y;
    res.y = v1.z * v2.x - v1.x * v2.z;
    res.z = v1.x * v2.y - v1.y * v2.x;

    return res;
}

Point operator+(const Vector &v, Point &p){
    Point res = v;
    res = res + p;
    return res;
}

Point operator+(Point &p, const Vector &v){
    Point res = v;
    res = res + p;
    return res;
}

Point operator-(const Vector &v, Point &p){
    Point res = v;
    res = res - p;
    return res;
}

Point operator-(Point &p, const Vector &v){
    Point res = v;
    res = res - p;
    return res;
}

// Point operator+(Point &p1, Point &p2) {
//     return Point(p1.x + p2.x, p1.y + p2.y, p1.z + p2.z);
// }
    
// Point operator-(Point &p1, Point &p2) {
//     return Point(p1.x - p2.x, p1.y - p2.y, p1.z - p2.z);
// }

void rotateVector(Vector& vec, Point degrees) {
    static const reel toRad = (M_PI / 180.0f);
    Vector vRot(degrees.x,degrees.y,degrees.z);
    // Convert angles from degrees to radians
    Vector rad = (toRad * vRot);
    // reel rad.x = degrees.x * M_PI / 180.0f;
    // reel rad.y = degrees.y * M_PI / 180.0f;
    // reel rad.z = degrees.z * M_PI / 180.0f;

    // Rotation matrix around X axis
    reel rotX[3][3] = {
        {1, 0, 0},
        {0, cos(rad.x), -sin(rad.x)},
        {0, sin(rad.x), cos(rad.x)}
    };

    // Rotation matrix around Y axis
    reel rotY[3][3] = {
        {cos(rad.y), 0, sin(rad.y)},
        {0, 1, 0},
        {-sin(rad.y), 0, cos(rad.y)}
    };

    // Rotation matrix around Z axis
    reel rotZ[3][3] = {
        {cos(rad.z), -sin(rad.z), 0},
        {sin(rad.z), cos(rad.z), 0},
        {0, 0, 1}
    };

    // Apply rotations in order: Z, Y, X
    vec = vec * rotZ;
    vec = vec * rotY;
    vec = vec * rotX;
}

