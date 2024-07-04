#ifndef GEOMETRY_H_INCLUDED
#define GEOMETRY_H_INCLUDED

#include <Arduino.h>

#ifndef M_PI
    #define M_PI		3.14159265358979323846
#endif

typedef float reel;

class Coordinates
{
public:
    reel x, y, z;
    Coordinates(reel xx=0, reel yy=0, reel zz=0) {x=xx; y=yy; z=zz;}
};


// Declaration in order to use it within Point methods
class Vector;

class Point : public Coordinates
{
public:
    // Point constructor calls the base class constructor and do nothing more
    Point(reel xx=0, reel yy=0, reel zz=0) : Coordinates(xx, yy, zz) {}
    void translate(const Vector &);

    Point operator+(Point& v) const {
        return Point(x + v.x, y + v.y, z + v.z);
    }

    Point operator-(Point& v) const {
        return Point(x - v.x, y - v.y, z - v.z);
    }
};

class Vector : public Coordinates
{
public:
    // Instantiates a Vector from its coordinates
    Vector(reel xx=0, reel yy=0, reel zz=0) : Coordinates(xx, yy, zz) {}
    // Or with two points
    Vector(Point, Point);
    // Compute the vector norm
    reel norm();
    
    Vector integral(reel delta_t);
    // Overloaded standard operators
    void operator+=(const Vector &v);

    operator Point() const {
        return Point(x, y, z);
    }

    Vector operator*(const reel matrix[3][3]) const {
        return Vector{
            x * matrix[0][0] + y * matrix[0][1] + z * matrix[0][2],
            x * matrix[1][0] + y * matrix[1][1] + z * matrix[1][2],
            x * matrix[2][0] + y * matrix[2][1] + z * matrix[2][2]
        };
    }

    
};


// Compute the distance between two points
reel distance(Point p1, Point p2);

// Overloaded standard operators
Vector operator+(const Vector &v1, const Vector &v2);
Vector operator-(const Vector &v);
Vector operator-(const Vector &v1, const Vector &v2);
Vector operator*(const reel &k, const Vector &v);
Point operator*(const reel &k, const Point &p);
// Scalar product
reel operator*(const Vector &v1, const Vector &v2);
// Vector product
Vector operator^(const Vector &v1, const Vector &v2);

Point operator+(const Vector &v, Point &p);
Point operator+(Point &p, const Vector &v);
Point operator-(const Vector &v, Point &p);
Point operator-(Point &p, const Vector &v);
// Point operator+(Point &p1, Point &p2);
// Point operator-(Point &p1, Point &p2);


void rotateVector(Vector& vec, Point degrees);


class Plan
{
private:
    Vector normal;
public:
    Vector v1, v2;
    Plan(){
        v1 = Vector(0,0,1);
        v2 = Vector(1,0,0);
        normal = v1 ^ v2;
    }
    Plan(Vector v1, Vector v2){
        v1 = v1;
        v2 = v2;
        normal = v1 ^ v2;
    }

    void rotateVectors(Point rot) {
        rotateVector(v1, rot);
        rotateVector(v2, rot);
        normal = v1 ^ v2;
    }

    void setVector(Vector v1, Vector v2){
        v1 = v1;
        v2 = v2;
        normal = v1 ^ v2;
    }

    Vector getNormal(){
        normal = v1 ^ v2;
        return normal;
    }

    Vector getNormal(Vector vOne, Vector vTwo){
        v1 = vOne;
        v2 = vTwo;
        normal = v1 ^ v2;
        return normal;
    }

};

#endif // GEOMETRY_H_INCLUDED