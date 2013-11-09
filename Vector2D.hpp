
#pragma once

#include <iostream>

class Vector2D
{
    private:
        double _x;
        double _y;

    public:
        static const Vector2D ORIGIN;
        static const Vector2D E1;
        static const Vector2D E2;

        Vector2D(double x = 0.0, double y = 0.0) : _x(x), _y(y){} 
        ~Vector2D(){}

        double x() const;
        double y() const;

        double dot(const Vector2D & other) const;
        double length() const;
        double cross(const Vector2D & other) const;
        bool isLeft(const Vector2D & startOfLine, const Vector2D & endOfLine) const;
        Vector2D mult(double k) const;
        Vector2D add(const Vector2D & other) const;
        Vector2D sub(const Vector2D & other) const;
        Vector2D neg() const;
        Vector2D normalize() const;
        Vector2D rotate(double theta) const;
        Vector2D rotate(const Vector2D & center, double theta) const;
};


std::ostream & operator<<(std::ostream & os, const Vector2D & v);

