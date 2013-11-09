
#include <cmath>

#include "Vector2D.hpp"

const Vector2D Vector2D::ORIGIN = Vector2D(0.0,0.0);
const Vector2D Vector2D::E1 = Vector2D(1.0,0.0);
const Vector2D Vector2D::E2 = Vector2D(0.0,1.0);

double Vector2D::x() const
{
    return _x;
}

double Vector2D::y() const
{
    return _y;
}

double Vector2D::dot(const Vector2D & other) const 
{
    return (_x*other._x + _y*other._y);
}

double Vector2D::length() const
{
    return std::sqrt(_x*_x + _y*_y);
}

double Vector2D::cross(const Vector2D & other) const
{
    return _x*other._y - other._x*_y;
}

bool Vector2D::isLeft(const Vector2D & startOfLine, const Vector2D & endOfLine) const
{
    Vector2D lineDir = endOfLine.sub(startOfLine);
    Vector2D pointDir = this->sub(startOfLine);

    return (lineDir.cross(pointDir) > 0.0);
}

Vector2D Vector2D::mult(double k) const
{
    return Vector2D(k*_x,k*_y);
}

Vector2D Vector2D::add(const Vector2D & other) const
{
    return Vector2D(_x + other._x, _y + other._y);
}

Vector2D Vector2D::sub(const Vector2D & other) const
{
    return Vector2D(_x - other._x, _y - other._y);
}

Vector2D Vector2D::neg() const
{
    return Vector2D(-_x,-_y);
}

Vector2D Vector2D::normalize() const
{
    double l = length();

    return Vector2D(_x/l,_y/l);
}

Vector2D Vector2D::rotate(double theta) const
{
    double c = std::cos(theta);
    double s = std::sin(theta);

    return Vector2D(_x*c - _x*s,_y*s + _y*c);
}

Vector2D Vector2D::rotate(const Vector2D & center, double theta) const
{
    Vector2D translated = sub(center);
    Vector2D rotated = translated.rotate(theta);

    return rotated.add(center);
}

std::ostream & operator<<(std::ostream & os, const Vector2D & v)
{
    os << "(" << v.x() << "," << v.y() << ")";

    return os;
}
