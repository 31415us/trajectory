
#pragma once

#include <vector>

#include "Vector2D.hpp"

class Polygon;

class Circle
{
    private:
        Vector2D _center;
        double _r;

    public:

        Circle(Vector2D c, double r) : _center(c), _r(r){}
        Circle(double x = 0.0, double y = 0.0, double r = 1.0) : _center(Vector2D(x,y)), _r(r){}
        ~Circle(){}

        Vector2D center() const;
        double r() const;

        Circle translate(const Vector2D & trans) const;
        
        bool collidesWith(const Circle & other) const;
        bool collidesWith(const Polygon & poly) const;
        
        bool completelyInside(const Polygon & poly) const;

        bool contains(const Vector2D & p) const;
};

class Polygon
{
    private:
        std::vector<Vector2D> _vertices;

    public:
        Polygon(std::vector<Vector2D> vert = std::vector<Vector2D>()) : _vertices(vert){}
        ~Polygon(){}

        std::vector<Vector2D> vertices() const;

        Polygon translate(const Vector2D & trans) const;
        Polygon rotate(const Vector2D & center, double theta) const;

        bool collidesWith(const Circle & circ) const;
        bool collidesWith(const Polygon & other) const;

        bool completelyContains(const Circle & circ) const;

        bool contains(const Vector2D & p) const;
};

