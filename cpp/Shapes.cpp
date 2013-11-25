
#include <cmath>
#include <iostream>

#include "Shapes.hpp"

std::vector<Vector2D> Polygon::vertices() const
{
    return _vertices;
}

Polygon Polygon::translate(const Vector2D & trans) const
{
    std::vector<Vector2D> newVertices;

    for(auto v : _vertices)
    {
        newVertices.push_back(v.add(trans));
    }

    return Polygon(newVertices);
}

Polygon Polygon::rotate(const Vector2D & center, double theta) const
{
    std::vector<Vector2D> nVert;

    for(auto v : _vertices)
    {
        nVert.push_back(v.rotate(center,theta));
    }

    return Polygon(nVert);
}

double getQ(double discr, double b)
{
    double root = std::sqrt(discr);

    if(b < 0.0)
    {
        return (root - b)/2.0;
    }
    else
    {
        return (-root - b)/2.0;
    }
}

std::vector<double> quadraticEquation(double a, double b, double c)
{
    std::vector<double> res;

    double discr = (b*b) - (4*a*c);

    if(discr >= 0.0)
    {
        double q = getQ(discr,b);

        res.push_back(q/a);
        res.push_back(c/q);
    }

    return res;
}

bool segmentCircle(Vector2D A, Vector2D B, Circle circ)
{
    Vector2D ab = B.sub(A);
    Vector2D ma = A.sub(circ.center());
    double r = circ.r();

    double a = ab.dot(ab);
    double b = 2 * ma.dot(ab);
    double c = ma.dot(ma) - r*r;

    auto ts = quadraticEquation(a,b,c);

    for(auto t : ts)
    {
        if(t <= 1.0 && t >= 0.0)
        {
            return true;
        }
    }

    return false;
}

bool Polygon::collidesWith(const Circle & circ) const
{
    // test if circle is inside polygon
    if(contains(circ.center()))
    {
        return true;
    }

    // test if any polygon edge intersects circle
    for(size_t i = 1; i < _vertices.size(); i++)
    {
        auto a = _vertices[i-1];
        auto b = _vertices[i];

        if(segmentCircle(a,b,circ))
        {
            return true;
        }
    }

    if(_vertices.size() > 0)
    {
        auto a = _vertices.back();
        auto b = _vertices.front();

        if(segmentCircle(a,b,circ))
        {
            return true;
        }
    }

    // necessary test if polygon is entirely inside circle
    for(auto v : _vertices)
    {
        if(circ.contains(v))
        {
            return true;
        }
    }

    return false;
}

//http://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
bool lineSegmentIntersection(const Vector2D & start1, const Vector2D & end1, const Vector2D & start2, const Vector2D & end2) 
{
    Vector2D p = start1;
    Vector2D r = end1.sub(start1);
    Vector2D q = start2;
    Vector2D s = end2.sub(start2);

    double rCrossS =r.cross(s);

    Vector2D qSubp = q.sub(p);

    double qSubpCrossR = qSubp.cross(s);

    if(rCrossS == 0.0)
    {
        return (qSubpCrossR == 0.0);
    }

    double t = (qSubp.cross(s))/rCrossS;
    double u = qSubpCrossR/rCrossS;

    bool tInRange = (t >= 0.0) && (t <= 1.0);
    bool uInRange = (u >= 0.0) && (u <= 1.0);

    return tInRange && uInRange;
}

bool Polygon::collidesWith(const Polygon & other) const
{
    // return true if any vertex of this polygon is inside other polygon
    for(auto v : _vertices)
    {
        if(other.contains(v))
        {
            return true;
        }
    }

    // return true if any vertex of the other polygon is inside this polygon
    for(auto v : other._vertices)
    {
        if(contains(v))
        {
            return true;
        }
    }

    // look for edge intersections
    Vector2D start1;
    Vector2D end1;
    Vector2D start2;
    Vector2D end2;
    for(size_t i = 0; i <= _vertices.size(); i++)
    {
        if(i == _vertices.size() - 1)
        {
            start1 = _vertices[i];
            end1 = _vertices[0];
        }
        else
        {
            start1 = _vertices[i];
            end1 = _vertices[i+1];
        }

        for(size_t j = 0; j <= other._vertices.size(); j++)
        {
            if(j == other._vertices.size() - 1)
            {
                start2 = other._vertices[j];
                end2 = other._vertices[0];
            }
            else
            {
                start2 = other._vertices[j];
                end2 = other._vertices[j+1];
            }

            if(lineSegmentIntersection(start1,end1,start2,end2))
            {
                return true;
            }
        }
    }

    return false;
}

bool Polygon::completelyContains(const Circle & circ) const
{
    // if center not inside polygon then certainly false
    if(!contains(circ.center()))
    {
        return false;
    }

    // circle mustn't intersect any edge of polygon to be completly inside
    Vector2D start;
    Vector2D end;
    for(size_t i = 0; i < _vertices.size(); i++)
    {
        if(i == _vertices.size() - 1)
        {
            start = _vertices[i];
            end = _vertices[0];
        }
        else
        {
            start = _vertices[i];
            end = _vertices[i+1];
        }

        if(segmentCircle(start,end,circ))
        {
            return false;
        }
    }

    return true;
}

// http://geomalgorithms.com/a03-_inclusion.html
bool Polygon::contains(const Vector2D & p) const
{
    int windings = 0;

    Vector2D start;
    Vector2D end;
    for(size_t i = 0; i < _vertices.size(); i++)
    {
        if(i == _vertices.size() - 1)
        {
            start = _vertices[i];
            end = _vertices[0];
        }
        else
        {
            start = _vertices[i];
            end = _vertices[i+1];
        }

        if(start.y() <= p.y())
        {
            if(end.y() > p.y())
            {
                if(p.isLeft(start,end))
                {
                    windings += 1;
                }

            }
        }
        else
        {
            if(end.y() <= p.y())
            {
                if(!p.isLeft(start,end))
                {
                    windings -= 1;
                }
            }
        }
    }

    return (windings != 0);
}

// circle implementation

Vector2D Circle::center() const
{
    return _center;
}

double Circle::r() const
{
    return _r;
}

Circle Circle::translate(const Vector2D & trans) const
{
    return Circle(_center.add(trans),_r);
}

bool Circle::collidesWith(const Circle & other) const
{
    double dist = _center.sub(other._center).length();
    double minDist = _r + other._r;
    return (dist <= minDist);
}

bool Circle::collidesWith(const Polygon & poly) const
{
    return poly.collidesWith(*this);
}

bool Circle::completelyInside(const Polygon & poly) const
{
    return poly.completelyContains(*this);
}

bool Circle::contains(const Vector2D & p) const
{
    Vector2D pc = _center.sub(p);

    return (pc.dot(pc) <= _r*_r);
}

