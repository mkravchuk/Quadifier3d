#pragma once
#include <complex>

class Complex
{
public:
    float real;
    float imag;
    Complex() = default;
    Complex(float _real, float _imag)
        : real(_real), imag(_imag)
    {
    }
    Complex(std::complex<float> c)
    {
        real = c.real();
        imag = c.imag();
    }

    friend inline  Complex operator * (Complex const & p1, Complex const & p2)
    {
        std::complex<float> c1 = { p1.real, p1.imag };
        std::complex<float> c2 = { p2.real, p2.imag };
        return Complex(c1*c2);
    }
};

class Polar
{
public:
    float r;
    float angle; // angle in radians: from 0 to PI
    Polar() = default;
    Polar(float _r)
        : r(_r), angle(0)
    {
    }
    Polar(float _r, float _angle)
        : r(_r), angle(_angle)
    {
    }

    inline Complex toComplex() const
    {
        return std::polar(r, angle);
    }

    operator Complex() const
    {
        return toComplex();
    }

    friend inline  Polar operator * (Polar const & p1, Polar const & p2)
    {
        // in polar coordinates (r,a)^2 == (r*r,a+a) == (r^2, 2a)
        return { p1.r*p2.r, p1.angle + p2.angle };
    }
};

class PolarInt
{
private:
    static constexpr double PI2 = 2 * 3.14159265358979323846;
    //static constexpr double UINT_MAX_in_double = UINT_MAX;
    static constexpr double RADIANS_IN_ONE_UNIT = PI2 / (0. + UINT_MAX);
    static constexpr double UNITS_IN_ONE_RADIAN = (0. + UINT_MAX) / PI2;
public:
    float r;
    unsigned int angle; // angle in int: from 0 to UINT_MAX
    PolarInt() = default;
    PolarInt(float _r, unsigned int _angle)
        : r(_r), angle(_angle)
    {
    }
    PolarInt(const Polar& polar)
    {
        r = polar.r;
        double radians = polar.angle;
        while (radians < 0) radians += PI2;
        while (radians > PI2) radians -= PI2;
        double mult = UNITS_IN_ONE_RADIAN * radians;
        angle = static_cast<unsigned int>(mult);
    }

    inline Polar toPolar() const
    {
        float angleDecimal = static_cast<float>(RADIANS_IN_ONE_UNIT * (0. + angle));
        return { r, angleDecimal };
    }
    inline Complex toComplex() const
    {
        return toPolar().toComplex();
    }

    operator Polar() const
    {
        return toPolar();
    }
    operator Complex() const
    {
        return toComplex();
    }

    friend inline  PolarInt operator * (PolarInt const & p1, PolarInt const & p2)
    {
        return { p1.r*p2.r, p1.angle + p2.angle };
    }
};
