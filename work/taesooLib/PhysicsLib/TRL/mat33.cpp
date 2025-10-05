
#include "physicsLib.h"
#include "Body.h"

inline void calcRodrigues(matrix33& out_R, const vector3& axis, double q)
{
    // E + a_hat*sin(q) + a_hat*a_hat*(1-cos(q))
    //
    //    |  0 -az  ay|
    // =E+| az   0 -ax|*s + a_hat*a_hat*v
    //    |-ay  ax   0|
    //
    //    |  0 -az  ay|     |-az*az-ay*ay        ax*ay        az*ax|
    // =E+| az   0 -ax|*s + |       ax*ay -az*az-ax*ax        ay*az|*v
    //    |-ay  ax   0|     |       az*ax        ay*az -ax*ax-ay*ay|
    //
    //  |1-az*az*v-ay*ay*v     -az*s+ax*ay*v      ay*s+az*ax*v|
    // =|     az*s+ax*ay*v 1-az*az*v-ax*ax*v     -ax*s+ay+az*v|
    //  |    -ay*s+az*ax*v      ax*s+ay*az*v 1-ax*ax*v-ay*ay*v|
    //

    const double sth = sin(q);
    const double vth = 1.0 - cos(q);

    double ax = axis.x;
    double ay = axis.y;
    double az = axis.z;

    const double axx = ax*ax*vth;
    const double ayy = ay*ay*vth;
    const double azz = az*az*vth;
    const double axy = ax*ay*vth;
    const double ayz = ay*az*vth;
    const double azx = az*ax*vth;

    ax *= sth;
    ay *= sth;
    az *= sth;

    out_R .setValue( 1.0 - azz - ayy, -az + axy,       ay + azx,
            az + axy,        1.0 - azz - axx, -ax + ayz,
            -ay + azx,       ax + ayz,        1.0 - ayy - axx);
}
matrix3 rodrigues(vector3 const& a, double q)
{
	matrix3 out;
	calcRodrigues(out, a, q);
	return out;
}
