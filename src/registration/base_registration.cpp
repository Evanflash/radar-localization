#include "base_registration.h"
#include "utils_func.h"

namespace registration{
std::ostream& operator<<(std::ostream& os, const SE2& se2)
{
    os << se2.x << " " << se2.y << " " << se2.theta;
    return os;
}

SE2::SE2(float _x, float _y, float _theta)
    : x(_x), y(_y), theta(_theta){}

POINT SE2::operator*(POINT p)
{
    Vec3d cur_p(p.x, p.y, 1.0);
    Mat3d T = get_trans();
    Vec3d nxt_p = T * cur_p;
    return POINT(nxt_p[0], nxt_p[1], 0, p.intensity);
}

SE2& SE2::operator*(SE2 se2){
    Mat3d T_1 = get_trans();
    Mat3d T_2 = se2.get_trans();
    T_1 = T_2 * T_1;

    this -> x = T_1(0, 2);
    this -> y = T_1(1, 2);
    float sin_theta = T_1(1, 0);
    float cos_theta = T_1(1, 1);

    this -> theta = utils::Utils::theta(sin_theta, cos_theta);

    return *this;
}

Mat3d SE2::get_trans() const
{
    Mat3d T;
    double cos_theta = cos(theta);
    double sin_theta = sin(theta);
    T << -cos_theta, sin_theta, x,
          sin_theta, cos_theta, y,
          0, 0, 1;
    return T;
}

} // namespace registration