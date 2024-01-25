#ifndef _RADAR_LOCALIZATION_GPS_SENSOR
#define _RADAR_LOCALIZATION_GPS_SENSOR

#include <string>
#include <vector>
#include <unordered_set>

#include <Eigen/Core>
#include <Eigen/Dense>

namespace gps
{

using ll = long long;
using Vec2d = Eigen::Vector2d;
using Vec3d = Eigen::Vector3d;

struct gps_data
{
    ll timestamp;
    double x_coor;
    double y_coor;
    gps_data() = default;
    gps_data(ll _timestamp, double _x_coor, double _y_coor)
        : timestamp(_timestamp), x_coor(_x_coor), y_coor(_y_coor){}
}; // struct gps_data

class GPSSensor
{
public:
    GPSSensor() = default;
    GPSSensor(const std::string file_path);
    Vec2d get_gps_data_by_timestamp(ll timestamp);
    bool is_bad_gps_data() const
    {
        return bad_gps;
    }

private:
    void search(ll timestamp, size_t &start, size_t &end);
    void check_timestamp(ll timestamp);

private:
    std::vector<gps_data> all_gps_data;
    std::unordered_set<ll> bad_gps_value;
    bool bad_gps;

}; // class GPSSensor

}; // namespace gps

#endif // _RADAR_LOCALIZATION_GPS_SENSOR