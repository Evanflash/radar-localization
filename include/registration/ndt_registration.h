#ifndef _RADAR_LOCALIZATION_NDT_REGISTRATION
#define _RADAR_LOCALIZATION_NDT_REGISTRATION

#include "base_registration.h"
#include <unordered_map>

namespace registration{
struct Options{
    
};

class NDTRegistration : public BaseRegistration
{
public:
    enum NearbyType {
        CENTER,
        NEARBY4
    };
    using KeyType = Eigen::Matrix<int, 3, 1>;
    struct voxel_data{
        voxel_data(){};
        voxel_data(size_t id){idx.emplace_back(id);};

        std::vector<size_t> idx;
        Vec3d mu = Vec3d::Zero();
        Mat3d sigma = Mat3d::Zero();
        Mat3d info = Mat3d::Zero();
    };

private:
    class hash{
    public:
        size_t operator()(const KeyType &key_type) const{
            return std::hash<int>()(key_type[0]) + std::hash<int>()(key_type[1])
                + std::hash<int>()(key_type[2]);
        }
    };
private:
    void build_voxels();

    void generate_nearby_grids();

    Vec3d target_center = Vec3d::Zero();
    Vec3d source_center = Vec3d::Zero();

    std::unordered_map<KeyType, voxel_data, hash> grids;
    std::vector<KeyType> nearby_grids;
}; //class NDTRegistration

} // namespace registration

#endif // _RADAR_LOCALIZATION_NDT_REGISTRATION