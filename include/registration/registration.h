#ifndef _RADAR_LOCALIZATION_REGISTRATION
#define _RADAR_LOCALIZATION_REGISTRATION

#include "base_registration.h"
#include "ndt_registration.h"
#include "icp_registration.h"

namespace registration{

class Registration{
public:
    enum registration_model {
        ICP2D,
        NDT2D
    };

public:
    Registration(registration_model _rgm, int _iterators);
    ~Registration();
    void set_target_cloud(CLOUD::Ptr target_cloud);
    void set_source_cloud(CLOUD::Ptr source_cloud);

    inline Sophus::SE2d get_cur_pose() const
    {
        return cur_pose;
    }

    inline void set_cur_pose(Sophus::SE2d pose){
        cur_pose = pose;
    }

    void registration();

private:
    // 位姿
    Sophus::SE2d cur_pose;

    BaseRegistration *registrationer = nullptr;
    registration_model rgm;

    // ICP2D
    int iterators;
};

} // namespace registration

#endif // _RADAR_LOCALIZATION_REGISTRATION