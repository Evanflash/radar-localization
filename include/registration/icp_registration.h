#ifndef _RADAR_LOCALIZATION_ICP_REGISTRATION
#define _RADAR_LOCALIZATION_ICP_REGISTRATION

#include "base_registration.h"

namespace registration {

class ICPRegistration : public BaseRegistration
{
public:
    bool registration(SE2 &init_pose, int iterations_) override;
    void set_target(CLOUD::Ptr target_scan_) override;
private:
    void build_target_kdtree();
    pcl::search::KdTree<POINT> kdtree; 

}; // class ICPRegistration

} // namespace registration

#endif // _RADAR_LOCALIZATION_ICP_REGISTRATION