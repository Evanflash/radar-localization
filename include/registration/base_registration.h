#ifndef _RADAR_LOCALIZATION_BASE_REGISTRATION
#define _RADAR_LOCALIZATION_BASE_REGISTRATION

#include <Eigen/Dense>

namespace registration{

class BaseRegistration
{
public:
    virtual void registration() {};
}; // class BaseRegistration

} // namespace registration

#endif // _RADAR_LOCALIZATION_BASE_REGISTRATION