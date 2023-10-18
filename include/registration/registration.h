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

private:
    BaseRegistration *registrationer = nullptr;
};

} // namespace registration

#endif // _RADAR_LOCALIZATION_REGISTRATION