#include "ndt_registration.h"

namespace registration{

void NDTRegistration::build_voxels()
{
    assert(target_scan != nullptr);
    assert(target_scan -> empty() == false);

    std::vector<size_t> index(target_scan -> size());
    std::for_each(index.begin(), index.end(), [idx = 0](size_t &i) mutable {i = idx++;});
    
}

} // namespace registration