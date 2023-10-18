#include "registration.h"

namespace registration{

Registration::Registration(registration_model _rgm, int _iterators)
    : rgm(_rgm), iterators(_iterators)
{
    registrationer = new ICPRegistration();
}

Registration::~Registration()
{
    delete registrationer;
}

void Registration::set_target_cloud(CLOUD::Ptr target_cloud)
{
    registrationer -> set_target(target_cloud);
}

void Registration::set_source_cloud(CLOUD::Ptr source_cloud)
{
    registrationer -> set_source(source_cloud);
}

void Registration::registration()
{
    if(rgm == ICP2D){
        registrationer -> registration(cur_pose, iterators);
    }
}

} // namespace registration