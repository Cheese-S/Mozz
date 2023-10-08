#include "pbr_material.hpp"

namespace mz::sg
{
PBRMaterial::PBRMaterial(const std::string &name) :
    Material(name)
{
}

std::type_index PBRMaterial::get_type()
{
	return typeid(PBRMaterial);
}
}        // namespace mz::sg