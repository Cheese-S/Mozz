#include "material.hpp"

namespace mz::sg
{
Material::Material(const std::string &name, size_t id) :
    Component(name, id)
{
}

std::type_index Material::get_type()
{
	return typeid(Material);
}

}        // namespace mz::sg