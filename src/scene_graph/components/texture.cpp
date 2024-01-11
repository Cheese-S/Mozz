#include "texture.hpp"

#include "common/error.hpp"

namespace mz::sg
{
Texture::Texture(const std::string &name, size_t id) :
    Component(name, id)
{
}

std::type_index Texture::get_type()
{
	return typeid(Texture);
}

}        // namespace mz::sg