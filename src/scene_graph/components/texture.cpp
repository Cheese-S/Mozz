#include "texture.hpp"

#include "common/error.hpp"

namespace mz::sg
{
Texture::Texture(const std::string &name) :
    Component(name)
{
}

std::type_index Texture::get_type()
{
	return typeid(Texture);
}

}        // namespace mz::sg