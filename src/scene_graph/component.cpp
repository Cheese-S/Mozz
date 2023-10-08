#include "component.hpp"

namespace mz::sg
{
Component::Component(const std::string &name) :
    name_(name){};

const std::string &Component::get_name() const
{
	return name_;
}
}        // namespace mz::sg