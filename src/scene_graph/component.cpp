#include "component.hpp"

namespace mz::sg
{
Component::Component(const std::string &name, size_t id) :
    name_(name),
    id_(id){};

Component::Component(Component &&rhs) :
    name_(std::move(rhs.name_)),
    id_(rhs.id_)
{
}

const std::string &Component::get_name() const
{
	return name_;
}

size_t Component::get_id() const
{
	return id_;
}

void Component::set_id(size_t id)
{
	id_ = id;
}
}        // namespace mz::sg