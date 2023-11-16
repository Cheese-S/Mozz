#pragma once

#include <string>
#include <typeindex>

namespace mz
{
namespace sg
{
class Component
{
  public:
	Component()          = default;
	virtual ~Component() = default;
	Component(const std::string &name, size_t id);
	Component(Component &&rhs);

	const std::string      &get_name() const;
	size_t                  get_id() const;
	virtual std::type_index get_type() = 0;

	void set_id(size_t id);

  private:
	size_t      id_;
	std::string name_;
};

}        // namespace sg
}        // namespace mz