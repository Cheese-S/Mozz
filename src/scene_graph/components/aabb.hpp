#pragma once
#include "common/glm_common.hpp"
#include "scene_graph/component.hpp"

namespace mz::sg
{
class AABB : public Component
{
  public:
	AABB();
	AABB(const glm::vec3 &min, const glm::vec3 &max);
	virtual ~AABB() = default;
	virtual std::type_index get_type() override;

	void      update(const glm::vec3 &pt);
	void      update(const glm::vec3 &min, const glm::vec3 &max);
	void      update(const AABB &b);
	AABB      transform(glm::mat4 T);
	glm::vec3 get_scale() const;
	glm::vec3 get_center() const;
	glm::vec3 get_min() const;
	glm::vec3 get_max() const;
	void      reset();

	std::string to_string() const;

  private:
	glm::vec3 min_;
	glm::vec3 max_;
};
}        // namespace mz::sg