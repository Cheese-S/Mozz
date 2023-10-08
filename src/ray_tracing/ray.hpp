#pragma once

#include "common/glm_common.hpp"

namespace mz
{
class Ray
{
  public:
	Ray();
	Ray(const glm::vec3 &orig, const glm::vec3 &dir);

	glm::vec3 at(float t) const;

	glm::vec3 get_orig() const;
	glm::vec3 get_dir() const;

  private:
	glm::vec3 orig_;
	glm::vec3 dir_;
};
}        // namespace mz