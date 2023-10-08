#include "ray.hpp"

namespace mz
{
Ray::Ray()
{
}

Ray::Ray(const glm::vec3 &orig, const glm::vec3 &dir) :
    orig_(orig),
    dir_(dir)
{
}

glm::vec3 Ray::at(float t) const
{
	return orig_ + dir_ * t;
}

glm::vec3 Ray::get_orig() const
{
	return orig_;
}

glm::vec3 Ray::get_dir() const
{
	return dir_;
}
}        // namespace mz