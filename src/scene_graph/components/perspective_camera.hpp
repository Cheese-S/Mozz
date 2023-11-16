#pragma once

#include "camera.hpp"
namespace mz::sg
{
class PerspectiveCamera : public Camera
{
  public:
	PerspectiveCamera(const std::string &name, size_t id);
	virtual ~PerspectiveCamera() = default;
	virtual glm::mat4 get_projection() override;

	void set_aspect_ratio(float aspect_ratio);
	void set_field_of_view(float fov);
	void set_far_plane(float zfar);
	void set_near_plane(float znear);

	float get_far_plane() const;
	float get_near_plane() const;
	float get_field_of_view() const;
	float get_aspect_ratio() const;

  private:
	float aspect_ratio_ = 1.0f;
	float fov_          = glm::radians(60.0f);
	float zfar_         = 100.0f;
	float znear_        = 0.1f;
};
}        // namespace mz::sg