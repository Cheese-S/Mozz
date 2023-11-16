#include "arc_ball_camera.hpp"

#include <iostream>

#include "glm/gtx/string_cast.hpp"
#include "scene_graph/components/aabb.hpp"
#include "scene_graph/components/perspective_camera.hpp"

namespace mz::sg
{

ArcBallCamera::ArcBallCamera(Node &camera_node, const AABB &scene_bd, size_t id) :
    NodeScript(camera_node, "", id)
{
	auto &T = get_node().get_transform();
	center_ = scene_bd.get_center();
	dist_   = glm::length(scene_bd.get_scale());

	T.set_tranlsation(glm::vec3(center_.x, center_.y, center_.z + dist_));
}

void ArcBallCamera::update(float delta_time)
{
	glm::vec3 delta_rotation(0.0f, 0.0f, 0.0f);
	float     delta_dist_ = 0.0f;
	if (mouse_button_pressed_[MouseButton::eLeft])
	{
		delta_rotation.x -= 4.8 * mouse_move_delta_.y;
		delta_rotation.y -= 4.8 * mouse_move_delta_.x;
	}

	if (scroll_delta_.y != 0)
	{
		delta_dist_ -= scroll_delta_.y * 3.0;
	}

	delta_rotation *= delta_time;

	if (delta_rotation != glm::vec3(0.0f) || delta_dist_ != 0)
	{
		auto &T         = get_node().get_transform();
		float cos_theta = glm::dot(glm::normalize(T.get_rotation() * glm::vec3(0.0f, 0.0f, -1.0f)), glm::vec3(0.0f, 1.0f, 0.0f));
		dist_ += delta_dist_;

		if (cos_theta * (delta_rotation.x >= 0 ? 1 : -1) > 0.99f)
		{
			delta_rotation.x = 0;
		}
		glm::quat qx = glm::angleAxis(delta_rotation.x, glm::vec3(1.0f, 0.0f, 0.0f));
		glm::quat qy = glm::angleAxis(delta_rotation.y, glm::vec3(0.0f, 1.0f, 0.0f));

		glm::quat orientation = glm::normalize(qy * T.get_rotation() * qx);
		T.set_tranlsation(center_ + orientation * glm::vec3(0.0f, 0.0f, dist_));
		T.set_rotation(orientation);
	}

	mouse_move_delta_ = {};
	scroll_delta_     = {};
}

void ArcBallCamera::process_event(const Event &event)
{
	if (event.type == EventType::eMouseButton)
	{
		const auto &mouse_event = static_cast<const MouseButtonEvent &>(event);
		glm::vec2   mouse_pos{std::floor(mouse_event.xpos), std::floor(mouse_event.ypos)};
		switch (mouse_event.action)
		{
			case MouseAction::eDown:
				mouse_button_pressed_[mouse_event.button] = true;
				mouse_last_pos_                           = mouse_pos;
				break;
			case MouseAction::eUp:
				mouse_button_pressed_[mouse_event.button] = false;
				break;
			case MouseAction::eMove:
				mouse_move_delta_ = mouse_pos - mouse_last_pos_;
				mouse_last_pos_   = mouse_pos;
				break;
			default:
				break;
		}
	}
	else if (event.type == EventType::eScroll)
	{
		const auto &scroll_event = static_cast<const ScrollEvent &>(event);
		scroll_delta_.x += scroll_event.x_offset;
		scroll_delta_.y += scroll_event.y_offset;
	}
}

void ArcBallCamera::resize(uint32_t width, uint32_t height)
{
	auto &camera_node = get_node();

	if (camera_node.has_component<Camera>())
	{
		if (auto camera = dynamic_cast<PerspectiveCamera *>(&camera_node.get_component<Camera>()))
		{
			camera->set_aspect_ratio(static_cast<float>(width) / height);
		}
	}
};

}        // namespace mz::sg