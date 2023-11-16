#include "free_camera.hpp"

#include <iostream>

#include "glm/gtx/string_cast.hpp"
#include "scene_graph/components/camera.hpp"
#include "scene_graph/components/perspective_camera.hpp"

namespace mz::sg
{
const float FreeCamera::ROTATION_MOVE_WEIGHT = 4.8f;

const float FreeCamera::TRANSLATION_MOVE_STEP = 10.0f;

const float FreeCamera::TRANSLATION_MOVE_WEIGHT = 10.0f;

const uint32_t FreeCamera::TRANSLATION_MOVE_SPEED = 4;

FreeCamera::FreeCamera(Node &node, size_t id) :
    NodeScript(node, "FreeCamera", id)
{
}

void FreeCamera::update(float delta_time)
{
	glm::vec3 delta_translation(0.0f, 0.0f, 0.0f);
	glm::vec3 delta_rotation(0.0f, 0.0f, 0.0f);

	if (key_pressed_[KeyCode::eW])
	{
		delta_translation.z -= TRANSLATION_MOVE_STEP;
	}

	if (key_pressed_[KeyCode::eS])
	{
		delta_translation.z += TRANSLATION_MOVE_STEP;
	}

	if (key_pressed_[KeyCode::eA])
	{
		delta_translation.x -= TRANSLATION_MOVE_STEP;
	}

	if (key_pressed_[KeyCode::eD])
	{
		delta_translation.x += TRANSLATION_MOVE_STEP;
	}

	if (mouse_button_pressed_[MouseButton::eLeft] && mouse_button_pressed_[MouseButton::eRight])
	{
		delta_rotation.z += ROTATION_MOVE_WEIGHT * mouse_move_delta_.x;
	}
	else if (mouse_button_pressed_[MouseButton::eMiddle])
	{
		delta_translation.x += TRANSLATION_MOVE_WEIGHT * mouse_move_delta_.x;
		delta_translation.y += TRANSLATION_MOVE_WEIGHT * -mouse_move_delta_.y;
	}
	else if (mouse_button_pressed_[MouseButton::eLeft])
	{
		delta_rotation.x -= ROTATION_MOVE_WEIGHT * mouse_move_delta_.y;
		delta_rotation.y -= ROTATION_MOVE_WEIGHT * mouse_move_delta_.x;
	}
	delta_translation *= speed_multiplier_ * delta_time;
	delta_rotation *= delta_time;

	if (delta_rotation != glm::vec3(0.0f) || delta_translation != glm::vec3(0.0f))
	{
		auto     &transform = get_node().get_transform();
		glm::quat qx        = glm::angleAxis(delta_rotation.x, glm::vec3(1.0f, 0.0f, 0.0f));
		glm::quat qy        = glm::angleAxis(delta_rotation.y, glm::vec3(0.0f, 1.0f, 0.0f));

		glm::quat orientation = glm::normalize(qy * transform.get_rotation() * qx);
		transform.set_tranlsation(transform.get_translation() +
		                          orientation * delta_translation);
		transform.set_rotation(orientation);
		glm::inverse(orientation);
	}

	mouse_move_delta_ = {};
}

void FreeCamera::process_event(const Event &event)
{
	if (event.type == EventType::eKeyInput)
	{
		const auto &key_event = static_cast<const KeyEvent &>(event);

		if (key_event.action == KeyAction::eDown || key_event.action == KeyAction::eRepeat)
		{
			key_pressed_[key_event.code] = true;
		}
		else
		{
			key_pressed_[key_event.code] = false;
		}
	}
	else if (event.type == EventType::eMouseButton)
	{
		const auto &mouse_event = static_cast<const MouseButtonEvent &>(event);
		glm::vec2   mouse_pos{std::floor(mouse_event.xpos), std::floor(mouse_event.ypos)};
		switch (mouse_event.action)
		{
			case MouseAction::eDown:
				mouse_button_pressed_[mouse_event.button] = true;
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
}

void FreeCamera::resize(uint32_t width, uint32_t height)
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
