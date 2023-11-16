#pragma once

#include "scene_graph/script.hpp"

namespace mz::sg
{
class AABB;
class ArcBallCamera : public NodeScript
{
  public:
	ArcBallCamera(Node &node, const AABB &scene_bd, size_t id);
	~ArcBallCamera() = default;
	void update(float delta_time) override;
	void process_event(const Event &event) override;
	void resize(uint32_t width, uint32_t height) override;

  private:
	void                                  update_camera_transform();
	float                                 dist_;
	glm::vec3                             center_;
	glm::vec2                             mouse_last_pos_;
	glm::vec2                             mouse_move_delta_;
	glm::vec2                             scroll_delta_;
	std::unordered_map<MouseButton, bool> mouse_button_pressed_;
};
}        // namespace mz::sg