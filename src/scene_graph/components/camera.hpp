#pragma once

#include "common/glm_common.hpp"
#include "scene_graph/component.hpp"

namespace mz::sg
{
class Node;
class Camera : public Component
{
  public:
	Camera(const std::string &name, size_t id);
	virtual ~Camera() = default;
	virtual std::type_index get_type() override;

	void set_node(Node &node);
	void set_pre_rotation(const glm::mat4 &pre_rotation);

	virtual glm::mat4 get_projection() = 0;
	glm::mat4         get_view();
	const glm::mat4   get_pre_rotation();
	Node             *get_node();

  private:
	Node     *pNode_{nullptr};
	glm::mat4 pre_rotation_{1.0f};
};
}        // namespace mz::sg