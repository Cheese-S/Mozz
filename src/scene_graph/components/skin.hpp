#pragma once

#include <array>
#include <unordered_map>

#include "common/glm_common.hpp"
#include "scene_graph/component.hpp"

namespace mz::sg
{
class Node;
class Scene;

class Skin : public Component
{
  public:
	static const int MAX_NUM_JOINTS = 256;
	Skin(const std::string &name, size_t id);

	void                                   compute_joint_Ms(sg::Scene &scene, glm::mat4 *p_joint_Ms) const;
	void                                   add_new_joint(int joint_id, uint32_t node_id);
	std::type_index                        get_type() override;
	std::array<glm::mat4, MAX_NUM_JOINTS> &get_IBMs();

  private:
	std::array<glm::mat4, MAX_NUM_JOINTS> IBMs_;
	std::unordered_map<uint32_t, int>     node_joint_map_;
	std::unordered_map<int, uint32_t>     joint_node_map_;
};
}        // namespace mz::sg