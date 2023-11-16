#include "skin.hpp"

#include <iostream>

#include "glm/gtx/string_cast.hpp"
#include "scene_graph/scene.hpp"

namespace mz::sg
{
Skin::Skin(const std::string &name, size_t id) :
    Component(name, id){};

void Skin::compute_joint_Ms(sg::Scene &scene, glm::mat4 *p_joint_Ms) const
{
	std::vector<sg::Node *> p_nodes = scene.get_nodes();
	for (int joint_id = 0; joint_id < joint_node_map_.size(); joint_id++)
	{
		Node *p_node         = p_nodes[joint_node_map_.at(joint_id)];
		Node *p_parent       = p_node->get_parent();
		p_joint_Ms[joint_id] = p_node->get_transform().get_world_M() * IBMs_[joint_id];
	}
}

void Skin::add_new_joint(int joint_id, uint32_t node_id)
{
	node_joint_map_[node_id]  = joint_id;
	joint_node_map_[joint_id] = node_id;
}

std::type_index Skin::get_type()
{
	return typeid(Skin);
}

std::array<glm::mat4, Skin::MAX_NUM_JOINTS> &Skin::get_IBMs()
{
	return IBMs_;
}

}        // namespace mz::sg