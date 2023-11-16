#include "transform.hpp"

#include "common/vk_common.hpp"
#include <glm/gtx/matrix_decompose.hpp>

#include "scene_graph/node.hpp"

namespace mz::sg
{

vk::TransformMatrixKHR Transform::to_vk_transform(glm::mat4 m)
{
	// glm is column major, vk::TransformMatrix is row major
	glm::mat4              transpose = glm::transpose(m);
	vk::TransformMatrixKHR T;
	memcpy(&T, &transpose, sizeof(T));
	return T;
}

Transform::Transform(Node &node) :
    Component("", -1),
    node_(node)
{
}

std::type_index Transform::get_type()
{
	return typeid(Transform);
}

glm::mat4 Transform::get_world_M()
{
	if (!need_update_)
	{
		return world_M_;
	}
	world_M_ = get_local_M();

	auto parent = node_.get_parent();

	if (parent)
	{
		auto &transform = parent->get_transform();
		world_M_        = transform.get_world_M() * world_M_;
	}

	need_update_ = false;
	return world_M_;
}

glm::mat4 Transform::get_local_M()
{
	return glm::translate(glm::mat4(1.0), translation_) * glm::mat4_cast(rotation_) *
	       glm::scale(glm::mat4(1.0), scale_);
}

glm::vec3 Transform::get_scale()
{
	return scale_;
}

glm::vec3 Transform::get_translation()
{
	return translation_;
}

glm::quat Transform::get_rotation()
{
	return rotation_;
}

void Transform::set_tranlsation(const glm::vec3 &translation)
{
	translation_ = translation;
	invalidate_world_M();
}
void Transform::set_rotation(const glm::quat &rotation)
{
	rotation_ = rotation;
	invalidate_world_M();
}

void Transform::set_scale(const glm::vec3 &scale)
{
	scale_ = scale;
	invalidate_world_M();
}

void Transform::set_local_M(const glm::mat4 &local_M)
{
	glm::vec3 skew;
	glm::vec4 perspective;
	glm::decompose(local_M, scale_, rotation_, translation_, skew, perspective);
	invalidate_world_M();
}

void Transform::invalidate_world_M()
{
	need_update_                       = true;
	std::vector<sg::Node *> p_children = node_.get_children();
	for (sg::Node *p_child : p_children)
	{
		Transform &child_T = p_child->get_transform();
		if (!child_T.need_update_)
		{
			child_T.invalidate_world_M();
		}
	}
}

}        // namespace mz::sg