#include "mesh.hpp"

namespace mz::sg
{

Mesh::Mesh(const std::string &name, size_t id) :
    Component(name, id){};

void Mesh::add_submesh(SubMesh &submesh)
{
	p_submeshs.push_back(&submesh);
}

void Mesh::add_node(Node &node)
{
	p_nodes.push_back(&node);
}

std::type_index Mesh::get_type()
{
	return typeid(Mesh);
}

const AABB &Mesh::get_bounds() const
{
	return bounds_;
}

AABB &Mesh::get_mut_bounds()
{
	return bounds_;
}

const std::vector<SubMesh *> &Mesh::get_p_submeshs() const
{
	return p_submeshs;
}

const std::vector<Node *> &Mesh::get_p_nodes() const
{
	return p_nodes;
}
}        // namespace mz::sg