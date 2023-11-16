#include "scene.hpp"

#include "component.hpp"
#include "node.hpp"

namespace mz::sg
{
Scene::Scene(const std::string &name) :
    name_(name)
{
}

Scene::Scene(Scene &&rhs) :
    name_(std::move(rhs.name_)),
    root_(rhs.root_),
    bound_(rhs.bound_.get_min(), rhs.bound_.get_max()),
    p_nodes_(std::move(rhs.p_nodes_)),
    p_components_(std::move(rhs.p_components_))
{
}

void Scene::add_node(std::unique_ptr<Node> &&pNode)
{
	p_nodes_.emplace_back(std::move(pNode));
}

void Scene::add_child(Node &child)
{
	root_->add_child(child);
}

void Scene::add_component(std::unique_ptr<Component> &&pComponent)
{
	if (pComponent)
	{
		p_components_[pComponent->get_type()].push_back(std::move(pComponent));
	}
}

void Scene::add_component_to_node(std::unique_ptr<Component> &&pComponent, Node &node)
{
	if (pComponent)
	{
		node.set_component(*pComponent);
		p_components_[pComponent->get_type()].push_back(std::move(pComponent));
	}
}

void Scene::set_components(const std::type_index                   type,
                           std::vector<std::unique_ptr<Component>> pComponents)
{
	p_components_[type] = std::move(pComponents);
}

void Scene::set_root_node(Node &node)
{
	root_ = &node;
}

void Scene::set_nodes(std::vector<std::unique_ptr<Node>> &&nodes)
{
	p_nodes_ = std::move(nodes);
}

std::vector<sg::Node *> Scene::get_nodes()
{
	std::vector<sg::Node *> res;
	res.reserve(p_nodes_.size());
	for (auto &p_node : p_nodes_)
	{
		res.push_back(p_node.get());
	}
	return res;
}

Node &Scene::get_root_node()
{
	return *root_;
}

Node &Scene::get_node_by_index(int idx)
{
	assert(idx >= 0 && idx < p_nodes_.size());
	return *p_nodes_[idx].get();
}

AABB &Scene::get_bound()
{
	return bound_;
}

Node *Scene::find_node(const std::string &name)
{
	for (auto &pNode : p_nodes_)
	{
		if (pNode->get_name() == name)
		{
			return pNode.get();
		}
	}

	return nullptr;
}

const std::vector<std::unique_ptr<Component>> &Scene::get_components(
    const std::type_index &type) const
{
	return p_components_.at(type);
}

bool Scene::has_component(const std::type_index &type) const
{
	return p_components_.count(type) > 0;
}
}        // namespace mz::sg