#include "node.hpp"

namespace mz::sg
{
Node::Node(const std::string &name, size_t id) :
    id_(id),
    name_(name),
    T_(*this)
{
	set_component(T_);
}

void Node::add_child(Node &child)
{
	children_.push_back(&child);
}

void Node::set_parent(Node &parent)
{
	parent_ = &parent;
	T_.invalidate_world_M();
}

void Node::set_component(Component &component)
{
	auto it = components_.find(component.get_type());
	if (it != components_.end())
	{
		it->second = &component;
	}
	else
	{
		components_.insert(std::make_pair(component.get_type(), &component));
	}
}

const size_t Node::get_id() const
{
	return id_;
};

const std::string &Node::get_name() const
{
	return name_;
};

Node *Node::get_parent() const
{
	return parent_;
};

const std::vector<Node *> &Node::get_children() const
{
	return children_;
};

Component &Node::get_component(const std::type_index index)
{
	return *components_.at(index);
};

Transform &Node::get_transform()
{
	return T_;
}

bool Node::has_component(const std::type_index index)
{
	return components_.count(index) > 0;
}
}        // namespace mz::sg