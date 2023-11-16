#include "script.hpp"

namespace mz::sg
{
Script::Script(const std::string &name, size_t id) :
    Component(name, id){};

std::type_index Script::get_type()
{
	return typeid(Script);
}

void Script::process_event(const Event &event)
{
}

void Script::resize(uint32_t width, uint32_t height)
{
}

NodeScript::NodeScript(Node &node, const std::string &name, size_t id) :
    Script(name, id),
    node_(node)
{
}

Node &NodeScript::get_node()
{
	return node_;
};

}        // namespace mz::sg