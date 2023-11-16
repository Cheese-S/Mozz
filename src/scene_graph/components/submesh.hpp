#pragma once

#include "common/glm_common.hpp"
#include "scene_graph/component.hpp"
#include <memory>

namespace vk
{
struct VertexInputAttributeDescription;
}

namespace mz
{
class Device;
class Buffer;
namespace sg
{

struct Vertex
{
	static std::array<vk::VertexInputAttributeDescription, 6> get_input_attr_descriptions();

	glm::vec3 pos;
	glm::vec3 norm;
	glm::vec2 uv;
	glm::vec4 joint;
	glm::vec4 weight;
	glm::vec4 color;
};

class Material;
class SubMesh : public Component
{
  public:
	SubMesh(const std::string &name, size_t id);

	virtual ~SubMesh();
	virtual std::type_index get_type() override;

	void set_material(const Material &material);

	const Material *get_material() const;

	std::uint32_t idx_offset_ = 0;
	std::uint32_t vert_count_ = 0;
	std::uint32_t idx_count_  = 0;

	std::unique_ptr<Buffer> p_vert_buf_;
	std::unique_ptr<Buffer> p_idx_buf_;

  private:
	const Material *p_material_ = nullptr;
};

}        // namespace sg
}        // namespace mz