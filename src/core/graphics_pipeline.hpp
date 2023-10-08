#pragma once

#include "common/vk_common.hpp"
#include "core/vulkan_object.hpp"

namespace mz
{
class Device;
class PipelineLayout;
class RenderPass;

struct VertexInputState
{
	vk::ArrayProxy<vk::VertexInputAttributeDescription> attribute_descriptions;
	vk::ArrayProxy<vk::VertexInputBindingDescription>   binding_descriptions;
};

struct InputAssemblyState
{
	vk::PrimitiveTopology topology                 = vk::PrimitiveTopology::eTriangleList;
	vk::Bool32            primitive_restart_enable = false;
};

// gltf's winding order is ccw. However, we flip its x coordinate during importing.
// Therefore, we use a cw winding order
struct RasterizationState
{
	vk::Bool32        depth_clamp_enable        = false;
	vk::Bool32        depth_bias_enable         = false;
	vk::Bool32        rasterizer_discard_enable = false;
	vk::PolygonMode   polygon_mode              = vk::PolygonMode::eFill;
	vk::CullModeFlags cull_mode                 = vk::CullModeFlagBits::eBack;
	vk::FrontFace     front_face                = vk::FrontFace::eClockwise;
};

struct MultisampleState
{
	vk::SampleCountFlagBits rasterization_samples = vk::SampleCountFlagBits::e1;
};

struct DepthStencilState
{
	vk::Bool32    depth_test_enable        = true;
	vk::Bool32    depth_write_enable       = true;
	vk::CompareOp depth_compare_op         = vk::CompareOp::eLess;
	vk::Bool32    depth_bounds_test_enable = false;
	vk::Bool32    stencil_test_enable      = false;
};

struct ColorBlendAttachmentState
{
	vk::Bool32              blend_enable     = false;
	vk::ColorComponentFlags color_write_mask = vk::ColorComponentFlagBits::eR | vk::ColorComponentFlagBits::eG | vk::ColorComponentFlagBits::eB | vk::ColorComponentFlagBits::eA;
};

struct ColorBlendState
{
	vk::Bool32  logic_op_enable = false;
	vk::LogicOp logic_op        = vk::LogicOp::eClear;
};

struct GraphicsPipelineState
{
	const char               *vert_shader_name;
	const char               *frag_shader_name;
	VertexInputState          vertex_input_state;
	InputAssemblyState        input_assembly_state;
	RasterizationState        rasterization_state;
	MultisampleState          multisample_state;
	DepthStencilState         depth_stencil_state;
	ColorBlendAttachmentState color_blend_attachment_state;
	ColorBlendState           color_blend_state;
};

class GraphicsPipeline : public VulkanObject<vk::Pipeline>
{
  public:
	GraphicsPipeline(Device &device, RenderPass &render_pass, GraphicsPipelineState &state, vk::PipelineLayoutCreateInfo &pl_layout_cinfo);
	GraphicsPipeline(GraphicsPipeline &&) = default;
	~GraphicsPipeline() override;

	vk::PipelineLayout get_pipeline_layout();

  private:
	vk::ShaderModule   create_shader_module(const std::string &name);
	Device            &device_;
	vk::PipelineLayout pl_layout_;
};

}        // namespace mz