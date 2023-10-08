#include "graphics_pipeline.hpp"

#include "common/file_utils.hpp"
#include "common/utils.hpp"
#include "device.hpp"
#include "pipeline_layout.hpp"
#include "render_pass.hpp"

namespace mz
{

GraphicsPipeline::GraphicsPipeline(Device &device, RenderPass &render_pass, GraphicsPipelineState &state, vk::PipelineLayoutCreateInfo &pl_layout_cinfo) :
    device_(device)
{
	vk::ShaderModule vert_shader_module = create_shader_module(state.vert_shader_name);
	vk::ShaderModule frag_shader_module = create_shader_module(state.frag_shader_name);

	vk::PipelineShaderStageCreateInfo vert_stage_cinfo{
	    .stage  = vk::ShaderStageFlagBits::eVertex,
	    .module = vert_shader_module,
	    .pName  = "main",
	};
	vk::PipelineShaderStageCreateInfo frag_stage_cinfo{
	    .stage  = vk::ShaderStageFlagBits::eFragment,
	    .module = frag_shader_module,
	    .pName  = "main",
	};

	std::array<vk::PipelineShaderStageCreateInfo, 2> shader_stages{vert_stage_cinfo, frag_stage_cinfo};

	vk::PipelineVertexInputStateCreateInfo vertex_input_cinfo{
	    .vertexBindingDescriptionCount   = to_u32(state.vertex_input_state.binding_descriptions.size()),
	    .pVertexBindingDescriptions      = state.vertex_input_state.binding_descriptions.data(),
	    .vertexAttributeDescriptionCount = to_u32(state.vertex_input_state.attribute_descriptions.size()),
	    .pVertexAttributeDescriptions    = state.vertex_input_state.attribute_descriptions.data(),
	};

	vk::PipelineInputAssemblyStateCreateInfo input_assembly_cinfo{
	    .topology               = state.input_assembly_state.topology,
	    .primitiveRestartEnable = state.input_assembly_state.primitive_restart_enable,
	};
	vk::PipelineViewportStateCreateInfo viewport_cinfo{
	    .viewportCount = 1,
	    .scissorCount  = 1,
	};

	vk::PipelineRasterizationStateCreateInfo rasterization_cinfo{
	    .depthClampEnable        = state.rasterization_state.depth_clamp_enable,
	    .rasterizerDiscardEnable = state.rasterization_state.rasterizer_discard_enable,
	    .polygonMode             = state.rasterization_state.polygon_mode,
	    .cullMode                = state.rasterization_state.cull_mode,
	    .frontFace               = state.rasterization_state.front_face,
	    .depthBiasEnable         = state.rasterization_state.depth_bias_enable,
	    .lineWidth               = 1.0f,
	};

	vk::PipelineMultisampleStateCreateInfo multisample_cinfo{
	    .rasterizationSamples = state.multisample_state.rasterization_samples,
	    .sampleShadingEnable  = false,
	};

	vk::PipelineDepthStencilStateCreateInfo depth_stencil_cinfo{
	    .depthTestEnable       = state.depth_stencil_state.depth_test_enable,
	    .depthWriteEnable      = state.depth_stencil_state.depth_write_enable,
	    .depthCompareOp        = state.depth_stencil_state.depth_compare_op,
	    .depthBoundsTestEnable = state.depth_stencil_state.depth_bounds_test_enable,
	    .stencilTestEnable     = state.depth_stencil_state.stencil_test_enable,
	};

	vk::PipelineColorBlendAttachmentState color_blend_attachment_cinfo{
	    .blendEnable    = state.color_blend_attachment_state.blend_enable,
	    .colorWriteMask = state.color_blend_attachment_state.color_write_mask,
	};

	vk::PipelineColorBlendStateCreateInfo color_blending_cinfo{
	    .logicOpEnable   = state.color_blend_state.logic_op_enable,
	    .logicOp         = state.color_blend_state.logic_op,
	    .attachmentCount = 1,
	    .pAttachments    = &color_blend_attachment_cinfo,
	    .blendConstants  = std::array<float, 4>{0.0f, 0.0f, 0.0f, 0.0f},
	};

	std::vector<vk::DynamicState>      dynamic_states{vk::DynamicState::eViewport, vk::DynamicState::eScissor};
	vk::PipelineDynamicStateCreateInfo dynamic_state_cinfo{
	    .dynamicStateCount = to_u32(dynamic_states.size()),
	    .pDynamicStates    = dynamic_states.data(),
	};

	pl_layout_ = device_.get_handle().createPipelineLayout(pl_layout_cinfo);

	vk::GraphicsPipelineCreateInfo graphics_pipeline_cinfo{
	    .stageCount          = to_u32(shader_stages.size()),
	    .pStages             = shader_stages.data(),
	    .pVertexInputState   = &vertex_input_cinfo,
	    .pInputAssemblyState = &input_assembly_cinfo,
	    .pViewportState      = &viewport_cinfo,
	    .pRasterizationState = &rasterization_cinfo,
	    .pMultisampleState   = &multisample_cinfo,
	    .pDepthStencilState  = &depth_stencil_cinfo,
	    .pColorBlendState    = &color_blending_cinfo,
	    .pDynamicState       = &dynamic_state_cinfo,
	    .layout              = pl_layout_,
	    .renderPass          = render_pass.get_handle(),
	    .subpass             = 0,
	};

	// ? might have to handle ePipelineCompileRequiredEXT
	handle_ = device_.get_handle().createGraphicsPipeline(nullptr, graphics_pipeline_cinfo).value;
	device_.get_handle().destroyShaderModule(vert_shader_module);
	device_.get_handle().destroyShaderModule(frag_shader_module);
}

GraphicsPipeline::~GraphicsPipeline()
{
	// In our design, we bind pipeline layout to a pipeline.
	if (handle_)
	{
		device_.get_handle().destroyPipelineLayout(pl_layout_);
		device_.get_handle().destroyPipeline(handle_);
	}
}

vk::ShaderModule GraphicsPipeline::create_shader_module(const std::string &name)
{
	std::vector<uint8_t>       binary = fu::read_shader_binary(name);
	vk::ShaderModuleCreateInfo shader_module_cinfo{
	    .codeSize = to_u32(binary.size()),
	    .pCode    = reinterpret_cast<const uint32_t *>(binary.data()),
	};
	return device_.get_handle().createShaderModule(shader_module_cinfo);
}

vk::PipelineLayout GraphicsPipeline::get_pipeline_layout()
{
	return pl_layout_;
}
}        // namespace mz