#pragma once

#include "common/vk_common.hpp"
#include "core/vulkan_object.hpp"

namespace mz
{

class Device;

class RaytracingPipeline : public VulkanObject<vk::Pipeline>
{
  public:
	enum StageIndices
	{
		eRaygen           = 0,
		eMiss             = 1,
		eClosestHit       = 2,
		eShaderGroupCount = 3,
	};

	RaytracingPipeline(Device &device, std::array<const char *, eShaderGroupCount> &shader_names, vk::PipelineLayoutCreateInfo &pl_layout_cinfo);

	~RaytracingPipeline();

	vk::PipelineLayout get_pipeline_layout() const;

  private:
	vk::ShaderModule create_shader_module(const std::string &name);

	Device            &device_;
	vk::PipelineLayout pl_layout_;
};
}        // namespace mz