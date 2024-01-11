#pragma once

#include "common/vk_common.hpp"
#include "core/vulkan_object.hpp"

namespace mz
{
class Device;

class PipelineLayout : public VulkanObject<vk::PipelineLayout>
{
  public:
	PipelineLayout(Device &device, vk::PipelineLayoutCreateInfo &pipeline_layout_cinfo);
	~PipelineLayout() override;

  private:
	Device &device_;
};
}        // namespace mz