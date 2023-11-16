#pragma once

#include "common/vk_common.hpp"
#include "core/vulkan_object.hpp"

namespace mz
{
class Device;
class Swapchain;

class RenderPass : public VulkanObject<vk::RenderPass>
{
  public:
	static vk::AttachmentDescription color_attachment(vk::Format format, vk::ImageLayout initial_layout = vk::ImageLayout::eUndefined, vk::ImageLayout final_layout = vk::ImageLayout::eColorAttachmentOptimal);

	static vk::AttachmentDescription depth_attachment(vk::Format format, vk::ImageLayout initial_layout = vk::ImageLayout::eUndefined, vk::ImageLayout final_layout = vk::ImageLayout::eDepthAttachmentOptimal);

	RenderPass(Device &device, vk::RenderPassCreateInfo render_pass_cinfo);
	~RenderPass() override;

  private:
	Device &device_;
};
}        // namespace mz