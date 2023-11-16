#pragma once
#include "common/vk_common.hpp"
#include "core/vulkan_object.hpp"

namespace mz
{
class Device;
class Swapchain;
class RenderPass;

class Framebuffer : public VulkanObject<vk::Framebuffer>
{
  public:
	Framebuffer(Device &device, vk::FramebufferCreateInfo framebuffer_cinfo);
	~Framebuffer() override;

  private:
	Device &device_;
};

class SwapchainFramebuffer
{
  public:
	SwapchainFramebuffer(Device &device, Swapchain &swapchain, RenderPass &render_pass);
	~SwapchainFramebuffer();

	void cleanup();
	void rebuild();

	const std::vector<vk::Framebuffer> &get_handles() const;
	const vk::Framebuffer              &get_handle(uint32_t idx) const;

  private:
	void build();

	Device                      &device_;
	Swapchain                   &swapchain_;
	RenderPass                  &render_pass_;
	std::vector<vk::Framebuffer> framebuffers_;
};
}        // namespace mz