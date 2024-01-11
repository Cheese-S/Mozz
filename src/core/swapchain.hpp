#pragma once

#include "common/vk_common.hpp"
#include "vulkan_object.hpp"
#include <memory>

namespace mz
{
class Context;
class ImageView;
class ImageResource;

struct SwapchainProperties
{
	vk::SurfaceFormatKHR surface_format;
	vk::PresentModeKHR   present_mode;
	vk::Extent2D         extent;
};

class Swapchain : public VulkanObject<vk::SwapchainKHR>
{
  public:
	Swapchain(Context &context, vk::Extent2D window_extent);
	~Swapchain() override;

	void       cleanup();
	void       rebuild(vk::Extent2D new_window_extent);
	void       build(vk::Extent2D window_extent);
	vk::Format choose_depth_format();

	const SwapchainProperties    &get_swapchain_properties() const;
	const std::vector<ImageView> &get_frame_image_views() const;
	const std::vector<vk::Image> &get_frame_images() const;
	const ImageResource          &get_depth_resource() const;

  private:
	void     choose_features();
	void     choose_format(const std::vector<vk::SurfaceFormatKHR> &formats);
	void     choose_present_mode(const std::vector<vk::PresentModeKHR> &present_modes);
	void     choose_extent(const vk::SurfaceCapabilitiesKHR &capabilities, vk::Extent2D window_extent);
	uint32_t calc_min_image_count(uint32_t min_image_count, uint32_t max_image_count);

	void create_frame_resources();

	Context                       &context_;
	SwapchainProperties            properties_;
	std::vector<vk::Image>         frame_images_;        // Special images owned by vulkan
	std::vector<ImageView>         frame_image_views_;
	std::unique_ptr<ImageResource> p_depth_resource_;
	// void createSwapchain();
	// void chooseSurfaceFormat(const std::vector<vk::SurfaceFormatKHR> &formats);
	// void choosePresentMode(const std::vector<vk::PresentModeKHR> &presentModes);
	// void chooseExtent(const vk::SurfaceCapabilitiesKHR &capabilities);
	// void createImageViews();
	// void createColorResources();
	// void createDepthResource();
	// void cleanup();

	// struct AttachmentResource
	// {
	// 	std::unique_ptr<DeviceMemory::Image> pImage = nullptr;
	// 	vk::raii::ImageView                  view   = nullptr;
	// };

	// Window                                 *pWindow_;
	// Instance                               *pInstance_;
	// Device                                 *pDevice_;
	// vk::SampleCountFlagBits                 mssaSamples_;
	// vk::SurfaceFormatKHR                    surfaceFormat_;
	// vk::PresentModeKHR                      presentMode_;
	// vk::Extent2D                            extent_;
	// std::unique_ptr<vk::raii::SwapchainKHR> swapchain_;
	// std::vector<vk::Image>                  images_;
	// std::vector<vk::raii::ImageView>        imageViews_;
	// std::vector<vk::raii::Framebuffer>      framebuffers_;
	// AttachmentResource                      depthResource_;
};
}        // namespace mz