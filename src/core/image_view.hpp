#pragma once

#include "common/vk_common.hpp"
#include "core/vulkan_object.hpp"

namespace mz
{
class Device;
class ImageView : public VulkanObject<vk::ImageView>
{
  public:
	static vk::ImageViewCreateInfo two_dim_view_cinfo(vk::Image image, vk::Format format, vk::ImageAspectFlags aspct_flags, uint32_t mip_levels);
	static vk::ImageViewCreateInfo cube_view_cinfo(vk::Image image, vk::Format format, vk::ImageAspectFlags aspct_flags, uint32_t mip_levels);

	ImageView(const Device &device, std::nullptr_t nptr);
	ImageView(const Device &device, vk::ImageViewCreateInfo &image_view_cinfo);
	ImageView(ImageView &&rhs);
	ImageView &operator=(ImageView &&rhs);
	~ImageView() override;

	const vk::ImageSubresourceRange &get_subresource_range() const;

  private:
	const Device             &device_;
	vk::ImageSubresourceRange subresource_range_;
};
}        // namespace mz