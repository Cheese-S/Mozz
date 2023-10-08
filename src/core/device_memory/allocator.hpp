#pragma once

#include "common/vk_common.hpp"
#include "core/vulkan_object.hpp"

#include <vk_mem_alloc.h>

namespace mz
{
class Device;
class Image;
class Buffer;

class DeviceMemoryAllocator : public VulkanObject<VmaAllocator>
{
  public:
	DeviceMemoryAllocator(Device &device);
	~DeviceMemoryAllocator();

	Buffer allocate_staging_buffer(size_t size) const;
	Buffer allocate_vertex_buffer(size_t size) const;
	Buffer allocate_index_buffer(size_t size) const;
	Buffer allocate_uniform_buffer(size_t size) const;
	Buffer allocate_buffer(vk::BufferCreateInfo &buffer_cinfo, VmaAllocationCreateInfo &alloc_cinfo) const;
	Buffer allocate_null_buffer() const;

	Image allocate_device_only_image(vk::ImageCreateInfo &image_cinfo) const;
	Image allocate_image(vk::ImageCreateInfo &image_cinfo, VmaAllocationCreateInfo &alloc_cinfo) const;
	Image allocate_null_image() const;
};

}        // namespace mz