#pragma once

#include "common/vk_common.hpp"
#include "core/device_memory/allocator.hpp"
#include "core/device_memory/device_memory_object.hpp"

namespace mz
{

template <typename T>
class Key;

class Image : public DeviceMemoryObject<vk::Image>
{
  public:
	Image(Key<DeviceMemoryAllocator> key, VmaAllocator allocator, std::nullptr_t nptr);
	Image(Key<DeviceMemoryAllocator> key, VmaAllocator allocator, vk::ImageCreateInfo &image_cinfo, VmaAllocationCreateInfo &allocation_cinfo);
	~Image();
	Image(Image &&rhs);
	Image &operator=(Image &&);

	vk::Extent3D get_base_extent();
	vk::Format   get_format();

  private:
	vk::Extent3D base_extent_;
	vk::Format   format_;
};
}        // namespace mz