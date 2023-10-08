#pragma once

#include "common/utils.hpp"
#include "common/vk_common.hpp"
#include "core/vulkan_object.hpp"
#include <vk_mem_alloc.h>

namespace mz
{

class DeviceMemoryAllocator;

struct DeviceAllocationDetails
{
	VmaAllocator          allocator;
	VmaAllocation         allocation;
	VmaAllocationInfo     allocation_info;
	VkMemoryPropertyFlags flags;
};

template <typename THandle>
class DeviceMemoryObject : public VulkanObject<THandle>
{
  public:
	DeviceMemoryObject(VmaAllocator allocator, Key<DeviceMemoryAllocator> const &key)
	{
		details_.allocator = allocator;
	}

	DeviceMemoryObject(DeviceMemoryObject &&rhs) :
	    VulkanObject<THandle>(std::move(rhs)),
	    details_(rhs.details_)
	{
		rhs.details_.allocator  = nullptr;
		rhs.details_.allocation = nullptr;
	}

	virtual ~DeviceMemoryObject() = default;

	void update_flags()
	{
		if (details_.allocator)
		{
			vmaGetAllocationMemoryProperties(details_.allocator, details_.allocation, &details_.flags);
		}
	}
	inline bool is_mappable()
	{
		return details_.flags & VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT;
	}

  protected:
	DeviceAllocationDetails details_;
};

}        // namespace mz