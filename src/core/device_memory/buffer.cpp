#include "buffer.hpp"
#include "common/error.hpp"
#include "common/utils.hpp"

namespace mz
{

Buffer::Buffer(Key<DeviceMemoryAllocator> key, VmaAllocator allocator, std::nullptr_t nptr) :
    DeviceMemoryObject(allocator, key)
{
	handle_ = nullptr;
}

Buffer::Buffer(Key<DeviceMemoryAllocator> key, VmaAllocator allocator, vk::BufferCreateInfo &buffer_cinfo, VmaAllocationCreateInfo &allocation_cinfo) :
    DeviceMemoryObject(allocator, key)
{
	details_.allocator = allocator;
	is_persistent_     = allocation_cinfo.flags & VMA_ALLOCATION_CREATE_MAPPED_BIT;
	VkBuffer c_buf_handle;
	VK_CHECK(vmaCreateBuffer(details_.allocator, reinterpret_cast<VkBufferCreateInfo *>(&buffer_cinfo), &allocation_cinfo, &c_buf_handle, &details_.allocation, &details_.allocation_info));
	handle_ = c_buf_handle;
	update_flags();
}

Buffer::~Buffer()
{
	if (handle_)
	{
		vmaDestroyBuffer(details_.allocator, handle_, details_.allocation);
	}
};

Buffer::Buffer(Buffer &&rhs) :
    DeviceMemoryObject(std::move(rhs)),
    is_persistent_(rhs.is_persistent_),
    p_mapped_data_(rhs.p_mapped_data_)
{
	rhs.p_mapped_data_ = nullptr;
}

void Buffer::update(void *p_data, size_t size, size_t offset)
{
	update(to_ubyte_ptr(p_data), size, offset);
}

void Buffer::update(const std::vector<uint8_t> &binary, size_t offset)
{
	update(binary.data(), binary.size(), offset);
}

void Buffer::update(const uint8_t *p_data, size_t size, size_t offset)
{
	if (is_persistent_)
	{
		std::copy(p_data, p_data + size, to_ubyte_ptr(details_.allocation_info.pMappedData) + offset);
	}
	else
	{
		map();
		std::copy(p_data, p_data + size, to_ubyte_ptr(p_mapped_data_) + offset);
		flush();
		unmap();
	}
}

void Buffer::map()
{
	assert(is_mappable());
	if (p_mapped_data_)
	{
		vmaMapMemory(details_.allocator, details_.allocation, &p_mapped_data_);
	}
}

void Buffer::unmap()
{
	if (p_mapped_data_)
	{
		vmaUnmapMemory(details_.allocator, details_.allocation);
		p_mapped_data_ = nullptr;
	}
}

void Buffer::flush()
{
	if (p_mapped_data_)
	{
		vmaFlushAllocation(details_.allocator, details_.allocation, 0, details_.allocation_info.size);
	}
}
}        // namespace mz