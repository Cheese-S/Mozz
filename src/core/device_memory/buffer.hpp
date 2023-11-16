#pragma once
#include "common/vk_common.hpp"
#include "core/device_memory/device_memory_object.hpp"
#include "core/vulkan_object.hpp"

namespace mz
{

template <typename T>
class Key;

class Buffer : public DeviceMemoryObject<vk::Buffer>
{
  public:
	Buffer(Key<DeviceMemoryAllocator> key, VmaAllocator allocator, std::nullptr_t nptr);
	Buffer(Key<DeviceMemoryAllocator> key, VmaAllocator allocator, vk::BufferCreateInfo &buffer_cinfo, VmaAllocationCreateInfo &allocation_cinfo);
	Buffer(Buffer &&);
	Buffer(Buffer const &)            = delete;
	Buffer &operator=(Buffer const &) = delete;
	Buffer &operator=(Buffer &&)      = delete;
	~Buffer();

	void update(void *p_data, size_t size, size_t offset = 0);
	void update(const std::vector<uint8_t> &binary, size_t offset = 0);
	void update(const uint8_t *p_data, size_t size, size_t offset = 0);

	template <typename T>
	void update(std::vector<T> &vec)
	{
		update(vec.data(), sizeof(T) * vec.size());
	}

  private:
	void map();
	void unmap();
	void flush();

	bool  is_persistent_;
	void *p_mapped_data_ = nullptr;
};

}        // namespace mz