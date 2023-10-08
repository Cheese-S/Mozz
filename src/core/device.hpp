#pragma once

#include <memory>

#include "common/vk_common.hpp"
#include "device_memory/allocator.hpp"
#include "vulkan_object.hpp"

namespace mz
{
class Instance;
class PhysicalDevice;
class DeviceMemoryAllocator;
class CommandPool;
class CommandBuffer;

class Device : public VulkanObject<typename vk::Device>
{
  public:
	static const std::vector<const char *> REQUIRED_EXTENSIONS;

	Device(Instance &instance, PhysicalDevice &physical_device);
	~Device() override;
	CommandBuffer begin_one_time_buf() const;
	void          end_one_time_buf(CommandBuffer &cmd_buf) const;

	const Instance              &get_instance() const;
	const PhysicalDevice        &get_physical_device() const;
	const vk::Queue             &get_graphics_queue() const;
	const vk::Queue             &get_present_queue() const;
	const vk::Queue             &get_compute_queue() const;
	const DeviceMemoryAllocator &get_device_memory_allocator() const;

  private:
	Instance                              &instance_;
	PhysicalDevice                        &physical_device_;
	std::unique_ptr<DeviceMemoryAllocator> p_device_memory_allocator_;
	vk::Queue                              graphics_queue_ = nullptr;
	vk::Queue                              present_queue_  = nullptr;
	vk::Queue                              compute_queue_  = nullptr;
	std::unique_ptr<CommandPool>           p_one_time_buf_pool_;
};
}        // namespace mz