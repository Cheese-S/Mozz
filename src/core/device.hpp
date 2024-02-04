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
class Queue;

class Device : public VulkanObject<typename vk::Device>
{
  public:
	static const std::vector<const char *> REQUIRED_EXTENSIONS;
	static const std::vector<const char *> RAY_TRACING_EXTENSIONS;

	Device(Instance &instance, PhysicalDevice &physical_device, std::vector<const char *> &requested_extensions, vk::PhysicalDeviceFeatures2 &requested_features);
	~Device() override;

	CommandBuffer begin_one_time_buf() const;
	void          end_one_time_buf(CommandBuffer &cmd_buf) const;

	vk::DeviceAddress get_buffer_device_address(Buffer &buffer);

	const Queue                 &get_graphics_queue() const;
	const Queue                 &get_present_queue() const;
	const Queue                 &get_compute_queue() const;
	const DeviceMemoryAllocator &get_device_memory_allocator() const;

  private:
	Instance                              &instance_;
	std::unique_ptr<Queue>                 p_graphics_queue_;
	std::unique_ptr<Queue>                 p_present_queue_;
	std::unique_ptr<Queue>                 p_compute_queue_;
	std::unique_ptr<DeviceMemoryAllocator> p_device_memory_allocator_;
	std::unique_ptr<CommandPool>           p_one_time_buf_pool_;
};
}        // namespace mz