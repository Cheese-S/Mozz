#include "device.hpp"

#include "command_buffer.hpp"
#include "command_pool.hpp"
#include "common/common.hpp"
#include "common/utils.hpp"
#include "device_memory/buffer.hpp"
#include "instance.hpp"
#include "physical_device.hpp"
#include "queue.hpp"

#include <set>

namespace mz
{
const std::vector<const char *> Device::REQUIRED_EXTENSIONS{
    VK_KHR_SWAPCHAIN_EXTENSION_NAME,
#ifdef __IS_ON_OSX__
    "VK_KHR_portability_subset"
#endif
};

const std::vector<const char *> Device::RAY_TRACING_EXTENSIONS = {
    VK_KHR_ACCELERATION_STRUCTURE_EXTENSION_NAME,
    VK_KHR_RAY_TRACING_PIPELINE_EXTENSION_NAME,
    VK_KHR_DEFERRED_HOST_OPERATIONS_EXTENSION_NAME,
};

Device::Device(Instance &instance, PhysicalDevice &physical_device, std::vector<const char *> &requested_extensions, vk::PhysicalDeviceFeatures2 &requested_features) :
    instance_(instance)
{
	QueueFamilyIndices indices        = physical_device.get_queue_family_indices();
	std::set<uint32_t> unique_indices = {indices.compute_index.value(), indices.graphics_index.value(), indices.present_index.value()};

	std::vector<vk::DeviceQueueCreateInfo> queue_cinfos;
	float                                  priority = 1.0f;
	for (uint32_t index : unique_indices)
	{
		vk::DeviceQueueCreateInfo queue_cinfo{};
		queue_cinfo.queueFamilyIndex = index;
		queue_cinfo.queueCount       = 1;
		queue_cinfo.pQueuePriorities = &priority;
		queue_cinfos.push_back(queue_cinfo);
	}

	vk::DeviceCreateInfo device_cinfo{
	    .pNext                   = &requested_features,
	    .flags                   = {},
	    .queueCreateInfoCount    = to_u32(queue_cinfos.size()),
	    .pQueueCreateInfos       = queue_cinfos.data(),
	    .enabledLayerCount       = to_u32(Instance::VALIDATION_LAYERS.size()),
	    .ppEnabledLayerNames     = instance.VALIDATION_LAYERS.data(),
	    .enabledExtensionCount   = to_u32(requested_extensions.size()),
	    .ppEnabledExtensionNames = requested_extensions.data(),
	};

	handle_ = physical_device.get_handle().createDevice(device_cinfo);

	VULKAN_HPP_DEFAULT_DISPATCHER.init(handle_);

	p_graphics_queue_ = std::make_unique<Queue>(*this, indices.graphics_index.value(), 0);
	p_present_queue_  = std::make_unique<Queue>(*this, indices.present_index.value(), 0);
	p_compute_queue_  = std::make_unique<Queue>(*this, indices.compute_index.value(), 0);

	p_device_memory_allocator_ = std::make_unique<DeviceMemoryAllocator>(instance, physical_device, *this);
	p_one_time_buf_pool_       = std::make_unique<CommandPool>(*this, *p_graphics_queue_, CommandPoolResetStrategy::eIndividual, vk::CommandPoolCreateFlagBits::eResetCommandBuffer | vk::CommandPoolCreateFlagBits::eTransient);
}

Device::~Device()
{
	p_one_time_buf_pool_.reset();
	p_device_memory_allocator_.reset();
	handle_.destroy();
}

CommandBuffer Device::begin_one_time_buf() const
{
	CommandBuffer cmd_buf = p_one_time_buf_pool_->allocate_command_buffer();
	cmd_buf.begin(vk::CommandBufferUsageFlagBits::eOneTimeSubmit);
	return cmd_buf;
}

void Device::end_one_time_buf(CommandBuffer &cmd_buf) const
{
	const auto &cmd_buf_handle = cmd_buf.get_handle();
	cmd_buf_handle.end();
	vk::SubmitInfo submit_info{
	    .pWaitSemaphores    = nullptr,
	    .pWaitDstStageMask  = nullptr,
	    .commandBufferCount = 1,
	    .pCommandBuffers    = &cmd_buf_handle,
	};

	p_graphics_queue_->get_handle().submit(submit_info);
	p_graphics_queue_->get_handle().waitIdle();
	p_one_time_buf_pool_->free_command_buffer(cmd_buf);
}

vk::DeviceAddress Device::get_buffer_device_address(Buffer &buffer)
{
	vk::BufferDeviceAddressInfo buf_ainfo{
	    .buffer = buffer.get_handle(),
	};
	return handle_.getBufferAddress(buf_ainfo);
}

const Queue &Device::get_graphics_queue() const
{
	return *p_graphics_queue_;
}

const Queue &Device::get_present_queue() const
{
	return *p_present_queue_;
}

const Queue &Device::get_compute_queue() const
{
	return *p_compute_queue_;
}

const DeviceMemoryAllocator &Device::get_device_memory_allocator() const
{
	return *p_device_memory_allocator_;
}

}        // namespace mz