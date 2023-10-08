#include "device.hpp"

#include "command_buffer.hpp"
#include "command_pool.hpp"
#include "common/common.hpp"
#include "common/utils.hpp"
#include "instance.hpp"
#include "physical_device.hpp"

#include <set>

namespace mz
{
const std::vector<const char *> Device::REQUIRED_EXTENSIONS = {
    VK_KHR_SWAPCHAIN_EXTENSION_NAME,
#ifdef __IS_ON_OSX__
    "VK_KHR_portability_subset"
#endif
};

Device::Device(Instance &instance, PhysicalDevice &physical_device) :
    instance_(instance),
    physical_device_(physical_device)
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

	vk::PhysicalDeviceFeatures required_features;
	required_features.samplerAnisotropy = true;
	required_features.sampleRateShading = true;

	vk::DeviceCreateInfo device_cinfo{
	    .flags                   = {},
	    .queueCreateInfoCount    = to_u32(queue_cinfos.size()),
	    .pQueueCreateInfos       = queue_cinfos.data(),
	    .enabledLayerCount       = to_u32(instance_.VALIDATION_LAYERS.size()),
	    .ppEnabledLayerNames     = instance_.VALIDATION_LAYERS.data(),
	    .enabledExtensionCount   = to_u32(REQUIRED_EXTENSIONS.size()),
	    .ppEnabledExtensionNames = REQUIRED_EXTENSIONS.data(),
	    .pEnabledFeatures        = &required_features,
	};

	handle_ = physical_device.get_handle().createDevice(device_cinfo);

	graphics_queue_ = handle_.getQueue(indices.graphics_index.value(), 0);
	present_queue_  = handle_.getQueue(indices.present_index.value(), 0);
	compute_queue_  = handle_.getQueue(indices.compute_index.value(), 0);

	p_device_memory_allocator_ = std::make_unique<DeviceMemoryAllocator>(*this);
	p_one_time_buf_pool_       = std::make_unique<CommandPool>(*this, graphics_queue_, indices.graphics_index.value(), CommandPoolResetStrategy::eIndividual, vk::CommandPoolCreateFlagBits::eResetCommandBuffer | vk::CommandPoolCreateFlagBits::eTransient);
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

	graphics_queue_.submit(submit_info);
	graphics_queue_.waitIdle();
	p_one_time_buf_pool_->free_command_buffer(cmd_buf);
}

const Instance &Device::get_instance() const
{
	return instance_;
}

const PhysicalDevice &Device::get_physical_device() const
{
	return physical_device_;
}

const vk::Queue &Device::get_graphics_queue() const
{
	return graphics_queue_;
}

const vk::Queue &Device::get_present_queue() const
{
	return present_queue_;
}

const vk::Queue &Device::get_compute_queue() const
{
	return compute_queue_;
}

const DeviceMemoryAllocator &Device::get_device_memory_allocator() const
{
	return *p_device_memory_allocator_;
}

}        // namespace mz