#include "physical_device.hpp"

#include <set>

#include "instance.hpp"

namespace mz
{

PhysicalDevice::PhysicalDevice(vk::PhysicalDevice handle, Instance &instance) :
    VulkanObject(handle),
    instance_(instance)
{
	if (handle_)
	{
		find_queue_familiy_indices();
		find_ray_tracing_props();
	};
};

bool PhysicalDevice::is_extensions_supported(const std::vector<const char *> &required_extensions) const
{
	auto                  avaliable_extensions = handle_.enumerateDeviceExtensionProperties();
	std::set<std::string> required_set(required_extensions.begin(), required_extensions.end());
	for (const auto &extension : avaliable_extensions)
	{
		required_set.erase(extension.extensionName);
	}
	return required_set.empty();
}

bool PhysicalDevice::is_features_supported(vk::PhysicalDeviceFeatures2 &required_features) const
{
	return true;
}

void PhysicalDevice::find_queue_familiy_indices()
{
	QueueFamilyIndices indices;
	vk::SurfaceKHR     surface        = instance_.get_surface();
	auto               queue_families = handle_.getQueueFamilyProperties();

	for (size_t i = 0; i < queue_families.size(); i++)
	{
		const auto &queue_family = queue_families[i];
		if (queue_family.queueFlags & vk::QueueFlagBits::eGraphics)
		{
			indices.graphics_index = i;
		}
		if (queue_family.queueFlags & vk::QueueFlagBits::eCompute)
		{
			indices.compute_index = i;
		}

		bool is_present_supported = handle_.getSurfaceSupportKHR(i, surface);

		if (is_present_supported)
		{
			indices.present_index = i;
		}

		if (indices.is_complete())
		{
			break;
		}
	}
	indices_ = indices;
}

void PhysicalDevice::find_ray_tracing_props()
{
	auto prop_chain    = handle_.getProperties2<vk::PhysicalDeviceProperties2, vk::PhysicalDeviceRayTracingPipelinePropertiesKHR>();
	ray_tracing_props_ = prop_chain.get<vk::PhysicalDeviceRayTracingPipelinePropertiesKHR>();
}

SwapchainSupportInfo PhysicalDevice::get_swapchain_support_details() const
{
	vk::SurfaceKHR surface = instance_.get_surface();
	return {
	    .capabilities  = handle_.getSurfaceCapabilitiesKHR(surface),
	    .formats       = handle_.getSurfaceFormatsKHR(surface),
	    .present_modes = handle_.getSurfacePresentModesKHR(surface),
	};
}

const vk::PhysicalDeviceRayTracingPipelinePropertiesKHR &PhysicalDevice::get_ray_tracing_props() const
{
	return ray_tracing_props_;
}

const QueueFamilyIndices &PhysicalDevice::get_queue_family_indices() const
{
	return indices_;
}

uint32_t PhysicalDevice::get_graphics_queue_family_index() const
{
	return indices_.graphics_index.value();
}

uint32_t PhysicalDevice::get_present_queue_family_index() const
{
	return indices_.present_index.value();
}

uint32_t PhysicalDevice::get_compute_queue_family_index() const
{
	return indices_.compute_index.value();
}

}        // namespace mz
