#pragma once

#include "common/vk_common.hpp"
#include "vulkan_object.hpp"

#include <optional>

namespace mz
{
class Instance;

struct QueueFamilyIndices
{
	std::optional<uint32_t> graphics_index;
	std::optional<uint32_t> present_index;
	std::optional<uint32_t> compute_index;

	bool is_complete() const
	{
		return graphics_index.has_value() && present_index.has_value() && compute_index.has_value();
	}
};

struct SwapchainSupportDetails
{
	vk::SurfaceCapabilitiesKHR        capabilities;
	std::vector<vk::SurfaceFormatKHR> formats;
	std::vector<vk::PresentModeKHR>   present_modes;
};

class PhysicalDevice : public VulkanObject<typename vk::PhysicalDevice>
{
  public:
	PhysicalDevice(vk::PhysicalDevice handle, Instance &instance);
	~PhysicalDevice() override                        = default;
	PhysicalDevice(const PhysicalDevice &)            = delete;
	PhysicalDevice &operator=(const PhysicalDevice &) = delete;
	PhysicalDevice &operator=(PhysicalDevice &&)      = delete;
	PhysicalDevice(PhysicalDevice &&)                 = default;
	bool is_all_extensions_supported(const std::vector<const char *> &required_extensions) const;

	SwapchainSupportDetails   get_swapchain_support_details() const;
	const QueueFamilyIndices &get_queue_family_indices() const;
	uint32_t                  get_graphics_queue_family_index() const;
	uint32_t                  get_compute_queue_family_index() const;
	uint32_t                  get_present_queue_family_index() const;

  private:
	void find_queue_familiy_indices();

	Instance          &instance_;
	QueueFamilyIndices indices_;
};
}        // namespace mz