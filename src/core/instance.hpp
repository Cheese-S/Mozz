#pragma once

#include "common/vk_common.hpp"
#include "vulkan_object.hpp"
#include <memory>

extern const std::vector<const char *> VALIDATION_LAYERS;
extern const std::vector<const char *> DEVICE_EXTENSIONS;

namespace mz
{
class Window;
class PhysicalDevice;

class Instance : public VulkanObject<vk::Instance>
{
  public:
	static const std::vector<const char *> VALIDATION_LAYERS;

	Instance(const std::string &app_name, Window &window);
	~Instance() override;
	Instance(const Instance &)            = delete;
	Instance &operator=(const Instance &) = delete;
	Instance(Instance &&)                 = delete;
	Instance &operator=(Instance &&)      = delete;

	PhysicalDevice pick_physical_device(const std::vector<const char *> &device_extensions, vk::PhysicalDeviceFeatures2 &required_features);

	const vk::SurfaceKHR &get_surface() const;

  private:
	void                      init_dispatcher();
	void                      create_instance(const std::string &app_name);
	bool                      is_validation_layer_supported();
	std::vector<const char *> get_required_extensions();
	std::vector<const char *> get_required_layers();
	void                      load_function_ptrs();
	void                      populate_debug_messenger_create_info(vk::DebugUtilsMessengerCreateInfoEXT &createInfo);
	void                      init_debug_messenger();
	inline static VKAPI_ATTR VkBool32 VKAPI_CALL
	     debug_callback(VkDebugUtilsMessageSeverityFlagBitsEXT      messageSeverity,
	                    VkDebugUtilsMessageTypeFlagsEXT             messageType,
	                    const VkDebugUtilsMessengerCallbackDataEXT *pCallbackData, void *pUserData);
	bool is_physical_device_suitable(const PhysicalDevice &device, const std::vector<const char *> &device_extensions, vk::PhysicalDeviceFeatures2 &required_features);

	vk::SurfaceKHR             surface_         = nullptr;
	vk::DebugUtilsMessengerEXT debug_messenger_ = nullptr;
};
}        // namespace mz