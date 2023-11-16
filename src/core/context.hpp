#pragma once

#include "device.hpp"
#include "instance.hpp"
#include "physical_device.hpp"

namespace mz
{

struct ContextCreateInfo
{
	// features
	std::string                  app_name;
	std::vector<const char *>    device_extensions;
	vk::PhysicalDeviceFeatures2 &requested_features;
	Window                      &window;
};

struct Context
{
  public:
	Context(ContextCreateInfo &context_cinfo);

	Instance       instance;
	PhysicalDevice physical_device;
	Device         device;
};
}        // namespace mz
