#pragma once

#include "common/vk_common.hpp"
#include "core/device_memory/buffer.hpp"
#include "core/vulkan_object.hpp"

namespace mz
{
class Device;
class Buffer;

class AccelerationStructure : public VulkanObject<vk::AccelerationStructureKHR>
{
  public:
	AccelerationStructure(Device &device, vk::AccelerationStructureCreateInfoKHR &as_cinfo);
	~AccelerationStructure() override;
	AccelerationStructure(AccelerationStructure &&rhs);

	vk::DeviceAddress get_as_device_address();

	Buffer &get_buffer();

  private:
	Device &device_;
	Buffer  buf_;
};        // namespace mz;
}        // namespace mz