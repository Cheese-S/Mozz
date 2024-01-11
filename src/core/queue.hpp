#pragma once

#include "common/vk_common.hpp"
#include "vulkan_object.hpp"

namespace mz
{
class Device;

class Queue : public VulkanObject<vk::Queue>
{
  public:
	Queue(Device &device, uint32_t family_idx, uint32_t queue_idx);

	uint32_t get_family_idx() const;

  private:
	uint32_t family_idx_;
};

}        // namespace mz
