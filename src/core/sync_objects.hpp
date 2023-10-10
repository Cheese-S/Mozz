#pragma once

#include "common/vk_common.hpp"
#include "core/vulkan_object.hpp"

namespace mz
{
class Device;

class Fence : public VulkanObject<vk::Fence>
{
  public:
	Fence(Device &device, vk::FenceCreateFlags flags);
	Fence(Fence &&) = default;
	~Fence() override;

  private:
	Device &device_;
};

class Semaphore : public VulkanObject<vk::Semaphore>
{
  public:
	Semaphore(Device &device);
	Semaphore(Semaphore &&) = default;
	~Semaphore() override;

  private:
	Device &device_;
};

}        // namespace mz