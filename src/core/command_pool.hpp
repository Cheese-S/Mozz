#pragma once

#include "common/vk_common.hpp"
#include "core/vulkan_object.hpp"

namespace mz
{

class Device;
class Queue;
class CommandBuffer;

enum class CommandPoolResetStrategy
{
	eIndividual,
	ePool
};

class CommandPool : public VulkanObject<vk::CommandPool>
{
  public:
	CommandPool(Device &device, const Queue &queue, CommandPoolResetStrategy strategy = CommandPoolResetStrategy::eIndividual, vk::CommandPoolCreateFlags flags = vk::CommandPoolCreateFlagBits::eResetCommandBuffer);

	~CommandPool() override;

	CommandBuffer              allocate_command_buffer(vk::CommandBufferLevel level = vk::CommandBufferLevel::ePrimary);
	std::vector<CommandBuffer> allocate_command_buffers(uint32_t count, vk::CommandBufferLevel level = vk::CommandBufferLevel::ePrimary);
	void                       free_command_buffers(std::vector<CommandBuffer> &cmd_bufs);
	void                       free_command_buffer(CommandBuffer &cmd_buf);
	void                       recycle_command_buffer(CommandBuffer &cmd_buf);

	void                     reset();
	CommandPoolResetStrategy get_reset_strategy();
	const Queue             &get_queue();
	const Device            &get_device();

  private:
	Device                    &device_;
	const Queue               &queue_;
	std::vector<CommandBuffer> primary_cmd_bufs_;
	std::vector<CommandBuffer> secondary_cmd_bufs_;
	CommandPoolResetStrategy   strategy_;
};

}        // namespace mz