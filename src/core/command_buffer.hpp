#pragma once
#include "common/vk_common.hpp"
#include "core/vulkan_object.hpp"

namespace mz
{
class Buffer;
class Image;
class ImageResource;
class CommandPool;

class CommandBuffer : public VulkanObject<vk::CommandBuffer>
{
	friend CommandPool;

  public:
	CommandBuffer(vk::CommandBuffer handle, CommandPool &pool, vk::CommandBufferLevel level);
	~CommandBuffer() override;
	CommandBuffer(CommandBuffer &&);
	CommandBuffer(const CommandBuffer &)            = delete;
	CommandBuffer &operator=(const CommandBuffer &) = delete;
	CommandBuffer &operator=(CommandBuffer &&)      = delete;

	void begin(vk::CommandBufferUsageFlags flag = {});
	void flush(vk::SubmitInfo submit_info);
	void reset();

	void set_image_layout(ImageResource &resource, vk::ImageLayout old_layout, vk::ImageLayout new_layout, vk::PipelineStageFlags src_stage_mask = vk::PipelineStageFlagBits::eAllCommands, vk::PipelineStageFlags dst_stage_mask = vk::PipelineStageFlagBits::eAllCommands);
	void update_image(ImageResource &resouce, Buffer &staging_buf);

	void copy_buffer(Buffer &src, Buffer &dst, size_t size);
	void copy_buffer(Buffer &src, Buffer &dst, vk::BufferCopy copy_region = {});

  private:
	std::vector<vk::BufferImageCopy> full_copy_regions(const vk::ImageSubresourceRange &subresource_range, vk::Extent3D base_extent, uint8_t bits_per_pixel);

  private:
	CommandPool           &pool_;
	vk::CommandBufferLevel level_;
};
}        // namespace mz