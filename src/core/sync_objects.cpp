#include "sync_objects.hpp"

#include "device.hpp"

namespace mz
{

Fence::Fence(Device &device, vk::FenceCreateFlags flags) :
    device_(device)
{
	vk::FenceCreateInfo fence_cinfo{
	    .flags = flags,
	};
	handle_ = device_.get_handle().createFence(fence_cinfo);
}

Fence::~Fence()
{
	if (handle_)
	{
		device_.get_handle().destroyFence(handle_);
	}
}

Semaphore::Semaphore(Device &device) :
    device_(device)
{
	vk::SemaphoreCreateInfo semaphore_cinfo{};
	handle_ = device_.get_handle().createSemaphore(semaphore_cinfo);
}

Semaphore::~Semaphore()
{
	if (handle_)
	{
		device_.get_handle().destroySemaphore(handle_);
	}
}

}        // namespace mz