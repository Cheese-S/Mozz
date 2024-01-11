#include "queue.hpp"

#include "device.hpp"

namespace mz
{

Queue::Queue(Device &device, uint32_t family_idx, uint32_t queue_idx) :
    family_idx_(family_idx)
{
	handle_ = device.get_handle().getQueue(family_idx_, queue_idx);
}

uint32_t Queue::get_family_idx() const
{
	return family_idx_;
}
}        // namespace mz