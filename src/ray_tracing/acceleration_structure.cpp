#include "acceleration_structure.hpp"

#include "core/device.hpp"

namespace mz
{

AccelerationStructure::AccelerationStructure(Device &device, vk::AccelerationStructureCreateInfoKHR &as_cinfo) :
    device_(device),
    buf_(device_.get_device_memory_allocator().allocate_AS_buffer(as_cinfo.size))
{
	as_cinfo.buffer = buf_.get_handle();
	handle_         = device_.get_handle().createAccelerationStructureKHR(as_cinfo);
}

AccelerationStructure::~AccelerationStructure()
{
	if (handle_)
	{
		device_.get_handle().destroyAccelerationStructureKHR(handle_);
	}
}

AccelerationStructure::AccelerationStructure(AccelerationStructure &&rhs) :
    VulkanObject(std::move(rhs)),
    device_(rhs.device_),
    buf_(std::move(rhs.buf_))
{
}

vk::DeviceAddress AccelerationStructure::get_as_device_address()
{
	vk::AccelerationStructureDeviceAddressInfoKHR as_ainfo{
	    .accelerationStructure = handle_,
	};
	return device_.get_handle().getAccelerationStructureAddressKHR(as_ainfo);
}

Buffer &AccelerationStructure::get_buffer()
{
	return buf_;
}

}        // namespace mz