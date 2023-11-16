#include "pipeline_layout.hpp"

#include "device.hpp"

namespace mz
{

PipelineLayout::PipelineLayout(Device &device, vk::PipelineLayoutCreateInfo &pipeline_layout_cinfo) :
    device_(device)
{
	handle_ = device_.get_handle().createPipelineLayout(pipeline_layout_cinfo);
}

PipelineLayout::~PipelineLayout()
{
	if (handle_)
	{
		device_.get_handle().destroyPipelineLayout(handle_);
	}
}
}        // namespace mz