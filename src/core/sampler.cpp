#include "sampler.hpp"

#include "device.hpp"
#include "physical_device.hpp"

namespace mz
{

vk::SamplerCreateInfo Sampler::linear_clamp_cinfo(const PhysicalDevice &physical_device, float max_lod)
{
	vk::SamplerCreateInfo sampler_cinfo{
	    .magFilter        = vk::Filter::eLinear,
	    .minFilter        = vk::Filter::eLinear,
	    .mipmapMode       = vk::SamplerMipmapMode::eLinear,
	    .addressModeU     = vk::SamplerAddressMode::eClampToEdge,
	    .addressModeV     = vk::SamplerAddressMode::eClampToEdge,
	    .addressModeW     = vk::SamplerAddressMode::eClampToEdge,
	    .mipLodBias       = 0.0f,
	    .anisotropyEnable = physical_device.get_handle().getFeatures().samplerAnisotropy,
	    .maxAnisotropy    = physical_device.get_handle().getProperties().limits.maxSamplerAnisotropy,
	    .compareOp        = vk::CompareOp::eNever,
	    .minLod           = 0.0f,
	    .maxLod           = max_lod,
	    .borderColor      = vk::BorderColor::eFloatOpaqueBlack,
	};
	return sampler_cinfo;
}

Sampler::Sampler(const Device &device, vk::SamplerCreateInfo &sampler_cinfo) :
    device_(device)
{
	handle_ = device_.get_handle().createSampler(sampler_cinfo);
}

Sampler::Sampler(Sampler &&rhs) :
    device_(rhs.device_),
    VulkanObject<vk::Sampler>(std::move(rhs)){};

Sampler::~Sampler()
{
	if (handle_)
	{
		device_.get_handle().destroySampler(handle_);
	}
}

}        // namespace mz