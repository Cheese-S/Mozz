#include "sampler.hpp"

#include "core/device.hpp"

namespace mz::sg
{
Sampler::Sampler(const Device &device, const std::string &name, size_t id, vk::SamplerCreateInfo &sampler_cinfo) :
    Component(name, id),
    device_(device),
    sampler_(device_, sampler_cinfo){};

std::type_index Sampler::get_type()
{
	return typeid(Sampler);
}

vk::Sampler Sampler::get_handle()
{
	return sampler_.get_handle();
}
}        // namespace mz::sg
