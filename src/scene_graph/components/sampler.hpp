#pragma once

#include "common/vk_common.hpp"
#include "core/sampler.hpp"
#include "scene_graph/component.hpp"

namespace mz
{
class Device;

namespace sg
{
class Sampler : public Component
{
  public:
	Sampler(const Device &device, const std::string &name, size_t id, vk::SamplerCreateInfo &sampler_cinfo);

	virtual ~Sampler() override = default;
	virtual std::type_index get_type() override;
	vk::Sampler             get_handle();

  private:
	const Device &device_;
	mz::Sampler   sampler_;
};
}        // namespace sg
}        // namespace mz