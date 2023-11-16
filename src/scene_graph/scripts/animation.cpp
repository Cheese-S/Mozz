#include "animation.hpp"

#include <iostream>

namespace mz::sg
{

inline glm::vec3 compute_cubic_spline(const std::vector<glm::vec3> &outputs, size_t i, float interp_val, float delta);

inline glm::quat compute_cubic_spline(const std::vector<glm::quat> &outputs, size_t i, float interp_val, float delta);

Animation::Animation(const std::string &name, size_t id) :
    Script(name, id)
{
}

void Animation::update(float delta_time)
{
	current_time_ += delta_time;
	if (current_time_ > end_time_)
	{
		current_time_ -= end_time_;
	}

	for (const auto &channel : channels_)
	{
		update_by_channel(channel);
	}
}

void Animation::update_by_channel(const AnimationChannel &channel)
{
	const AnimationSampler &sampler = channel.sampler;
	for (size_t i = 0; i < sampler.inputs.size() - 1; i++)
	{
		if (current_time_ >= sampler.inputs[i] && current_time_ <= sampler.inputs[i + 1])
		{
			if (sampler.type == AnimationType::eLinear)
			{
				linear_update(channel, i);
			}
			else if (sampler.type == AnimationType::eStep)
			{
				step_update(channel, i);
			}
			else if (sampler.type == AnimationType::eCubicSpline)
			{
				cubic_spline_update(channel, i);
			}
		}
	}
}

void Animation::linear_update(const AnimationChannel &channel, size_t i)
{
	Transform &T          = channel.node.get_transform();
	float      interp_val = (current_time_ - channel.sampler.inputs[i]) / (channel.sampler.inputs[i + 1] - channel.sampler.inputs[i]);

	switch (channel.target)
	{
		case AnimationTarget::eTranslation:
		{
			const std::vector<glm::vec3> &vecs = channel.sampler.get_vecs();
			T.set_tranlsation(glm::mix(vecs[i], vecs[i + 1], interp_val));
			break;
		}
		case AnimationTarget::eRotation:
		{
			const std::vector<glm::quat> &quats = channel.sampler.get_quats();
			T.set_rotation(glm::normalize(glm::slerp(quats[i], quats[i + 1], interp_val)));
			break;
		}

		case AnimationTarget::eScale:
		{
			const std::vector<glm::vec3> &vecs = channel.sampler.get_vecs();
			T.set_scale(glm::mix(vecs[i], vecs[i + 1], interp_val));
			break;
		}
	}
}

void Animation::step_update(const AnimationChannel &channel, size_t i)
{
	Transform &T = channel.node.get_transform();
	switch (channel.target)
	{
		case AnimationTarget::eTranslation:
		{
			T.set_tranlsation(channel.sampler.get_vecs()[i]);
			break;
		}
		case AnimationTarget::eRotation:
		{
			T.set_rotation(glm::normalize(channel.sampler.get_quats()[i]));
			break;
		}
		case AnimationTarget::eScale:
		{
			T.set_scale(channel.sampler.get_vecs()[i]);
			break;
		}
	}
}

void Animation::cubic_spline_update(const AnimationChannel &channel, size_t i)
{
	Transform              &T          = channel.node.get_transform();
	const AnimationSampler &sampler    = channel.sampler;
	float                   delta      = sampler.inputs[i + 1] - channel.sampler.inputs[i];
	float                   interp_val = (current_time_ - sampler.inputs[i]) / (sampler.inputs[i + 1] - sampler.inputs[i]);

	switch (channel.target)
	{
		case AnimationTarget::eTranslation:
			T.set_tranlsation(compute_cubic_spline(sampler.get_vecs(), i, interp_val, delta));
			break;
		case AnimationTarget::eRotation:

			T.set_rotation(glm::normalize(compute_cubic_spline(sampler.get_quats(), i, interp_val, delta)));
			break;
		case AnimationTarget::eScale:
			T.set_scale(compute_cubic_spline(sampler.get_vecs(), i, interp_val, delta));
			break;
	}
}

void Animation::add_channel(Node &node, const AnimationTarget &target, const AnimationSampler &sampler)
{
	channels_.push_back({node, target, sampler});
}

void Animation::update_interval()
{
	for (auto &channel : channels_)
	{
		for (float time : channel.sampler.inputs)
		{
			if (time < start_time_)
			{
				start_time_ = time;
			}
			if (time > end_time_)
			{
				end_time_ = time;
			}
		}
	}
}

void Animation::set_channels(std::vector<AnimationChannel> &&channels)
{
	channels_ = std::move(channels);
}

inline glm::vec3 compute_cubic_spline(const std::vector<glm::vec3> &outputs, size_t i, float interp_val, float delta)
{
	glm::vec3 p0 = outputs[i * 3 + 1];
	glm::vec3 p1 = outputs[(i + 1) * 3 + 1];

	glm::vec3 m0 = delta * outputs[i * 3 + 2];
	glm::vec3 m1 = delta * outputs[(i + 1) * 3 + 0];

	glm::vec3 result = (2.0f * glm::pow(interp_val, 3.0f) - 3.0f * glm::pow(interp_val, 2.0f) + 1.0f) * p0 + (glm::pow(interp_val, 3.0f) - 2.0f * glm::pow(interp_val, 2.0f) + interp_val) * m0 + (-2.0f * glm::pow(interp_val, 3.0f) + 3.0f * glm::pow(interp_val, 2.0f)) * p1 + (glm::pow(interp_val, 3.0f) - glm::pow(interp_val, 2.0f)) * m1;

	return result;
}

inline glm::quat compute_cubic_spline(const std::vector<glm::quat> &outputs, size_t i, float interp_val, float delta)
{
	glm::quat p0 = outputs[i * 3 + 1];
	glm::quat p1 = outputs[(i + 1) * 3 + 1];

	glm::quat m0 = delta * outputs[i * 3 + 2];
	glm::quat m1 = delta * outputs[(i + 1) * 3 + 0];

	glm::quat result = (2.0f * glm::pow(interp_val, 3.0f) - 3.0f * glm::pow(interp_val, 2.0f) + 1.0f) * p0 + (glm::pow(interp_val, 3.0f) - 2.0f * glm::pow(interp_val, 2.0f) + interp_val) * m0 + (-2.0f * glm::pow(interp_val, 3.0f) + 3.0f * glm::pow(interp_val, 2.0f)) * p1 + (glm::pow(interp_val, 3.0f) - glm::pow(interp_val, 2.0f)) * m1;

	return result;
}
}        // namespace mz::sg