#pragma once

#include <variant>

#include "scene_graph/script.hpp"

namespace mz::sg
{

enum class AnimationType
{
	eLinear,
	eStep,
	eCubicSpline,
};

enum class AnimationTarget
{
	eTranslation,
	eRotation,
	eScale
};

struct AnimationSampler
{
	void init_vecs()
	{
		outputs_.emplace<std::vector<glm::vec3>>();
	};

	void init_quats()
	{
		outputs_.emplace<std::vector<glm::quat>>();
	};

	const std::vector<glm::vec3> &get_vecs() const
	{
		assert(outputs_.index() == 0);
		return std::get<std::vector<glm::vec3>>(outputs_);
	}

	const std::vector<glm::quat> &get_quats() const
	{
		assert(outputs_.index() == 1);
		return std::get<std::vector<glm::quat>>(outputs_);
	}

	std::vector<glm::vec3> &get_mut_vecs()
	{
		assert(outputs_.index() == 0);
		return std::get<std::vector<glm::vec3>>(outputs_);
	}

	std::vector<glm::quat> &get_mut_quats()
	{
		assert(outputs_.index() == 1);
		return std::get<std::vector<glm::quat>>(outputs_);
	}

	AnimationType      type;
	std::vector<float> inputs;

  private:
	std::variant<
	    std::vector<glm::vec3>,
	    std::vector<glm::quat>>
	    outputs_;
};

struct AnimationChannel
{
	Node            &node;
	AnimationTarget  target;
	AnimationSampler sampler;
};

class Animation : public Script
{
  public:
	Animation(const std::string &name, size_t id);

	void update(float delta_time) override;

	void update_interval();
	void set_channels(std::vector<AnimationChannel> &&channels);
	void add_channel(Node &node, const AnimationTarget &target, const AnimationSampler &sampler);

  private:
	void update_by_channel(const AnimationChannel &channel);
	void linear_update(const AnimationChannel &channel, size_t i);
	void step_update(const AnimationChannel &channel, size_t i);
	void cubic_spline_update(const AnimationChannel &channel, size_t i);

	std::vector<AnimationChannel> channels_;
	float                         current_time_{0.0f};
	float                         start_time_{std::numeric_limits<float>::max()};
	float                         end_time_{std::numeric_limits<float>::min()};
};

}        // namespace mz::sg