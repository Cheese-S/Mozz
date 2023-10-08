#pragma once

#include "scene_graph/components/image.hpp"

namespace mz::sg
{
class Stb : public Image
{
  public:
	Stb(const std::string &name, const std::vector<uint8_t> &data);
	virtual ~Stb() = default;
};
}        // namespace mz::sg