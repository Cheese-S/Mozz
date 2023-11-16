#pragma once

#include <stdint.h>

#include "common/vk_common.hpp"
#include "core/image_resource.hpp"
#include "scene_graph/component.hpp"

namespace mz
{

class Device;

namespace sg
{
class Image : public Component
{
  public:
	Image(ImageResource &&resrc, const std::string &name, size_t id);
	Image(Image &&);

	virtual ~Image() = default;
	virtual std::type_index get_type() override;

	ImageResource           &get_resource();
	const ImageTransferInfo &get_image_transfer_info();

	void set_resource(ImageResource &&resource);

  private:
	ImageResource resource_;
};
}        // namespace sg

}        // namespace mz