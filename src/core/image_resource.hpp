#pragma once

#include "common/vk_common.hpp"
#include "core/image_view.hpp"
#include "device_memory/image.hpp"

#include <memory>

namespace mz
{

struct ImageLoadResult;

struct ImageMetaInfo
{
	vk::Extent3D extent;
	vk::Format   format;
	uint32_t     levels;
};

struct ImageTransferInfo
{
	std::vector<uint8_t> binary;
	ImageMetaInfo        meta;
};

ImageTransferInfo stb_load(const std::string &path);
ImageTransferInfo gli_load(const std::string &path);

class ImageView;

class ImageResource
{
  public:
	static uint8_t           format_to_bits_per_pixel(vk::Format format);
	static ImageTransferInfo load_two_dim_image(const std::string &path);
	static ImageTransferInfo load_cubic_image(const std::string &path);
	static ImageResource     create_empty_two_dim_img_resrc(const Device &device, const ImageMetaInfo &meta);
	static ImageResource     create_empty_cubic_img_resrc(const Device &device, const ImageMetaInfo &meta);

	ImageResource(const Device &device, std::nullptr_t nptr);
	ImageResource(Image &&image, ImageView &&view);
	ImageResource(ImageResource &&rhs);
	ImageResource &operator=(ImageResource &&rhs);
	~ImageResource();

	Image           &get_image();
	const ImageView &get_view() const;

  private:
	Image     image_;
	ImageView view_;
};

struct ImageLoadResult
{
	ImageResource     resource;
	ImageTransferInfo image_tifno;
};

};        // namespace mz