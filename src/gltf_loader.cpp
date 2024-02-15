#include "gltf_loader.hpp"

#include <glm/gtc/type_ptr.hpp>
#include <queue>

#include <iostream>

#include "glm/gtx/string_cast.hpp"

#include "common/error.hpp"
#include "common/file_utils.hpp"
#include "common/utils.hpp"
#include "core/command_buffer.hpp"
#include "core/context.hpp"
#include "core/device.hpp"
#include "core/device_memory/buffer.hpp"
#include "core/image_view.hpp"
#include "core/instance.hpp"
#include "core/physical_device.hpp"

#include "scene_graph/components/aabb.hpp"
#include "scene_graph/components/camera.hpp"
#include "scene_graph/components/image.hpp"
#include "scene_graph/components/mesh.hpp"
#include "scene_graph/components/pbr_material.hpp"
#include "scene_graph/components/perspective_camera.hpp"
#include "scene_graph/components/sampler.hpp"
#include "scene_graph/components/skin.hpp"
#include "scene_graph/components/submesh.hpp"
#include "scene_graph/components/texture.hpp"
#include "scene_graph/components/transform.hpp"
#include "scene_graph/node.hpp"
#include "scene_graph/scene.hpp"
#include "scene_graph/scripts/animation.hpp"

namespace mz
{

inline vk::Filter             to_vk_min_filter(int min_filter);
inline vk::Filter             to_vk_mag_filter(int mag_filter);
inline vk::SamplerMipmapMode  to_vk_mipmap_mode(int mipmap_mode);
inline vk::SamplerAddressMode to_vk_wrap_mode(int wrap_mode);
sg::PBRMaterialFlagBits       to_sg_material_flag_bit(const std::string &texture_name);
inline void                   to_sg_channel_output(sg::AnimationChannel &channel);
inline sg::AnimationType      to_sg_animation_type(const std::string &interpolation);
inline sg::AnimationTarget    to_sg_animation_target(const std::string &target);
inline void                   to_mz_vector_in_place(glm::vec3 &vec);
inline void                   to_mz_quaternion_in_place(glm::quat &quat);
inline void                   to_mz_matrix_in_place(glm::mat4 &M);
void                          to_mz_output_data_in_place(sg::AnimationSampler &sampler, sg::AnimationTarget target);
inline vk::Format             get_attr_format(const tinygltf::Model &model, uint32_t accessor_id);
inline std::vector<uint8_t>   get_attr_data(const tinygltf::Model &model, uint32_t accessor_id);
inline std::vector<uint8_t>   convert_data_stride(const std::vector<uint8_t> &src, uint32_t src_stride, uint32_t dst_stride);

const glm::vec3 DEFAULT_NORMAL = glm::vec3(0.0f);
const glm::vec2 DEFAULT_UV     = glm::vec2(0.0f);
const glm::vec4 DEFAULT_JOINT  = glm::vec4(0.0f);
const glm::vec4 DEFAULT_WEIGHT = glm::vec4(0.0f);
const glm::vec4 DEFAULT_COLOR  = glm::vec4(0.0f, 0.0f, 0.0f, 0.0f);

const glm::vec3 GLTFLoader::MZ_CONVERSION_SCALE = glm::vec3(-1, 1, 1);
const uint64_t  GLTFLoader::MAX_TEXUTRE_ID      = 1024;

template <class T, class Y>
struct TypeCast
{
	Y operator()(T value) const noexcept
	{
		return static_cast<Y>(value);
	}
};

GLTFLoader::GLTFLoader(const Context &ctx) :
    ctx_(ctx)
{
}

std::unique_ptr<sg::SubMesh> GLTFLoader::read_model_from_file(const std::string &file_name, int mesh_idx)
{
	load_gltf_model(file_name);
	return parse_submesh(nullptr, gltf_model_.meshes[mesh_idx].primitives[0]);
}

std::unique_ptr<sg::Scene> GLTFLoader::read_scene_from_file(const std::string &file_name,
                                                            int                scene_index)
{
	load_gltf_model(file_name);
	return std::make_unique<sg::Scene>(parse_scene(scene_index));
}

void GLTFLoader::load_gltf_model(const std::string &file_name)
{
	std::string err;
	std::string warn;

	tinygltf::TinyGLTF gltf_loader;

	std::string gltf_file_path = fu::compute_abs_path(fu::FileType::eModelAsset, file_name);
	std::string file_extension = fu::get_file_extension(gltf_file_path);
	bool        load_result;

	if (file_extension == "glb")
	{
		load_result =
		    gltf_loader.LoadBinaryFromFile(&gltf_model_, &err, &warn, gltf_file_path.c_str());
	}
	else if (file_extension == "gltf")
	{
		load_result =
		    gltf_loader.LoadASCIIFromFile(&gltf_model_, &err, &warn, gltf_file_path.c_str());
	}
	else
	{
		LOGE("Unsupported file type .{} for gltf models!", file_extension);
		abort();
	}

	if (!err.empty())
	{
		throw std::runtime_error(err);
	}

	if (!warn.empty())
	{
		throw std::runtime_error(warn);
	}

	if (!load_result)
	{
		throw std::runtime_error("Unable to load gltf file.");
	}

	size_t pos  = gltf_file_path.find_last_of('/');
	model_path_ = gltf_file_path.substr(0, pos);
	if (pos == std::string::npos)
	{
		model_path_.clear();
	}
}

sg::Scene GLTFLoader::parse_scene(int scene_idx)
{
	sg::Scene scene = sg::Scene("gltf_scene");
	p_scene_        = &scene;

	load_samplers();
	load_images();
	load_textures();
	load_materials();
	batch_upload_images();
	load_meshs();
	load_skins();
	load_cameras();
	load_nodes(scene_idx);
	load_default_camera();
	load_animations();
	init_scene_bound();
	return scene;
}

void GLTFLoader::init_scene_bound()
{
	std::vector<sg::Node *> p_nodes  = p_scene_->get_nodes();
	sg::AABB               &scene_bd = p_scene_->get_bound();

	for (sg::Node *p_node : p_nodes)
	{
		if (p_node->has_component<sg::Mesh>())
		{
			sg::Mesh &mesh = p_node->get_component<sg::Mesh>();
			scene_bd.update(mesh.get_mut_bounds().transform(p_node->get_transform().get_world_M()));
		}
	};
}

void GLTFLoader::load_samplers()
{
	std::vector<std::unique_ptr<sg::Sampler>> samplers(
	    gltf_model_.samplers.size());
	for (size_t i = 0; i < gltf_model_.samplers.size(); i++)
	{
		std::unique_ptr<sg::Sampler> sampler = parse_sampler(gltf_model_.samplers[i]);
		samplers[i]                          = std::move(sampler);
	}
	p_scene_->set_components(std::move(samplers));
}

std::unique_ptr<sg::Sampler> GLTFLoader::parse_sampler(
    tinygltf::Sampler &gltf_sampler)
{
	auto name = gltf_sampler.name;

	vk::Filter min_filter = to_vk_min_filter(gltf_sampler.minFilter);
	vk::Filter mag_filter = to_vk_mag_filter(gltf_sampler.magFilter);

	vk::SamplerMipmapMode mipmap_mode = to_vk_mipmap_mode(gltf_sampler.minFilter);

	vk::SamplerAddressMode address_mode_u = to_vk_wrap_mode(gltf_sampler.wrapS);
	vk::SamplerAddressMode address_mode_v = to_vk_wrap_mode(gltf_sampler.wrapT);
	vk::SamplerAddressMode address_mode_w = to_vk_wrap_mode(gltf_sampler.wrapS);

	vk::SamplerCreateInfo sampler_cinfo{
	    .magFilter     = mag_filter,
	    .minFilter     = min_filter,
	    .mipmapMode    = mipmap_mode,
	    .addressModeU  = address_mode_u,
	    .addressModeV  = address_mode_v,
	    .addressModeW  = address_mode_w,
	    .maxAnisotropy = ctx_.physical_device.get_handle().getProperties().limits.maxSamplerAnisotropy,
	    .maxLod        = std::numeric_limits<float>::max(),
	    .borderColor   = vk::BorderColor::eIntOpaqueWhite,
	};

	return std::make_unique<sg::Sampler>(ctx_.device, name, sampler_id_++, sampler_cinfo);
}

std::unique_ptr<sg::Sampler> GLTFLoader::create_default_sampler()
{
	tinygltf::Sampler gltf_sampler;
	gltf_sampler.minFilter = TINYGLTF_TEXTURE_FILTER_LINEAR;
	gltf_sampler.magFilter = TINYGLTF_TEXTURE_FILTER_LINEAR;

	gltf_sampler.wrapS = TINYGLTF_TEXTURE_WRAP_REPEAT;
	gltf_sampler.wrapT = TINYGLTF_TEXTURE_WRAP_REPEAT;

	return parse_sampler(gltf_sampler);
}

void GLTFLoader::load_images()
{
	std::vector<std::unique_ptr<sg::Image>> p_images;
	p_images.reserve(gltf_model_.images.size());
	img_tinfos_.reserve(gltf_model_.images.size());

	for (size_t i = 0; i < gltf_model_.images.size(); i++)
	{
		p_images.emplace_back(parse_image(gltf_model_.images[i]));
	}

	p_scene_->set_components(std::move(p_images));
}

std::unique_ptr<sg::Image> GLTFLoader::parse_image(tinygltf::Image &gltf_image)
{
	if (!gltf_image.image.empty())
	{
		img_tinfos_.push_back({
		    .binary = std::move(gltf_image.image),
		    .meta   = {
		          .extent = {
		              .width  = to_u32(gltf_image.width),
		              .height = to_u32(gltf_image.height),
		              .depth  = 1,
                },
		          .format = vk::Format::eR8G8B8A8Unorm,
		          .levels = 1,
            },
		});
	}
	else
	{
		std::string path = model_path_ + "/" + gltf_image.uri;
		img_tinfos_.push_back(ImageResource::load_two_dim_image(path));
	}

	return std::make_unique<sg::Image>(
	    ImageResource(ctx_.device, nullptr),
	    gltf_image.name,
	    image_id_++);
}

void GLTFLoader::batch_upload_images()
{
	std::vector<sg::Image *> p_images = p_scene_->get_components<sg::Image>();

	size_t i = 0;
	// we ignore the last image b/c it's the default image we've created for default texture.
	size_t        count  = p_images.size() - 1;
	const Device &device = ctx_.device;
	while (i < count)
	{
		std::vector<Buffer> staging_bufs;
		CommandBuffer       cmd_buf    = device.begin_one_time_buf();
		size_t              batch_size = 0;

		while (i < count && batch_size < 64 * 1024 * 1024)
		{
			sg::Image               *p_image   = p_images[i];
			const ImageTransferInfo &img_tinfo = img_tinfos_[i];
			size_t                   img_size  = img_tinfo.binary.size();

			create_image_resource(*p_image, i);

			batch_size += img_size;
			staging_bufs.emplace_back(device.get_device_memory_allocator().allocate_staging_buffer(img_size));

			staging_bufs.back().update(img_tinfo.binary);

			cmd_buf.set_image_layout(p_image->get_resource(), vk::ImageLayout::eUndefined, vk::ImageLayout::eTransferDstOptimal, vk::PipelineStageFlagBits::eHost, vk::PipelineStageFlagBits::eTransfer);

			cmd_buf.update_image(p_image->get_resource(), staging_bufs.back());

			cmd_buf.set_image_layout(p_image->get_resource(), vk::ImageLayout::eTransferDstOptimal, vk::ImageLayout::eShaderReadOnlyOptimal, vk::PipelineStageFlagBits::eTransfer, vk::PipelineStageFlagBits::eFragmentShader);

			i++;
		}
		device.end_one_time_buf(cmd_buf);
	}
};

void GLTFLoader::create_image_resource(sg::Image &image, size_t idx)
{
	const ImageTransferInfo &img_tinfo = img_tinfos_[idx];
	vk::ImageCreateInfo      img_cinfo{
	         .imageType   = vk::ImageType::e2D,
	         .format      = img_tinfo.meta.format,
	         .extent      = img_tinfo.meta.extent,
	         .mipLevels   = img_tinfo.meta.levels,
	         .arrayLayers = 1,
	         .samples     = vk::SampleCountFlagBits::e1,
	         .tiling      = vk::ImageTiling::eOptimal,
	         .usage       = vk::ImageUsageFlagBits::eSampled | vk::ImageUsageFlagBits::eTransferDst,
	         .sharingMode = vk::SharingMode::eExclusive,
    };

	Image vk_image = ctx_.device.get_device_memory_allocator().allocate_device_only_image(img_cinfo);

	vk::ImageViewCreateInfo view_cinfo = ImageView::two_dim_view_cinfo(vk_image.get_handle(), img_cinfo.format, vk::ImageAspectFlagBits::eColor, img_cinfo.mipLevels);

	image.set_resource(ImageResource(std::move(vk_image), ImageView(ctx_.device, view_cinfo)));
}

void GLTFLoader::load_textures()
{
	std::unique_ptr<sg::Sampler> p_default_sampler = create_default_sampler();
	std::vector<sg::Sampler *>   p_samplers        = p_scene_->get_components<sg::Sampler>();
	std::vector<sg::Image *>     p_images          = p_scene_->get_components<sg::Image>();

	for (auto &gltf_texture : gltf_model_.textures)
	{
		std::unique_ptr<sg::Texture> p_texture = parse_texture(gltf_texture);
		assert(gltf_texture.source < p_images.size());
		p_texture->p_resource_ = &p_images[gltf_texture.source]->get_resource();

		if (gltf_texture.sampler >= 0 && gltf_texture.sampler < static_cast<int>(p_samplers.size()))
		{
			p_texture->p_sampler_ = p_samplers[gltf_texture.sampler];
		}
		else
		{
			if (gltf_texture.name.empty())
			{
				gltf_texture.name = p_images[gltf_texture.source]->get_name();
			}
			p_texture->p_sampler_ = p_default_sampler.get();
		}
		p_scene_->add_component(std::move(p_texture));
	}

	p_scene_->add_component(create_default_texture(*p_default_sampler));
	p_scene_->add_component(std::move(p_default_sampler));
}

std::unique_ptr<sg::Texture> GLTFLoader::create_default_texture(sg::Sampler &default_sampler)
{
	std::unique_ptr<sg::Texture> p_texture = std::make_unique<sg::Texture>("default_texture", MAX_TEXUTRE_ID);
	std::unique_ptr<sg::Image>   p_image   = create_default_texture_image();
	p_texture->p_resource_                 = &p_image->get_resource();
	p_texture->p_sampler_                  = &default_sampler;
	p_scene_->add_component(std::move(p_image));
	return p_texture;
}

std::unique_ptr<sg::Image> GLTFLoader::create_default_texture_image()
{
	vk::ImageCreateInfo image_cinfo{
	    .imageType = vk::ImageType::e2D,
	    .format    = vk::Format::eR8G8B8A8Srgb,
	    .extent    = {
	           .width  = 1,
	           .height = 1,
	           .depth  = 1,
        },
	    .mipLevels   = 1,
	    .arrayLayers = 1,
	    .samples     = vk::SampleCountFlagBits::e1,
	    .tiling      = vk::ImageTiling::eOptimal,
	    .usage       = vk::ImageUsageFlagBits::eSampled | vk::ImageUsageFlagBits::eTransferDst,
	    .sharingMode = vk::SharingMode::eExclusive,
	};

	Image img = ctx_.device.get_device_memory_allocator().allocate_device_only_image(image_cinfo);

	vk::ImageViewCreateInfo view_cinfo = ImageView::two_dim_view_cinfo(img.get_handle(), image_cinfo.format, vk::ImageAspectFlagBits::eColor, 1);
	ImageResource           resource   = ImageResource(std::move(img), ImageView(ctx_.device, view_cinfo));

	std::vector<uint8_t> binary = {0u, 0u, 0u, 0u};

	Buffer staging_buf = ctx_.device.get_device_memory_allocator().allocate_staging_buffer(binary.size());
	staging_buf.update(binary);

	CommandBuffer cmd_buf = ctx_.device.begin_one_time_buf();

	cmd_buf.set_image_layout(resource, vk::ImageLayout::eUndefined, vk::ImageLayout::eTransferDstOptimal, vk::PipelineStageFlagBits::eHost, vk::PipelineStageFlagBits::eTransfer);
	cmd_buf.update_image(resource, staging_buf);
	cmd_buf.set_image_layout(resource, vk::ImageLayout::eTransferDstOptimal, vk::ImageLayout::eShaderReadOnlyOptimal, vk::PipelineStageFlagBits::eTransfer, vk::PipelineStageFlagBits::eFragmentShader);

	ctx_.device.end_one_time_buf(cmd_buf);

	return std::make_unique<sg::Image>(std::move(resource), "default_image", image_id_++);
}

std::unique_ptr<sg::Texture> GLTFLoader::parse_texture(
    tinygltf::Texture &gltf_texture)
{
	return std::make_unique<sg::Texture>(gltf_texture.name, texture_id_++);
}

void GLTFLoader::load_materials()
{
	std::vector<sg::Texture *> p_textures;
	if (p_scene_->has_component<sg::Texture>())
	{
		p_textures = p_scene_->get_components<sg::Texture>();
	}

	for (auto &gltf_material : gltf_model_.materials)
	{
		std::unique_ptr<sg::PBRMaterial> p_material = parse_material(gltf_material);
		append_textures_to_material(gltf_material.values, p_textures, p_material.get());
		append_textures_to_material(gltf_material.additionalValues, p_textures, p_material.get());
		p_scene_->add_component(std::move(p_material));
	}
}

std::unique_ptr<sg::PBRMaterial> GLTFLoader::parse_material(
    tinygltf::Material &gltf_material)
{
	auto material = std::make_unique<sg::PBRMaterial>(gltf_material.name, material_id_++);

	for (auto &gltf_value : gltf_material.values)
	{
		if (gltf_value.first == "baseColorFactor")
		{
			const auto &color_factor = gltf_value.second.ColorFactor();
			material->base_color_factor_ =
			    glm::vec4(color_factor[0], color_factor[1], color_factor[2], color_factor[3]);
		}
		else if (gltf_value.first == "metallicFactor")
		{
			material->metallic_factor = static_cast<float>(gltf_value.second.number_value);
		}
		else if (gltf_value.first == "roughnessFactor")
		{
			material->roughness_factor = static_cast<float>(gltf_value.second.number_value);
		}
	}

	for (auto &gltf_value : gltf_material.additionalValues)
	{
		if (gltf_value.first == "emissiveFactor")
		{
			const auto &emissive_factor = gltf_value.second.number_array;
			material->emissive_ =
			    glm::vec3(emissive_factor[0], emissive_factor[1], emissive_factor[2]);
		}
		else if (gltf_value.first == "alphaMode")
		{
			if (gltf_value.second.string_value == "BLEND")
			{
				material->alpha_mode_ = sg::AlphaMode::Blend;
			}
			else if (gltf_value.second.string_value == "OPAQUE")
			{
				material->alpha_mode_ = sg::AlphaMode::Opaque;
			}
			else if (gltf_value.second.string_value == "MASK")
			{
				material->alpha_mode_ = sg::AlphaMode::Mask;
			}
		}
		else if (gltf_value.first == "alphaCutoff")
		{
			material->alpha_cutoff_ = static_cast<float>(gltf_value.second.number_value);
		}
		else if (gltf_value.first == "doubleSided")
		{
			material->is_double_sided = gltf_value.second.bool_value;
		}
	}

	return material;
}

void GLTFLoader::append_textures_to_material(tinygltf::ParameterMap &parameter_map, std::vector<sg::Texture *> &p_textures, sg::PBRMaterial *p_material)
{
	for (auto &value : parameter_map)
	{
		if (value.first.find("Texture") != std::string::npos)
		{
			int                     texture_idx  = value.second.TextureIndex();
			std::string             texture_name = to_snake_case(to_string(value.first));
			sg::PBRMaterialFlagBits flag_bit     = to_sg_material_flag_bit(texture_name);
			assert(texture_idx < p_textures.size());

			if (flag_bit == sg::PBRMaterialFlagBits::eBaseColorTexture || flag_bit == sg::PBRMaterialFlagBits::eEmissiveTexture)
			{
				img_tinfos_[gltf_model_.textures[texture_idx].source].meta.format = vk::Format::eR8G8B8A8Srgb;
			}
			p_material->texture_map_[texture_name] = p_textures[value.second.TextureIndex()];
			p_material->flag |= flag_bit;
		}
	}
}

std::unique_ptr<sg::PBRMaterial> GLTFLoader::create_default_material()
{
	tinygltf::Material gltf_material;

	std::unique_ptr<sg::PBRMaterial> p_material = parse_material(gltf_material);

	p_material->texture_map_["base_color_texture"] =
	    p_material->texture_map_["normal_texture"] =
	        p_material->texture_map_["occlusion_texture"] =
	            p_material->texture_map_["emissive_texture"] =
	                p_material->texture_map_["metallic_roughness_texture"] =
	                    p_scene_->get_components<sg::Texture>().back();

	return p_material;
}

void GLTFLoader::load_meshs()
{
	std::unique_ptr<sg::PBRMaterial> p_default_material = create_default_material();
	std::vector<sg::PBRMaterial *>   p_materials        = p_scene_->get_components<sg::PBRMaterial>();

	for (auto &gltf_mesh : gltf_model_.meshes)
	{
		std::unique_ptr<sg::Mesh> p_mesh = parse_mesh(gltf_mesh);

		for (auto &primitive : gltf_mesh.primitives)
		{
			std::unique_ptr<sg::SubMesh> p_submesh = parse_submesh(p_mesh.get(), primitive);
			if (primitive.material >= 0)
			{
				assert(primitive.material < p_materials.size());
				p_submesh->set_material(*p_materials[primitive.material]);
			}
			else
			{
				p_submesh->set_material(*p_default_material);
			}
			p_mesh->add_submesh(*p_submesh);
			p_scene_->add_component(std::move(p_submesh));
		}

		p_scene_->add_component(std::move(p_mesh));
	}

	p_scene_->add_component(std::move(p_default_material));
}

std::unique_ptr<sg::Mesh> GLTFLoader::parse_mesh(tinygltf::Mesh &gltf_mesh)
{
	return std::make_unique<sg::Mesh>(gltf_mesh.name, mesh_id_++);
}

std::unique_ptr<sg::SubMesh> GLTFLoader::parse_submesh(sg::Mesh *p_mesh, tinygltf::Primitive &gltf_submesh)
{
	std::vector<Buffer>          transient_bufs;
	std::unique_ptr<sg::SubMesh> p_submesh = std::make_unique<sg::SubMesh>("", submesh_id_++);
	p_submesh->vert_count_                 = get_submesh_vertex_count(gltf_submesh);
	if (p_mesh)
	{
		update_parent_mesh_bound(p_mesh, gltf_submesh);
	}
	std::vector<sg::Vertex> vertexs;
	vertexs.reserve(p_submesh->vert_count_);

	DataAccessInfo<float>    pos    = get_attr_data_ptr<float>(gltf_submesh, "POSITION");
	DataAccessInfo<float>    norm   = get_attr_data_ptr<float>(gltf_submesh, "NORMAL");
	DataAccessInfo<float>    uv     = get_attr_data_ptr<float>(gltf_submesh, "TEXCOORD_0");
	DataAccessInfo<uint16_t> joint  = get_attr_data_ptr<uint16_t>(gltf_submesh, "JOINTS_0");
	DataAccessInfo<float>    weight = get_attr_data_ptr<float>(gltf_submesh, "WEIGHTS_0");
	DataAccessInfo<float>    color  = get_attr_data_ptr<float>(gltf_submesh, "COLOR_0");

	bool is_skinned = joint.p_data && weight.p_data;

	for (size_t i = 0; i < p_submesh->vert_count_; i++)
	{
		vertexs.emplace_back(sg::Vertex{
		    .pos    = glm::make_vec3(&pos.p_data[i * pos.stride]),
		    .norm   = norm.p_data ? glm::normalize(glm::make_vec3(&norm.p_data[i * norm.stride])) : DEFAULT_NORMAL,
		    .uv     = uv.p_data ? glm::make_vec2(&uv.p_data[i * uv.stride]) : DEFAULT_UV,
		    .joint  = is_skinned ? glm::vec4(glm::make_vec4(&joint.p_data[i * joint.stride])) : DEFAULT_JOINT,
		    .weight = is_skinned ? glm::make_vec4(&weight.p_data[i * weight.stride]) : DEFAULT_WEIGHT,
		    .color  = color.p_data ? glm::make_vec4(&color.p_data[i * color.stride]) : DEFAULT_COLOR,
		});
		to_mz_vector_in_place(vertexs.back().pos);
		to_mz_vector_in_place(vertexs.back().norm);
	}

	size_t vertex_buf_size    = vertexs.size() * sizeof(sg::Vertex);
	Buffer vertex_staging_buf = ctx_.device.get_device_memory_allocator().allocate_staging_buffer(vertex_buf_size);
	Buffer vertex_buf         = ctx_.device.get_device_memory_allocator().allocate_vertex_buffer(vertex_buf_size);
	vertex_staging_buf.update(vertexs.data(), vertex_buf_size);
	CommandBuffer cmd_buf = ctx_.device.begin_one_time_buf();
	cmd_buf.copy_buffer(vertex_staging_buf, vertex_buf, vertex_buf_size);
	p_submesh->p_vert_buf_ = std::make_unique<Buffer>(std::move(vertex_buf));
	transient_bufs.push_back(std::move(vertex_staging_buf));

	if (gltf_submesh.indices >= 0)
	{
		const tinygltf::Accessor &accessor = gltf_model_.accessors[gltf_submesh.indices];
		p_submesh->idx_count_              = accessor.count;

		vk::Format           format = get_attr_format(gltf_model_, gltf_submesh.indices);
		std::vector<uint8_t> indexs = get_attr_data(gltf_model_, gltf_submesh.indices);

		switch (format)
		{
			case vk::Format::eR32Uint:
				break;
			case vk::Format::eR16Uint:
			{
				indexs = convert_data_stride(indexs, 2, 4);
				break;
			}
			case vk::Format::eR8Uint:
			{
				indexs = convert_data_stride(indexs, 1, 4);
				break;
			}
			default:
				// unreachable;
				break;
		}

		Buffer idx_staging_buf = ctx_.device.get_device_memory_allocator().allocate_staging_buffer(indexs.size());
		Buffer idx_buf         = ctx_.device.get_device_memory_allocator().allocate_index_buffer(indexs.size());
		idx_staging_buf.update(indexs);
		cmd_buf.copy_buffer(idx_staging_buf, idx_buf, indexs.size());
		transient_bufs.push_back(std::move(idx_staging_buf));
		p_submesh->p_idx_buf_ = std::make_unique<Buffer>(std::move(idx_buf));
	}

	ctx_.device.end_one_time_buf(cmd_buf);
	return std::move(p_submesh);
}

size_t GLTFLoader::get_submesh_vertex_count(tinygltf::Primitive &submesh)
{
	// GLTF gurantees that a vertex will always have position attribute
	const tinygltf::Accessor accessor = gltf_model_.accessors[submesh.attributes.find("POSITION")->second];
	return accessor.count;
}

void GLTFLoader::update_parent_mesh_bound(sg::Mesh *p_mesh, tinygltf::Primitive &submesh)
{
	const tinygltf::Accessor accessor = gltf_model_.accessors[submesh.attributes.find("POSITION")->second];
	p_mesh->get_mut_bounds().update(
	    glm::vec3(accessor.minValues[0], accessor.minValues[1], accessor.minValues[2]),
	    glm::vec3(accessor.maxValues[0], accessor.maxValues[1], accessor.maxValues[2]));
}

void GLTFLoader::load_cameras()
{
	for (tinygltf::Camera &camera : gltf_model_.cameras)
	{
		p_scene_->add_component(parse_camera(camera));
	}
}

std::unique_ptr<sg::Camera> GLTFLoader::parse_camera(
    tinygltf::Camera &gltf_camera)
{
	std::unique_ptr<sg::Camera> camera;

	if (gltf_camera.type == "perspective")
	{
		auto perspective_camera = std::make_unique<sg::PerspectiveCamera>(gltf_camera.name, camera_id_++);

		perspective_camera->set_aspect_ratio(
		    static_cast<float>(gltf_camera.perspective.aspectRatio));
		perspective_camera->set_field_of_view(static_cast<float>(gltf_camera.perspective.yfov));
		perspective_camera->set_near_plane(static_cast<float>(gltf_camera.perspective.znear));
		perspective_camera->set_far_plane(static_cast<float>(gltf_camera.perspective.zfar));

		camera = std::move(perspective_camera);
	}
	else
	{
		throw std::runtime_error("Camera type not supported");
	}

	return camera;
}

void GLTFLoader::load_default_camera()
{
	std::unique_ptr<sg::Node>   p_camera_node = std::make_unique<sg::Node>("default_camera", node_id_++);
	std::unique_ptr<sg::Camera> p_camera      = create_default_camera();

	p_camera->set_node(*p_camera_node);
	p_camera_node->set_component(*p_camera);

	p_scene_->add_component(std::move(p_camera));
	p_scene_->get_root_node().add_child(*p_camera_node);
	p_scene_->add_node(std::move(p_camera_node));
}

std::unique_ptr<sg::Camera> GLTFLoader::create_default_camera()
{
	tinygltf::Camera gltf_camera;
	gltf_camera.name = "default_camera";
	gltf_camera.type = "perspective";

	gltf_camera.perspective.aspectRatio = 1.77f;
	gltf_camera.perspective.yfov        = 1.0f;
	gltf_camera.perspective.znear       = 0.1f;
	gltf_camera.perspective.zfar        = 1000.0f;

	return parse_camera(gltf_camera);
}

void GLTFLoader::load_nodes(int scene_idx)
{
	std::vector<std::unique_ptr<sg::Node>> p_nodes      = parse_nodes();
	tinygltf::Scene                       *p_gltf_scene = pick_scene(scene_idx);
	std::unique_ptr<sg::Node>              root         = std::make_unique<sg::Node>(p_gltf_scene->name, node_id_++);

	init_node_hierarchy(p_gltf_scene, p_nodes, *root);

	p_scene_->set_root_node(*root);
	p_nodes.push_back(std::move(root));
	p_scene_->set_nodes(std::move(p_nodes));
}

std::vector<std::unique_ptr<sg::Node>> GLTFLoader::parse_nodes()
{
	std::vector<sg::Camera *>              p_cameras = p_scene_->get_components<sg::Camera>();
	std::vector<sg::Mesh *>                p_meshs   = p_scene_->get_components<sg::Mesh>();
	std::vector<sg::Skin *>                p_skins   = p_scene_->get_components<sg::Skin>();
	std::vector<std::unique_ptr<sg::Node>> p_nodes;
	p_nodes.reserve(gltf_model_.nodes.size());

	for (size_t i = 0; i < gltf_model_.nodes.size(); i++)
	{
		tinygltf::Node           &gltf_node = gltf_model_.nodes[i];
		std::unique_ptr<sg::Node> p_node    = parse_node(gltf_node);

		if (gltf_node.mesh >= 0)
		{
			assert(gltf_node.mesh < p_meshs.size());
			sg::Mesh *p_mesh = p_meshs[gltf_node.mesh];
			p_node->set_component(*p_mesh);
			p_mesh->add_node(*p_node);
		}

		if (gltf_node.camera >= 0)
		{
			assert(gltf_node.camera < p_cameras.size());
			sg::Camera *p_camera = p_cameras[gltf_node.camera];
			p_node->set_component(*p_camera);
			p_camera->set_node(*p_node);
		}

		if (gltf_node.skin >= 0)
		{
			assert(gltf_node.skin < p_skins.size());
			sg::Skin *p_skin = p_skins[gltf_node.skin];
			p_node->set_component(*p_skin);
		}

		p_nodes.push_back(std::move(p_node));
	};
	return p_nodes;
}

std::unique_ptr<sg::Node> GLTFLoader::parse_node(tinygltf::Node &gltf_node)
{
	auto node = std::make_unique<sg::Node>(gltf_node.name, node_id_++);

	auto &transform = node->get_transform();

	if (!gltf_node.translation.empty())
	{
		glm::vec3 translation;
		std::transform(gltf_node.translation.begin(), gltf_node.translation.end(), glm::value_ptr(translation), TypeCast<double, float>{});
		to_mz_vector_in_place(translation);
		transform.set_tranlsation(translation);
	}

	if (!gltf_node.rotation.empty())
	{
		glm::quat rotation;
		rotation.x = gltf_node.rotation[0];
		rotation.y = gltf_node.rotation[1];
		rotation.z = gltf_node.rotation[2];
		rotation.w = gltf_node.rotation[3];
		to_mz_quaternion_in_place(rotation);
		transform.set_rotation(rotation);
	}

	if (!gltf_node.scale.empty())
	{
		glm::vec3 scale;
		std::transform(gltf_node.scale.begin(), gltf_node.scale.end(), glm::value_ptr(scale), TypeCast<double, float>{});
		transform.set_scale(scale);
	}

	if (!gltf_node.matrix.empty())
	{
		glm::mat4 matrix;
		std::transform(gltf_node.matrix.begin(), gltf_node.matrix.end(), glm::value_ptr(matrix), TypeCast<double, float>{});
		to_mz_matrix_in_place(matrix);
		transform.set_local_M(matrix);
	}

	return node;
}

void GLTFLoader::init_node_hierarchy(tinygltf::Scene *p_gltf_scene, std::vector<std::unique_ptr<sg::Node>> &p_nodes, sg::Node &root)
{
	struct NodeTraversal
	{
		sg::Node &parent;
		int       curr_idx;
	};

	std::queue<NodeTraversal> q;

	for (int i : p_gltf_scene->nodes)
	{
		q.push({
		    .parent   = std::ref(root),
		    .curr_idx = i,
		});
	}

	while (!q.empty())
	{
		NodeTraversal traversal = q.front();
		q.pop();
		assert(traversal.curr_idx < p_nodes.size());
		sg::Node &curr = *p_nodes[traversal.curr_idx];
		traversal.parent.add_child(curr);
		curr.set_parent(traversal.parent);

		for (int child_idx : gltf_model_.nodes[traversal.curr_idx].children)
		{
			q.push({
			    .parent   = std::ref(curr),
			    .curr_idx = child_idx,
			});
		}
	}
}

void GLTFLoader::load_animations()
{
	std::vector<sg::Node *>                     p_nodes = p_scene_->get_nodes();
	std::vector<std::unique_ptr<sg::Animation>> p_animations;
	p_animations.reserve(gltf_model_.animations.size());
	for (size_t i = 0; i < gltf_model_.animations.size(); i++)
	{
		const tinygltf::Animation     &gltf_animation = gltf_model_.animations[i];
		std::unique_ptr<sg::Animation> p_animation    = std::make_unique<sg::Animation>(gltf_animation.name, animation_id_++);
		p_animation->set_channels(parse_animation_channels(gltf_animation, p_nodes));
		p_animation->update_interval();
		p_animations.push_back(std::move(p_animation));
	}
	p_scene_->set_components(std::move(p_animations));
}

std::vector<sg::AnimationChannel> GLTFLoader::parse_animation_channels(const tinygltf::Animation &gltf_animation, std::vector<sg::Node *> p_nodes)
{
	std::vector<sg::AnimationSampler> samplers = parse_animation_samplers(gltf_animation);
	std::vector<sg::AnimationChannel> channels;
	channels.reserve(gltf_animation.channels.size());

	for (size_t i = 0; i < gltf_animation.channels.size(); i++)
	{
		const tinygltf::AnimationChannel &gltf_channel = gltf_animation.channels[i];

		channels.push_back(sg::AnimationChannel{
		    .node    = *p_nodes[gltf_channel.target_node],
		    .target  = to_sg_animation_target(gltf_channel.target_path),
		    .sampler = samplers[gltf_channel.sampler],
		});
		to_mz_output_data_in_place(channels.back().sampler, channels.back().target);
	}

	return channels;
}

std::vector<sg::AnimationSampler> GLTFLoader::parse_animation_samplers(const tinygltf::Animation &gltf_animation)
{
	std::vector<sg::AnimationSampler> samplers;
	samplers.resize(gltf_animation.samplers.size());

	for (size_t i = 0; i < gltf_animation.samplers.size(); i++)
	{
		auto                 &gltf_sampler = gltf_animation.samplers[i];
		sg::AnimationSampler &sampler      = samplers[i];
		sampler.type                       = to_sg_animation_type(gltf_sampler.interpolation);
		parse_animation_input_data(gltf_sampler, sampler);
		parse_animation_output_data(gltf_sampler, sampler);
	}

	return samplers;
}

void GLTFLoader::parse_animation_input_data(const tinygltf::AnimationSampler &gltf_sampler, sg::AnimationSampler &sampler)
{
	const tinygltf::Accessor &input_accessor = gltf_model_.accessors[gltf_sampler.input];
	std::vector<uint8_t>      input_data     = get_attr_data(gltf_model_, gltf_sampler.input);
	const float              *p_input_data   = reinterpret_cast<const float *>(input_data.data());
	for (size_t i = 0; i < input_accessor.count; i++)
	{
		sampler.inputs.push_back(p_input_data[i]);
	}
}

void GLTFLoader::parse_animation_output_data(const tinygltf::AnimationSampler &gltf_sampler, sg::AnimationSampler &sampler)
{
	const tinygltf::Accessor &output_accessor = gltf_model_.accessors[gltf_sampler.output];
	std::vector<uint8_t>      output_data     = get_attr_data(gltf_model_, gltf_sampler.output);
	switch (output_accessor.type)
	{
		case TINYGLTF_TYPE_VEC3:
		{
			sampler.init_vecs();
			std::vector<glm::vec3> &sampler_outputs = sampler.get_mut_vecs();

			const glm::vec3 *p_vecs = reinterpret_cast<const glm::vec3 *>(output_data.data());
			for (size_t i = 0; i < output_accessor.count; i++)
			{
				sampler_outputs.push_back(glm::vec3(p_vecs[i]));
			}
			break;
		}
		case TINYGLTF_TYPE_VEC4:
		{
			sampler.init_quats();
			std::vector<glm::quat> &sampler_outputs = sampler.get_mut_quats();

			const float *p_float = reinterpret_cast<const float *>(output_data.data());
			for (size_t i = 0; i < output_accessor.count; i++)
			{
				// gltf passes in quat as (x, y, z, w)
				sampler_outputs.push_back(glm::quat::wxyz(
				    p_float[i * 4 + 3],
				    p_float[i * 4 + 0],
				    p_float[i * 4 + 1],
				    p_float[i * 4 + 2]));
			}
			break;
		}
		default:
		{
			LOGE("Unsupported output data type.");
			abort();
			break;
		}
	}
}

void GLTFLoader::load_skins()
{
	std::vector<std::unique_ptr<sg::Skin>> p_skins;
	p_skins.reserve(gltf_model_.skins.size());
	for (auto &gltf_skin : gltf_model_.skins)
	{
		p_skins.push_back(parse_skin(gltf_skin));
	}
	p_scene_->set_components(std::move(p_skins));
}

std::unique_ptr<sg::Skin> GLTFLoader::parse_skin(tinygltf::Skin &gltf_skin)
{
	const std::vector<int> &joints = gltf_skin.joints;
	if (joints.size() > sg::Skin::MAX_NUM_JOINTS)
	{
		LOGE("Skin {} exceeds the joint limits.", gltf_skin.name);
		abort();
	}
	std::unique_ptr<sg::Skin> p_skin = std::make_unique<sg::Skin>(gltf_skin.name, skin_id_++);

	auto                 &IBMs = p_skin->get_IBMs();
	DataAccessInfo<float> IBM  = get_accessor_data_ptr<float>(gltf_skin.inverseBindMatrices);

	for (int joint_id = 0; joint_id < joints.size(); joint_id++)
	{
		p_skin->add_new_joint(joint_id, joints[joint_id]);
		glm::mat4 m = glm::make_mat4(&IBM.p_data[joint_id * IBM.stride]);
		to_mz_matrix_in_place(m);
		IBMs[joint_id] = m;
	}

	return p_skin;
}

tinygltf::Scene *GLTFLoader::pick_scene(int scene_idx)
{
	tinygltf::Scene *gltf_scene  = nullptr;
	int              scene_count = static_cast<int>(gltf_model_.scenes.size());

	if (scene_idx >= 0 && scene_idx < scene_count)
	{
		gltf_scene = &gltf_model_.scenes[scene_idx];
	}
	else if (gltf_model_.defaultScene >= 0 && gltf_model_.defaultScene < scene_count)
	{
		gltf_scene = &gltf_model_.scenes[gltf_model_.defaultScene];
	}
	else if (gltf_model_.scenes.size() > 0)
	{
		gltf_scene = &gltf_model_.scenes[0];
	}

	if (!gltf_scene)
	{
		LOGE("Couldn't determine which scene to load");
		abort();
	}

	return gltf_scene;
}

inline vk::Filter to_vk_min_filter(int min_filter)
{
	switch (min_filter)
	{
		case TINYGLTF_TEXTURE_FILTER_NEAREST:
		case TINYGLTF_TEXTURE_FILTER_NEAREST_MIPMAP_LINEAR:
		case TINYGLTF_TEXTURE_FILTER_NEAREST_MIPMAP_NEAREST:
			return vk::Filter::eNearest;
		case TINYGLTF_TEXTURE_FILTER_LINEAR:
		case TINYGLTF_TEXTURE_FILTER_LINEAR_MIPMAP_LINEAR:
		case TINYGLTF_TEXTURE_FILTER_LINEAR_MIPMAP_NEAREST:
			return vk::Filter::eLinear;
		default:
			return vk::Filter::eLinear;
	}
}

inline vk::Filter to_vk_mag_filter(int mag_filter)
{
	switch (mag_filter)
	{
		case TINYGLTF_TEXTURE_FILTER_LINEAR:
			return vk::Filter::eLinear;
		case TINYGLTF_TEXTURE_FILTER_NEAREST:
			return vk::Filter::eNearest;
		default:
			return vk::Filter::eLinear;
	}
}

inline vk::SamplerMipmapMode to_vk_mipmap_mode(int mipmap_mode)
{
	switch (mipmap_mode)
	{
		case TINYGLTF_TEXTURE_FILTER_NEAREST_MIPMAP_LINEAR:
		case TINYGLTF_TEXTURE_FILTER_NEAREST_MIPMAP_NEAREST:
			return vk::SamplerMipmapMode::eNearest;
		case TINYGLTF_TEXTURE_FILTER_LINEAR_MIPMAP_LINEAR:
		case TINYGLTF_TEXTURE_FILTER_LINEAR_MIPMAP_NEAREST:
			return vk::SamplerMipmapMode::eLinear;
		default:
			return vk::SamplerMipmapMode::eLinear;
	}
}

inline vk::SamplerAddressMode to_vk_wrap_mode(int wrap_mode)
{
	switch (wrap_mode)
	{
		case TINYGLTF_TEXTURE_WRAP_REPEAT:
			return vk::SamplerAddressMode::eRepeat;
		case TINYGLTF_TEXTURE_WRAP_CLAMP_TO_EDGE:
			return vk::SamplerAddressMode::eClampToEdge;
		case TINYGLTF_TEXTURE_WRAP_MIRRORED_REPEAT:
			return vk::SamplerAddressMode::eMirroredRepeat;
		default:
			return vk::SamplerAddressMode::eRepeat;
	}
}

sg::PBRMaterialFlagBits to_sg_material_flag_bit(const std::string &texture_name)
{
	static std::unordered_map<std::string, sg::PBRMaterialFlagBits> name_to_flag_bit_map = {
	    {"base_color_texture", sg::PBRMaterialFlagBits::eBaseColorTexture},
	    {"normal_texture", sg::PBRMaterialFlagBits::eNormalTexture},
	    {"occlusion_texture", sg::PBRMaterialFlagBits::eOcclusionTexture},
	    {"emissive_texture", sg::PBRMaterialFlagBits::eEmissiveTexture},
	    {"metallic_roughness_texture", sg::PBRMaterialFlagBits::eMetallicRoughnessTexture},
	};
	assert(name_to_flag_bit_map.count(texture_name));
	return name_to_flag_bit_map[texture_name];
}

inline sg::AnimationType to_sg_animation_type(const std::string &interpolation)
{
	if (interpolation == "LINEAR")
	{
		return sg::AnimationType::eLinear;
	}
	else if (interpolation == "STEP")
	{
		return sg::AnimationType::eStep;
	}
	else if (interpolation == "CUBICSPLINE")
	{
		return sg::AnimationType::eCubicSpline;
	}
	LOGW("Unkown interpolation value {}.", interpolation);
	return sg::AnimationType::eLinear;
}

inline sg::AnimationTarget to_sg_animation_target(const std::string &target)
{
	if (target == "translation")
	{
		return sg::AnimationTarget::eTranslation;
	}
	else if (target == "rotation")
	{
		return sg::AnimationTarget::eRotation;
	}
	else if (target == "scale")
	{
		return sg::AnimationTarget::eScale;
	}
	LOGW("Animation target {} is not supported!", target);
	return sg::AnimationTarget::eTranslation;
}

inline void to_mz_vector_in_place(glm::vec3 &vec)
{
	vec *= GLTFLoader::MZ_CONVERSION_SCALE;
}

inline void to_mz_quaternion_in_place(glm::quat &quat)
{
	// We need to flip the handiness
	float     flip_scale        = -1;
	glm::vec3 new_axis_rotation = flip_scale * glm::vec3(quat.x, quat.y, quat.z) * GLTFLoader::MZ_CONVERSION_SCALE;
	quat.x                      = new_axis_rotation.x;
	quat.y                      = new_axis_rotation.y;
	quat.z                      = new_axis_rotation.z;
}

inline void to_mz_matrix_in_place(glm::mat4 &M)
{
	glm::mat4 convert = glm::scale(glm::mat4(1.0f), GLTFLoader::MZ_CONVERSION_SCALE);
	M                 = convert * M * convert;
}

void to_mz_output_data_in_place(sg::AnimationSampler &sampler, sg::AnimationTarget target)
{
	switch (target)
	{
		case sg::AnimationTarget::eTranslation:
		{
			std::vector<glm::vec3> &vecs = sampler.get_mut_vecs();
			for (auto &vec : vecs)
			{
				to_mz_vector_in_place(vec);
			}
			break;
		}
		case sg::AnimationTarget::eRotation:
		{
			std::vector<glm::quat> &quats = sampler.get_mut_quats();
			for (auto &quat : quats)
			{
				to_mz_quaternion_in_place(quat);
			}
			break;
		}
		case sg::AnimationTarget::eScale:
			break;
	}
}

inline vk::Format get_attr_format(const tinygltf::Model &model, uint32_t accessor_id)
{
	assert(accessor_id < model.accessors.size());
	auto &accessor = model.accessors[accessor_id];

	vk::Format format;

	switch (accessor.componentType)
	{
		case TINYGLTF_COMPONENT_TYPE_BYTE:
		{
			static const std::map<int, vk::Format> mapped_format = {
			    {TINYGLTF_TYPE_SCALAR, vk::Format::eR8Sint},
			    {TINYGLTF_TYPE_VEC2, vk::Format::eR8G8Sint},
			    {TINYGLTF_TYPE_VEC3, vk::Format::eR8G8B8Sint},
			    {TINYGLTF_TYPE_VEC4, vk::Format::eR8G8B8A8Sint}};

			format = mapped_format.at(accessor.type);

			break;
		}
		case TINYGLTF_COMPONENT_TYPE_UNSIGNED_BYTE:
		{
			static const std::map<int, vk::Format> mapped_format = {
			    {TINYGLTF_TYPE_SCALAR, vk::Format::eR8Uint},
			    {TINYGLTF_TYPE_VEC2, vk::Format::eR8G8Uint},
			    {TINYGLTF_TYPE_VEC3, vk::Format::eR8G8B8Uint},
			    {TINYGLTF_TYPE_VEC4, vk::Format::eR8G8B8A8Uint}};

			static const std::map<int, vk::Format> mapped_format_normalize = {
			    {TINYGLTF_TYPE_SCALAR, vk::Format::eR8Unorm},
			    {TINYGLTF_TYPE_VEC2, vk::Format::eR8G8Unorm},
			    {TINYGLTF_TYPE_VEC3, vk::Format::eR8G8B8Unorm},
			    {TINYGLTF_TYPE_VEC4, vk::Format::eR8G8B8A8Unorm}};

			if (accessor.normalized)
			{
				format = mapped_format_normalize.at(accessor.type);
			}
			else
			{
				format = mapped_format.at(accessor.type);
			}

			break;
		}
		case TINYGLTF_COMPONENT_TYPE_SHORT:
		{
			static const std::map<int, vk::Format> mapped_format = {
			    {TINYGLTF_TYPE_SCALAR, vk::Format::eR16Sint},
			    {TINYGLTF_TYPE_VEC2, vk::Format::eR16G16Sint},
			    {TINYGLTF_TYPE_VEC3, vk::Format::eR16G16B16Sint},
			    {TINYGLTF_TYPE_VEC4, vk::Format::eR16G16B16A16Sint}};

			format = mapped_format.at(accessor.type);

			break;
		}
		case TINYGLTF_COMPONENT_TYPE_UNSIGNED_SHORT:
		{
			static const std::map<int, vk::Format> mapped_format = {
			    {TINYGLTF_TYPE_SCALAR, vk::Format::eR16Uint},
			    {TINYGLTF_TYPE_VEC2, vk::Format::eR16G16Uint},
			    {TINYGLTF_TYPE_VEC3, vk::Format::eR16G16B16Uint},
			    {TINYGLTF_TYPE_VEC4, vk::Format::eR16G16B16A16Uint}};

			static const std::map<int, vk::Format> mapped_format_normalize = {
			    {TINYGLTF_TYPE_SCALAR, vk::Format::eR16Unorm},
			    {TINYGLTF_TYPE_VEC2, vk::Format::eR16G16Unorm},
			    {TINYGLTF_TYPE_VEC3, vk::Format::eR16G16B16Unorm},
			    {TINYGLTF_TYPE_VEC4, vk::Format::eR16G16B16A16Unorm}};
			if (accessor.normalized)
			{
				format = mapped_format_normalize.at(accessor.type);
			}
			else
			{
				format = mapped_format.at(accessor.type);
			}

			break;
		}
		case TINYGLTF_COMPONENT_TYPE_INT:
		{
			static const std::map<int, vk::Format> mapped_format = {
			    {TINYGLTF_TYPE_SCALAR, vk::Format::eR32Sint},
			    {TINYGLTF_TYPE_VEC2, vk::Format::eR32G32Sint},
			    {TINYGLTF_TYPE_VEC3, vk::Format::eR32G32B32Sint},
			    {TINYGLTF_TYPE_VEC4, vk::Format::eR32G32B32A32Sint}};

			format = mapped_format.at(accessor.type);

			break;
		}
		case TINYGLTF_COMPONENT_TYPE_UNSIGNED_INT:
		{
			static const std::map<int, vk::Format> mapped_format = {
			    {TINYGLTF_TYPE_SCALAR, vk::Format::eR32Uint},
			    {TINYGLTF_TYPE_VEC2, vk::Format::eR32G32Uint},
			    {TINYGLTF_TYPE_VEC3, vk::Format::eR32G32B32Uint},
			    {TINYGLTF_TYPE_VEC4, vk::Format::eR32G32B32A32Uint}};
			format = mapped_format.at(accessor.type);

			break;
		}
		case TINYGLTF_COMPONENT_TYPE_FLOAT:
		{
			static const std::map<int, vk::Format> mapped_format = {
			    {TINYGLTF_TYPE_SCALAR, vk::Format::eR32Sfloat},
			    {TINYGLTF_TYPE_VEC2, vk::Format::eR32G32Sfloat},
			    {TINYGLTF_TYPE_VEC3, vk::Format::eR32G32B32Sfloat},
			    {TINYGLTF_TYPE_VEC4, vk::Format::eR32G32B32A32Sfloat}};

			format = mapped_format.at(accessor.type);

			break;
		}
		default:
		{
			format = vk::Format::eUndefined;
			break;
		}
	}

	return format;
}

std::vector<uint8_t> get_attr_data(const tinygltf::Model &model, uint32_t accessor_id)
{
	assert(accessor_id < model.accessors.size());
	auto &accessor = model.accessors[accessor_id];
	assert(accessor.bufferView < model.bufferViews.size());
	auto &buffer_view = model.bufferViews[accessor.bufferView];
	assert(buffer_view.buffer < model.buffers.size());
	auto &buffer = model.buffers[buffer_view.buffer];

	size_t stride     = accessor.ByteStride(buffer_view);
	size_t start_byte = accessor.byteOffset + buffer_view.byteOffset;
	size_t end_byte   = start_byte + accessor.count * stride;

	return {buffer.data.begin() + start_byte, buffer.data.begin() + end_byte};
}

std::vector<uint8_t> convert_data_stride(const std::vector<uint8_t> &src,
                                         uint32_t                    src_stride,
                                         uint32_t                    dst_stride)
{
	auto                 elem_count = src.size() / src_stride;
	std::vector<uint8_t> dst(elem_count * dst_stride);

	for (uint32_t src_idx = 0, dst_idx = 0; src_idx < src.size() && dst_idx < dst.size();
	     src_idx += src_stride, dst_idx += dst_stride)
	{
		std::copy(src.begin() + src_idx, src.begin() + src_idx + src_stride, dst.begin() + dst_idx);
	}

	return dst;
}

}        // namespace mz