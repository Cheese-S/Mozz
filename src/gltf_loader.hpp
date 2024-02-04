#pragma once

#include <tiny_gltf.h>

#include <memory>

#include "common/glm_common.hpp"

namespace mz
{
class Context;

namespace sg
{
class Scene;
class Node;
class Camera;
class Image;
class Mesh;
class SubMesh;
class PBRMaterial;
class Sampler;
class Texture;
class AABB;
class Skin;
struct AnimationSampler;
struct AnimationChannel;
struct Vertex;
};        // namespace sg

struct ImageTransferInfo;

class GLTFLoader
{
  public:
	static const glm::vec3 MZ_CONVERSION_SCALE;
	static const uint64_t  MAX_TEXUTRE_ID;

	GLTFLoader(const Context &ctx);
	virtual ~GLTFLoader() = default;
	std::unique_ptr<sg::Scene>   read_scene_from_file(const std::string &file_name,
	                                                  int                scene_index = -1);
	std::unique_ptr<sg::SubMesh> read_model_from_file(const std::string &file_name, int mesh_idx);

  private:
	void      load_gltf_model(const std::string &file_name);
	sg::Scene parse_scene(int scene_idx = -1);

	void load_samplers();
	void load_images();
	void load_textures();
	void load_materials();
	void load_meshs();
	void load_cameras();
	void load_nodes(int scene_idx);
	void load_animations();
	void load_skins();
	void load_default_camera();

	std::vector<std::unique_ptr<sg::Node>> parse_nodes();
	std::unique_ptr<sg::Node>              parse_node(tinygltf::Node &gltf_node);
	std::unique_ptr<sg::Camera>            parse_camera(tinygltf::Camera &gltf_camera);
	std::unique_ptr<sg::Mesh>              parse_mesh(tinygltf::Mesh &gltf_mesh);
	std::unique_ptr<sg::SubMesh>           parse_submesh(sg::Mesh *p_mesh, tinygltf::Primitive &gltf_submesh);
	std::unique_ptr<sg::PBRMaterial>       parse_material(tinygltf::Material &gltf_material);
	std::unique_ptr<sg::Image>             parse_image(tinygltf::Image &gltf_image);
	std::unique_ptr<sg::Sampler>           parse_sampler(tinygltf::Sampler &gltf_sampler);
	std::unique_ptr<sg::Texture>           parse_texture(tinygltf::Texture &gltf_texture);
	std::unique_ptr<sg::SubMesh>           parse_submesh_as_model(
	              tinygltf::Primitive &gltf_primitive);
	std::vector<sg::AnimationSampler> parse_animation_samplers(const tinygltf::Animation &gltf_animation);
	void                              parse_animation_input_data(const tinygltf::AnimationSampler &gltf_sampler, sg::AnimationSampler &sampler);
	void                              parse_animation_output_data(const tinygltf::AnimationSampler &gltf_sampler, sg::AnimationSampler &sampler);
	std::vector<sg::AnimationChannel> parse_animation_channels(const tinygltf::Animation &gltf_animation, std::vector<sg::Node *> p_nodes);
	std::unique_ptr<sg::Skin>         parse_skin(tinygltf::Skin &gltf_skin);

	std::unique_ptr<sg::PBRMaterial> create_default_material();
	std::unique_ptr<sg::Texture>     create_default_texture(sg::Sampler &default_sampler);
	std::unique_ptr<sg::Image>       create_default_texture_image();
	std::unique_ptr<sg::Sampler>     create_default_sampler();
	std::unique_ptr<sg::Camera>      create_default_camera();

	void             batch_upload_images();
	void             create_image_resource(sg::Image &image, size_t idx);
	void             append_textures_to_material(tinygltf::ParameterMap &parameter_map, std::vector<sg::Texture *> &p_textures, sg::PBRMaterial *p_material);
	size_t           get_submesh_vertex_count(tinygltf::Primitive &submesh);
	void             update_parent_mesh_bound(sg::Mesh *p_mesh, tinygltf::Primitive &gltf_submesh);
	tinygltf::Scene *pick_scene(int scene_idx);
	void             init_node_hierarchy(tinygltf::Scene *p_gltf_scene, std::vector<std::unique_ptr<sg::Node>> &p_nodes, sg::Node &root);
	void             init_scene_bound();

	template <typename T>
	struct DataAccessInfo
	{
		const T *p_data;
		size_t   stride;        // This is stride is not BYTE stride, but T stride.
	};

	template <typename T>
	DataAccessInfo<T> get_attr_data_ptr(const tinygltf::Primitive &submesh, const char *name) const
	{
		auto it = submesh.attributes.find(name);
		if (it == submesh.attributes.end())
		{
			return {
			    .p_data = nullptr,
			    .stride = 0,
			};
		}
		return get_accessor_data_ptr<T>(it->second);
	}

	template <typename T>
	DataAccessInfo<T> get_accessor_data_ptr(int accessor_id) const
	{
		assert(accessor_id < gltf_model_.accessors.size());
		const tinygltf::Accessor &accessor = gltf_model_.accessors[accessor_id];
		assert(accessor.bufferView < gltf_model_.bufferViews.size());
		const tinygltf::BufferView &buffer_view = gltf_model_.bufferViews[accessor.bufferView];
		assert(buffer_view.buffer < gltf_model_.buffers.size());
		const tinygltf::Buffer &buffer = gltf_model_.buffers[buffer_view.buffer];

		return {
		    .p_data = reinterpret_cast<const T *>(&buffer.data[accessor.byteOffset + buffer_view.byteOffset]),
		    .stride = accessor.ByteStride(buffer_view) / sizeof(T),
		};
	}

	const Context                 &ctx_;
	sg::Scene                     *p_scene_;
	tinygltf::Model                gltf_model_;
	std::string                    model_path_;
	std::vector<ImageTransferInfo> img_tinfos_;

	size_t material_id_  = 0;
	size_t submesh_id_   = 0;
	size_t mesh_id_      = 0;
	size_t texture_id_   = 0;
	size_t node_id_      = 0;
	size_t camera_id_    = 0;
	size_t sampler_id_   = 0;
	size_t skin_id_      = 0;
	size_t image_id_     = 0;
	size_t animation_id_ = 0;
};

}        // namespace mz
