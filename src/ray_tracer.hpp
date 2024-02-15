#pragma once

#include "common/timer.hpp"
#include "common/vk_common.hpp"

#include "core/command_buffer.hpp"
#include "core/device_memory/buffer.hpp"
#include "core/image_resource.hpp"
#include "core/sampler.hpp"
#include "core/sync_objects.hpp"
#include "pbr_baker.hpp"
#include "ray_tracing/as_builder.hpp"
#include "scene_graph/components/skin.hpp"

namespace mz
{

namespace sg
{
class PBRMaterial;
class Texture;
class Camera;
}        // namespace sg

class Window;
class Context;
class Swapchain;
class RenderPass;
class SwapchainFramebuffer;
class AccelerationStructure;
class RaytracingPipeline;

struct DescriptorState;
struct Event;

class Raytracer
{
  public:
	Raytracer();
	~Raytracer();

	void start();
	void process_event(const Event &event);

  private:
	static const uint32_t NUM_INFLIGHT_FRAMES;

	struct FrameResource
	{
		CommandBuffer     cmd_buf;
		Buffer            camera_buf;
		Semaphore         image_avaliable_semaphore;
		Semaphore         render_finished_semaphore;
		Fence             in_flight_fence;
		vk::DescriptorSet scene_set;
	};

	struct RaytraceResource
	{
		std::array<vk::DescriptorSetLayout, 4>              desc_layout_ring;
		std::vector<std::unique_ptr<AccelerationStructure>> p_BLASs;
		std::unique_ptr<AccelerationStructure>              p_TLAS;
		std::unique_ptr<RaytracingPipeline>                 p_pl;
		std::unique_ptr<Buffer>                             p_sbt_buf;
		vk::StridedDeviceAddressRegionKHR                   rgen_region;
		vk::StridedDeviceAddressRegionKHR                   miss_region;
		vk::StridedDeviceAddressRegionKHR                   hit_region;
		vk::StridedDeviceAddressRegionKHR                   call_region;
		vk::DescriptorSet                                   global_set;
	};

	enum DescriptorRingAccessor
	{
		eGlobal   = 0,
		eMaterial = 1,
	};

	struct RTCameraUBO
	{
		glm::mat4 view_inverse;
		glm::mat4 proj_inverse;
	};

	struct RTPCO
	{
		glm::vec4 clear_color;
		glm::vec4 light_position;
		float     light_intensity;
	};

	struct SubmeshAccessInfo
	{
		vk::DeviceAddress vert_buf_addr;
		vk::DeviceAddress idx_buf_addr;
		uint64_t          material_idx;
	};

	struct Material
	{
		glm::vec4 base_color;
		glm::vec4 metallic_roughness_ior;
		uint64_t  albedo_texture_idx;
		uint64_t  normal_texture_idx;
		uint64_t  occlusion_texture_idx;
		uint64_t  emissive_texture_idx;
		uint64_t  metallic_roughness_texture_idx;
		float     ior;
		float     pad;
	};

	struct Light
	{
		glm::vec4 pos;
	};

	void main_loop();
	void update();
	void render_frame();
	void raytrace(CommandBuffer &cmd_buf);

	uint32_t sync_acquire_next_image();
	void     sync_submit_commands();
	void     sync_present(uint32_t img_idx);
	void     record_draw_commands(uint32_t img_idx);
	void     update_frame_buffer_image(CommandBuffer &cmd_buf, uint32_t img_idx);

	void update_camera_ubo();

	void           resize();
	void           resize_raytrace_resource();
	void           update_storage_image_descriptor();
	FrameResource &get_current_frame_resource();

	void load_scene(const char *scene_name);
	void create_context();
	void create_rendering_resources();
	void create_frame_resources();

	void create_pbr_resource();
	void create_raytrace_resources();
	void create_scene_ubos();

	void create_material_ubo();
	void create_submesh_ainfo_ubo();
	void create_light_ubo();
	void create_raytrace_descriptors();
	void create_storage_image();
	void create_ASs();
	void create_raytrace_descriptor_resources();
	void create_raytrace_pipeline();
	void created_shader_binding_table();

	std::unique_ptr<Window>               p_window_;
	std::unique_ptr<Context>              p_ctx_;
	std::unique_ptr<Swapchain>            p_swapchain_;
	std::unique_ptr<RenderPass>           p_render_pass_;
	std::unique_ptr<SwapchainFramebuffer> p_sframe_buffer_;
	std::unique_ptr<DescriptorState>      p_descriptor_state_;
	std::unique_ptr<CommandPool>          p_cmd_pool_;
	std::unique_ptr<sg::Scene>            p_scene_;
	std::unique_ptr<ImageResource>        p_storage_img_;
	std::unique_ptr<Buffer>               p_submesh_ainfos_buf;
	std::unique_ptr<Buffer>               p_material_buf;
	std::unique_ptr<Buffer>               p_light_buf;

	sg::Node *p_camera_node_ = nullptr;

	Timer                      timer_;
	uint32_t                   frame_idx_ = 0;
	std::vector<FrameResource> frame_resources_;
	RaytraceResource           raytrace_;
	PBR                        baked_pbr_;
	bool                       is_window_resized_ = false;
};
}        // namespace mz
