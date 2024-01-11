#pragma once

#include "common/timer.hpp"
#include "common/vk_common.hpp"

#include "command_buffer.hpp"
#include "core/image_resource.hpp"
#include "core/sampler.hpp"
#include "device_memory/buffer.hpp"
#include "pbr_baker.hpp"
#include "ray_tracing/as_builder.hpp"
#include "scene_graph/components/skin.hpp"
#include "sync_objects.hpp"

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
class PipelineResource;
class AccelerationStructure;

struct DescriptorState;
struct Event;

class Renderer
{
  public:
	Renderer();
	~Renderer();

	void start();
	void process_event(const Event &event);

  private:
	static const uint32_t NUM_INFLIGHT_FRAMES;

	struct FrameResource
	{
		CommandBuffer     cmd_buf;
		Buffer            camera_buf;
		Buffer            joint_buf;
		Semaphore         image_avaliable_semaphore;
		Semaphore         render_finished_semaphore;
		Fence             in_flight_fence;
		vk::DescriptorSet pbr_set;
		vk::DescriptorSet skybox_set;
	};

	struct RaytraceResource
	{
		std::vector<std::unique_ptr<AccelerationStructure>> p_BLASs;
		std::unique_ptr<AccelerationStructure>              p_TLAS;
		std::array<vk::DescriptorSetLayout, 4>              desc_layout_ring;
		vk::DescriptorSet                                   global_set;
	};

	struct PipelineResource
	{
		std::unique_ptr<GraphicsPipeline>      p_pl;
		std::array<vk::DescriptorSetLayout, 4> desc_layout_ring;
	};

	enum DescriptorRingAccessor
	{
		eGlobal   = 0,
		eMaterial = 1,
	};

	struct JointUBO
	{
		glm::mat4 joint_Ms[sg::Skin::MAX_NUM_JOINTS];
		float     is_skinned;
	};

	struct CameraUBO
	{
		glm::mat4 proj_view;
		glm::vec3 cam_pos;
	};

	struct SkyboxPCO
	{
		glm::mat4 proj;
		glm::mat4 view;
	};

	struct PBRPCO
	{
		glm::mat4 model;
		glm::vec4 base_color;
		glm::vec4 metallic_roughness;
		uint32_t  material_flag;
	};

	void main_loop();
	void update();
	void render_frame();

	uint32_t sync_acquire_next_image();
	void     sync_submit_commands();
	void     sync_present(uint32_t img_idx);
	void     record_draw_commands(uint32_t img_idx);

	void update_camera_ubo();
	void set_dynamic_states(CommandBuffer &cmd_buf);
	void begin_render_pass(CommandBuffer &cmd_buf, vk::Framebuffer framebuffer);
	void draw_scene(CommandBuffer &cmd_buf);
	void draw_skybox(CommandBuffer &cmd_buf);
	void draw_node(CommandBuffer &cmd_buf, sg::Node &node);
	void draw_submesh(CommandBuffer &cmd_buf, sg::SubMesh &submesh);
	void bind_material(CommandBuffer &cmd_buf, const sg::PBRMaterial &material, PBRPCO &pco);
	void bind_skin(CommandBuffer &cmd_buf, const sg::Skin &skin);
	void disable_skin(CommandBuffer &cmd_buf);
	void push_node_model_matrix(CommandBuffer &cmd_buf, sg::Node &node);

	void           resize();
	FrameResource &get_current_frame_resource();

	void load_scene(const char *scene_name);
	void create_pbr_resources();
	void create_rendering_resources();
	void create_frame_resources();
	void create_descriptor_resources();
	void create_skybox_desc_resources();
	void create_pbr_desc_resources();
	void create_materials_desc_resources();
	void create_render_pass();
	void create_pipeline_resources();

	void create_raytrace_resources();
	void create_storage_image();
	void create_ASs();
	void create_raytrace_descriptor_resources();

	std::unique_ptr<Window>               p_window_;
	std::unique_ptr<Context>              p_ctx_;
	std::unique_ptr<Swapchain>            p_swapchain_;
	std::unique_ptr<RenderPass>           p_render_pass_;
	std::unique_ptr<SwapchainFramebuffer> p_sframe_buffer_;
	std::unique_ptr<DescriptorState>      p_descriptor_state_;
	std::unique_ptr<CommandPool>          p_cmd_pool_;
	std::unique_ptr<sg::Scene>            p_scene_;
	std::unique_ptr<ImageResource>        p_storage_img_;
	sg::Node                             *p_camera_node_ = nullptr;

	Timer                      timer_;
	uint32_t                   frame_idx_ = 0;
	std::vector<FrameResource> frame_resources_;
	PipelineResource           skybox_;
	PipelineResource           pbr_;
	RaytraceResource           raytrace_;
	PBR                        baked_pbr_;
	bool                       is_window_resized_ = false;
};

// class Renderer
// {
//   public:
// 	Renderer();
// 	~Renderer();

// 	void start();

// 	void process_resize();
// 	void process_input_event(const InputEvent &input_event);

//   private:
// 	struct TempTexture
// 	{
// 		std::unique_ptr<SceneGraph::Image> texture;
// 		vk::raii::Sampler                  sampler;
// 	};

// 	struct LUT
// 	{
// 		std::unique_ptr<DeviceMemory::Image> image;
// 		vk::raii::ImageView                  view;
// 		vk::raii::Sampler                    sampler;
// 	};

// 	void loop();
// 	void update();
// 	void drawFrame();

// 	void                   updateUniformBuffer();
// 	void                   recordDrawCommands(const vk::raii::CommandBuffer &commandBuffer, uint32_t imageIndex);
// 	void                   draw_scene(const vk::raii::CommandBuffer &command_buffer);
// 	void                   draw_submesh(const vk::raii::CommandBuffer &command_buffer, SceneGraph::SubMesh *submesh);
// 	void                   draw_skybox(const vk::raii::CommandBuffer &command_buffer);
// 	void                   setup_scene();
// 	void                   load_texture_cubemap();
// 	void                   initVulkan();
// 	void                   createRenderPass();
// 	void                   createDescriptorSetLayout();
// 	void                   createFrameDatas();
// 	void                   createCommandBuffers();
// 	void                   createUniformBuffers();
// 	void                   createSyncObjects();
// 	void                   createDescriptorSets();
// 	void                   createGraphicsPipeline();
// 	vk::raii::ShaderModule createShaderModule(const std::string &filename);
// 	void                   perform_resize();
// 	void                   compute_irraidiance();
// 	void                   compute_prefilter_cube();
// 	void                   compute_BRDF_LUT();

// 	struct UniformBufferObject
// 	{
// 		glm::mat4 model;
// 		glm::mat4 view;
// 		glm::mat4 proj;
// 	};

// 	struct PushConstantObject
// 	{
// 		glm::mat4 model;
// 		glm::vec3 cam_pos;
// 	};

// 	struct FrameResource
// 	{
// 		vk::raii::CommandBuffer               commandBuffer           = nullptr;
// 		vk::raii::Semaphore                   imageAvaliableSemaphore = nullptr;
// 		vk::raii::Semaphore                   renderFinishedSemaphore = nullptr;
// 		vk::raii::Fence                       inflightFence           = nullptr;
// 		vk::DescriptorSet                     descriptorSet           = nullptr;
// 		vk::DescriptorSet                     skyboxDescriptorSet     = nullptr;
// 		std::unique_ptr<DeviceMemory::Buffer> uniformBuffer           = nullptr;
// 	};

// 	inline const FrameResource &currentFrame()
// 	{
// 		return frameResources_[currentFrameIdx_];
// 	}

// 	Config config_;
// 	Window window_;
// 	Timer  timer_;
// 	bool   window_resized_ = false;

// 	/* ------------------------------ VULKAN STATE ------------------------------ */
// 	Instance  instance_;
// 	Device    device_;
// 	Swapchain swapchain_;
// 	struct DescriptorManager
// 	{
// 		DescriptorManager(Device &device) :
// 		    allocator(&device),
// 		    cache(device)
// 		{
// 		}
// 		DescriptorAllocator                    allocator;
// 		DescriptorLayoutCache                  cache;
// 		std::array<vk::DescriptorSetLayout, 2> layouts;
// 	} descriptor_manager_;
// 	vk::raii::RenderPass                                                   renderPass_      = nullptr;
// 	vk::raii::PipelineLayout                                               layout_          = nullptr;
// 	vk::raii::PipelineLayout                                               skybox_layout_   = nullptr;
// 	vk::raii::Pipeline                                                     pipeline_        = nullptr;
// 	vk::raii::Pipeline                                                     skybox_pipeline_ = nullptr;
// 	std::array<vk::DescriptorSetLayout, 1>                                 skybox_layouts_;
// 	uint32_t                                                               currentFrameIdx_ = 0;
// 	std::vector<FrameResource>                                             frameResources_;
// 	std::unique_ptr<SceneGraph::Scene>                                     pScene_ = nullptr;
// 	SceneGraph::Node                                                      *pCamera_node;
// 	std::unordered_map<const SceneGraph::PBRMaterial *, vk::DescriptorSet> descriptor_set_map_;
// 	TempTexture                                                            background_    = {nullptr, nullptr};
// 	TempTexture                                                            irradiance_    = {nullptr, nullptr};
// 	TempTexture                                                            prefilter_     = {nullptr, nullptr};
// 	LUT                                                                    brdf_lut_      = {nullptr, nullptr, nullptr};
// 	std::unique_ptr<SceneGraph::Scene>                                     pSkybox_scene_ = nullptr;
// };
}        // namespace mz