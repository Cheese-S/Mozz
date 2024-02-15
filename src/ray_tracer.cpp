#include "ray_tracer.hpp"

#include "glm/gtx/string_cast.hpp"
#include <iostream>
#include <queue>

#include "gltf_loader.hpp"

#include "common/cvar.hpp"
#include "common/error.hpp"
#include "common/file_utils.hpp"
#include "common/logging.hpp"
#include "common/utils.hpp"

#include "core/command_pool.hpp"
#include "core/context.hpp"
#include "core/descriptor_allocator.hpp"
#include "core/device.hpp"
#include "core/framebuffer.hpp"
#include "core/graphics_pipeline.hpp"
#include "core/image_resource.hpp"
#include "core/image_view.hpp"
#include "core/instance.hpp"
#include "core/physical_device.hpp"
#include "core/queue.hpp"
#include "core/render_pass.hpp"
#include "core/swapchain.hpp"
#include "core/window.hpp"

#include "scene_graph/components/camera.hpp"
#include "scene_graph/components/mesh.hpp"
#include "scene_graph/components/pbr_material.hpp"
#include "scene_graph/components/sampler.hpp"
#include "scene_graph/components/submesh.hpp"
#include "scene_graph/components/texture.hpp"
#include "scene_graph/scene.hpp"
#include "scene_graph/script.hpp"
#include "scene_graph/scripts/animation.hpp"

#include "ray_tracing/ray_tracing_pipeline.hpp"

namespace mz
{
const uint32_t Raytracer::NUM_INFLIGHT_FRAMES = 2;

Raytracer::Raytracer()
{
	p_window_ = std::make_unique<Window>("Wolfie3D");
	p_window_->register_callbacks(*this);
	create_context();

	p_descriptor_state_ = std::make_unique<DescriptorState>(p_ctx_->device);
	p_cmd_pool_         = std::make_unique<CommandPool>(p_ctx_->device, p_ctx_->device.get_graphics_queue());
	p_swapchain_        = std::make_unique<Swapchain>(*p_ctx_, p_window_->get_extent());
	load_scene("2.0/ToyCar/glTF/test2.0.gltf");
	create_rendering_resources();
	create_pbr_resource();
	create_raytrace_resources();
	// p_sframe_buffer_ = std::make_unique<SwapchainFramebuffer>(p_ctx_->device, *p_swapchain_, *p_render_pass_);
}

Raytracer::~Raytracer(){};

void Raytracer::start()
{
	main_loop();
	timer_.start();
}

void Raytracer::main_loop()
{
	while (!p_window_->should_close())
	{
		timer_.tick();
		render_frame();
		update();
		p_window_->poll_events();
	}
	p_ctx_->device.get_handle().waitIdle();
}

void Raytracer::update()
{
	double delta_time = timer_.tick();
	p_camera_node_->get_component<sg::Script>().update(delta_time);
}

void Raytracer::render_frame()
{
	uint32_t img_idx = sync_acquire_next_image();
	update_camera_ubo();
	record_draw_commands(img_idx);
	sync_submit_commands();
	sync_present(img_idx);
	frame_idx_ = (frame_idx_ + 1) % NUM_INFLIGHT_FRAMES;
}

void Raytracer::raytrace(CommandBuffer &cmd_buf)
{
	vk::Extent2D extent = p_window_->get_extent();

	RTPCO rtpco{
	    .clear_color     = glm::vec4(0.54f, 0.81f, 0.94f, 1.0f),
	    .light_position  = glm::vec4(12.0, 0.0, 0.0, 0.0),
	    .light_intensity = 1.0f,
	};

	cmd_buf.get_handle().bindPipeline(vk::PipelineBindPoint::eRayTracingKHR, raytrace_.p_pl->get_handle());
	cmd_buf.get_handle().bindDescriptorSets(vk::PipelineBindPoint::eRayTracingKHR, raytrace_.p_pl->get_pipeline_layout(), 0, {raytrace_.global_set, get_current_frame_resource().scene_set}, nullptr);
	cmd_buf.get_handle().pushConstants<RTPCO>(raytrace_.p_pl->get_pipeline_layout(), vk::ShaderStageFlagBits::eRaygenKHR | vk::ShaderStageFlagBits::eClosestHitKHR | vk::ShaderStageFlagBits::eMissKHR, 0, rtpco);
	cmd_buf.get_handle().traceRaysKHR(raytrace_.rgen_region, raytrace_.miss_region, raytrace_.hit_region, raytrace_.call_region, extent.width, extent.height, 1);
}

uint32_t Raytracer::sync_acquire_next_image()
{
	FrameResource &frame    = get_current_frame_resource();
	vk::Device     device_h = p_ctx_->device.get_handle();
	uint32_t       img_idx;

	while (true)
	{
		while (vk::Result::eTimeout ==
		       device_h.waitForFences({frame.in_flight_fence.get_handle()}, true, UINT64_MAX))
		{
			;
		}

		// Opted for plain vkAacquireNextImageKHR here because we want to deal with the error ourselves.
		// Otherwise, vulkan.hpp would've thrown the error and we have to catch it (slow).
		// See, https://github.com/KhronosGroup/Vulkan-Hpp/issues/599
		vk::Result acquired_res = static_cast<vk::Result>(vkAcquireNextImageKHR(device_h, p_swapchain_->get_handle(), UINT64_MAX, frame.image_avaliable_semaphore.get_handle(), VK_NULL_HANDLE, &img_idx));

		if (acquired_res == vk::Result::eErrorOutOfDateKHR)
		{
			resize();
		}
		else if (acquired_res != vk::Result::eSuccess && acquired_res != vk::Result::eSuboptimalKHR)
		{
			LOGE("failed to acquire swapchain image!");
			abort();
		}
		else
		{
			break;
		}
	}

	device_h.resetFences(frame.in_flight_fence.get_handle());

	return img_idx;
};

void Raytracer::sync_submit_commands()
{
	FrameResource         &frame       = get_current_frame_resource();
	vk::PipelineStageFlags wait_stages = vk::PipelineStageFlagBits::eColorAttachmentOutput;
	vk::SubmitInfo         submit_info{
	            .waitSemaphoreCount   = 1,
	            .pWaitSemaphores      = &frame.image_avaliable_semaphore.get_handle(),
	            .pWaitDstStageMask    = &wait_stages,
	            .commandBufferCount   = 1,
	            .pCommandBuffers      = &frame.cmd_buf.get_handle(),
	            .signalSemaphoreCount = 1,
	            .pSignalSemaphores    = &frame.render_finished_semaphore.get_handle(),
    };
	p_ctx_->device.get_graphics_queue().get_handle().submit(submit_info, frame.in_flight_fence.get_handle());
	p_ctx_->device.get_handle().waitIdle();
}

void Raytracer::sync_present(uint32_t img_idx)
{
	p_ctx_->device.get_handle().waitIdle();
	FrameResource     &frame = get_current_frame_resource();
	vk::PresentInfoKHR present_info{
	    .waitSemaphoreCount = 1,
	    .pWaitSemaphores    = &frame.render_finished_semaphore.get_handle(),
	    .swapchainCount     = 1,
	    .pSwapchains        = &p_swapchain_->get_handle(),
	    .pImageIndices      = &img_idx,
	};

	// Same reasoing as sync_acquire.
	// See, https://github.com/KhronosGroup/Vulkan-Hpp/issues/599
	vk::Result present_res = static_cast<vk::Result>(
	    vkQueuePresentKHR(p_ctx_->device.get_present_queue().get_handle(), reinterpret_cast<VkPresentInfoKHR *>(&present_info)));

	if (present_res == vk::Result::eErrorOutOfDateKHR || present_res == vk::Result::eSuboptimalKHR || is_window_resized_)
	{
		is_window_resized_ = false;
		resize();
	}
	else if (present_res != vk::Result::eSuccess)
	{
		LOGE("Failed to present swapchain image");
		abort();
	}
	p_ctx_->device.get_handle().waitIdle();
}

void Raytracer::resize()
{
	vk::Extent2D extent = p_window_->wait_for_non_zero_extent();
	p_ctx_->device.get_handle().waitIdle();
	p_swapchain_->rebuild(p_window_->get_extent());
	p_sframe_buffer_->rebuild();
	p_camera_node_->get_component<sg::Script>().resize(extent.width, extent.height);
	resize_raytrace_resource();
}

void Raytracer::resize_raytrace_resource()
{
	p_storage_img_.reset();
	create_storage_image();
	update_storage_image_descriptor();
}

void Raytracer::update_storage_image_descriptor()
{
	vk::DescriptorImageInfo storage_image_info{
	    .imageView   = p_storage_img_->get_view().get_handle(),
	    .imageLayout = vk::ImageLayout::eGeneral,
	};

	vk::WriteDescriptorSet write{
	    .dstSet          = raytrace_.global_set,
	    .dstBinding      = 1,
	    .descriptorCount = 1,
	    .descriptorType  = vk::DescriptorType::eStorageImage,
	    .pImageInfo      = &storage_image_info,
	};

	p_ctx_->device.get_handle().updateDescriptorSets(write, nullptr);
}

void Raytracer::record_draw_commands(uint32_t img_idx)
{
	CommandBuffer &cmd_buf = get_current_frame_resource().cmd_buf;
	cmd_buf.reset();
	cmd_buf.begin();
	update_camera_ubo();
	raytrace(cmd_buf);
	update_frame_buffer_image(cmd_buf, img_idx);
	cmd_buf.get_handle().end();
}

void Raytracer::update_camera_ubo()
{
	sg::Camera &camera = p_camera_node_->get_component<sg::Camera>();
	Buffer     &buf    = get_current_frame_resource().camera_buf;

	RTCameraUBO ubo{
	    .view_inverse = glm::inverse(camera.get_view()),
	    .proj_inverse = glm::inverse(camera.get_projection()),
	};

	// LOGI("View:{}\n View Inverse:{}\n", glm::to_string(camera.get_view()), glm::to_string(ubo.view_inverse));

	buf.update(&ubo, sizeof(ubo));
}

void Raytracer::update_frame_buffer_image(CommandBuffer &cmd_buf, uint32_t img_idx)
{
	vk::Extent2D           extent        = p_swapchain_->get_swapchain_properties().extent;
	vk::Image              swapchain_img = p_swapchain_->get_frame_images()[img_idx];
	vk::ImageMemoryBarrier swapchain_img_barrier{
	    .srcAccessMask       = {},
	    .dstAccessMask       = vk::AccessFlagBits::eTransferWrite,
	    .oldLayout           = vk::ImageLayout::eUndefined,
	    .newLayout           = vk::ImageLayout::eTransferDstOptimal,
	    .srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
	    .dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED,
	    .image               = swapchain_img,
	    .subresourceRange    = {
	           .aspectMask     = vk::ImageAspectFlagBits::eColor,
	           .baseMipLevel   = 0,
	           .levelCount     = 1,
	           .baseArrayLayer = 0,
	           .layerCount     = 1,
        },
	};
	cmd_buf.get_handle().pipelineBarrier(vk::PipelineStageFlagBits::eTopOfPipe, vk::PipelineStageFlagBits::eTransfer, {}, {}, {}, swapchain_img_barrier);

	cmd_buf.set_image_layout(*p_storage_img_, vk::ImageLayout::eGeneral, vk::ImageLayout::eTransferSrcOptimal, vk::PipelineStageFlagBits::eAllCommands, vk::PipelineStageFlagBits::eTransfer);

	vk::ImageCopy copy_region{
	    .srcSubresource = {
	        .aspectMask     = vk::ImageAspectFlagBits::eColor,
	        .mipLevel       = 0,
	        .baseArrayLayer = 0,
	        .layerCount     = 1,
	    },
	    .srcOffset      = {0, 0, 0},
	    .dstSubresource = {
	        .aspectMask     = vk::ImageAspectFlagBits::eColor,
	        .mipLevel       = 0,
	        .baseArrayLayer = 0,
	        .layerCount     = 1,
	    },
	    .dstOffset = {0, 0, 0},
	    .extent    = {extent.width, extent.height, 1},
	};

	cmd_buf.get_handle().copyImage(p_storage_img_->get_image().get_handle(), vk::ImageLayout::eTransferSrcOptimal, swapchain_img, vk::ImageLayout::eTransferDstOptimal, copy_region);

	swapchain_img_barrier.oldLayout     = vk::ImageLayout::eTransferDstOptimal;
	swapchain_img_barrier.newLayout     = vk::ImageLayout::ePresentSrcKHR;
	swapchain_img_barrier.srcAccessMask = vk::AccessFlagBits::eTransferWrite;
	swapchain_img_barrier.dstAccessMask = {};

	cmd_buf.get_handle().pipelineBarrier(vk::PipelineStageFlagBits::eTransfer, vk::PipelineStageFlagBits::eBottomOfPipe, {}, {}, {}, swapchain_img_barrier);

	cmd_buf.set_image_layout(*p_storage_img_, vk::ImageLayout::eTransferSrcOptimal, vk::ImageLayout::eGeneral, vk::PipelineStageFlagBits::eTransfer, vk::PipelineStageFlagBits::eAllCommands);
}

Raytracer::FrameResource &Raytracer::get_current_frame_resource()
{
	return frame_resources_[frame_idx_];
};

void Raytracer::process_event(const Event &event)
{
	if (event.type == EventType::eResize)
	{
		is_window_resized_ = true;
	}
	else
	{
		std::vector<sg::Script *> p_scripts = p_scene_->get_components<sg::Script>();
		for (sg::Script *p_script : p_scripts)
		{
			p_script->process_event(event);
		}
	}
}

void Raytracer::load_scene(const char *scene_name)
{
	GLTFLoader loader(*p_ctx_);
	p_scene_ = loader.read_scene_from_file(scene_name);

	vk::Extent2D window_extent = p_window_->get_extent();
	p_camera_node_             = add_arc_ball_camera_script(*p_scene_, "main_camera", window_extent.width, window_extent.height);
}

void Raytracer::create_pbr_resource()
{
	PBRBaker baker(*p_ctx_);
	baked_pbr_ = baker.bake();
}

void Raytracer::create_context()
{
	vk::StructureChain<
	    vk::PhysicalDeviceFeatures2,
	    vk::PhysicalDeviceAccelerationStructureFeaturesKHR,
	    vk::PhysicalDeviceRayTracingPipelineFeaturesKHR,
	    vk::PhysicalDeviceBufferDeviceAddressFeatures,
	    vk::PhysicalDeviceHostQueryResetFeatures,
	    vk::PhysicalDeviceDescriptorIndexingFeatures>
	    requested_features;

	ContextCreateInfo ctx_cinfo{
	    .app_name           = "Wolfie3D",
	    .device_extensions  = {},
	    .requested_features = requested_features.get<vk::PhysicalDeviceFeatures2>(),
	    .window             = *p_window_,
	};

	for (const char *extension_name : Device::REQUIRED_EXTENSIONS)
	{
		ctx_cinfo.device_extensions.push_back(extension_name);
	}

	for (const char *extension_name : Device::RAY_TRACING_EXTENSIONS)
	{
		ctx_cinfo.device_extensions.push_back(extension_name);
	}

	auto &core_features                      = requested_features.get<vk::PhysicalDeviceFeatures2>();
	core_features.features.samplerAnisotropy = true;
	core_features.features.sampleRateShading = true;
	core_features.features.shaderInt64       = true;

	auto &acc_struct_features                 = requested_features.get<vk::PhysicalDeviceAccelerationStructureFeaturesKHR>();
	acc_struct_features.accelerationStructure = true;

	auto &ray_tracing_ppl_features              = requested_features.get<vk::PhysicalDeviceRayTracingPipelineFeaturesKHR>();
	ray_tracing_ppl_features.rayTracingPipeline = true;

	auto &device_address_features               = requested_features.get<vk::PhysicalDeviceBufferDeviceAddressFeatures>();
	device_address_features.bufferDeviceAddress = true;

	auto &host_query_reset_features          = requested_features.get<vk::PhysicalDeviceHostQueryResetFeatures>();
	host_query_reset_features.hostQueryReset = true;

	auto &desc_index_feautres                                     = requested_features.get<vk::PhysicalDeviceDescriptorIndexingFeatures>();
	desc_index_feautres.runtimeDescriptorArray                    = true;
	desc_index_feautres.shaderSampledImageArrayNonUniformIndexing = true;
	desc_index_feautres.descriptorBindingVariableDescriptorCount  = true;
	desc_index_feautres.descriptorBindingPartiallyBound           = true;

	p_ctx_ = std::make_unique<Context>(ctx_cinfo);
}

void Raytracer::create_raytrace_resources()
{
	create_storage_image();
	create_ASs();
	create_raytrace_descriptor_resources();
	create_raytrace_pipeline();
	created_shader_binding_table();
}

void Raytracer::create_storage_image()
{
	vk::Extent2D        extent = p_swapchain_->get_swapchain_properties().extent;
	vk::ImageCreateInfo img_cinfo{
	    .imageType = vk::ImageType::e2D,
	    .format    = vk::Format::eB8G8R8A8Unorm,
	    .extent    = {
	           .width  = extent.width,
	           .height = extent.height,
	           .depth  = 1,
        },
	    .mipLevels   = 1,
	    .arrayLayers = 1,
	    .samples     = vk::SampleCountFlagBits::e1,
	    .tiling      = vk::ImageTiling::eOptimal,
	    .usage       = vk::ImageUsageFlagBits::eTransferSrc | vk::ImageUsageFlagBits::eStorage,
	};

	Image img = p_ctx_->device.get_device_memory_allocator().allocate_device_only_image(img_cinfo);

	vk::ImageViewCreateInfo view_cinfo = ImageView::two_dim_view_cinfo(img.get_handle(), vk::Format::eB8G8R8A8Unorm, vk::ImageAspectFlagBits::eColor, 1);

	p_storage_img_ = std::make_unique<ImageResource>(std::move(img), ImageView(p_ctx_->device, view_cinfo));

	CommandBuffer cmd_buf = p_ctx_->device.begin_one_time_buf();
	cmd_buf.set_image_layout(*p_storage_img_, vk::ImageLayout::eUndefined, vk::ImageLayout::eGeneral);
	p_ctx_->device.end_one_time_buf(cmd_buf);
}

void Raytracer::create_ASs()
{
	ASBuilder                 builder(p_ctx_->device);
	std::vector<sg::Node *>   p_nodes = p_scene_->get_nodes();
	std::vector<sg::Node *>   p_meshful_nodes;
	std::vector<BLASMeshInfo> blas_minfos;
	for (sg::Node *p_node : p_nodes)
	{
		if (p_node->has_component<sg::Mesh>())
		{
			blas_minfos.emplace_back(ASBuilder::mesh_to_blas_minfo(p_ctx_->device, p_node->get_component<sg::Mesh>()));
			p_meshful_nodes.push_back(p_node);
		}
	}
	raytrace_.p_BLASs = builder.build_BLASs(blas_minfos, vk::BuildAccelerationStructureFlagBitsKHR::eAllowCompaction);

	std::vector<vk::AccelerationStructureInstanceKHR> tlas;
	tlas.reserve(p_meshful_nodes.size());
	for (uint32_t i = 0; i < p_meshful_nodes.size(); i++)
	{
		sg::Node *p_node = p_meshful_nodes[i];
		tlas.push_back(vk::AccelerationStructureInstanceKHR{
		    .transform                              = sg::Transform::to_vk_transform(p_node->get_transform().get_world_M()),
		    .instanceCustomIndex                    = to_u32(p_node->get_component<sg::Mesh>().get_p_submeshs()[0]->get_id()),        // should give you the idx to SubmeshInfo
		    .mask                                   = 0xFF,
		    .instanceShaderBindingTableRecordOffset = 0,
		    .flags                                  = VK_GEOMETRY_INSTANCE_TRIANGLE_FRONT_COUNTERCLOCKWISE_BIT_KHR,
		    .accelerationStructureReference         = raytrace_.p_BLASs[i]->get_as_device_address(),
		});
		LOGI(glm::to_string(p_node->get_transform().get_world_M()));
	}
	raytrace_.p_TLAS = std::make_unique<AccelerationStructure>(builder.build_TLASs(tlas, vk::BuildAccelerationStructureFlagBitsKHR::ePreferFastTrace));
}

void Raytracer::create_rendering_resources()
{
	create_frame_resources();
	// create_descriptor_resources();
	// create_render_pass();
	// create_pipeline_resources();
}

void Raytracer::create_frame_resources()
{
	Device &device = p_ctx_->device;
	for (uint32_t i = 0; i < NUM_INFLIGHT_FRAMES; i++)
	{
		frame_resources_.push_back({
		    .cmd_buf                   = std::move(p_cmd_pool_->allocate_command_buffer()),
		    .camera_buf                = std::move(device.get_device_memory_allocator().allocate_uniform_buffer(sizeof(RTCameraUBO))),
		    .image_avaliable_semaphore = std::move(Semaphore(device)),
		    .render_finished_semaphore = std::move(Semaphore(device)),
		    .in_flight_fence           = std::move(Fence(device, vk::FenceCreateFlagBits::eSignaled)),
		});
	}
}

void Raytracer::create_raytrace_descriptor_resources()
{
	create_scene_ubos();
	create_raytrace_descriptors();
}

void Raytracer::create_scene_ubos()
{
	create_material_ubo();
	create_submesh_ainfo_ubo();
}

void Raytracer::create_material_ubo()
{
	sg::Texture                   *default_texture = p_scene_->get_components<sg::Texture>().back();
	std::vector<sg::PBRMaterial *> p_materials     = p_scene_->get_components<sg::PBRMaterial>();
	std::vector<Material>          materials;
	materials.reserve(p_materials.size());

	for (size_t i = 0; i < p_materials.size(); i++)
	{
		sg::PBRMaterial *p_material = p_materials[i];
		Material         mat{
		            .base_color                     = p_material->base_color_factor_,
		            .metallic_roughness_ior         = glm::vec4{0.0, p_material->roughness_factor, p_material->metallic_factor, 0.0},
		            .albedo_texture_idx             = p_material->texture_map_.count("base_color_texture") ? p_material->texture_map_["base_color_texture"]->get_id() : default_texture->get_id(),
		            .normal_texture_idx             = p_material->texture_map_.count("normal_texture") ? p_material->texture_map_["normal_texture"]->get_id() : default_texture->get_id(),
		            .occlusion_texture_idx          = p_material->texture_map_.count("occlusion_texture") ? p_material->texture_map_["occlusion_texture"]->get_id() : default_texture->get_id(),
		            .emissive_texture_idx           = p_material->texture_map_.count("emissive_texture") ? p_material->texture_map_["emissive_texture"]->get_id() : default_texture->get_id(),
		            .metallic_roughness_texture_idx = p_material->texture_map_.count("metallic_roughness_texture") ? p_material->texture_map_["metallic_roughness_texture"]->get_id() : default_texture->get_id(),
		            .ior                            = -1,
        };
		if (p_material->get_name() == "Material.003")
		{
			mat.ior = 1.5;
		}
		materials.push_back(mat);
	}
	size_t buf_size = sizeof(Material) * materials.size();

	p_material_buf     = std::make_unique<Buffer>(p_ctx_->device.get_device_memory_allocator().allocate_storage_buffer(buf_size));
	Buffer staging_buf = p_ctx_->device.get_device_memory_allocator().allocate_staging_buffer(buf_size);
	staging_buf.update(materials);
	CommandBuffer cmd_buf = p_ctx_->device.begin_one_time_buf();
	cmd_buf.copy_buffer(staging_buf, *p_material_buf, vk::BufferCopy{.size = buf_size});
	p_ctx_->device.end_one_time_buf(cmd_buf);
}

void Raytracer::create_submesh_ainfo_ubo()
{
	std::vector<sg::SubMesh *>     p_submeshes = p_scene_->get_components<sg::SubMesh>();
	std::vector<SubmeshAccessInfo> submesh_ainfos(p_submeshes.size());
	for (size_t i = 0; i < p_submeshes.size(); i++)
	{
		sg::SubMesh *p_submesh          = p_submeshes[i];
		submesh_ainfos[i].idx_buf_addr  = p_ctx_->device.get_buffer_device_address(*p_submesh->p_idx_buf_);
		submesh_ainfos[i].vert_buf_addr = p_ctx_->device.get_buffer_device_address(*p_submesh->p_vert_buf_);
		submesh_ainfos[i].material_idx  = p_submesh->get_material()->get_id();
	}
	size_t buf_size = sizeof(SubmeshAccessInfo) * submesh_ainfos.size();

	p_submesh_ainfos_buf = std::make_unique<Buffer>(p_ctx_->device.get_device_memory_allocator().allocate_storage_buffer(buf_size));
	Buffer staging_buf   = p_ctx_->device.get_device_memory_allocator().allocate_staging_buffer(buf_size);
	staging_buf.update(submesh_ainfos);
	CommandBuffer cmd_buf = p_ctx_->device.begin_one_time_buf();
	cmd_buf.copy_buffer(staging_buf, *p_submesh_ainfos_buf, vk::BufferCopy{.size = buf_size});
	p_ctx_->device.end_one_time_buf(cmd_buf);
}

void Raytracer::create_raytrace_descriptors()
{
	vk::DescriptorImageInfo storage_img_info{
	    .imageView   = p_storage_img_->get_view().get_handle(),
	    .imageLayout = vk::ImageLayout::eGeneral,
	};

	vk::WriteDescriptorSetAccelerationStructureKHR as_write{
	    .accelerationStructureCount = 1,
	    .pAccelerationStructures    = &(raytrace_.p_TLAS->get_handle()),
	};

	vk::DescriptorBufferInfo material_ubo_info{
	    .buffer = p_material_buf->get_handle(),
	    .offset = 0,
	    .range  = VK_WHOLE_SIZE,
	};

	vk::DescriptorBufferInfo submesh_ainfo_ubo_info{
	    .buffer = p_submesh_ainfos_buf->get_handle(),
	    .offset = 0,
	    .range  = VK_WHOLE_SIZE,
	};

	vk::DescriptorImageInfo sampler_dinfo{
	    .sampler = VK_NULL_HANDLE,
	};

	vk::DescriptorImageInfo background{
	    .sampler     = baked_pbr_.p_background->sampler.get_handle(),
	    .imageView   = baked_pbr_.p_background->resource.get_view().get_handle(),
	    .imageLayout = vk::ImageLayout::eShaderReadOnlyOptimal,
	};

	vk::DescriptorImageInfo irradiance{
	    .sampler     = baked_pbr_.p_irradiance->sampler.get_handle(),
	    .imageView   = baked_pbr_.p_irradiance->resource.get_view().get_handle(),
	    .imageLayout = vk::ImageLayout::eShaderReadOnlyOptimal,
	};

	vk::DescriptorImageInfo prefilter{
	    .sampler     = baked_pbr_.p_prefilter->sampler.get_handle(),
	    .imageView   = baked_pbr_.p_prefilter->resource.get_view().get_handle(),
	    .imageLayout = vk::ImageLayout::eShaderReadOnlyOptimal,
	};

	vk::DescriptorImageInfo brdf_lut{
	    .sampler     = baked_pbr_.p_brdf_lut->sampler.get_handle(),
	    .imageView   = baked_pbr_.p_brdf_lut->resource.get_view().get_handle(),
	    .imageLayout = vk::ImageLayout::eShaderReadOnlyOptimal,
	};

	std::vector<vk::DescriptorImageInfo> texture_img_dinfos;
	std::vector<sg::Texture *>           p_textures = p_scene_->get_components<sg::Texture>();

	for (sg::Texture *p_texture : p_textures)
	{
		texture_img_dinfos.push_back(vk::DescriptorImageInfo{
		    .sampler     = p_texture->p_sampler_->get_handle(),
		    .imageView   = p_texture->p_resource_->get_view().get_handle(),
		    .imageLayout = vk::ImageLayout::eShaderReadOnlyOptimal,
		});
	}

	DescriptorAllocation global_set_allocation =
	    DescriptorBuilder::begin(p_descriptor_state_->cache, p_descriptor_state_->allocator)
	        .bind_tlas(0, as_write, vk::DescriptorType::eAccelerationStructureKHR, vk::ShaderStageFlagBits::eRaygenKHR | vk::ShaderStageFlagBits::eClosestHitKHR)
	        .bind_image(1, storage_img_info, vk::DescriptorType::eStorageImage, vk::ShaderStageFlagBits::eRaygenKHR)
	        .bind_buffer(2, material_ubo_info, vk::DescriptorType::eStorageBuffer, vk::ShaderStageFlagBits::eClosestHitKHR)
	        .bind_buffer(3, submesh_ainfo_ubo_info, vk::DescriptorType::eStorageBuffer, vk::ShaderStageFlagBits::eClosestHitKHR)
	        .bind_image(4, irradiance, vk::DescriptorType::eCombinedImageSampler, vk::ShaderStageFlagBits::eClosestHitKHR)
	        .bind_image(5, prefilter, vk::DescriptorType::eCombinedImageSampler, vk::ShaderStageFlagBits::eClosestHitKHR)
	        .bind_image(6, brdf_lut, vk::DescriptorType::eCombinedImageSampler, vk::ShaderStageFlagBits::eClosestHitKHR)
	        .bind_image(7, background, vk::DescriptorType::eCombinedImageSampler, vk::ShaderStageFlagBits::eMissKHR)
	        .bind_unbounded_array(8, texture_img_dinfos, 1024, vk::ShaderStageFlagBits::eClosestHitKHR)
	        .build();

	raytrace_.global_set          = global_set_allocation.set;
	raytrace_.desc_layout_ring[0] = global_set_allocation.set_layout;

	for (int i = 0; i < NUM_INFLIGHT_FRAMES; i++)
	{
		FrameResource &frame = frame_resources_[i];

		vk::DescriptorBufferInfo camera_ubo_info{
		    .buffer = frame_resources_[i].camera_buf.get_handle(),
		    .range  = sizeof(RTCameraUBO),
		};

		DescriptorAllocation scene_set_allocation =
		    DescriptorBuilder::begin(p_descriptor_state_->cache, p_descriptor_state_->allocator)
		        .bind_buffer(0, camera_ubo_info, vk::DescriptorType::eUniformBuffer, vk::ShaderStageFlagBits::eRaygenKHR)
		        .build();

		frame.scene_set               = scene_set_allocation.set;
		raytrace_.desc_layout_ring[1] = scene_set_allocation.set_layout;
	}
}

void Raytracer::create_raytrace_pipeline()
{
	std::array<const char *, RaytracingPipeline::eShaderGroupCount> shader_names = {
	    "raygen.rgen.spv",
	    "miss.rmiss.spv",
	    "shadow.rmiss.spv",
	    "closesthit.rchit.spv",
	};

	vk::PushConstantRange pc_range{
	    .stageFlags = vk::ShaderStageFlagBits::eRaygenKHR | vk::ShaderStageFlagBits::eClosestHitKHR | vk::ShaderStageFlagBits::eMissKHR,
	    .offset     = 0,
	    .size       = sizeof(RTPCO),
	};

	vk::PipelineLayoutCreateInfo layout_cinfo{
	    .setLayoutCount         = 2,
	    .pSetLayouts            = raytrace_.desc_layout_ring.data(),
	    .pushConstantRangeCount = 1,
	    .pPushConstantRanges    = &pc_range,
	};

	raytrace_.p_pl = std::make_unique<RaytracingPipeline>(p_ctx_->device, shader_names, layout_cinfo);
}

void Raytracer::created_shader_binding_table()
{
	const vk::PhysicalDeviceRayTracingPipelinePropertiesKHR &props = p_ctx_->physical_device.get_ray_tracing_props();

	uint32_t miss_cnt    = 2;
	uint32_t hit_cnt     = 1;
	uint32_t handle_cnt  = 1 + miss_cnt + hit_cnt;
	uint32_t handle_size = props.shaderGroupHandleSize;

	uint32_t aligned_handle_size = align_up(handle_size, props.shaderGroupHandleAlignment);

	raytrace_.rgen_region.stride = align_up(aligned_handle_size, props.shaderGroupBaseAlignment);
	raytrace_.rgen_region.size   = raytrace_.rgen_region.stride;

	raytrace_.miss_region.stride = aligned_handle_size;
	raytrace_.miss_region.size   = align_up(miss_cnt * aligned_handle_size, props.shaderGroupBaseAlignment);

	raytrace_.hit_region.stride = aligned_handle_size;
	raytrace_.hit_region.size   = align_up(hit_cnt * aligned_handle_size, props.shaderGroupBaseAlignment);

	uint32_t             data_size = handle_cnt * handle_size;
	std::vector<uint8_t> handles(data_size);
	auto                 res = p_ctx_->device.get_handle().getRayTracingShaderGroupHandlesKHR(raytrace_.p_pl->get_handle(), 0, handle_cnt, data_size, handles.data());

	vk::DeviceSize sbt_size = raytrace_.rgen_region.size + raytrace_.miss_region.size + raytrace_.call_region.size + raytrace_.hit_region.size;
	raytrace_.p_sbt_buf     = std::make_unique<Buffer>(p_ctx_->device.get_device_memory_allocator().allocate_BST_buffer(sbt_size));

	vk::DeviceAddress sbt_buf_addr      = p_ctx_->device.get_buffer_device_address(*raytrace_.p_sbt_buf);
	raytrace_.rgen_region.deviceAddress = sbt_buf_addr;
	raytrace_.miss_region.deviceAddress = sbt_buf_addr + raytrace_.rgen_region.size;
	raytrace_.hit_region.deviceAddress  = raytrace_.miss_region.deviceAddress + raytrace_.miss_region.size;

	size_t offset     = 0;
	int    handle_idx = 0;
	auto   get_handle = [&](int i) { return handles.data() + i * handle_size; };
	raytrace_.p_sbt_buf->update(get_handle(handle_idx++), handle_size, offset);
	offset = raytrace_.rgen_region.size;
	for (int i = 0; i < miss_cnt; i++)
	{
		raytrace_.p_sbt_buf->update(get_handle(handle_idx++), handle_size, offset);
		offset += raytrace_.miss_region.stride;
	}

	offset = raytrace_.rgen_region.size + raytrace_.miss_region.size;
	raytrace_.p_sbt_buf->update(get_handle(handle_idx++), handle_size, offset);
}

}        // namespace mz
