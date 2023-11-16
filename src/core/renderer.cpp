#include "renderer.hpp"

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

namespace mz
{
const uint32_t Renderer::NUM_INFLIGHT_FRAMES = 2;

Renderer::Renderer()
{
	p_window_ = std::make_unique<Window>("Wolfie3D");
	p_window_->register_callbacks(*this);

	vk::StructureChain<vk::PhysicalDeviceFeatures2,
	                   vk::PhysicalDeviceBufferDeviceAddressFeatures>
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

	p_ctx_              = std::make_unique<Context>(ctx_cinfo);
	p_descriptor_state_ = std::make_unique<DescriptorState>(p_ctx_->device);
	p_cmd_pool_         = std::make_unique<CommandPool>(p_ctx_->device, p_ctx_->device.get_graphics_queue());
	p_swapchain_        = std::make_unique<Swapchain>(*p_ctx_, p_window_->get_extent());
	load_scene("2.0/DamagedHelmet/glTF/DamagedHelmet.gltf");
	create_pbr_resources();
	create_rendering_resources();
	create_raytrace_resources();
	p_sframe_buffer_ = std::make_unique<SwapchainFramebuffer>(p_ctx_->device, *p_swapchain_, *p_render_pass_);
}

Renderer::~Renderer(){};

void Renderer::start()
{
	main_loop();
	timer_.start();
}

void Renderer::main_loop()
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

void Renderer::update()
{
	double delta_time = timer_.tick();
	p_camera_node_->get_component<sg::Script>().update(delta_time);
	std::vector<sg::Animation *> p_animations = p_scene_->get_components<sg::Animation>();
	for (auto p_animation : p_animations)
	{
		// if (p_animation->get_name() == "Survey")
		{
			p_animation->update(delta_time);
		};
	}
}

void Renderer::render_frame()
{
	uint32_t img_idx = sync_acquire_next_image();
	record_draw_commands(img_idx);
	sync_submit_commands();
	sync_present(img_idx);
	frame_idx_ = (frame_idx_ + 1) % NUM_INFLIGHT_FRAMES;
}

uint32_t Renderer::sync_acquire_next_image()
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

void Renderer::sync_submit_commands()
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
}

void Renderer::sync_present(uint32_t img_idx)
{
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
}

void Renderer::resize()
{
	vk::Extent2D extent = p_window_->wait_for_non_zero_extent();
	p_ctx_->device.get_handle().waitIdle();
	p_swapchain_->rebuild(p_window_->get_extent());
	p_sframe_buffer_->rebuild();
	p_camera_node_->get_component<sg::Script>().resize(extent.width, extent.height);
}

void Renderer::record_draw_commands(uint32_t img_idx)
{
	CommandBuffer &cmd_buf = get_current_frame_resource().cmd_buf;
	cmd_buf.reset();
	cmd_buf.begin();
	update_camera_ubo();
	set_dynamic_states(cmd_buf);
	begin_render_pass(cmd_buf, p_sframe_buffer_->get_handle(img_idx));
	draw_skybox(cmd_buf);
	draw_scene(cmd_buf);
	cmd_buf.get_handle().endRenderPass();
	cmd_buf.get_handle().end();
}

void Renderer::update_camera_ubo()
{
	sg::Camera &camera = p_camera_node_->get_component<sg::Camera>();
	Buffer     &buf    = get_current_frame_resource().camera_buf;

	CameraUBO ubo{
	    .proj_view = camera.get_projection() * camera.get_view(),
	    .cam_pos   = p_camera_node_->get_transform().get_translation(),
	};

	buf.update(&ubo, sizeof(ubo));
}

void Renderer::set_dynamic_states(CommandBuffer &cmd_buf)
{
	vk::Extent2D swapchain_extent = p_swapchain_->get_swapchain_properties().extent;
	vk::Viewport viewport{
	    .x        = 0,
	    .y        = 0,
	    .width    = static_cast<float>(swapchain_extent.width),
	    .height   = static_cast<float>(swapchain_extent.height),
	    .minDepth = 0.0f,
	    .maxDepth = 1.0f,
	};
	vk::Rect2D scissor{
	    .offset = {
	        .x = 0,
	        .y = 0,
	    },
	    .extent = swapchain_extent,
	};

	cmd_buf.get_handle().setViewport(0, viewport);
	cmd_buf.get_handle().setScissor(0, scissor);
}

void Renderer::begin_render_pass(CommandBuffer &cmd_buf, vk::Framebuffer framebuffer)
{
	std::array<vk::ClearValue, 2> clear_values{
	    std::array<float, 4>{0.54f, 0.81f, 0.94f, 1.0f},
	};
	clear_values[1].depthStencil = vk::ClearDepthStencilValue{1.0f, 0};

	vk::RenderPassBeginInfo render_pass_binfo{
	    .renderPass  = p_render_pass_->get_handle(),
	    .framebuffer = framebuffer,
	    .renderArea  = {
	         .offset = {
	             .x = 0,
	             .y = 0,
            },
	         .extent = p_swapchain_->get_swapchain_properties().extent,
        },
	    .clearValueCount = clear_values.size(),
	    .pClearValues    = clear_values.data(),
	};

	cmd_buf.get_handle().beginRenderPass(render_pass_binfo, vk::SubpassContents::eInline);
}

void Renderer::draw_skybox(CommandBuffer &cmd_buf)
{
	sg::Camera    &camera = p_camera_node_->get_component<sg::Camera>();
	FrameResource &frame  = get_current_frame_resource();
	SkyboxPCO      pco{
	         .proj = camera.get_projection(),
	         .view = camera.get_view(),
    };
	cmd_buf.get_handle().bindPipeline(
	    vk::PipelineBindPoint::eGraphics,
	    skybox_.p_pl->get_handle());
	cmd_buf.get_handle().bindDescriptorSets(
	    vk::PipelineBindPoint::eGraphics,
	    skybox_.p_pl->get_pipeline_layout(),
	    0,
	    frame.skybox_set,
	    {});
	cmd_buf.get_handle().pushConstants<SkyboxPCO>(
	    skybox_.p_pl->get_pipeline_layout(),
	    vk::ShaderStageFlagBits::eVertex,
	    0,
	    pco);
	draw_submesh(cmd_buf, *baked_pbr_.p_box);
}        // namespace mz

void Renderer::draw_scene(CommandBuffer &cmd_buf)
{
	vk::PipelineLayout pl_layout = pbr_.p_pl->get_pipeline_layout();
	cmd_buf.get_handle().bindPipeline(
	    vk::PipelineBindPoint::eGraphics,
	    pbr_.p_pl->get_handle());
	cmd_buf.get_handle().bindDescriptorSets(
	    vk::PipelineBindPoint::eGraphics,
	    pbr_.p_pl->get_pipeline_layout(),
	    0,
	    get_current_frame_resource().pbr_set,
	    {});

	std::queue<sg::Node *> p_nodes;
	p_nodes.push(&p_scene_->get_root_node());
	while (!p_nodes.empty())
	{
		sg::Node *p_node = p_nodes.front();
		p_nodes.pop();

		draw_node(cmd_buf, *p_node);

		std::vector<sg::Node *> p_children = p_node->get_children();
		for (sg::Node *p_child : p_children)
		{
			p_nodes.push(p_child);
		}
	}
}

void Renderer::draw_node(CommandBuffer &cmd_buf, sg::Node &node)
{
	if (node.has_component<sg::Mesh>())
	{
		if (node.has_component<sg::Skin>())
		{
			bind_skin(cmd_buf, node.get_component<sg::Skin>());
		}
		else
		{
			disable_skin(cmd_buf);
		}

		PBRPCO pbr_pco{
		    .model = node.get_transform().get_world_M(),
		};
		std::vector<sg::SubMesh *> p_submeshs = node.get_component<sg::Mesh>().get_p_submeshs();
		for (sg::SubMesh *p_submesh : p_submeshs)
		{
			const sg::PBRMaterial *p_pbr_material = dynamic_cast<const sg::PBRMaterial *>(p_submesh->get_material());
			bind_material(cmd_buf, *p_pbr_material, pbr_pco);
			pbr_pco.material_flag = p_pbr_material->flag;
			cmd_buf.get_handle().pushConstants<PBRPCO>(pbr_.p_pl->get_pipeline_layout(), vk::ShaderStageFlagBits::eVertex | vk::ShaderStageFlagBits::eFragment, 0, pbr_pco);
			draw_submesh(cmd_buf, *p_submesh);
		}
	}
}

void Renderer::push_node_model_matrix(CommandBuffer &cmd_buf, sg::Node &node)
{
	glm::mat4 rotated_m = node.get_transform().get_world_M();
	PBRPCO    pco       = {
	             .model = rotated_m,
    };
	cmd_buf.get_handle().pushConstants<PBRPCO>(pbr_.p_pl->get_pipeline_layout(), vk::ShaderStageFlagBits::eVertex, 0, pco);
}

void Renderer::bind_material(CommandBuffer &cmd_buf, const sg::PBRMaterial &material, PBRPCO &pco)
{
	pco.material_flag        = material.flag;
	pco.base_color           = material.base_color_factor_;
	pco.metallic_roughness.g = material.roughness_factor;
	pco.metallic_roughness.b = material.metallic_factor;
	cmd_buf.get_handle().bindDescriptorSets(
	    vk::PipelineBindPoint::eGraphics,
	    pbr_.p_pl->get_pipeline_layout(),
	    1,
	    material.set,
	    {});
}

void Renderer::bind_skin(CommandBuffer &cmd_buf, const sg::Skin &skin)
{
	Buffer &buf = get_current_frame_resource().joint_buf;

	JointUBO ubo{
	    .is_skinned = 1,
	};

	skin.compute_joint_Ms(*p_scene_, ubo.joint_Ms);

	buf.update(&ubo, sizeof(ubo));
}

void Renderer::disable_skin(CommandBuffer &cmd_buf)
{
	Buffer &buf        = get_current_frame_resource().joint_buf;
	float   is_skinned = 0;
	buf.update(&is_skinned, sizeof(is_skinned), offsetof(JointUBO, is_skinned));
}

void Renderer::draw_submesh(CommandBuffer &cmd_buf, sg::SubMesh &submesh)
{
	cmd_buf.get_handle().bindVertexBuffers(0, submesh.p_vert_buf_->get_handle(), {0});
	if (submesh.p_idx_buf_)
	{
		cmd_buf.get_handle().bindIndexBuffer(submesh.p_idx_buf_->get_handle(), 0, vk::IndexType::eUint32);
		cmd_buf.get_handle().drawIndexed(submesh.idx_count_, 1, 0, 0, 0);
	}
	else
	{
		cmd_buf.get_handle().draw(submesh.vert_count_, 1, 0, 0);
	}
}

Renderer::FrameResource &Renderer::get_current_frame_resource()
{
	return frame_resources_[frame_idx_];
};

void Renderer::process_event(const Event &event)
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

void Renderer::load_scene(const char *scene_name)
{
	GLTFLoader loader(*p_ctx_);
	p_scene_ = loader.read_scene_from_file(scene_name);

	vk::Extent2D window_extent = p_window_->get_extent();
	p_camera_node_             = add_arc_ball_camera_script(*p_scene_, "main_camera", window_extent.width, window_extent.height);
}

void Renderer::create_raytrace_resources()
{
	create_storage_image();
	create_ASs();
	create_raytrace_descriptor_resources();
}

void Renderer::create_pbr_resources()
{
	PBRBaker baker(*p_ctx_);
	baked_pbr_ = baker.bake();
}

void Renderer::create_rendering_resources()
{
	create_frame_resources();
	create_descriptor_resources();
	create_render_pass();
	create_pipeline_resources();
}

void Renderer::create_frame_resources()
{
	Device &device = p_ctx_->device;
	for (uint32_t i = 0; i < NUM_INFLIGHT_FRAMES; i++)
	{
		frame_resources_.push_back({
		    .cmd_buf                   = std::move(p_cmd_pool_->allocate_command_buffer()),
		    .camera_buf                = std::move(device.get_device_memory_allocator().allocate_uniform_buffer(sizeof(CameraUBO))),
		    .joint_buf                 = std::move(device.get_device_memory_allocator().allocate_uniform_buffer(sizeof(JointUBO))),
		    .image_avaliable_semaphore = std::move(Semaphore(device)),
		    .render_finished_semaphore = std::move(Semaphore(device)),
		    .in_flight_fence           = std::move(Fence(device, vk::FenceCreateFlagBits::eSignaled)),
		});
	}
}

void Renderer::create_descriptor_resources()
{
	create_pbr_desc_resources();
	create_skybox_desc_resources();
	create_materials_desc_resources();
}

void Renderer::create_pbr_desc_resources()
{
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

	for (uint32_t i = 0; i < NUM_INFLIGHT_FRAMES; i++)
	{
		vk::DescriptorBufferInfo camera_bbinfo{
		    .buffer = frame_resources_[i].camera_buf.get_handle(),
		    .offset = 0,
		    .range  = sizeof(CameraUBO),
		};

		vk::DescriptorBufferInfo joint_bbinfo{
		    .buffer = frame_resources_[i].joint_buf.get_handle(),
		    .offset = 0,
		    .range  = sizeof(JointUBO),
		};

		DescriptorAllocation allocation =
		    DescriptorBuilder::begin(p_descriptor_state_->cache, p_descriptor_state_->allocator)
		        .bind_buffer(0, camera_bbinfo, vk::DescriptorType::eUniformBuffer, vk::ShaderStageFlagBits::eVertex | vk::ShaderStageFlagBits::eFragment)
		        .bind_buffer(1, joint_bbinfo, vk::DescriptorType::eUniformBuffer, vk::ShaderStageFlagBits::eVertex)
		        .bind_image(2, irradiance, vk::DescriptorType::eCombinedImageSampler, vk::ShaderStageFlagBits::eFragment)
		        .bind_image(3, prefilter, vk::DescriptorType::eCombinedImageSampler, vk::ShaderStageFlagBits::eFragment)
		        .bind_image(4, brdf_lut, vk::DescriptorType::eCombinedImageSampler, vk::ShaderStageFlagBits::eFragment)
		        .build();

		frame_resources_[i].pbr_set                            = allocation.set;
		pbr_.desc_layout_ring[DescriptorRingAccessor::eGlobal] = allocation.set_layout;
	}
}

void Renderer::create_skybox_desc_resources()
{
	vk::DescriptorImageInfo background{
	    .sampler     = baked_pbr_.p_background->sampler.get_handle(),
	    .imageView   = baked_pbr_.p_background->resource.get_view().get_handle(),
	    .imageLayout = vk::ImageLayout::eShaderReadOnlyOptimal,
	};

	for (uint32_t i = 0; i < NUM_INFLIGHT_FRAMES; i++)
	{
		DescriptorAllocation skybox_allocation =
		    DescriptorBuilder::begin(p_descriptor_state_->cache, p_descriptor_state_->allocator)
		        .bind_image(0, background, vk::DescriptorType::eCombinedImageSampler, vk::ShaderStageFlagBits::eFragment)
		        .build();

		frame_resources_[i].skybox_set                            = skybox_allocation.set;
		skybox_.desc_layout_ring[DescriptorRingAccessor::eGlobal] = skybox_allocation.set_layout;
	}
}

void Renderer::create_materials_desc_resources()
{
	static const std::vector<std::string> pbr_texture_names = {
	    "base_color_texture",
	    "normal_texture",
	    "occlusion_texture",
	    "emissive_texture",
	    "metallic_roughness_texture",
	};
	sg::Texture *p_default_texture = p_scene_->find_component<sg::Texture>("default_texture");

	std::vector<sg::PBRMaterial *> p_materials = p_scene_->get_components<sg::PBRMaterial>();
	for (sg::PBRMaterial *p_material : p_materials)
	{
		DescriptorBuilder builder =
		    DescriptorBuilder::begin(p_descriptor_state_->cache, p_descriptor_state_->allocator);

		std::vector<vk::DescriptorImageInfo> desc_iinfos;
		desc_iinfos.reserve(pbr_texture_names.size());

		for (int i = 0; i < pbr_texture_names.size(); i++)
		{
			const std::string &name      = pbr_texture_names[i];
			sg::Texture       *p_texture = p_default_texture;
			auto               it        = p_material->texture_map_.find(name);
			if (it != p_material->texture_map_.end())
			{
				p_texture = it->second;
			}
			desc_iinfos.emplace_back(vk::DescriptorImageInfo{
			    .sampler     = p_texture->p_sampler_->get_handle(),
			    .imageView   = p_texture->p_resource_->get_view().get_handle(),
			    .imageLayout = vk::ImageLayout::eShaderReadOnlyOptimal,
			});
			builder.bind_image(i, desc_iinfos.back(), vk::DescriptorType::eCombinedImageSampler, vk::ShaderStageFlagBits::eFragment);
		}

		DescriptorAllocation desc_allocation = builder.build();

		p_material->set                                          = desc_allocation.set;
		pbr_.desc_layout_ring[DescriptorRingAccessor::eMaterial] = desc_allocation.set_layout;
	}
}

void Renderer::create_render_pass()
{
	std::array<vk::AttachmentDescription, 2> attachemnts;

	attachemnts[0] = RenderPass::color_attachment(p_swapchain_->get_swapchain_properties().surface_format.format, vk::ImageLayout::eUndefined, vk::ImageLayout::ePresentSrcKHR);
	vk::AttachmentReference color_attachment_ref{
	    .attachment = 0,
	    .layout     = vk::ImageLayout::eColorAttachmentOptimal,
	};

	attachemnts[1] = RenderPass::depth_attachment(p_swapchain_->choose_depth_format(), vk::ImageLayout::eUndefined, vk::ImageLayout::eDepthStencilAttachmentOptimal);

	vk::AttachmentReference depth_attachemnt_ref{
	    .attachment = 1,
	    .layout     = vk::ImageLayout::eDepthStencilAttachmentOptimal,
	};

	vk::SubpassDescription subpass{
	    .pipelineBindPoint       = vk::PipelineBindPoint::eGraphics,
	    .colorAttachmentCount    = 1,
	    .pColorAttachments       = &color_attachment_ref,
	    .pDepthStencilAttachment = &depth_attachemnt_ref,
	};

	vk::SubpassDependency dependency{
	    .srcSubpass   = VK_SUBPASS_EXTERNAL,
	    .dstSubpass   = 0,
	    .srcStageMask = vk::PipelineStageFlagBits::eColorAttachmentOutput |
	                    vk::PipelineStageFlagBits::eEarlyFragmentTests,
	    .dstStageMask = vk::PipelineStageFlagBits::eColorAttachmentOutput |
	                    vk::PipelineStageFlagBits::eEarlyFragmentTests,
	    .srcAccessMask = {},
	    .dstAccessMask = vk::AccessFlagBits::eColorAttachmentRead |
	                     vk::AccessFlagBits::eColorAttachmentWrite |
	                     vk::AccessFlagBits::eDepthStencilAttachmentWrite,
	};

	vk::RenderPassCreateInfo render_pass_cinfo{
	    .attachmentCount = 2,
	    .pAttachments    = attachemnts.data(),
	    .subpassCount    = 1,
	    .pSubpasses      = &subpass,
	    .dependencyCount = 1,
	    .pDependencies   = &dependency,
	};

	p_render_pass_ = std::make_unique<RenderPass>(p_ctx_->device, render_pass_cinfo);
}

void Renderer::create_pipeline_resources()
{
	std::array<vk::VertexInputBindingDescription, 1> binding_descriptions;
	binding_descriptions[0] = vk::VertexInputBindingDescription{
	    .binding   = 0,
	    .stride    = sizeof(sg::Vertex),
	    .inputRate = vk::VertexInputRate::eVertex,
	};
	GraphicsPipelineState pl_state{
	    .vert_shader_name   = "pbr.vert.spv",
	    .frag_shader_name   = "pbr.frag.spv",
	    .vertex_input_state = {
	        .attribute_descriptions = sg::Vertex::get_input_attr_descriptions(),
	        .binding_descriptions   = binding_descriptions,
	    },
	};

	std::array<vk::PushConstantRange, 1> pbr_push_const_ranges;
	pbr_push_const_ranges[0] = {
	    .stageFlags = vk::ShaderStageFlagBits::eVertex | vk::ShaderStageFlagBits::eFragment,
	    .offset     = 0,
	    .size       = sizeof(PBRPCO),
	};
	vk::PipelineLayoutCreateInfo pbr_pl_layout_cinfo{
	    .setLayoutCount         = 2,
	    .pSetLayouts            = pbr_.desc_layout_ring.data(),
	    .pushConstantRangeCount = to_u32(pbr_push_const_ranges.size()),
	    .pPushConstantRanges    = pbr_push_const_ranges.data(),
	};

	pbr_.p_pl = std::make_unique<GraphicsPipeline>(p_ctx_->device, *p_render_pass_, pl_state, pbr_pl_layout_cinfo);

	vk::PushConstantRange skybox_push_const_range{
	    .stageFlags = vk::ShaderStageFlagBits::eVertex,
	    .offset     = 0,
	    .size       = sizeof(SkyboxPCO),
	};
	vk::PipelineLayoutCreateInfo skybox_pl_layout_cinfo{
	    .setLayoutCount         = 1,
	    .pSetLayouts            = skybox_.desc_layout_ring.data(),
	    .pushConstantRangeCount = 1,
	    .pPushConstantRanges    = &skybox_push_const_range,
	};
	pl_state.vert_shader_name                       = "skybox.vert.spv";
	pl_state.frag_shader_name                       = "skybox.frag.spv";
	pl_state.rasterization_state.cull_mode          = vk::CullModeFlagBits::eFront;
	pl_state.depth_stencil_state.depth_test_enable  = false;
	pl_state.depth_stencil_state.depth_write_enable = false;
	skybox_.p_pl                                    = std::make_unique<GraphicsPipeline>(p_ctx_->device, *p_render_pass_, pl_state, skybox_pl_layout_cinfo);
}

void Renderer::create_ASs()
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
		    .instanceCustomIndex                    = i,
		    .mask                                   = 0xFF,
		    .instanceShaderBindingTableRecordOffset = 0,
		    .flags                                  = VK_GEOMETRY_INSTANCE_TRIANGLE_FRONT_COUNTERCLOCKWISE_BIT_KHR,
		    .accelerationStructureReference         = p_ctx_->device.get_buffer_device_address(raytrace_.p_BLASs[i]->get_buffer()),
		});
	}
	raytrace_.p_TLAS = std::make_unique<AccelerationStructure>(builder.build_TLASs(tlas, vk::BuildAccelerationStructureFlagBitsKHR::ePreferFastTrace));
}

void Renderer::create_raytrace_descriptor_resources()
{
	vk::DescriptorImageInfo storage_img_info{
	    .imageView   = p_storage_img_->get_view().get_handle(),
	    .imageLayout = vk::ImageLayout::eGeneral,
	};

	vk::WriteDescriptorSetAccelerationStructureKHR as_write{
	    .accelerationStructureCount = 1,
	    .pAccelerationStructures    = &(raytrace_.p_TLAS->get_handle()),
	};

	DescriptorAllocation desc_allocation =
	    DescriptorBuilder::begin(p_descriptor_state_->cache, p_descriptor_state_->allocator)
	        .bind_tlas(0, as_write, vk::DescriptorType::eAccelerationStructureKHR, vk::ShaderStageFlagBits::eRaygenKHR)
	        .bind_image(1, storage_img_info, vk::DescriptorType::eStorageImage, vk::ShaderStageFlagBits::eRaygenKHR)
	        .build();

	raytrace_.desc_layout_ring[0] = desc_allocation.set_layout;
	raytrace_.global_set          = desc_allocation.set;
}

void Renderer::create_storage_image()
{
	vk::Extent2D        extent = p_window_->get_extent();
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

	vk::ImageViewCreateInfo view_cinfo = ImageView::two_dim_view_cinfo(img.get_handle(), vk::Format::eB8G8R8A8Unorm, vk::ImageAspectFlagBits::eColor, 0);

	p_storage_img_ = std::make_unique<ImageResource>(std::move(img), ImageView(p_ctx_->device, view_cinfo));

	CommandBuffer cmd_buf = p_ctx_->device.begin_one_time_buf();
	cmd_buf.set_image_layout(*p_storage_img_, vk::ImageLayout::eUndefined, vk::ImageLayout::eGeneral);
	p_ctx_->device.end_one_time_buf(cmd_buf);
}

}        // namespace mz
