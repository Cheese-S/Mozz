#include "ray_tracing_pipeline.hpp"

#include "core/device.hpp"

#include "common/file_utils.hpp"
#include "common/utils.hpp"

namespace mz
{

RaytracingPipeline::RaytracingPipeline(Device &device, std::array<const char *, StageIndices::eShaderGroupCount> &shader_names, vk::PipelineLayoutCreateInfo &pl_layout_cinfo) :
    device_(device)
{
	std::array<vk::PipelineShaderStageCreateInfo, eShaderGroupCount> stage_cinfos;

	vk::PipelineShaderStageCreateInfo stage_cinfo{
	    .pName = "main",
	};

	stage_cinfo.stage     = vk::ShaderStageFlagBits::eRaygenKHR;
	stage_cinfo.module    = create_shader_module(shader_names[eRaygen]);
	stage_cinfos[eRaygen] = stage_cinfo;

	stage_cinfo.stage   = vk::ShaderStageFlagBits::eMissKHR;
	stage_cinfo.module  = create_shader_module(shader_names[eMiss]);
	stage_cinfos[eMiss] = stage_cinfo;

	stage_cinfo.stage     = vk::ShaderStageFlagBits::eMissKHR;
	stage_cinfo.module    = create_shader_module(shader_names[eShadow]);
	stage_cinfos[eShadow] = stage_cinfo;

	stage_cinfo.stage         = vk::ShaderStageFlagBits::eClosestHitKHR;
	stage_cinfo.module        = create_shader_module(shader_names[eClosestHit]);
	stage_cinfos[eClosestHit] = stage_cinfo;

	std::vector<vk::RayTracingShaderGroupCreateInfoKHR> group_cinfos;

	vk::RayTracingShaderGroupCreateInfoKHR group_cinfo{
	    .generalShader      = VK_SHADER_UNUSED_KHR,
	    .closestHitShader   = VK_SHADER_UNUSED_KHR,
	    .anyHitShader       = VK_SHADER_UNUSED_KHR,
	    .intersectionShader = VK_SHADER_UNUSED_KHR,
	};

	group_cinfo.type          = vk::RayTracingShaderGroupTypeKHR::eGeneral;
	group_cinfo.generalShader = eRaygen;
	group_cinfos.push_back(group_cinfo);

	group_cinfo.type          = vk::RayTracingShaderGroupTypeKHR::eGeneral;
	group_cinfo.generalShader = eMiss;
	group_cinfos.push_back(group_cinfo);

	group_cinfo.type          = vk::RayTracingShaderGroupTypeKHR::eGeneral;
	group_cinfo.generalShader = eShadow;
	group_cinfos.push_back(group_cinfo);

	group_cinfo.type             = vk::RayTracingShaderGroupTypeKHR::eTrianglesHitGroup;
	group_cinfo.generalShader    = VK_SHADER_UNUSED_KHR;
	group_cinfo.closestHitShader = eClosestHit;
	group_cinfos.push_back(group_cinfo);

	pl_layout_ = device_.get_handle().createPipelineLayout(pl_layout_cinfo);

	vk::RayTracingPipelineCreateInfoKHR pl_cinfo{
	    .stageCount                   = to_u32(stage_cinfos.size()),
	    .pStages                      = stage_cinfos.data(),
	    .groupCount                   = to_u32(group_cinfos.size()),
	    .pGroups                      = group_cinfos.data(),
	    .maxPipelineRayRecursionDepth = 2,
	    .layout                       = pl_layout_,
	};

	auto res = device_.get_handle().createRayTracingPipelineKHR({}, {}, pl_cinfo);
	handle_  = res.value;

	for (auto &stage_cinfo : stage_cinfos)
	{
		device_.get_handle().destroyShaderModule(stage_cinfo.module);
	}
}

RaytracingPipeline::~RaytracingPipeline()
{
	device_.get_handle().destroyPipelineLayout(pl_layout_);
	device_.get_handle().destroyPipeline(handle_);
}

vk::ShaderModule RaytracingPipeline::create_shader_module(const std::string &name)
{
	std::vector<uint8_t>       binary = fu::read_shader_binary(name);
	vk::ShaderModuleCreateInfo shader_module_cinfo{
	    .codeSize = to_u32(binary.size()),
	    .pCode    = reinterpret_cast<const uint32_t *>(binary.data()),
	};
	return device_.get_handle().createShaderModule(shader_module_cinfo);
}

vk::PipelineLayout RaytracingPipeline::get_pipeline_layout() const
{
	return pl_layout_;
}

}        // namespace mz