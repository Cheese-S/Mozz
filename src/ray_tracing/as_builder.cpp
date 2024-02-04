#include "as_builder.hpp"

#include "common/utils.hpp"

#include "core/command_buffer.hpp"
#include "core/device.hpp"
#include "core/device_memory/buffer.hpp"
#include "scene_graph/components/mesh.hpp"
#include "scene_graph/components/submesh.hpp"

namespace mz
{

void append_submesh_blas_info(BLASMeshInfo &blas_minfo, Device &device, sg::SubMesh &submesh);

BLASMeshInfo ASBuilder::mesh_to_blas_minfo(Device &device, sg::Mesh &mesh)
{
	BLASMeshInfo               blas_minfo;
	std::vector<sg::SubMesh *> p_submeshes = mesh.get_p_submeshs();
	for (sg::SubMesh *p_submesh : p_submeshes)
	{
		append_submesh_blas_info(blas_minfo, device, *p_submesh);
	}
	return blas_minfo;
}

void append_submesh_blas_info(BLASMeshInfo &blas_minfo, Device &device, sg::SubMesh &submesh)
{
	vk::DeviceAddress vert_buf_addr = device.get_buffer_device_address(*submesh.p_vert_buf_);
	vk::DeviceAddress idx_buf_addr  = device.get_buffer_device_address(*submesh.p_idx_buf_);

	uint32_t max_trig_cnt = submesh.idx_count_ / 3;

	vk::AccelerationStructureGeometryKHR geometry{
	    .geometryType = vk::GeometryTypeKHR::eTriangles,
	    .geometry     = {
	            .triangles = {
	                .vertexFormat = vk::Format::eR32G32B32Sfloat,
	                .vertexData   = {
	                      .deviceAddress = vert_buf_addr,
                },
	                .vertexStride = sizeof(sg::Vertex),
	                .maxVertex    = submesh.vert_count_ - 1,
	                .indexType    = vk::IndexType::eUint32,
	                .indexData    = {
	                       .deviceAddress = idx_buf_addr,
                },
            },
        },
	    .flags = vk::GeometryFlagBitsKHR::eOpaque,
	};

	vk::AccelerationStructureBuildRangeInfoKHR range_info{
	    .primitiveCount  = max_trig_cnt,
	    .primitiveOffset = 0,
	    .firstVertex     = 0,
	    .transformOffset = 0,
	};

	blas_minfo.geometrys.push_back(geometry);
	blas_minfo.range_infos.push_back(range_info);
}

ASBuilder::ASBuilder(Device &device) :
    device_(device)
{
}

AccelerationStructure ASBuilder::build_TLASs(std::vector<vk::AccelerationStructureInstanceKHR> &tlas_infos, vk::BuildAccelerationStructureFlagsKHR flags, bool update)
{
	uint32_t instance_cnt      = tlas_infos.size();
	uint32_t instance_buf_size = instance_cnt * sizeof(vk::AccelerationStructureInstanceKHR);

	CommandBuffer cmd_buf = device_.begin_one_time_buf();

	Buffer instance_buf = device_.get_device_memory_allocator().allocate_AS_build_buffer(instance_buf_size);
	Buffer staging_buf  = device_.get_device_memory_allocator().allocate_staging_buffer(instance_buf_size);
	staging_buf.update(tlas_infos);
	cmd_buf.copy_buffer(staging_buf, instance_buf, instance_buf_size);

	vk::DeviceAddress instance_buf_addr = device_.get_buffer_device_address(instance_buf);
	vk::MemoryBarrier barrier{
	    .srcAccessMask = vk::AccessFlagBits::eTransferWrite,
	    .dstAccessMask = vk::AccessFlagBits::eAccelerationStructureWriteKHR,
	};
	cmd_buf.get_handle().pipelineBarrier(vk::PipelineStageFlagBits::eTransfer, vk::PipelineStageFlagBits::eAccelerationStructureBuildKHR, {}, barrier, nullptr, nullptr);

	vk::AccelerationStructureGeometryKHR TLAS_geometry{
	    .geometryType = vk::GeometryTypeKHR::eInstances,
	    .geometry     = {
	            .instances = {
	                .data = {
	                    .deviceAddress = instance_buf_addr,
                },
            },
        },
	    .flags = vk::GeometryFlagBitsKHR::eOpaque,
	};

	vk::AccelerationStructureBuildGeometryInfoKHR build_ginfo{
	    .type          = vk::AccelerationStructureTypeKHR::eTopLevel,
	    .flags         = flags,
	    .mode          = update ? vk::BuildAccelerationStructureModeKHR::eUpdate : vk::BuildAccelerationStructureModeKHR::eBuild,
	    .geometryCount = 1,
	    .pGeometries   = &TLAS_geometry,
	};

	vk::AccelerationStructureBuildSizesInfoKHR build_sinfo = device_.get_handle().getAccelerationStructureBuildSizesKHR(vk::AccelerationStructureBuildTypeKHR::eDevice, build_ginfo, instance_cnt);

	vk::AccelerationStructureCreateInfoKHR as_cinfo{
	    .size = build_sinfo.accelerationStructureSize,
	    .type = vk::AccelerationStructureTypeKHR::eTopLevel,
	};

	AccelerationStructure tlas(device_, as_cinfo);

	Buffer scratch_buf = device_.get_device_memory_allocator().allocate_scratch_buffer(build_sinfo.buildScratchSize);

	build_ginfo.dstAccelerationStructure  = tlas.get_handle();
	build_ginfo.scratchData.deviceAddress = device_.get_buffer_device_address(scratch_buf);

	vk::AccelerationStructureBuildRangeInfoKHR build_rinfo{
	    .primitiveCount = instance_cnt,
	};

	cmd_buf.get_handle().buildAccelerationStructuresKHR(build_ginfo, &build_rinfo);

	device_.end_one_time_buf(cmd_buf);

	return tlas;
}

std::vector<std::unique_ptr<AccelerationStructure>> ASBuilder::build_BLASs(std::vector<BLASMeshInfo> &blas_minfos, vk::BuildAccelerationStructureFlagsKHR flags)
{
	uint32_t       blas_cnt          = blas_minfos.size();
	uint32_t       compact_cnt       = 0;
	vk::DeviceSize max_scratching_sz = 0;
	vk::DeviceSize total_sz          = 0;

	std::vector<BuildAS> build_ASs(blas_cnt);
	for (uint32_t i = 0; i < blas_cnt; i++)
	{
		BuildAS      &build_AS   = build_ASs[i];
		BLASMeshInfo &blas_minfo = blas_minfos[i];

		build_AS.build_info = {
		    .type          = vk::AccelerationStructureTypeKHR::eBottomLevel,
		    .flags         = blas_minfo.flags | flags,
		    .mode          = vk::BuildAccelerationStructureModeKHR::eBuild,
		    .geometryCount = to_u32(blas_minfo.geometrys.size()),
		    .pGeometries   = blas_minfo.geometrys.data(),
		};

		build_AS.p_range_info = blas_minfos[i].range_infos.data();

		std::vector<uint32_t> max_prim_count(blas_minfo.range_infos.size());
		for (uint32_t j = 0; j < blas_minfo.range_infos.size(); j++)
		{
			max_prim_count[j] = blas_minfo.range_infos[j].primitiveCount;
		}

		device_.get_handle().getAccelerationStructureBuildSizesKHR(vk::AccelerationStructureBuildTypeKHR::eDevice, &build_AS.build_info, max_prim_count.data(), &build_AS.size_info);

		total_sz += build_AS.size_info.accelerationStructureSize;
		max_scratching_sz = std::max(max_scratching_sz, build_AS.size_info.buildScratchSize);

		compact_cnt += has_flag(build_AS.build_info.flags, vk::BuildAccelerationStructureFlagBitsKHR::eAllowCompaction);
	}

	Buffer            scratch_buf      = device_.get_device_memory_allocator().allocate_scratch_buffer(max_scratching_sz);
	vk::DeviceAddress scratch_buf_addr = device_.get_buffer_device_address(scratch_buf);

	vk::QueryPool query_pool{VK_NULL_HANDLE};
	if (compact_cnt)
	{
		assert(compact_cnt == blas_cnt);
		vk::QueryPoolCreateInfo query_pool_cinfo{
		    .queryType  = vk::QueryType::eAccelerationStructureCompactedSizeKHR,
		    .queryCount = blas_cnt,
		};
		query_pool = device_.get_handle().createQueryPool(query_pool_cinfo, nullptr);
	}

	std::vector<uint32_t> indices;
	vk::DeviceSize        batch_sz    = 0;
	vk::DeviceSize        batch_limit = 256000000;        // 256 mb
	for (uint32_t i = 0; i < blas_cnt; i++)
	{
		indices.push_back(i);
		batch_sz += build_ASs[i].size_info.accelerationStructureSize;

		if (batch_sz >= batch_limit || i == blas_cnt - 1)
		{
			CommandBuffer cmd_buf = device_.begin_one_time_buf();
			create_blas(cmd_buf, indices, build_ASs, scratch_buf_addr, query_pool);
			device_.end_one_time_buf(cmd_buf);

			if (query_pool)
			{
				CommandBuffer compact_cmd_buf = device_.begin_one_time_buf();
				compact_blas(compact_cmd_buf, indices, build_ASs, query_pool);
				device_.end_one_time_buf(compact_cmd_buf);
			}

			batch_sz = 0;
			indices.clear();
		}
	}

	std::vector<std::unique_ptr<AccelerationStructure>> res;
	for (BuildAS &build_AS : build_ASs)
	{
		res.push_back(std::move(build_AS.p_as));
	}
	device_.get_handle().destroyQueryPool(query_pool);
	return res;
}

void ASBuilder::create_blas(CommandBuffer &cmd_buf, std::vector<uint32_t> &indices, std::vector<BuildAS> &buildASs, vk::DeviceAddress scratch_addr, vk::QueryPool query_pool)
{
	uint32_t query_cnt = 0;
	if (query_pool)
	{
		device_.get_handle().resetQueryPool(query_pool, 0, indices.size());
	}

	for (const auto i : indices)
	{
		BuildAS &build_AS = buildASs[i];

		vk::AccelerationStructureCreateInfoKHR as_cinfo = {
		    .size = build_AS.size_info.accelerationStructureSize,
		    .type = vk::AccelerationStructureTypeKHR::eBottomLevel,
		};

		build_AS.p_as = std::make_unique<AccelerationStructure>(device_, as_cinfo);

		build_AS.build_info.dstAccelerationStructure  = build_AS.p_as->get_handle();
		build_AS.build_info.scratchData.deviceAddress = scratch_addr;

		cmd_buf.get_handle().buildAccelerationStructuresKHR(build_AS.build_info, build_AS.p_range_info);

		vk::MemoryBarrier barrier{
		    .srcAccessMask = vk::AccessFlagBits::eAccelerationStructureWriteKHR,
		    .dstAccessMask = vk::AccessFlagBits::eAccelerationStructureReadKHR,
		};
		cmd_buf.get_handle().pipelineBarrier(vk::PipelineStageFlagBits::eAccelerationStructureBuildKHR, vk::PipelineStageFlagBits::eAccelerationStructureBuildKHR, {}, barrier, nullptr, nullptr);

		if (query_pool)
		{
			cmd_buf.get_handle().writeAccelerationStructuresPropertiesKHR(build_AS.build_info.dstAccelerationStructure, vk::QueryType::eAccelerationStructureCompactedSizeKHR, query_pool, query_cnt++);
		}
	}
}

void ASBuilder::compact_blas(CommandBuffer &cmd_buf, std::vector<uint32_t> &indices, std::vector<BuildAS> &buildASs, vk::QueryPool query_pool)
{
	uint32_t                    query_cnt = 0;
	std::vector<vk::DeviceSize> compact_sizes(indices.size());
	auto                        result = device_.get_handle().getQueryPoolResults(query_pool, 0, to_u32(compact_sizes.size()), compact_sizes.size() * sizeof(VkDeviceSize), compact_sizes.data(), sizeof(vk::DeviceSize), vk::QueryResultFlagBits::eWait);

	for (uint32_t idx : indices)
	{
		BuildAS &buildAS                            = buildASs[idx];
		buildAS.p_cleanup_as                        = std::move(buildAS.p_as);
		buildAS.size_info.accelerationStructureSize = compact_sizes[query_cnt++];

		vk::AccelerationStructureCreateInfoKHR as_cinfo{
		    .size = buildAS.size_info.accelerationStructureSize,
		    .type = vk::AccelerationStructureTypeKHR::eBottomLevel,
		};

		buildAS.p_as = std::make_unique<AccelerationStructure>(device_, as_cinfo);

		vk::CopyAccelerationStructureInfoKHR copy_info{
		    .src  = buildAS.build_info.dstAccelerationStructure,
		    .dst  = buildAS.p_as->get_handle(),
		    .mode = vk::CopyAccelerationStructureModeKHR::eCompact,
		};

		cmd_buf.get_handle().copyAccelerationStructureKHR(copy_info);
	}
}

};        // namespace mz