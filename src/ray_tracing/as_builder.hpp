#pragma once

#include <memory>
#include <vector>

#include "acceleration_structure.hpp"
#include "common/vk_common.hpp"

namespace mz
{

class Device;
class CommandBuffer;
class Buffer;

namespace sg
{
class Mesh;
class SubMesh;
};        // namespace sg

struct BLASMeshInfo
{
	std::vector<vk::AccelerationStructureGeometryKHR>       geometrys;
	std::vector<vk::AccelerationStructureBuildRangeInfoKHR> range_infos;
	vk::BuildAccelerationStructureFlagsKHR                  flags;
};

struct BuildAS
{
	vk::AccelerationStructureBuildGeometryInfoKHR     build_info;
	vk::AccelerationStructureBuildSizesInfoKHR        size_info;
	const vk::AccelerationStructureBuildRangeInfoKHR *p_range_info;

	std::unique_ptr<AccelerationStructure> p_as;
	std::unique_ptr<AccelerationStructure> p_cleanup_as;
};

class ASBuilder
{
  public:
	static BLASMeshInfo mesh_to_blas_minfo(Device &device, sg::Mesh &mesh);

	ASBuilder(Device &device);
	std::vector<std::unique_ptr<AccelerationStructure>> build_BLASs(std::vector<BLASMeshInfo> &mesh_infos, vk::BuildAccelerationStructureFlagsKHR flags);
	AccelerationStructure                               build_TLASs(std::vector<vk::AccelerationStructureInstanceKHR> &tlas_infos, vk::BuildAccelerationStructureFlagsKHR flags, bool update = false);

  private:
	void create_blas(CommandBuffer &cmd_buf, std::vector<uint32_t> &indices, std::vector<BuildAS> &buildAS, vk::DeviceAddress scratch_addr, vk::QueryPool query_pool);
	void compact_blas(CommandBuffer &cmd_buf, std::vector<uint32_t> &indices, std::vector<BuildAS> &buildASs, vk::QueryPool query_pool);

	Device &device_;
};
}        // namespace mz