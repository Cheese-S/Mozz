#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace mz::fu
{

enum class FileType
{
	eShader,
	eModelAsset,
	eImage,
};

std::vector<uint8_t> read_shader_binary(const std::string &filename);
std::vector<uint8_t> read_binary(const std::string &filename);
std::string          get_file_extension(const std::string &filename);
const std::string    compute_abs_path(const FileType type, const std::string &file);

}        // namespace mz::fu