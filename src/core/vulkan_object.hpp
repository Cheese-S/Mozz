#pragma once

namespace mz
{
template <typename THandle>
class VulkanObject
{
  public:
	VulkanObject(THandle handle = nullptr) :
	    handle_(handle){};

	VulkanObject(const VulkanObject &)            = delete;
	VulkanObject &operator=(const VulkanObject &) = delete;
	VulkanObject &operator=(VulkanObject &&)      = delete;
	VulkanObject(VulkanObject &&rhs) :
	    handle_(rhs.handle_)
	{
		rhs.handle_ = nullptr;
	}

	virtual ~VulkanObject() = default;

	inline const THandle &get_handle() const
	{
		return handle_;
	}

  protected:
	THandle handle_;
};
}        // namespace mz