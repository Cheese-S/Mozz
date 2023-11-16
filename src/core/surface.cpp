#include "surface.hpp"
#include "GLFW/glfw3.h"
#include "instance.hpp"
#include "window.hpp"

namespace mz
{
Surface::Surface(Instance &instance, Window &window) :
    instance_(instance)
{
	handle_ = window.create_surface(instance_);
}

Surface::~Surface()
{
	instance_.get_handle().destroySurfaceKHR(handle_);
}

}        // namespace mz