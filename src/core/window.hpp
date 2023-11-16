#pragma once

#include <stdint.h>
#include <vector>

const int DEFAULT_WIDTH  = 800;
const int DEFAULT_HEIGHT = 600;

struct GLFWwindow;

namespace vk
{
class SurfaceKHR;
struct Extent2D;
}        // namespace vk

namespace mz
{
class Raytracer;
class Renderer;
class Instance;
class Window
{
  public:
	static const int DEFAULT_WINDOW_WIDTH;
	static const int DEFAULT_WINDOW_HEIGHT;
	static void      push_required_extensions(std::vector<const char *> &extensions);

	Window(const char *title, int width = DEFAULT_WIDTH,
	       int height = DEFAULT_HEIGHT);
	~Window();

	void           register_callbacks(Renderer &renderer);
	void           register_callbacks(Raytracer &raytracer);
	vk::SurfaceKHR create_surface(Instance &instance);
	vk::Extent2D   wait_for_non_zero_extent();
	bool           should_close();
	void           poll_events();
	void           wait_events();

	GLFWwindow  *get_handle();
	vk::Extent2D get_extent() const;

  private:
	GLFWwindow *handle_;
};
}        // namespace mz