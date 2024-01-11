#include <stdlib.h>

#include <exception>
#include <iostream>

#include "core/renderer.hpp"
#include "ray_tracer.hpp"

int main()
{
	mz::Raytracer raytracer;
	// mz::Renderer renderer;
	try
	{
		raytracer.start();
	}
	catch (const std::exception &e)
	{
		std::cerr << e.what() << std::endl;
		return EXIT_FAILURE;
	}

	return EXIT_SUCCESS;
};