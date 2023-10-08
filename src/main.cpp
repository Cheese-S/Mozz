#include <stdlib.h>

#include <exception>
#include <iostream>

#include "core/renderer.hpp"

int main()
{
	mz::Renderer renderer;
	try
	{
		renderer.start();
	}
	catch (const std::exception &e)
	{
		std::cerr << e.what() << std::endl;
		return EXIT_FAILURE;
	}

	return EXIT_SUCCESS;
};