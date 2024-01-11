#include "timer.hpp"

namespace mz
{
Timer::Timer() :
    start_time_(Clock::now()),
    previous_tick_(Clock::now())
{
}

void Timer::start()
{
	if (!running_)
	{
		running_    = true;
		start_time_ = Clock::now();
	}
}

bool Timer::is_running() const
{
	return running_;
}
}        // namespace mz