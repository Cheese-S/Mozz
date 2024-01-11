#pragma once

#if defined(__APPLE__) && defined(__MACH__)
constexpr const bool IS_ON_OSX = true;
#else
constexpr const bool IS_ON_OSX = false;
#endif

extern const char *APP_NAME;