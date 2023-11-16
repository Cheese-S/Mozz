#pragma once

#include <stdint.h>
#include <type_traits>

namespace mz
{
template <typename T>
class BitFlags
{
  public:
	using FlagType = typename std::underlying_type<T>::type;

	constexpr BitFlags() :
	    flag_(0)
	{}

	constexpr BitFlags(T bit) :
	    flag_(static_cast<FlagType>(bit))
	{}

	constexpr BitFlags(BitFlags<T> const &rhs) :
	    flag_(rhs.flag_)
	{
	}

	constexpr BitFlags<T> operator&(BitFlags<T> const &rhs) const noexcept
	{
		return BitFlags<T>(flag_ & rhs.flag_);
	}

	constexpr BitFlags<T> operator|(BitFlags<T> const &rhs) const noexcept
	{
		return BitFlags<T>(flag_ | rhs.flag_);
	}
	constexpr BitFlags<T> operator^(BitFlags<T> const &rhs) const noexcept
	{
		return BitFlags<T>(flag_ ^ rhs.flag_);
	}

	constexpr BitFlags<T> &operator=(BitFlags<T> const &rhs) = default;

	constexpr BitFlags<T> &operator&=(BitFlags<T> const &rhs)
	{
		flag_ &= rhs.flag_;
		return *this;
	}

	constexpr BitFlags<T> &operator|=(BitFlags<T> const &rhs)
	{
		flag_ |= rhs.flag_;
		return *this;
	}

	constexpr BitFlags<T> &operator^=(BitFlags<T> const &rhs)
	{
		flag_ |= rhs.flag_;
		return *this;
	}

	constexpr operator FlagType() const noexcept
	{
		return flag_;
	}

  private:
	FlagType flag_;
};
}        // namespace mz