#pragma once

#include <utility>

namespace crs_lib
{
	template<class T>
	union Uninitialized final
	{
		char dummy{};
		T v;

		void init(auto&& ... args)
		{
			new(v) T(std::forward<decltype(args)>(args)...);
		}

		void destroy()
		{
			v.~T();
		}

		~Uninitialized()
		{}
	};
}