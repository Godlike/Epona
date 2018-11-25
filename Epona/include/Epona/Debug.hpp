/*
* Copyright (C) 2019 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#ifndef EPONA_DEBUG_HPP
#define EPONA_DEBUG_HPP

#ifdef EPONA_DEBUG
#include <Epona/debug/DebugImplementation.hpp>
#else
#include <Epona/debug/DebugDummy.hpp>
#endif

namespace epona
{
namespace debug
{
#ifdef EPONA_DEBUG
    using Debug = DebugImplementation;
#else
    using Debug = DebugDummy;
#endif
} // namespace debug
} // namespace epona

#endif // EPONA_DEBUG_HPP
