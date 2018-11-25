/*
* Copyright (C) 2019 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#ifndef EPONA_DEBUG_DUMMY_HPP
#define EPONA_DEBUG_DUMMY_HPP

#include <functional>
#include <vector>
#include <glm/glm.hpp>

namespace epona {
    template <typename T>
    class QuickhullConvexHull;
} // namespace epona

namespace epona
{
namespace debug
{

/* Provides dummy callback interface for debugging */
class DebugDummy
{
public:
    /**
     * @brief QuickhullConvexHull debug dummy call function
     *
     * @note This is a dummy method that should be optimized away
     *
     * @tparam VertexBuffer convex hull vertex buffer type
     *
     * @param[in] convexHull   current convex hull instance
     * @param[in] vertexBuffer source vertices for the convex hull
     */
    template < typename VertexBuffer >
    static void QuickhullConvexHullCall(
            QuickhullConvexHull<VertexBuffer>& convexHull, VertexBuffer& vertexBuffer
        )
    {
    }

    /**
    * @brief Initializes QuickhullConvexHull callback instance
    *
    * @note This is a dummy method that should be optimized away
    *
    * @tparam VertexBuffer convex hull vertex buffer type
    *
    * @param callback callback function object
    */
    template < typename BufferType >
    static void SetQuickhullConvexHullCallback(
        std::function<void(QuickhullConvexHull<BufferType>&, BufferType&)> callback
    )
    {
    }
};

} // namespace debug
} // namespace epona

#endif // EPONA_DEBUG_DUMMY_HPP
