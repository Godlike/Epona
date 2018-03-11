/*
* Copyright (C) 2018 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#ifndef EPONA_DEBUG_IMPLEMENTATION_HPP
#define EPONA_DEBUG_IMPLEMENTATION_HPP

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

/* Provides callback interface for debugging */
class DebugImplementation
{
public:
    /**
     * @brief QuickhullConvexHull debug call function
     *
     * This method is called from within the QuickhullConvexHull class
     * during the calculation. It then proxies all the arguments to the currently
     * set callback functor.
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
        GetQuickhullConvexHullCallback<VertexBuffer>()(convexHull, vertexBuffer);
    }

    /**
    * @brief Initializes QuickhullConvexHull callback instance
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
        GetQuickhullConvexHullCallback<BufferType>() = callback;
    }

private:
    /**
    * @brief Returns reference to the QuickhullConvexHull callback instance
    *
    * @tparam VertexBuffer convex hull vertex buffer type
    *
    * @return callback function reference
    */
    template < typename BufferType >
    static std::function<void(QuickhullConvexHull<BufferType>&, BufferType&)>&
        GetQuickhullConvexHullCallback()
    {
        static std::function<void(QuickhullConvexHull<BufferType>&, BufferType&)> convexHull = {};
        return convexHull;
    }
};

} // namespace debug
} // namespace epona

#endif // EPONA_DEBUG_IMPLEMENTATION_HPP
