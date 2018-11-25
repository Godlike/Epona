/*
* Copyright (C) 2019 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#ifndef EPONA_DEBUG_IMPLEMENTATION_HPP
#define EPONA_DEBUG_IMPLEMENTATION_HPP

#include <functional>
#include <vector>
#include <glm/glm.hpp>

namespace epona {
struct ConvexHull;
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
     * @param[in] convexHull   current convex hull instance
     * @param[in] vertexBuffer source vertices for the convex hull
     */
    static void QuickhullConvexHullCall(
        epona::ConvexHull& convexHull, std::vector<glm::vec3>& vertexBuffer
    )
    {
        GetQuickhullConvexHullCallback()(convexHull, vertexBuffer);
    }

    /**
    * @brief Initializes QuickhullConvexHull callback instance
    *
    * @param callback callback function object
    */
    static void SetQuickhullConvexHullCallback(
        std::function<void(epona::ConvexHull&, std::vector<glm::vec3>&)> callback
    )
    {
        GetQuickhullConvexHullCallback() = callback;
    }

private:
    /**
    * @brief Returns reference to the QuickhullConvexHull callback instance
    *
    * @return callback function reference
    */
    static std::function<void(epona::ConvexHull&, std::vector<glm::vec3>&)>&
        GetQuickhullConvexHullCallback()
    {
        static std::function<void(epona::ConvexHull&, std::vector<glm::vec3>&)> convexHull = DummyQuickhullConvexHullCallback;
        return convexHull;
    }

    /**
    * @brief QuickhullConvexHull debug call function
    */
    static void DummyQuickhullConvexHullCallback(epona::ConvexHull&, std::vector<glm::vec3>&)
    {
    }
};

} // namespace debug
} // namespace epona

#endif // EPONA_DEBUG_IMPLEMENTATION_HPP
