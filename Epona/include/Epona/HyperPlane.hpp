/*
* Copyright (C) 2017 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#ifndef EPONA_HYPER_PLANE_HPP
#define EPONA_HYPER_PLANE_HPP

#include <glm/glm.hpp>

namespace epona
{
/**
 * @brief HyperPlane calculation algorithm
 */
class HyperPlane
{
public:
    HyperPlane() = default;

    /**
     * @brief Constructs a plane in Hessian Normal Form
     *
     * Allows for the normal direction correction
     * using below the hyperplane point if one is specified.
     * @param[in] normal plane's normal vector of unit length
     * @param[in] point point on the plane
     * @param[in] below point below the plane, allows for the normal direction correction
     */
    HyperPlane(glm::vec3 const& normal,
        glm::vec3 const& point,
        glm::vec3 const* below = nullptr
    );

    /**
     * @brief Constructs a plane in Hessian Normal Form
     *
     * Constructs a hyperplane from the given vertices
     * and allows for the normal direction correction
     * using below the hyperplane point if one is specified.
     * @param[in] a point on the plane
     * @param[in] b point on the plane
     * @param[in] c point on the plane
     * @param[in] below point below the plane, allows for the normal direction correction
     */
    HyperPlane(glm::vec3 const& a,
        glm::vec3 const& b,
        glm::vec3 const& c,
        glm::vec3 const* below = nullptr
    );

    /**
     * @brief Constructs a plane in Hessian Normal Form
     *
     * Constructs a hyperplane from the given vertices
     * and allows for the normal direction correction
     * using below the hyperplane point if one is specified.
     * @param[in] vertices points on the plane
     * @param[in] below point below the plane
     */
    HyperPlane(
        glm::mat3 const& vertices,
        glm::vec3 const* below = nullptr
    );

    /** brief Returns point on the plane */
    glm::vec3 const& GetPoint() const;

    /** Returns plane normal vector */
    glm::vec3 const& GetNormal() const;

    /** Returns a distance from the plane to the origin */
    float GetDistance() const;

    /** Sets the plane normal vector */
    void SetNormal(glm::vec3 const& normal);

    /** Sets a point on the plane */
    void SetPoint(glm::vec3 const& point);

    /**
     * @brief Calculates absolute distance from the plane to a point
     * @param[in] point the point of interest
     * @return absolute distance from the point to the plane
     */
    float Distance(glm::vec3 const& point) const;

    /**
     * @brief Calculates signed distance from the plane to a point
     * @param[in] point the point of interest
     * @return signed distance from the plane to the point
     */
    float SignedDistance(glm::vec3 const& point) const;

    /**
     * @brief Calculates whether a ray and the plane are intersecting
     * @param[in] rayNormal ray direction vector
     * @param[in] rayPoint point on the ray
     * @param[out] resultPoint intersection point
     * @return @c true if there is an intersection point, @c false otherwise
     */
    bool RayIntersection(
        glm::vec3 const& rayNormal, glm::vec3 const& rayPoint, glm::vec3& resultPoint
    ) const;

    /**
     * @brief Calculates whether a line segment and the plane are intersecting
     * @param[in] lineStart start of the line segment
     * @param[in] lineEnd end of the line segment
     * @param[out] resultPoint intersection point
     * @return @c true if there is intersection point, @c false otherwise
     */
    bool LineSegmentIntersection(
        glm::vec3 const& lineStart, glm::vec3 const& lineEnd, glm::vec3& resultPoint
    ) const;

    /**
     *  @brief  Calculates closest point on the plane to a given point
     *
     *  @param  point point of interest
     *
     *  @return closest point on the plane
     */
    glm::vec3 ClosestPoint(glm::vec3 const& point) const;

private:
    glm::vec3 m_normal;
    glm::vec3 m_point;
    float m_distance;
};
} // namespace epona
#endif // EPONA_HYPER_PLANE_HPP
