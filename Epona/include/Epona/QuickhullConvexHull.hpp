/*
* Copyright (C) 2017-2018 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#ifndef EPONA_CONVEX_HULL_HPP
#define EPONA_CONVEX_HULL_HPP

#include <Epona/Debug.hpp>
#include <Epona/Analysis.hpp>
#include <Epona/HyperPlane.hpp>
#include <Epona/HalfEdgeDataStructure.hpp>
#include <Epona/JacobiEigenvalue.hpp>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/norm.hpp>
#include <glm/glm.hpp>

#include <set>
#include <unordered_set>
#include <array>
#include <list>
#include <algorithm>
#include <numeric>

namespace epona
{

//! Stores convex hull data
struct ConvexHull
{
    //! Indices of the convex hull vertices
    std::vector<uint32_t> indices;

    //! Convex hull face indices
    std::vector<glm::u32vec3> faces;

    //! Convex hull face hyperplanes
    std::vector<HyperPlane> planes;

    //! Half-Edge Data Structure of the convex hull
    HalfEdgeDataStructure heds;
};

} // namespace epona


namespace
{

//! Stores per face outlier and extreme vertex data
struct FaceOutliers {
    std::vector<std::vector<uint32_t>> outliers;
    std::vector<uint32_t> extremeIndices;
    std::vector<float> extremeSignedDistances;
};

//! Stores horizon ridge vertices
struct Ridge {
    uint32_t aVertex;
    uint32_t bVertex;

    bool operator==(Ridge other) const {
        return (aVertex == other.aVertex && bVertex == other.bVertex)
            || (aVertex == other.bVertex && bVertex == other.aVertex);
    }

    bool operator==(uint32_t index) const {
        return aVertex == index || bVertex == index;
    }
};

/**
 * @brief Calculates initial tetrehedron for the convex hull
 *
 * @note vertices size must be >= 4 and nondegenerate
 *
 * @param vertices input vertex data
 * @return convex hull
 */
epona::ConvexHull CalculateConvexHullInitialTetrahedron(std::vector<glm::vec3> const& vertices)
{
    //Find directions of the maximum spread for the calculation of the first approximation of the convex hull
    glm::mat3 basis = epona::CalculateJacobiEigenvectors(
        epona::CalculateCovarianceMatrix(
            vertices.begin(), vertices.end(), epona::CalculateExpectedValue(vertices.begin(), vertices.end())));
    basis = { glm::normalize(basis[0]), glm::normalize(basis[1]), glm::normalize(basis[2]) };

    float distances[4] = {};
    uint32_t indices[4] = {};

    //Find the furthest point in the direction of the first eigenvector of the covariance matrix
    for (uint32_t i = 0; i < vertices.size(); ++i)
    {
        float const distance = glm::abs(glm::dot(basis[0], vertices[i]));

        if (distance > distances[0])
        {
            distances[0] = distance;
            indices[0] = i;
        }
    }

    //Find the furthest point in the direction of the second eigenvector of the covariance matrix
    for (uint32_t i = 0; i < vertices.size(); ++i)
    {
        float const distance = glm::abs(glm::dot(basis[1], vertices[i]));

        if (distance > distances[1] && i != indices[0])
        {
            distances[1] = distance;
            indices[1] = i;
        }
    }

    //Find the furthest point in the direction of the third eigenvector of the covariance matrix
    for (uint32_t i = 0; i < vertices.size(); ++i)
    {
        float const distance = glm::abs(glm::dot(basis[2], vertices[i]));

        if (distance > distances[2] && i != indices[0] && i != indices[1])
        {
            distances[2] = distance;
            indices[2] = i;
        }
    }

    //Find the furthest point in the direction of the normal of the hyperplane formed by the extreme points
    epona::HyperPlane const hyperPlane(vertices[indices[0]], vertices[indices[1]], vertices[indices[2]]);
    for (uint32_t i = 0; i < vertices.size(); ++i)
    {
        float const distance = hyperPlane.Distance(vertices[i]);

        if (distance > distances[3] && indices[0] != i && indices[1] != i && indices[2] != i)
        {
            distances[3] = distance;
            indices[3] = i;
        }
    }

    glm::vec3 mean(0);
    for (uint32_t index : indices)
        mean += vertices[index];
    mean /= 4;

    assert( !epona::OneLine(vertices[indices[0]], vertices[indices[1]], vertices[indices[2]]) );
    assert( !epona::OneLine(vertices[indices[0]], vertices[indices[1]], vertices[indices[3]]) );
    assert( !epona::OneLine(vertices[indices[0]], vertices[indices[2]], vertices[indices[3]]) );
    assert( !epona::OneLine(vertices[indices[1]], vertices[indices[2]], vertices[indices[3]]) );

    epona::HalfEdgeDataStructure heds;
    heds.MakeFace(indices[0], indices[1], indices[2], 0 );
    heds.MakeFace(indices[0], indices[1], indices[3], 1 );
    heds.MakeFace(indices[0], indices[2], indices[3], 2 );
    heds.MakeFace(indices[1], indices[2], indices[3], 3 );

    //Create initial convex hull tetrahedron
    return {{ indices[0], indices[1], indices[2], indices[3] },
        {   glm::u32vec3{indices[0], indices[1], indices[2]},
            glm::u32vec3{indices[0], indices[1], indices[3]},
            glm::u32vec3{indices[0], indices[2], indices[3]},
            glm::u32vec3{indices[1], indices[2], indices[3]} },
        {
            epona::HyperPlane{vertices[indices[0]], vertices[indices[1]], vertices[indices[2]], &mean },
            epona::HyperPlane{vertices[indices[0]], vertices[indices[1]], vertices[indices[3]], &mean },
            epona::HyperPlane{vertices[indices[0]], vertices[indices[2]], vertices[indices[3]], &mean },
            epona::HyperPlane{vertices[indices[1]], vertices[indices[2]], vertices[indices[3]], &mean },
        },
        std::move(heds)
    };
}

} // namespace ::

namespace epona
{

/**
 * @brief Updates convex hull based on the vertices data
 *
 * @note vertices size must be >= 4 and nondegenerate
 * @attention The only way to update vertex data is to append some additional vertices
 *
 * @param[in, out] hull convex hull to update
 * @param[in] vertices updated input vertex data on which hull was based
 * @param[in] maxIterations maximum number of iterations
 * @param[in] distanceThreshold inlier vertex distance threshold
 */
inline bool RecalculateConvexHull(ConvexHull& hull, std::vector<glm::vec3> const& vertices, uint32_t maxIterations = 100, float distanceThreshold = 1e-3f)
{
    bool changed = false;

    std::vector<uint32_t> indices;
    indices.reserve(vertices.size());
    for (uint32_t v = 0; v < vertices.size(); ++v)
        if (std::find(hull.indices.begin(), hull.indices.end(), v) == hull.indices.end())
            indices.push_back(v);
    std::sort(hull.indices.begin(), hull.indices.end(), std::greater<uint32_t>());

    uint32_t totalOutliersCount = 0;
    ::FaceOutliers faceOutliers;
    std::vector<uint32_t> visibleFaces;
    std::vector<::Ridge> boundary;
    std::vector<uint32_t> temporaryOutliers;

    do {
        glm::vec3 mean(0);
        for (auto i : hull.indices)
            mean += vertices[i];
        mean /= hull.indices.size();

        //Find outliers and extreme vertices
        uint32_t newOutliersCount = 0;
        faceOutliers.outliers.resize(hull.faces.size());
        faceOutliers.extremeIndices.resize(hull.faces.size(), UINT32_MAX);
        faceOutliers.extremeSignedDistances.resize(hull.faces.size(), -FLT_MAX);
        for (uint32_t v : indices)
        {
            for (uint32_t f = 0; f < hull.faces.size(); ++f)
            {
                float const signedDistance = hull.planes[f].SignedDistance(vertices[v]);
                if (signedDistance > distanceThreshold) {
                    faceOutliers.outliers[f].push_back(v);
                    newOutliersCount++;
                    if (signedDistance > faceOutliers.extremeSignedDistances[f]) {
                        faceOutliers.extremeSignedDistances[f] = signedDistance;
                        faceOutliers.extremeIndices[f] = v;
                    }
                    break;
                }
            }
        }

        //Find visible faces from the first extreme outlier
        uint32_t extremeOutlierIndex = UINT32_MAX;
        visibleFaces.clear();
        visibleFaces.reserve(hull.faces.size() / 2);
        for (uint32_t f = 0; f < hull.faces.size(); ++f)
        {
            if (!faceOutliers.outliers[f].empty())
            {
                visibleFaces.push_back(f);
                std::vector<uint32_t> visitedFaces(1, f);
                visitedFaces.reserve(hull.faces.size() / 2);
                glm::vec3 const extremeVertex = vertices[faceOutliers.extremeIndices[f]];

                for (uint32_t vf = 0; vf < visibleFaces.size(); ++vf)
                {
                    auto adjFaceIt = hull.heds.GetFace(hull.faces[visibleFaces[vf]])->GetAdjacentFaceIterator();

                    for (uint8_t i = 0; i < 3; ++i, ++adjFaceIt) {
                        //NOTE: will work faster than open address map till ~ visitedFaces.size() < 100
                        if (std::find(visitedFaces.begin(), visitedFaces.end(), adjFaceIt->index) == visitedFaces.end())
                        {
                            visitedFaces.push_back(adjFaceIt->index);
                            float const signedDistance = hull.planes[adjFaceIt->index].SignedDistance(extremeVertex);
                            if (fp::IsGreater(signedDistance, 0.0f))
                                visibleFaces.push_back(adjFaceIt->index);
                        }
                    }
                }
                extremeOutlierIndex = faceOutliers.extremeIndices[f];
                break;
            }
        }
        std::sort(visibleFaces.begin(), visibleFaces.end());

        if (visibleFaces.empty())
        {
            indices.clear();
        }
        else
        {
            changed = true;
            assert(extremeOutlierIndex != UINT32_MAX);

            //Find boundary ridges
            boundary.clear();
            boundary.reserve(visibleFaces.size());

            for (uint32_t vf : visibleFaces)
            {
                auto adjFaceIt = hull.heds.GetFace(hull.faces[vf])->GetAdjacentFaceIterator();
                for (uint8_t i = 0; i < 3; ++i, ++adjFaceIt)
                {
                    if (std::find(visibleFaces.begin(), visibleFaces.end(), adjFaceIt->index) == visibleFaces.end())
                    {
                        auto halfEdge = static_cast<HalfEdgeDataStructure::HalfEdge*>(adjFaceIt);
                        ::Ridge const ridge{ static_cast<uint32_t>(halfEdge->vertexIndex), static_cast<uint32_t>(halfEdge->prev->vertexIndex) };
                        //NOTE: will work faster than open address map till ~ boundary.size() < 100
                        if (std::find(boundary.begin(), boundary.end(), ridge) == boundary.end())
                        {
                            boundary.push_back(ridge);
                        }
                    }
                }
            }
            //FixMe: Checks for degenerate case, that should be prevented by threshold check, is there a way to make it better?
            if (std::find(boundary.begin(), boundary.end(), extremeOutlierIndex) != boundary.end())
                return true;

            //Collect inliers as outliers and remove them from hull indices
            temporaryOutliers.clear();
            temporaryOutliers.reserve(visibleFaces.size() * 3 + newOutliersCount);
            for (uint32_t vf : visibleFaces) {
                auto face = hull.faces[vf];
                for (uint8_t i = 0; i < 3; ++i) {
                    //NOTE: will work faster than open address map till ~ (boundary/outliers).size() < 100
                    if (std::find(boundary.begin(), boundary.end(), face[i]) == boundary.end()
                        && std::find(temporaryOutliers.begin(), temporaryOutliers.end(), face[i]) == temporaryOutliers.end())
                    {
                        temporaryOutliers.push_back(face[i]);
                        //NOTE: will work faster than open address map till ~ hull.indieces.size() < 100
                        auto it = std::find(hull.indices.begin(), hull.indices.end(), face[i]);
                        if (it != hull.indices.end())
                            hull.indices.erase(it);
                    }
                }
            }

            //Remove visible faces and collect unassigned outliers
            for (uint32_t vf : visibleFaces)
            {
                hull.heds.RemoveFace(hull.heds.GetFace(hull.faces[vf]));
                temporaryOutliers.insert(temporaryOutliers.end(),
                    std::make_move_iterator(faceOutliers.outliers[vf].begin()),
                    std::make_move_iterator(faceOutliers.outliers[vf].end())
                );
            }
            indices.swap(temporaryOutliers);
            temporaryOutliers.clear();

            //Create new faces and assign outliers
            for (::Ridge ridge : boundary)
            {
                uint32_t faceIndex = static_cast<uint32_t>(hull.faces.size());
                if (!visibleFaces.empty())
                {
                    faceIndex = visibleFaces.back();
                    hull.faces[faceIndex] = { extremeOutlierIndex, ridge.aVertex, ridge.bVertex };
                    hull.planes[faceIndex] = { vertices[extremeOutlierIndex], vertices[ridge.aVertex], vertices[ridge.bVertex], &mean };
                    visibleFaces.pop_back();

                    faceOutliers.extremeIndices[faceIndex] = UINT32_MAX;
                    faceOutliers.extremeSignedDistances[faceIndex] = -FLT_MAX;
                    faceOutliers.outliers[faceIndex].clear();
                }
                else
                {
                    hull.faces.emplace_back(extremeOutlierIndex, ridge.aVertex, ridge.bVertex);
                    hull.planes.emplace_back(vertices[extremeOutlierIndex], vertices[ridge.aVertex], vertices[ridge.bVertex], &mean);
                }
                hull.heds.MakeFace(extremeOutlierIndex, ridge.aVertex, ridge.bVertex, faceIndex);
                //NOTE: will work faster than open address map till ~ hull.indieces.size() < 100
                if (std::find(hull.indices.begin(), hull.indices.end(), extremeOutlierIndex) == hull.indices.end())
                    hull.indices.push_back(extremeOutlierIndex);
            }

            //Restore structure
            if (!visibleFaces.empty())
            {
                std::sort(visibleFaces.begin(), visibleFaces.end());
                std::vector<glm::u32vec3> faces;
                std::vector<HyperPlane> planes;
                faces.reserve(hull.faces.size() - visibleFaces.size());
                planes.reserve(hull.faces.size() - visibleFaces.size());
                FaceOutliers tmpFaceOutliers;
                tmpFaceOutliers.extremeIndices.reserve(faceOutliers.extremeIndices.size());
                tmpFaceOutliers.extremeSignedDistances.reserve(faceOutliers.extremeSignedDistances.size());
                tmpFaceOutliers.outliers.reserve(faceOutliers.outliers.size());
                for (uint32_t i = 0, j = 0; i < hull.faces.size(); ++i)
                {
                    if (j == visibleFaces.size() || i != visibleFaces[j])
                    {
                        faces.push_back(hull.faces[i]);
                        planes.push_back(hull.planes[i]);
                        hull.heds.GetFace(hull.faces[i])->index = static_cast<uint32_t>(faces.size() - 1);
                        tmpFaceOutliers.extremeIndices.push_back(faceOutliers.extremeIndices[i]);
                        tmpFaceOutliers.extremeSignedDistances.push_back(faceOutliers.extremeSignedDistances[i]);
                        tmpFaceOutliers.outliers.push_back(std::move(faceOutliers.outliers[i]));
                    }
                    else { ++j; }
                }
                faceOutliers.extremeIndices.swap(tmpFaceOutliers.extremeIndices);
                faceOutliers.extremeSignedDistances.swap(tmpFaceOutliers.extremeSignedDistances);
                faceOutliers.outliers.swap(tmpFaceOutliers.outliers);
                hull.faces.swap(faces);
                hull.planes.swap(planes);
            }
        }

        totalOutliersCount = newOutliersCount + static_cast<uint32_t>(indices.size());
        for (auto& outliers : faceOutliers.outliers)
            totalOutliersCount += static_cast<uint32_t>(outliers.size());
    } while (totalOutliersCount && --maxIterations);

    return changed;
}

/**
 * @brief Calculates convex hull based on the vertices data using Quickhull convex hull algorithm
 *
 * @note vertices size must be >= 4 and nondegenerate
 *
 * @param[in] vertices updated input vertex data on which hull was based
 * @param[in] maxIterations maximum number of iterations
 * @param[in] distanceThreshold inlier vertex distance threshold
 * @return convex hull object
 */
inline ConvexHull CalculateConvexHull(std::vector<glm::vec3> const& vertices, uint32_t maxIterations = 100, float distanceThreshold = 1e-3f)
{
    ConvexHull hull = ::CalculateConvexHullInitialTetrahedron(vertices);

    RecalculateConvexHull(hull, vertices, maxIterations);

    return hull;
}

} // namespace epona
#endif // EPONA_CONVEX_HULL_HPP
