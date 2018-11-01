/*
* Copyright (C) 2018 by Godlike
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/
#ifndef EPONA_HEDS_HPP
#define EPONA_HEDS_HPP

#include <Epona/HyperPlane.hpp>
#include <iterator>
#include <unordered_map>
#include <list>

namespace epona
{
/**
 * @brief Data structure dedicated to store the mesh with most of the information contained in half-edges
 */
class HalfEdgeDataStructure
{
public:
    struct HalfEdge;
    class Face;

    using face_iterator = std::list<Face>::iterator;
    using const_face_iterator = std::list<Face>::const_iterator;

    HalfEdgeDataStructure() = default;

    HalfEdgeDataStructure(HalfEdgeDataStructure const&) = delete;
    
    HalfEdgeDataStructure& operator=(HalfEdgeDataStructure const& heds) = delete;
    
    HalfEdgeDataStructure(HalfEdgeDataStructure&& heds)
    {
        m_halfEdgeList = std::move(heds.m_halfEdgeList);
        m_halfEdgePointerIteratorMap = std::move(heds.m_halfEdgePointerIteratorMap);
        m_halfEdgeVerticesIteratorMap = std::move(heds.m_halfEdgeVerticesIteratorMap);

        m_facesList = std::move(heds.m_facesList);
        m_faceIteratorMap = std::move(heds.m_faceIteratorMap);
        m_faceVerticesIteratorMap = std::move(heds.m_faceVerticesIteratorMap);
    }

    HalfEdgeDataStructure& operator=(HalfEdgeDataStructure&& heds)
    {
        m_halfEdgeList = std::move(heds.m_halfEdgeList);
        m_halfEdgePointerIteratorMap = std::move(heds.m_halfEdgePointerIteratorMap);
        m_halfEdgeVerticesIteratorMap = std::move(heds.m_halfEdgeVerticesIteratorMap);

        m_facesList = std::move(heds.m_facesList);
        m_faceIteratorMap = std::move(heds.m_faceIteratorMap);
        m_faceVerticesIteratorMap = std::move(heds.m_faceVerticesIteratorMap);

		return *this;
    }

    /**
     * @brief HEDS Face data container
     */
    class Face
    {
    public:
        template <typename HalfEdgeType>
        class HalfEdgeCirculator;
        template <typename FaceType>
        class AdjacentFaceCirculator;

        using edge_iterator = HalfEdgeCirculator<HalfEdge>;
        using const_edge_iterator = HalfEdgeCirculator<HalfEdge const>;
        using face_iterator = AdjacentFaceCirculator<Face>;
        using const_face_iterator = AdjacentFaceCirculator<Face const>;

        /**
         * @brief Face half-edges circular iterator
         *
         * Iterator of a circular data structure,
         * does not have an end state. If the data structure is in
         * valid state always dereferencable.
         * Under the hood iterates along the inner half-edges of the face.
         * @tparam HalfEdgeType half-edge type
         */
        template < typename HalfEdgeType = HalfEdge >
        class HalfEdgeCirculator : public std::iterator<std::bidirectional_iterator_tag, HalfEdgeType>
        {
        public:
            /**
             * @brief Constructs a circular iterator from a given half-edge
             * @param halfEdge
             */
            explicit HalfEdgeCirculator(HalfEdgeType& halfEdge)
                : m_currentHalfEdge(&halfEdge)
            {
            }

            /**
             * @brief Returns a reference to the current half-edge
             * @return reference to the current half-edge
             */
            HalfEdgeType& operator*() const
            {
                return *m_currentHalfEdge;
            }

            /**
             * @brief Returns a pointer to the current half-edge
             * @return pointer to the current half-edge
             */
            HalfEdgeType* operator->() const
            {
                return m_currentHalfEdge;
            }

            /**
             * @brief Increments an iterator and returns its reference
             * @return reference to the incremented iterator
             */
            HalfEdgeCirculator& operator++()
            {
                m_currentHalfEdge = m_currentHalfEdge->next;
                return *this;
            }

            /**
             * @brief Increments an iterator and returns the iterator before the increment
             * @return iterator before the increment
             */
            HalfEdgeCirculator operator++(int)
            {
                HalfEdgeCirculator result(*this);
                operator++();
                return result;
            }

            /**
             * @brief Decrements an iterator and returns its reference
             * @return reference to the decremented iterator
             */
            HalfEdgeCirculator& operator--()
            {
                m_currentHalfEdge = m_currentHalfEdge->prev;
                return *this;
            }

            /**
             * @brief Decrements an iterator and returns the iterator before the decrement
             * @return iterator before the decrement
             */
            HalfEdgeCirculator operator--(int)
            {
                HalfEdgeCirculator result(*this);
                operator--();
                return result;
            }

            /**
             * @brief Performs a memberwise comparison of the iterators and returns true if they are equal
             * @param[in] other right hand side iterator reference
             * @return true if iterators are equal
             */
            bool operator==(HalfEdgeCirculator const& other) const
            {
                return m_currentHalfEdge == other.m_currentHalfEdge;
            }

            /**
             * @brief Performs a memberwise comparison of the iterators and returns true if they are not equal
             * @param[in] other right hand side iterator reference
             * @return false if iterators are equal
             */
            bool operator!=(HalfEdgeCirculator const& other) const
            {
                return !operator==(other);
            }

            /**
            * @brief Swaps two iterators content
            * @param[in] lhs left hand side iterator
            * @param[in] rhs right hand side iterator
            */
            friend void swap(HalfEdgeCirculator& lhs, HalfEdgeCirculator& rhs) noexcept
            {
                std::swap(lhs.m_currentHalfEdge, rhs.m_currentHalfEdge);
            }

            /**
            * @brief Constructs constant iterator from the current iterator and returns it
            */
            operator HalfEdgeCirculator<HalfEdgeType const>() const
            {
                return HalfEdgeCirculator(*this);
            }

        private:
            HalfEdgeType* m_currentHalfEdge;
        };

        /**
         * @brief Adjacent faces iterator
         *
         * Iterator of a circular data structure,
         * does not have an end state. If the data structure is in
         * valid state always dereferencable.
         * Under the hood iterates along the inner half-edges of the face.
         * Access to adjacent faces is performed via twin half-edge.
         * @tparam FaceType face type
         */
        template < typename FaceType = Face>
        class AdjacentFaceCirculator : public std::iterator<std::bidirectional_iterator_tag, FaceType>
        {
        public:
            /**
             * @brief Constructs an adjacent face iterator from the given half-edge
             * @param[in] halfEdge a half-edge object
             */
            explicit AdjacentFaceCirculator(HalfEdge& halfEdge)
                : m_currentHalfEdge(&halfEdge)
            {
            }

            /**
             * @brief Returns a reference to the current adjacent face
             * @return a reference to the current adjacent face
             */
            FaceType& operator*() const
            {
                return *m_currentHalfEdge->twin->face;
            }

            /**
             * @brief Returns a pointer to the current adjacent face
             * @return a pointer to the current adjacent face
             */
            FaceType* operator->() const
            {
                return m_currentHalfEdge->twin->face;
            }

            /**
             * @brief Increments an iterator to the next adjacent face
             * @return incremented iterator reference
             */
            AdjacentFaceCirculator& operator++()
            {
                m_currentHalfEdge = m_currentHalfEdge->next;
                return *this;
            }

            /**
             * @brief Increments an iterator to the next adjacent face
             * @return iterator before increment
             */
            AdjacentFaceCirculator operator++(int)
            {
                AdjacentFaceCirculator result(*this);
                operator++();
                return result;
            }

            /**
             * @brief Decrements an iterator to the previous adjacent face
             * @return decremented iterator
             */
            AdjacentFaceCirculator& operator--()
            {
                m_currentHalfEdge = m_currentHalfEdge->prev;
                return *this;
            }

            /**
             * @brief Decrements an iterator to the previous adjacent face
             * @return iterator before decrement
             */
            AdjacentFaceCirculator operator--(int)
            {
                AdjacentFaceCirculator result(*this);
                operator++();
                return result;
            }

            /**
             * @brief Performs a memberwise comparison and returns true if iterators are equal
             * @param[in] other reference to the right hand side iterator
             * @return result of the comparison
             */
            bool operator==(AdjacentFaceCirculator const& other) const
            {
                return m_currentHalfEdge == other.m_currentHalfEdge;
            }

            /**
             * @brief Performs a memberwise comparison and returns true if iterators are different
             * @param[in] other reference to the right hand side iterator
             * @return result of the comparison
             */
            bool operator!=(AdjacentFaceCirculator const& other) const
            {
                return !operator==(other);
            }

            /**
             * @brief Swaps two iterators content
             * @param[in] lhs left hand side iterator
             * @param[in] rhs right hand side iterator
             */
            friend void swap(AdjacentFaceCirculator& lhs, AdjacentFaceCirculator& rhs) noexcept
            {
                std::swap(lhs.m_currentHalfEdge, lhs.m_currentHalfEdge);
            }

            /**
             * @brief Constructs constant iterator from the current iterator and returns it
             */
            operator AdjacentFaceCirculator<FaceType const>() const
            {
                return AdjacentFaceCirculator<FaceType const>(*this);
            }
			
			operator HalfEdge*() const
			{
				return m_currentHalfEdge;
			}

        private:
            HalfEdge* m_currentHalfEdge;
        };

        /**
         * @brief Constructs a face from a given half-edge
         * @param[in] halfEdge face half-edge
         */
        explicit Face(HalfEdge& halfEdge);

        /**
         * @brief Returns half-edge iterator
         * @return half-edge iterator of current face
         */
        edge_iterator GetHalfEdgeIterator();

        /**
        * @brief Returns half-edge constant iterator
        * @return half-edge constant iterator
        */
        const_edge_iterator GetHalfEdgeIterator() const;

        /**
         * @brief Returns iterator of adjacent faces
         * @return adjacent face iterator
         */
        face_iterator GetAdjacentFaceIterator();

        /**
        * @brief Returns iterator to constant adjacent faces
        * @return adjacent face constant iterator
        */
        const_face_iterator GetAdjacentFaceIterator() const;

		//! Stores face hyperplane
		HyperPlane hyperPlane;

		//! Stores face index
		uint32_t index;

    private:
        HalfEdge* m_halfEdge;
    };

    /**
     * @brief Object that can be used as a key for associative containers
     */
    struct FaceVertices
    {
        uint64_t a;
        uint64_t b;
        uint64_t c;

        /* Hasher for the key object */
        struct Hasher
        {
            /**
            * @brief Calculates a hash sum over the given FaceVertices object
            * @param face key for the hash calculation
            * @return hash sum value
            */
            size_t operator()(FaceVertices const& face) const;
        };

        bool operator==(FaceVertices const& other) const;
    };

    /**
     * @brief Data structure that describes one side of the edge
     */
    struct HalfEdge
    {
        HalfEdge* next;
        HalfEdge* prev;
        HalfEdge* twin;
        Face* face;
        uint64_t vertexIndex;
    };

	/**
	 * @brief Inserts face into the current data structure
	 * @param[in] a vertex index
	 * @param[in] b vertex index
	 * @param[in] c vertex index
	 */
	void MakeFace(uint64_t a, uint64_t b, uint64_t c)
	{
		MakeFace(a, b, c, {}, 0);
	}

    /**
     * @brief Inserts face into the current data structure
     * @param a vertex index
     * @param b vertex index
     * @param c vertex index
	 * @param hp hyperplane containing input vertices
	 * @param index outside face index
     */
    void MakeFace(uint64_t a, uint64_t b, uint64_t c, HyperPlane hp, uint32_t index);

	/**
	 * @brief Inserts face into the current data structure
	 * @type	T		array-like type with overaloded operator[]
	 * @param	index	face indices array of size 3
	 */
	template < typename T >
	decltype(auto) GetFace(T index)
	{
		return GetFace(index[0], index[1], index[2]);
	}

	/**
	 * @brief Inserts face into the current data structure
	 * @type	T		array-like type with overaloded operator[]
	 * @param	index	face indices array of size 3
	 */
	template < typename T >
	decltype(auto) GetFace(T index) const
	{
		return GetFace(index[0], index[1], index[2]);
	}

    /**
     * @brief Returns a face iterator
     * @param[in] a vertex index
     * @param[in] b vertex index
     * @param[in] c vertex index
     * @return face iterator
     */
    face_iterator GetFace(uint64_t a, uint64_t b, uint64_t c);

    /**
    * @brief Returns a constant face iterator
    * @param[in] a vertex index
    * @param[in] b vertex index
    * @param[in] c vertex index
    * @return face constant iterator
    */
    const_face_iterator GetFace(uint64_t a, uint64_t b, uint64_t c) const;

    /**
     * @brief Returns end face iterator
     * @return end face constant iterator
     */
    const_face_iterator GetFaceEnd() const;

    /**
     * @brief Removes face from the current data structure
     * @param[in] a vertex index
     * @param[in] b vertex index
     * @param[in] c vertex index
     */
    void RemoveFace(uint64_t a, uint64_t b, uint64_t c);

    /**
     * @brief Removes a face from the current data structure
     * @param[in] faceIterator iterator pointing to the element to be removed
     */
    void RemoveFace(face_iterator faceIterator);

private:
    /**
     * @brief Object that can be used as a key for associative containers
     */
    struct HalfEdgeVertices
    {
        uint64_t vertexIndexFrom;
        uint64_t vertexIndexTo;

        /* Hasher for the key object */
        struct Hasher
        {
            /**
            * @brief Calculates a hash sum over the given HalfEdgeVertices object
            * @param edge key for the hash calculation
            * @return hash sum value
            */
            size_t operator()(HalfEdgeVertices const& edge) const;
        };

        bool operator==(HalfEdgeVertices const& other) const;
    };

    std::list<HalfEdge> m_halfEdgeList;
    std::unordered_map<
        HalfEdge const*, std::list<HalfEdge>::iterator
    > m_halfEdgePointerIteratorMap;
    std::unordered_map<
        HalfEdgeVertices, std::list<HalfEdge>::iterator, HalfEdgeVertices::Hasher
    > m_halfEdgeVerticesIteratorMap;

    std::list<Face> m_facesList;
    std::unordered_map<
        Face const*, std::list<Face>::iterator
    > m_faceIteratorMap;
    std::unordered_map<
        FaceVertices, std::list<Face>::iterator, FaceVertices::Hasher
    > m_faceVerticesIteratorMap;

    /**
     * @brief Initializes given half-edge
     * @param[in] he half-edge iterator
     * @param[in] next next half-edge pointer
     * @param[in] prev previous half-edge pointer
     * @param[in] face half-edge face pointer
     * @param[in] vertexIndexFrom start edge vertex index
     * @param[in] vertexIndexTo end edge vertex index
     */
    void IntializeHalfEdge(
        std::list<HalfEdge>::iterator he,
        HalfEdge* next, HalfEdge* prev,
        Face* face,
        uint64_t vertexIndexFrom,
        uint64_t vertexIndexTo
    );
};

template < typename T >
std::tuple<uint64_t, uint64_t> GetRidge(HalfEdgeDataStructure::face_iterator it)
{


	return {};
}
} // namespace epona
#endif // EPONA_HEDS_HPP
