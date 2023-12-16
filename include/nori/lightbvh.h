/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob, Romain Prévost

    Nori is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Nori is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#if !defined(__NORI_LIGHTBVH_H)
#define __NORI_LIGHTBVH_H

#include <nori/emitter.h>

NORI_NAMESPACE_BEGIN

struct LightBVHQueryRecord {
    //// Shading point
    Point3f p;
    //// Normal at shading point
    Normal3f n;
    //// Selected emitter
    float pdf;

    LightBVHQueryRecord() {}

    LightBVHQueryRecord(Point3f p, Normal3f n) : p(p), n(n) {}

};

/**
 * \brief Bounding Volume Hierarchy for fast ray intersection queries
 *
 * This class builds a Bounding Volume Hierarchy (BVH) using a greedy
 * divide and conquer build strategy, which locally maximizes a criterion
 * known as the Surface Area Heuristic (SAH) to obtain a tree that is
 * particularly well-suited for ray intersection queries.
 *
 * Construction of a BVH is generally slow; the implementation here runs
 * in parallel to accelerate this process much as possible. For details
 * on how this works, refer to the paper
 *
 * "Fast and Parallel Construction of SAH-based Bounding Volume Hierarchies"
 * by Ingo Wald (Proc. IEEE/EG Symposium on Interactive Ray Tracing, 2007)
 *
 * \author Wenzel Jakob
 */


class LightBVH {
    friend class LightBVHBuildTask;
public:

    /// Create a new and empty BVH
    LightBVH() { m_shapeOffset.push_back(0u); }

    /// Release all resources
    virtual ~LightBVH() { clear(); };

    /// Release all resources
    void clear();

    /**
     * \brief Register a shape for inclusion in the BVH.
     *
     * This function can only be used before \ref build() is called
     */
    void addEmitter(Emitter *em);

    /// Build the BVH
    void build();

    /// Traverse the tree to pick an emitter, returns the corresponding pdf
    const Emitter *sample(LightBVHQueryRecord &lRec, Sampler *sampler) const;

    /// Return the total number of shapes registered with the BVH
    uint32_t getEmitterCount() const { return (uint32_t) m_emitters.size(); }

    /// Return the total number of internally represented primitives
    uint32_t getPrimitiveCount() const { return m_shapeOffset.back(); }

    /// Return one of the registered shapes
    //Emitter *getEmitter(uint32_t idx) { return m_emitters[idx]; }
    
    /// Return one of the registered shapes (const version)
    const Emitter *getEmitter(uint32_t idx) const { return m_emitters[idx]; }

    //// Return an axis-aligned bounding box containing the entire tree
    const BoundingBox3f &getBoundingBox() const {
        return m_bbox;
    }

    //// Return the light cone of the entire tree
    const LightCone getLightCone() const {
        return m_lightcone;
    }

protected:
    /**
     * \brief Compute the shape and primitive indices corresponding to
     * a primitive index used by the underlying generic BVH implementation. 
     */
    uint32_t findShape(uint32_t &idx) const {
        auto it = std::lower_bound(m_shapeOffset.begin(), m_shapeOffset.end(), idx+1) - 1;
        idx -= *it;
        return (uint32_t) (it - m_shapeOffset.begin());
    }

    //// Return an axis-aligned bounding box containing the given primitive
    BoundingBox3f getBoundingBox(uint32_t index) const {
        uint32_t emIdx = findShape(index);
        return m_emitters[emIdx]->getBoundingBox();
    }
    
    //// Return the centroid of the given primitive
    Point3f getCentroid(uint32_t index) const {
        uint32_t shapeIdx = findShape(index);
        return m_emitters[shapeIdx]->getBoundingBox().getCenter();
    }

    //// Return the lightcone of the given primitive
    struct LightCone getLightCone(uint32_t index) const {
        uint32_t shapeIdx = findShape(index);
        return m_emitters[shapeIdx]->getLightCone();
    }

    float getPower(uint32_t index) const {
        uint32_t shapeIdx = findShape(index);
        return m_emitters[shapeIdx]->getPower();
    }

    /// Compute internal tree statistics
    std::pair<float, uint32_t> statistics(uint32_t index = 0) const;

    /* LightBVH node in ?? bytes */
    struct LightBVHNode {
        union {
            struct {
                unsigned flag : 1;
                uint32_t size : 31;
                uint32_t start;
            } leaf;

            struct {
                unsigned flag : 1;
                uint32_t axis : 31;
                uint32_t rightChild;
            } inner;

            uint64_t data;
        };
        BoundingBox3f bbox;
        struct LightCone cone;
        float power;

        bool isLeaf() const {
            return leaf.flag == 1;
        }

        bool isInner() const {
            return leaf.flag == 0;
        }

        bool isUnused() const {
            return data == 0;
        }

        uint32_t start() const {
            return leaf.start;
        }

        uint32_t end() const {
            return leaf.start + leaf.size;
        }

        float getOrientationCost() {return cone.getOrientationCost();}

        float getImportance(Point3f p, Normal3f n) const {
            return 1 / (bbox.getCenter() - p).squaredNorm();
            /*
            if (bbox.contains(p)) return power;
            Vector3f pc = (bbox.getCenter() - p).normalized();
            float dSquared = (bbox.getCenter() - p).squaredNorm();
            float theta_i = std::acos(n.dot(pc));
            float theta_u = 0.f;
            for (int i = 0; i < 8; i++) {
                Point3f corner = bbox.getCorner(i);
                float theta_toCorner = std::acos(pc.dot((corner - p).normalized()));
                theta_u = std::max(theta_u, theta_toCorner);
            }
            float theta = std::acos(-pc.dot(cone.axis));
            float thetaP = std::max(theta - cone.theta_o - theta_u, 0.f);
            float thetaP_i = std::max(theta_i - theta_u, 0.f);
            return thetaP < cone.theta_e ? (cos(thetaP_i)) * power / dSquared * cos(thetaP) : 0.f;
            */
        }
    };

private:
    std::vector<Emitter *> m_emitters;       //< List of meshes registered with the BVH
    std::vector<uint32_t> m_shapeOffset; //< Index of the first triangle for each shape
    std::vector<LightBVHNode> m_nodes;   //< BVH nodes
    std::vector<uint32_t> m_indices;     //< Index references by BVH nodes
    BoundingBox3f m_bbox;                //< Bounding box of the entire BVH
    LightCone m_lightcone;               //< LightCone of the entire BVH
};

NORI_NAMESPACE_END

#endif /* __NORI_BVH_H */
