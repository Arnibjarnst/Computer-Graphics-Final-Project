#if !defined(__NORI_SUBSCENE_H)
#define __NORI_SUBSCENE_H

#include <nori/object.h>
#include <nori/shape.h>
#include <nori/bvh.h>
#include <nori/lightbvh.h>

NORI_NAMESPACE_BEGIN

class SubScene : public NoriObject {
public:
    SubScene(const PropertyList &);

    virtual ~SubScene();

    virtual void addChild(NoriObject *obj);

    const Shape *getMesh() const {return m_mesh;}

    const bool isEmitter() {return m_mesh->isEmitter();}

    int getID() {return m_id;}

    virtual EClassType getClassType() const override {return ESubScene;}

    virtual std::string toString() const override;

    bool rayIntersect(const Ray3f &ray, Intersection &its) const {
        return m_bvh->rayIntersect(ray, its, false);
    }

    bool rayIntersect(const Ray3f &ray) const {
        Intersection its; /* Unused */
        return m_bvh->rayIntersect(ray, its, true);
    }

    const BoundingBox3f &getBoundingBox() const {
        return m_bvh->getBoundingBox();
    }

    const LightCone getLightCone() {
        return m_lbvh->getLightCone();
    }

protected:
    int m_id;
    Shape *m_mesh = nullptr;
    BVH *m_bvh = nullptr;
    LightBVH *m_lbvh = nullptr;
};

NORI_NAMESPACE_END

#endif /* __NORI_SHAPE_H */