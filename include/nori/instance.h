#include <nori/object.h>
#include <nori/frame.h>
#include <nori/bbox.h>
#include <nori/shape.h>
#include <nori/subscene.h>
#include <Eigen/Geometry>

NORI_NAMESPACE_BEGIN

class Instance : public Shape {
public:
    Instance(const PropertyList &propList);

    void linkSubScene(SubScene* subscene) {
        //std::cout << tfm::format("Linked subscene %d (m_subscene = %d)", subscene->getID(), m_subscene) << endl;
        if (m_subscene)
            throw NoriException("There can only be one subscene per instance!");
        m_subscene = subscene;

        m_bsdf = const_cast<BSDF *>(subscene->getMesh()->getBSDF());
        if (subscene->getMesh()->isEmitter()){
            m_emitter = const_cast<Emitter *>(subscene->getMesh()->getEmitter());
            m_emitter->setShape(this);
        } else {
            m_emitter = nullptr;
        }
        m_bbox = BoundingBox3f();
        BoundingBox3f og_bbox = m_subscene->getBoundingBox();
        for (int i = 0; i < 8; i++) {
            m_bbox.expandBy(toWorld(og_bbox.getCorner(i)));
        }
    }

    bool linked() {return m_subscene;}


    int getSubsceneId() {
        return m_subsceneID;
    }

    Point3f toWorld(const Point3f point) const {
        return m_ToWorld * point;
    }

    Vector3f toWorld(const Vector3f dir) const {
        return m_ToWorld * dir;
    }

    Normal3f toWorld(const Normal3f n) const {
        return m_ToWorld * n;
    }

    Ray3f toWorld(const Ray3f &ray) const {
        return m_ToWorld * ray;
    }

    Point3f toLocal(const Point3f point) const {
        return m_ToLocal * point;
    }

    Vector3f toLocal(const Vector3f dir) const {
        return m_ToLocal * dir;
    }

    Normal3f toLocal(const Normal3f n) const {
        return m_ToLocal * n;
    }

    Ray3f toLocal(const Ray3f &ray) const {
        return m_ToLocal * ray;
    }

    virtual BoundingBox3f getBoundingBox(uint32_t index) const override{
        return m_bbox;
    }

    virtual LightCone getLightCone() const override{
        return m_subscene->getLightCone();
    }

    virtual LightCone getLightCone(uint32_t index) const override {
        return getLightCone();
    }

    bool isEmitter() const { return m_emitter != nullptr; }

    Emitter *getEmitter() { return m_emitter; }

    const Emitter *getEmitter() const { return m_emitter; }

    const BSDF *getBSDF() const { return m_bsdf; }

    uint32_t getPrimitiveCount() const {return 1;}

    virtual EClassType getClassType() const override {return EInstance;}

    virtual bool rayIntersect(uint32_t index, const Ray3f &ray, float &u, float &v, float &t) const {
        Intersection its;
        bool found_intersection = m_subscene->rayIntersect(toLocal(ray), its);
        //bool found_intersection = m_subscene->getMesh()->rayIntersect(index, toLocal(ray), u, v, t);
        u = its.uv.x();
        v = its.uv.y();
        t = its.t;
        return found_intersection;
    }

    virtual void setHitInformation(uint32_t index, const Ray3f &ray, Intersection & its) const {
        m_subscene->rayIntersect(toLocal(ray), its);
        its.p = toWorld(its.p);
        its.geoFrame = Frame(toWorld(its.geoFrame.n));
        its.shFrame = Frame(toWorld(its.shFrame.n));
        /*m_subscene->getMesh()->setHitInformation(index, toLocal(ray), its);
        its.p = toWorld(its.p);
        its.geoFrame = Frame(toWorld(its.geoFrame.n));
        its.shFrame = Frame(toWorld(its.shFrame.n));*/
    }

    Point3f getCentroid(uint32_t index) const {
        return m_subscene->getMesh()->getCentroid(index);
    }

    void sampleSurface(ShapeQueryRecord & sRec, const Point2f & sample) const {
        sRec.ref = toLocal(sRec.ref);
        sRec.p = toLocal(sRec.p);
        sRec.n = toLocal(sRec.n);
        m_subscene->getMesh()->sampleSurface(sRec, sample);
        sRec.ref = toWorld(sRec.ref);
        sRec.p = toWorld(sRec.p);
        sRec.n = toWorld(sRec.n);
    }

    float pdfSurface(const ShapeQueryRecord & sRec) const {
        ShapeQueryRecord sRec1;
        sRec1.ref = toLocal(sRec.ref);
        sRec1.p = toLocal(sRec.p);
        sRec1.n = toLocal(sRec.n);
        return m_subscene->getMesh()->pdfSurface(sRec1);
    }

    virtual std::string toString() const override {
        return tfm::format(
            "Instance[\n"
            "   subscene id = %d\n"
            "   toWorld = %s\n"
            "]",
            m_subsceneID,
            m_ToWorld.toString()
        );
    }

protected:
    int m_subsceneID;
    SubScene *m_subscene = nullptr;
    Transform m_ToWorld;
    Transform m_ToLocal;
};

NORI_NAMESPACE_END
