#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/sampler.h>
#include <nori/bsdf.h>

NORI_NAMESPACE_BEGIN

class Direct_ems : public Integrator {
public:

    Direct_ems(const PropertyList& props) {}
    /**
     * \brief Sample the incident radiance along a ray
     *
     * \param scene
     *    A pointer to the underlying scene
     * \param sampler
     *    A pointer to a sample generator
     * \param ray
     *    The ray in question
     * \return
     *    A (usually) unbiased estimate of the radiance in this direction
     */
    virtual Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const override {
        Intersection its;
        if (!scene->rayIntersect(ray, its))
            return Color3f(0.0f);
       

        const Emitter* light = scene->getRandomEmitter(sampler->next1D());
        
        EmitterQueryRecord q = EmitterQueryRecord();
        q.ref = its.p;

        const Color3f radiance = light->sample(q, sampler->next2D());

        if (scene->rayIntersect(q.shadowRay)) return Color3f(0.0f);

        const float cos = its.shFrame.n.dot(q.wi);

        if (cos <= 0) return Color3f(0.0f);

        const BSDF* bsdf = its.mesh->getBSDF();
        BSDFQueryRecord bsdfQuery = BSDFQueryRecord(
            its.shFrame.toLocal(q.wi),
            its.shFrame.toLocal(-ray.d),
            ESolidAngle);
        bsdfQuery.uv = its.uv;
        bsdfQuery.p = its.p;
        Color3f color = bsdf->eval(bsdfQuery);

        return radiance * cos * color * scene->getLights().size();
    };

    std::string toString() const {
        return "Direct_ems[]";
    }
};

NORI_REGISTER_CLASS(Direct_ems, "direct_ems");
NORI_NAMESPACE_END
