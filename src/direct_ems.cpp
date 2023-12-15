#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/sampler.h>
#include <nori/bsdf.h>

NORI_NAMESPACE_BEGIN

class DirectEms : public Integrator {
public:

    DirectEms(const PropertyList& props) {}
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
       
        // light emitted from intersection point
        Color3f Le = its.mesh->isEmitter() ? its.mesh->getEmitter()->eval(EmitterQueryRecord(ray.o, its.p, its.shFrame.n)) : Color3f(0.0f);

        // random light in scene
        LightBVHQueryRecord lqr(its.p, its.shFrame.n);
        const Emitter* light = scene->getRandomEmitter(lqr);
        //const Emitter* light = scene->getRandomEmitter(sampler->next1D());
        
        // radiance of a sampled point on light source
        EmitterQueryRecord q = EmitterQueryRecord(its.p);
        const Color3f radiance = light->sample(q, sampler->next2D());

        // if light is not visible return emitted light
        if (scene->rayIntersect(q.shadowRay)) return Le;

        const float cos = its.shFrame.n.dot(q.wi);
        // light source is behind (so not visible)
        if (cos <= 0) return Le;

        // bsdf contribution
        const BSDF* bsdf = its.mesh->getBSDF();
        BSDFQueryRecord bsdfQuery = BSDFQueryRecord(
            its.shFrame.toLocal(q.wi),
            its.shFrame.toLocal(-ray.d),
            ESolidAngle);
        bsdfQuery.uv = its.uv;
        bsdfQuery.p = its.p;
        Color3f scatter = bsdf->eval(bsdfQuery);

        // multiply light contriubtion by number of lights in scene to get the correct pdf
        // might overcompensate small lights ???
        //return Le + radiance * cos * scatter * scene->getLights().size();
        //cout << lqr.pdf << endl;
        return lqr.pdf > Epsilon ? Le + radiance * cos * scatter / lqr.pdf : Le;
    };

    std::string toString() const {
        return "DirectEms[]";
    }
};

NORI_REGISTER_CLASS(DirectEms, "direct_ems");
NORI_NAMESPACE_END
