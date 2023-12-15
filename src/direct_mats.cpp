#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/sampler.h>
#include <nori/bsdf.h>

NORI_NAMESPACE_BEGIN

class DirectMats : public Integrator {
public:

    DirectMats(const PropertyList& props) {}
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

        const BSDF* brdf = its.mesh->getBSDF();
        BSDFQueryRecord bsdfQuery = BSDFQueryRecord(its.shFrame.toLocal(-ray.d));
        bsdfQuery.its = &its;

        Color3f brdfValue = brdf->sample(bsdfQuery, sampler->next2D());

        Ray3f ray2(its.p, its.shFrame.toWorld(bsdfQuery.wo));
        Intersection its2;
        if (!scene->rayIntersect(ray2, its2) || !its2.mesh->isEmitter())
            return Le;

        EmitterQueryRecord eq(its.p, its2.p, its2.shFrame.n);
        Color3f radiance = its2.mesh->getEmitter()->eval(eq);

        return Le + brdfValue * radiance;
    };

    std::string toString() const {
        return "DirectMats[]";
    }
};

NORI_REGISTER_CLASS(DirectMats, "direct_mats");
NORI_NAMESPACE_END
