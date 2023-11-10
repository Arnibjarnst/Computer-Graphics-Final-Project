#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/sampler.h>
#include <nori/bsdf.h>

NORI_NAMESPACE_BEGIN

class PathMats : public Integrator {
public:

    PathMats(const PropertyList& props) {}
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
        Color3f L = 0.0f;
        Color3f t = 1.0f;
        Ray3f recursive_ray(ray);
        Intersection its;
        while (scene->rayIntersect(recursive_ray, its)) {

            // light emitted from intersection point
            if (its.mesh->isEmitter())
                L += t * its.mesh->getEmitter()->eval(EmitterQueryRecord(recursive_ray.o, its.p, its.shFrame.n));

            float successProbability = std::min(0.99f, t.maxCoeff());
            if (sampler->next1D() > successProbability) break;
            t /= successProbability;

            const BSDF* bsdf = its.mesh->getBSDF();
            BSDFQueryRecord bsdfQuery = BSDFQueryRecord(its.shFrame.toLocal(-recursive_ray.d));
            bsdfQuery.uv = its.uv;
            bsdfQuery.p = its.p;

            t *= bsdf->sample(bsdfQuery, sampler->next2D());

            recursive_ray = Ray3f(its.p, its.shFrame.toWorld(bsdfQuery.wo));
        }

        return L;
    };

    std::string toString() const {
        return "PathMats[]";
    }
};

NORI_REGISTER_CLASS(PathMats, "path_mats");
NORI_NAMESPACE_END
