#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/sampler.h>
#include <nori/bsdf.h>
#include <nori/warp.h>
#include <nori/camera.h>
#include <nori/medium.h>

NORI_NAMESPACE_BEGIN

class VolPathMats : public Integrator {
public:

    VolPathMats(const PropertyList& props) {}
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
    virtual Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray0) const override {
        const long START_ROULETTE = -1;

        Color3f L = 0.0f;
        Color3f t = 1.0f;
        Ray3f ray(ray0);
        Intersection its;

        for (long i = 0; ; i++) {

            if (!scene->rayIntersect(ray, its) && ray.medium == nullptr) break;

            MediumQueryRecord mRec(ray.d, its.t); // what is its.t when no intersection is found?
            if (ray.medium) t *= ray.medium->sample(mRec, sampler);
            else mRec.t = its.t;

            if (mRec.t < mRec.maxT) {
                if (i > START_ROULETTE) {
                    float successProbability = std::min(0.99f, t.maxCoeff());
                    if (sampler->next1D() > successProbability) break;
                    t /= successProbability;
                }
                ray.o = ray(mRec.t);
                ray.d = mRec.wo;
                ray.update();
            }
            else {
                // light emitted from intersection point
                if (its.mesh->isEmitter())
                    L += t * its.mesh->getEmitter()->eval(EmitterQueryRecord(ray.o, its.p, its.shFrame.n));

                if (i > START_ROULETTE) {
                    float successProbability = std::min(0.99f, t.maxCoeff());
                    if (sampler->next1D() > successProbability) break;
                    t /= successProbability;
                }

                const BSDF* bsdf = its.mesh->getBSDF();
                BSDFQueryRecord bsdfQuery = BSDFQueryRecord(its.shFrame.toLocal(-ray.d));
                bsdfQuery.its = &its;

                t *= bsdf->sample(bsdfQuery, sampler->next2D());

                ray.o = its.p;
                ray.d = its.shFrame.toWorld(bsdfQuery.wo);
                ray.update();
                if (its.mesh->getInterior() || its.mesh->getExterior())
                    ray.medium = its.shFrame.n.dot(ray.d) > 0 ? its.mesh->getExterior() : its.mesh->getInterior();
            }
        }

        return L;
    };

    std::string toString() const {
        return "VolPathMats[]";
    }
};

NORI_REGISTER_CLASS(VolPathMats, "vol_path_mats");
NORI_NAMESPACE_END
