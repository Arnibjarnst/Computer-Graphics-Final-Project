#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/sampler.h>
#include <nori/bsdf.h>
#include <nori/warp.h>
#include <nori/camera.h>

NORI_NAMESPACE_BEGIN

class VolPathMisSimple : public Integrator {
public:

    VolPathMisSimple(const PropertyList& props) {}
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
        const long START_ROULETTE = 5;

        Color3f L = 0.0f;
        Color3f t = 1.0f;
        Ray3f ray(ray0);
        Intersection its;
        float bsdfPdf = -1.0; // this means that we have discrete sampling (so infinty)

        for (long i = 0; ; i++) {
            // Find ray intersection
            if (!scene->rayIntersect(ray, its) && ray.medium == nullptr) {
                break;
            }

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
                if (its.mesh->isEmitter()) {
                    EmitterQueryRecord emitterEval(ray.o, its.p, its.shFrame.n);
                    const Emitter* emitter = its.mesh->getEmitter();
                    if (bsdfPdf < 0) L += t * emitter->eval(emitterEval); // discrete sampling so wMat == 1
                    else {
                        const float wMat = bsdfPdf / (bsdfPdf + emitter->pdf(emitterEval) / scene->getLights().size());
                        L += wMat * t * emitter->eval(emitterEval);
                    }
                }

                if (i > START_ROULETTE) {
                    float successProbability = std::min(0.99f, t.maxCoeff());
                    if (sampler->next1D() > successProbability) break;
                    t /= successProbability;
                }

                const BSDF* bsdf = its.mesh->getBSDF();
                const Vector3f wi = its.shFrame.toLocal(-ray.d);
                BSDFQueryRecord bsdfQuery = BSDFQueryRecord(wi);
                bsdfQuery.its = &its;

                Color3f bsdfValue = bsdf->sample(bsdfQuery, sampler->next2D());

                if (bsdfQuery.measure == ESolidAngle) { // If measure is discrete then wEm == 0

                    // random light in scene
                    const Emitter* light = scene->getRandomEmitter(sampler->next1D());

                    // radiance of a sampled point on light source
                    EmitterQueryRecord lightQuery = EmitterQueryRecord(its.p);
                    const Color3f radiance = light->sample(lightQuery, sampler->next2D());

                    if (lightQuery.pdf > 0 && !scene->rayIntersect(lightQuery.shadowRay)) {
                        Vector3f wo = its.shFrame.toLocal(lightQuery.wi);
                        BSDFQueryRecord bsdfEvalQuery = BSDFQueryRecord(
                            wi,
                            wo,
                            ESolidAngle);
                        bsdfEvalQuery.its = &its;
                        Color3f bsdfValueToLight = bsdf->eval(bsdfEvalQuery);

                        const float wEm = lightQuery.pdf / (lightQuery.pdf / scene->getLights().size() + bsdf->pdf(bsdfEvalQuery));

                        Color3f transmittance = ray.medium ? ray.medium->tr(lightQuery.shadowRay.maxt) : 1.0f;

                        L += wEm * t * std::abs(wo.z()) * bsdfValueToLight * radiance * transmittance;
                    }
                    bsdfPdf = bsdf->pdf(bsdfQuery);
                }
                else {
                    bsdfPdf = -1.0f; // infinity
                }

                ray.o = its.p;
                ray.d = its.shFrame.toWorld(bsdfQuery.wo);
                ray.update();
                if (its.mesh->getInterior() || its.mesh->getExterior()) {
                    ray.medium = its.shFrame.n.dot(ray.d) > 0 ? its.mesh->getExterior() : its.mesh->getInterior();
                }

                t *= bsdfValue;
            }
        }

        return L;
    };

    std::string toString() const {
        return "VolPathMisSimple[]";
    }
};

NORI_REGISTER_CLASS(VolPathMisSimple, "vol_path_mis_simple");
NORI_NAMESPACE_END