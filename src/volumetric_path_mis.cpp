#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/sampler.h>
#include <nori/bsdf.h>
#include <nori/warp.h>
#include <nori/camera.h>

NORI_NAMESPACE_BEGIN

class VolPathMis : public Integrator {
public:

    VolPathMis(const PropertyList& props) {}
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
        const long START_ROULETTE = 5;

        Color3f L = 0.0f;
        Color3f t = 1.0f;
        Ray3f recursive_ray(ray);
        Intersection its;
        const Medium* medium = scene->getCamera()->getMedium();
        float bsdfPdf = -1.0; // this means that we have discrete sampling (so infinty)

        for (long i = 0; ; i++) {
            // Find ray intersection
            if (!scene->rayIntersect(recursive_ray, its) && medium == nullptr) {
                break;
            }

            float dt = medium != nullptr ? medium->fp(sampler->next1D()) : its.t;

            if (dt < its.t) {
                if (i > START_ROULETTE) {
                    float successProbability = std::min(0.99f, t.maxCoeff());
                    if (sampler->next1D() > successProbability) break;
                    t /= successProbability;
                }

                MediumQueryRecord mRec(recursive_ray.d);
                t *= medium->sample(mRec, sampler->next2D());
                recursive_ray = Ray3f(recursive_ray.o + dt * mRec.wi, mRec.wo);
            }
            else {
                // light emitted from intersection point
                if (its.mesh->isEmitter()) {
                    EmitterQueryRecord emitterEval(recursive_ray.o, its.p, its.shFrame.n);
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
                const Vector3f wi = its.shFrame.toLocal(-recursive_ray.d);
                BSDFQueryRecord bsdfQuery = BSDFQueryRecord(wi);
                bsdfQuery.uv = its.uv;
                bsdfQuery.p = its.p;

                Color3f bsdfValue = bsdf->sample(bsdfQuery, sampler->next2D());

                if (bsdfQuery.measure == ESolidAngle) { // If measure is discrete then wEm == 0

                    // random light in scene
                    const Emitter* light = scene->getRandomEmitter(sampler->next1D());

                    // radiance of a sampled point on light source
                    EmitterQueryRecord lightQuery = EmitterQueryRecord(its.p);
                    const Color3f radiance = light->sample(lightQuery, sampler->next2D());

                    // if light is visible
                    if (lightQuery.pdf > 0 && !scene->rayIntersect(lightQuery.shadowRay)) {
                        Vector3f wo = its.shFrame.toLocal(lightQuery.wi);
                        BSDFQueryRecord bsdfEvalQuery = BSDFQueryRecord(
                            wi,
                            wo,
                            ESolidAngle);
                        bsdfEvalQuery.uv = its.uv;
                        bsdfEvalQuery.p = its.p;
                        Color3f bsdfValueToLight = bsdf->eval(bsdfEvalQuery);

                        const float wEm = lightQuery.pdf / (lightQuery.pdf / scene->getLights().size() + bsdf->pdf(bsdfEvalQuery));
                        Color3f transmittance = medium ? medium->tr(lightQuery.shadowRay.maxt) : 1.0f;
                        L += wEm * t * std::abs(wo.z()) * bsdfValueToLight * radiance * transmittance;
                    }
                    bsdfPdf = bsdf->pdf(bsdfQuery);
                }
                else {
                    bsdfPdf = -1.0f; // infinity
                }

                recursive_ray = Ray3f(its.p, its.shFrame.toWorld(bsdfQuery.wo));
                
                if (its.mesh->getInterior() || its.mesh->getExterior()) {
                    medium = its.shFrame.n.dot(recursive_ray.d) > 0 ? its.mesh->getExterior() : its.mesh->getInterior();
                }

                t *= bsdfValue;
            }
        }

        return L;
    };

    std::string toString() const {
        return "VolPathMis[]";
    }
};

NORI_REGISTER_CLASS(VolPathMis, "vol_path_mis");
NORI_NAMESPACE_END
