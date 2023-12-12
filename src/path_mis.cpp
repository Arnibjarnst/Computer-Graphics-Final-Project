#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/sampler.h>
#include <nori/bsdf.h>

NORI_NAMESPACE_BEGIN

class PathMis : public Integrator {
public:

    PathMis(const PropertyList& props) {}
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

        Color3f L = 0.0f;
        Color3f t = 1.0f;
        Ray3f ray(ray0);
        Intersection its;
        float bsdfPdf = -1.0; // this means that we have discrete sampling (so infinty)
        while (scene->rayIntersect(ray, its)) {
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

            const float successProbability = std::min(t.maxCoeff(), 0.99f);
            if (sampler->next1D() > successProbability) break;
            t /= successProbability;

            const BSDF* bsdf = its.mesh->getBSDF();
            const Vector3f wi = its.shFrame.toLocal(-ray.d);
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
                if (!scene->rayIntersect(lightQuery.shadowRay) && lightQuery.pdf > 0) {
                    Vector3f wo = its.shFrame.toLocal(lightQuery.wi);
                    BSDFQueryRecord bsdfEvalQuery = BSDFQueryRecord(
                        wi,
                        wo,
                        ESolidAngle);
                    bsdfEvalQuery.uv = its.uv;
                    bsdfEvalQuery.p = its.p;
                    Color3f bsdfValueToLight = bsdf->eval(bsdfEvalQuery);

                    const float wEm = lightQuery.pdf / (lightQuery.pdf / scene->getLights().size() + bsdf->pdf(bsdfEvalQuery));
                    L += wEm * t * std::abs(wo.z()) * bsdfValueToLight * radiance;
                }
                bsdfPdf = bsdf->pdf(bsdfQuery);
            }
            else {
                bsdfPdf = -1.0f; // infinity
            }

            bool isDifferential = ray.isDifferential;
            ray = Ray3f(its.p, its.shFrame.toWorld(bsdfQuery.wo));
            if (isDifferential) {
                ray.ox = ray.o + its.dpdx;
                ray.oy = ray.o + its.dpdy;
                //Normal3f dndx = isect.shading.dndu * isect.dudx +
                //    isect.shading.dndv * isect.dvdx;
                //Normal3f dndy = isect.shading.dndu * isect.dudy +
                //    isect.shading.dndv * isect.dvdy;
                //Vector3f dwodx = -ray.rxDirection - wo, dwody = -ray.ryDirection - wo;
                //Float dDNdx = Dot(dwodx, ns) + Dot(wo, dndx);
                //Float dDNdy = Dot(dwody, ns) + Dot(wo, dndy);
                //rd.rxDirection = wi - dwodx +
                //    2.f * Vector3f(Dot(wo, ns) * dndx + dDNdx * ns);
                //rd.ryDirection = wi - dwody +
                //    2.f * Vector3f(Dot(wo, ns) * dndy + dDNdy * ns);
            }

            t *= bsdfValue;
        }

        return L;
    };

    std::string toString() const {
        return "PathMis[]";
    }
};

NORI_REGISTER_CLASS(PathMis, "path_mis");
NORI_NAMESPACE_END
