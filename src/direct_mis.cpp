#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/sampler.h>
#include <nori/bsdf.h>

NORI_NAMESPACE_BEGIN

class DirectMis : public Integrator {
public:

    DirectMis(const PropertyList& props) {}
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
        Color3f Le = its.mesh->isEmitter() ? its.mesh->getEmitter()->eval(EmitterQueryRecord(ray.o, its.p, its.shFrame.n)) : 0.0f;

        const BSDF* bsdf = its.mesh->getBSDF();


        // random light in scene
        LightBVHQueryRecord lqr(its.p, its.shFrame.n);
        const Emitter* light = scene->getRandomEmitter(lqr);

        // radiance of a sampled point on light source
        EmitterQueryRecord lightQuery = EmitterQueryRecord(its.p);
        const Color3f radiance = light->sample(lightQuery, sampler->next2D());

        Color3f Lem(0.0f);
        // if light is visible
        if (!scene->rayIntersect(lightQuery.shadowRay)) {
            const float cos = its.shFrame.n.dot(lightQuery.wi);
            // light source is not behind
            if (cos > 0) {
                BSDFQueryRecord bsdfEvalQuery = BSDFQueryRecord(
                    its.shFrame.toLocal(-ray.d),
                    its.shFrame.toLocal(lightQuery.wi),
                    ESolidAngle);
                bsdfEvalQuery.uv = its.uv;
                bsdfEvalQuery.p = its.p;
                Color3f emitterScatter = bsdf->eval(bsdfEvalQuery);

                const float wEm = (lightQuery.pdf * lqr.pdf) / (lightQuery.pdf * lqr.pdf + bsdf->pdf(bsdfEvalQuery));

                Lem = lqr.pdf > Epsilon ? wEm * radiance * cos * emitterScatter : Color3f(0.f);
            }
        }

        BSDFQueryRecord bsdfQuery = BSDFQueryRecord(its.shFrame.toLocal(-ray.d));
        bsdfQuery.uv = its.uv;
        bsdfQuery.p = its.p;
        const Color3f bsdfValue = bsdf->sample(bsdfQuery, sampler->next2D());

        Color3f Lmat(0.0f);

        if (bsdfQuery.wo.z() > 0) {
            Ray3f matRay(its.p, its.shFrame.toWorld(bsdfQuery.wo));
            Intersection itsMat;
            if (scene->rayIntersect(matRay, itsMat) && itsMat.mesh->isEmitter()) {
                EmitterQueryRecord eq(its.p, itsMat.p, itsMat.shFrame.n);
                Color3f radiance = itsMat.mesh->getEmitter()->eval(eq);

                const float pdfMat = bsdf->pdf(bsdfQuery);
                const float wMat = pdfMat / (pdfMat + itsMat.mesh->getEmitter()->pdf(eq) / scene->getLights().size());

                Lmat = wMat * bsdfValue * radiance;
            }
        }

        return Le + Lem + Lmat;
    };

    std::string toString() const {
        return "DirectMis[]";
    }
};

NORI_REGISTER_CLASS(DirectMis, "direct_mis");
NORI_NAMESPACE_END
