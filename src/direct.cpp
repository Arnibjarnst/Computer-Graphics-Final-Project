#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/sampler.h>
#include <nori/bsdf.h>

NORI_NAMESPACE_BEGIN

class Direct : public Integrator {
public:

    Direct(const PropertyList& props) {}
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

        const BSDF* bsdf = its.mesh->getBSDF();
        const std::vector<Emitter*> emitters = scene->getLights();

        Color3f totalColor = Color3f(0.0f);

        for (Emitter* emitter : emitters) {
            EmitterQueryRecord q = EmitterQueryRecord();
            q.ref = its.p;
            const Color3f color = emitter->sample(q, sampler->next2D());
            float distPointToLight = q.wi.norm();
            Ray3f sampleRay = Ray3f(its.p, q.wi);

            float cos = its.shFrame.n.dot(q.wi) / distPointToLight;

            if (cos <= 0) continue;

            Intersection sampleIntersection;
            scene->rayIntersect(sampleRay, sampleIntersection);

            if (sampleIntersection.t > distPointToLight - 0.001 && sampleIntersection.t < distPointToLight + 0.001) {
                BSDFQueryRecord bsdfQuery = BSDFQueryRecord(its.shFrame.toLocal(q.wi), its.shFrame.toLocal(-ray.d), ESolidAngle);
                float pdfBSDF = its.mesh->getBSDF()->pdf(bsdfQuery);
                if (pdfBSDF > 0) totalColor += color * cos / pdfBSDF;
            }
        }
        return totalColor;
    };

    std::string toString() const {
        return "Direct[]";
    }
};

NORI_REGISTER_CLASS(Direct, "direct");
NORI_NAMESPACE_END
