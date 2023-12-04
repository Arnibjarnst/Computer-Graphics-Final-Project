#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/bsdf.h>
#include <nori/sampler.h>
#include <nori/warp.h>

NORI_NAMESPACE_BEGIN

/*
Density used for importance: g = pdf_mat * pdf_ems
Density used for sampling:   p = Warp::squareToCosineEmisphere
Weight for given sample:     g / p = pdf_mat * pdf_ems / Warp::squareToCosineHemispherePdf
*/
class DirectRISHEMIIntegrator : public Integrator {
public:
    DirectRISHEMIIntegrator(const PropertyList &props) {}

    struct SampleRecord{
        // Lighting intensity
        Color3f le;
        // BSDF value
        Color3f bsdf;
        // Outgoing direction
        Vector3f wo;
        // Weight
        float weight;
        // Density used for importance
        float g;
    };

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
    /* Find the surface that is visible in the requested direction */
        Intersection its;
        if (!scene->rayIntersect(ray, its))
            return Color3f(0.0f);
        
        Color3f res;

        // Light emitted at the intersection
        if (its.mesh->isEmitter()){
            EmitterQueryRecord eqr = EmitterQueryRecord(ray.o, its.p, its.shFrame.n);
            res += its.mesh->getEmitter()->eval(eqr);
        }

        // sample cosine hemisphere M times
        const auto emitters_count = scene->getLights().size();
        const Vector3f wi = its.shFrame.toLocal(-ray.d);
        float weigth_sum = 0.f;
        float cos_sum = 0.f;
        const int M = 5;
        SampleRecord samples[M];

        for (int i = 0; i < M; i++) { 
            Vector3f wo = Warp::squareToCosineHemisphere(sampler->next2D());
            samples[i].wo = wo;
            Intersection sec_its;
            const Ray3f sec_ray = Ray3f(its.p, its.shFrame.toWorld(wo));
            // If g of sample is zero
            if (!scene->rayIntersect(sec_ray, sec_its) || // No intersection
                !sec_its.mesh->isEmitter() ||             // Intersection with non-emitter
                !its.mesh->getBSDF()->isDiffuse())        // Discrete BSDF
                samples[i].weight = 0.f;
            else {
                EmitterQueryRecord eqr = EmitterQueryRecord(its.p, sec_its.p, sec_its.shFrame.n);
                BSDFQueryRecord bqr = BSDFQueryRecord(wi, wo, ESolidAngle);
                const Emitter *e = sec_its.mesh->getEmitter();
                const BSDF *b = its.mesh->getBSDF();
                samples[i].le = e->eval(eqr) * Frame::cosTheta(wo);
                samples[i].bsdf = b->eval(bqr);
                samples[i].weight = e->pdf(eqr) * b->pdf(bqr) / Warp::squareToCosineHemispherePdf(wo);
                
                samples[i].g = e->pdf(eqr) * b->pdf(bqr);
                weigth_sum += samples[i].weight; 
            }
            
        }
        
        if (weigth_sum <= 0.f) return res;

        // Choose one sample w/ prob proportional to weight
        float sample = sampler->next1D() * weigth_sum;
        int idx = -1;
        while (sample > 0){
            sample -= samples[++idx].weight;
        }

        Color3f res_chosen = samples[idx].bsdf * samples[idx].le / samples[idx].g; // f(Y) / g(Y)
        return res +  res_chosen  / M * weigth_sum;
    }

    std::string toString() const {
        return "DirectRISHEMIIntegrator[]";
    }
};

NORI_REGISTER_CLASS(DirectRISHEMIIntegrator, "direct_ris_hemi");
NORI_NAMESPACE_END