#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/bsdf.h>
#include <nori/sampler.h>

NORI_NAMESPACE_BEGIN

/*
Density used for importance: g = pdf_mat * pdf_ems
Density used for sampling:   p = pdf_mat
Weight for given sample:     g / p = pdf_ems
*/
class DirectRISMATIntegrator : public Integrator {
public:
    DirectRISMATIntegrator(const PropertyList &props) {}

    struct SampleRecord{
        // Lighting intensity
        Color3f le;
        // BSDF sample
        Color3f bsdf;
        // Weight
        float pdf_ems;
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

        // sample BSDF M times
        float weigth_sum = 0.f;
        const int M = 5;
        SampleRecord samples[M];

        for (int i = 0; i < M; i++) { 
            BSDFQueryRecord bqr = BSDFQueryRecord(its.shFrame.toLocal(-ray.d));
            samples[i].bsdf = its.mesh->getBSDF()->sample(bqr, sampler->next2D());

            Intersection sec_its;
            const Ray3f sec_ray = Ray3f(its.p, its.shFrame.toWorld(bqr.wo));
            // If g of sample is zero
            if (!scene->rayIntersect(sec_ray, sec_its) || !sec_its.mesh->isEmitter()) //No intersection or non-emitter
                samples[i].pdf_ems = 0.f;
            else {
                EmitterQueryRecord eqr = EmitterQueryRecord(its.p, sec_its.p, sec_its.shFrame.n);
                const Emitter *e = sec_its.mesh->getEmitter();
                samples[i].le = e->eval(eqr);
                samples[i].pdf_ems = e->pdf(eqr);
                weigth_sum += e->pdf(eqr);   
            }
            
        }
        
        if (weigth_sum <= 0.f) return res;

        // Choose one sample w/ prob proportional to weight
        float sample = sampler->next1D() * weigth_sum;
        int idx = -1;
        while (sample > 0){
            sample -= samples[++idx].pdf_ems;
        }

        Color3f res_chosen = samples[idx].bsdf * samples[idx].le / samples[idx].pdf_ems; // f(Y) / g(Y)
        return res +  res_chosen  / M * weigth_sum;
    }

    std::string toString() const {
        return "DirectRISMATIntegrator[]";
    }
};

NORI_REGISTER_CLASS(DirectRISMATIntegrator, "direct_ris_mat");
NORI_NAMESPACE_END