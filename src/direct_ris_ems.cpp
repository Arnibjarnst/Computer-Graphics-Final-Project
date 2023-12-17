#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/bsdf.h>
#include <nori/sampler.h>

NORI_NAMESPACE_BEGIN

/*
Density used for importance: g = pdf_mat * pdf_ems
Density used for sampling:   p = pdf_ems
Weight for given sample:     g / p = pdf_mat
*/
class DirectRISEMSIntegrator : public Integrator {
public:
    DirectRISEMSIntegrator(const PropertyList &props) {}

    struct SampleRecord{
        // Direction to emitter
        Vector3f wi;
        // Ray to emitter
        Ray3f shadowRay;
        // Lighting intensity
        Color3f le;
        // BSDF sample
        Color3f bsdf;
        // Weight (g = pdf_mat * pdf_ems, p = pdf_ems)
        float pdf_mat;
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

        // sample emitter and generate shadow ray M times
        const auto emitters_count = scene->getLights().size();
        const Vector3f wo = its.shFrame.toLocal(-ray.d);
        float weigth_sum = 0.f;
        const int M = 5;
        SampleRecord samples[M];
        for (int i = 0; i < M; i++) { 
            const Emitter *e = scene->getRandomEmitter(sampler->next1D());
            EmitterQueryRecord eqr = EmitterQueryRecord(its.p);
            samples[i].le = e->sample(eqr, sampler->next2D());
            samples[i].wi = its.shFrame.toLocal(eqr.shadowRay.d); 
            samples[i].shadowRay = eqr.shadowRay;

            BSDFQueryRecord bqr = BSDFQueryRecord(samples[i].wi, wo, ESolidAngle);
            bqr.its = &its;
            samples[i].bsdf = its.mesh->getBSDF()->eval(bqr);
            samples[i].pdf_mat = its.mesh->getBSDF()->pdf(bqr);
            weigth_sum += samples[i].pdf_mat;            
        }
        // Choose one sample w/ prob proportional to weight
        float sample = sampler->next1D() * weigth_sum;
        int idx = -1;
        while (sample > 0){
            sample -= samples[++idx].pdf_mat;
        }
        
        // Check visibility
        if (scene->rayIntersect(samples[idx].shadowRay)) return res;

        Color3f res_chosen = samples[idx].bsdf * samples[idx].le * Frame::cosTheta(samples[idx].wi) * emitters_count / samples[idx].pdf_mat; // f(Y) / g(Y)
        return res + res_chosen / M * weigth_sum;
    }

    std::string toString() const {
        return "DirectRISEMSIntegrator[]";
    }
protected:
    int m_M;
};

NORI_REGISTER_CLASS(DirectRISEMSIntegrator, "direct_ris_ems");
NORI_NAMESPACE_END