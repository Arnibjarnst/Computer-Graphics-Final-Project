#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/bsdf.h>
#include <nori/sampler.h>

NORI_NAMESPACE_BEGIN

class DirectMISIntegrator : public Integrator {
public:
    DirectMISIntegrator(const PropertyList &props) {}

    Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
        Intersection its;
        if (!scene->rayIntersect(ray, its))
            return Color3f(0.0f);
        
        Color3f res = Color3f(0.f);

        // Light emitted at intersection
        if (its.mesh->isEmitter()){
            EmitterQueryRecord eqr = EmitterQueryRecord(ray.o, its.p, its.shFrame.n);
            res += its.mesh->getEmitter()->eval(eqr);
        }

        const auto emitters_count = scene->getLights().size();
        
        // ems
        // sample emitter and generate shadow ray
        const Emitter *e1 = scene->getRandomEmitter(sampler->next1D());
        EmitterQueryRecord eqr1 = EmitterQueryRecord(its.p);
        Color3f sample_ems_ems = e1->sample(eqr1, sampler->next2D());

        Vector3f wo = its.shFrame.toLocal(-ray.d);
        Vector3f wi = its.shFrame.toLocal(eqr1.shadowRay.d);

        if (!scene->rayIntersect(eqr1.shadowRay)) {
            // query bsdf at intersection
            BSDFQueryRecord bqr1 = BSDFQueryRecord(wo, wi, ESolidAngle);
            bqr1.its = &its;
            const BSDF *b1 = its.mesh->getBSDF();
            Color3f sample_mat_ems = b1->eval(bqr1);
            
            // compute ems weight
            float pdf_ems_ems = eqr1.pdf / emitters_count;
            float pdf_mat_ems = b1->pdf(bqr1); 
            float w_ems = pdf_ems_ems / (pdf_ems_ems + pdf_mat_ems);

            if ((pdf_ems_ems + pdf_mat_ems) > Epsilon)
                res += w_ems * sample_mat_ems * sample_ems_ems * Frame::cosTheta(wi) * emitters_count;
        }

        //mats
        // sample bsdf for wo
        BSDFQueryRecord bqr2 = BSDFQueryRecord(its.toLocal(-ray.d));
        bqr2.its = &its;
        const BSDF *b2 = its.mesh->getBSDF();
        const Color3f sample_mat_mat = b2->sample(bqr2, sampler->next2D());

        // generate secondary ray
        Intersection sec_its;
        const Ray3f sec_ray = Ray3f(its.p, its.toWorld(bqr2.wo));
        if (scene->rayIntersect(sec_ray, sec_its) && sec_its.mesh->isEmitter()){
            // query emitter at secondary intersection
            EmitterQueryRecord eqr2 = EmitterQueryRecord(its.p, sec_its.p, sec_its.shFrame.n);
            const Emitter *e2 = sec_its.mesh->getEmitter();
            const Color3f sample_ems_mat = e2->eval(eqr2);

            // compute mats weight
            float pdf_mat_mat = b2->pdf(bqr2) ;
            float pdf_ems_mat = e2->pdf(eqr2) / emitters_count;
            float w_mat = pdf_mat_mat / (pdf_ems_mat + pdf_mat_mat);

            if ((pdf_ems_mat + pdf_mat_mat) > Epsilon)
                res += w_mat * sample_mat_mat * sample_ems_mat;

        }
        return res;
    }

    std::string toString() const {
        return "DirectMISIntegrator[]";
    }
};

NORI_REGISTER_CLASS(DirectMISIntegrator, "direct_mis");
NORI_NAMESPACE_END
