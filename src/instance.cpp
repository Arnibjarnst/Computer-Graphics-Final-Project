#include <nori/instance.h>
#include <nori/object.h>
#include <nori/frame.h>
#include <nori/bbox.h>
#include <nori/shape.h>
#include <nori/subscene.h>
#include <Eigen/Geometry>

NORI_NAMESPACE_BEGIN

Instance::Instance(const PropertyList &propList) {
    m_ToWorld = propList.getTransform("toWorld", Transform());
    m_ToLocal = m_ToWorld.inverse();
    m_subsceneID = propList.getInteger("subscene", -1);
}

void Instance::linkSubScene(SubScene *subscene) {
    std::cout << tfm::format("Linked subscene %d (m_subscene = %d)", subscene->getID(), m_subscene) << endl;
    if (m_subscene)
        throw NoriException("There can only be one subscene per instance!");
    m_subscene = subscene;

    m_bsdf = const_cast<BSDF *>(subscene->getMesh()->getBSDF());
    m_emitter = subscene->getMesh()->isEmitter() ? const_cast<Emitter *> (subscene->getMesh()->getEmitter()) : nullptr;
}

int Instance::getSubsceneId() {
    return m_subsceneID;
}


NORI_REGISTER_CLASS(Instance, "instance");
NORI_NAMESPACE_END