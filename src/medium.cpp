#include <nori/medium.h>

NORI_NAMESPACE_BEGIN

Medium::~Medium() {
	delete m_pf;
}

void Medium::addChild(NoriObject* obj) {
    if (obj->getClassType() == EPhaseFunction) {
        if (m_pf)
            throw NoriException(
                "Medium: tried to register multiple phase functions");
        m_pf = static_cast<PhaseFunction*>(obj);
    }
    else {
        throw NoriException("Medium::addChild(<%s>) is not supported!",
            classTypeName(obj->getClassType()));
    }
}

NORI_NAMESPACE_END