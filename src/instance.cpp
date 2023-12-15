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


NORI_REGISTER_CLASS(Instance, "instance");
NORI_NAMESPACE_END