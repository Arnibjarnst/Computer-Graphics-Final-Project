#include <nori/subscene.h>

NORI_NAMESPACE_BEGIN

SubScene::SubScene (const PropertyList &propList) {
    m_id = propList.getInteger("id", -1);
    m_bvh = new BVH();
}

SubScene::~SubScene(){
    delete m_mesh;
}

void SubScene::addChild(NoriObject *obj) {
        switch (obj->getClassType())
        {
        case EMesh:
        std::cout << "Mesh linked!" << endl;
            if (m_mesh) 
                throw NoriException("There can be only one shape per subscene!");
            m_mesh = static_cast<Shape *>(obj);
            m_bvh->addShape(m_mesh);
            m_bvh->build();
            break;
        
        default:
            throw NoriException("SubScene::addChild(<%s>) is not supported!",
                classTypeName(obj->getClassType()));
        }
    }

std::string SubScene::toString() const{
    return tfm::format(
        "SubScene[\n"
        "   shape = %s\n"
        "   id = %d\n"
        "]",
        m_mesh->toString(),
        m_id
    );
}

NORI_REGISTER_CLASS(SubScene, "subscene");
NORI_NAMESPACE_END