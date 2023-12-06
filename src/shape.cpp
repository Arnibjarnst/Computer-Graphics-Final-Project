/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Romain Prévost

    Nori is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Nori is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <nori/shape.h>
#include <nori/bsdf.h>
#include <nori/emitter.h>
//#include <nori/warp.h>
//#include <Eigen/Geometry>

NORI_NAMESPACE_BEGIN

Shape::~Shape() {
    delete m_bsdf;
    //delete m_emitter; // scene is responsible for deleting the emitter
}

void Shape::activate() {
    if (!m_bsdf) {
        /* If no material was assigned, instantiate a diffuse BRDF */
        m_bsdf = static_cast<BSDF *>(
            NoriObjectFactory::createInstance("diffuse", PropertyList()));
        m_bsdf->activate();
    }
}

void Shape::addChild(NoriObject *obj) {
    switch (obj->getClassType()) {
        case EBSDF:
            if (m_bsdf)
                throw NoriException(
                    "Shape: tried to register multiple BSDF instances!");
            m_bsdf = static_cast<BSDF *>(obj);
            break;

        case EEmitter:
            if (m_emitter)
                throw NoriException(
                    "Shape: tried to register multiple Emitter instances!");
            m_emitter = static_cast<Emitter *>(obj);
            m_emitter->setShape(static_cast<Shape*>(this));
            break;
        case EMedium:
            if (obj->getIdName() == "interior") {
                if (m_interior)
                    throw NoriException("There can only be one interior medium per mesh!");

                m_interior = static_cast<Medium*>(obj);
            }
            else if (obj->getIdName() == "exterior") {
                if (m_exterior)
                    throw NoriException("There can only be one exterior medium per mesh!");
                m_exterior = static_cast<Medium*>(obj);
            }
            else {
                throw NoriException("medium name can only be interior/exterior");
            }
            break;
        default:
            throw NoriException("Shape::addChild(<%s>) is not supported!",
                                classTypeName(obj->getClassType()));
    }
}

std::string Intersection::toString() const {
    if (!mesh)
        return "Intersection[invalid]";

    return tfm::format(
        "Intersection[\n"
        "  p = %s,\n"
        "  t = %f,\n"
        "  uv = %s,\n"
        "  shFrame = %s,\n"
        "  geoFrame = %s,\n"
        "  mesh = %s\n"
        "]",
        p.toString(),
        t,
        uv.toString(),
        indent(shFrame.toString()),
        indent(geoFrame.toString()),
        mesh ? mesh->toString() : std::string("null")
    );
}

NORI_NAMESPACE_END
