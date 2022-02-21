#include <rw/core/DOMElem.hpp>
#include <rw/core/DOMParser.hpp>
#include <rw/core/Property.hpp>
#include <rw/core/PropertyMap.hpp>
#include <rw/loaders/dom/DOMBasisTypes.hpp>
#include <rw/loaders/dom/DOMWorkCellSaver.hpp>
#include <rwlibs/control/JointController.hpp>
#include <rwsim/control/BeamJointController.hpp>
#include <rwsim/control/BodyController.hpp>
#include <rwsim/control/PDController.hpp>
#include <rwsim/control/PoseController.hpp>
#include <rwsim/control/SerialDeviceController.hpp>
#include <rwsim/control/SpringJointController.hpp>
#include <rwsim/control/SyncPDController.hpp>
#include <rwsim/control/VelRampController.hpp>
#include <rwsim/dynamics/BeamBody.hpp>
#include <rwsim/dynamics/DynamicWorkCell.hpp>
#include <rwsim/dynamics/FixedBody.hpp>
#include <rwsim/dynamics/FixedLink.hpp>
#include <rwsim/dynamics/KinematicBody.hpp>
#include <rwsim/dynamics/KinematicDevice.hpp>
#include <rwsim/dynamics/Link.hpp>
#include <rwsim/dynamics/RigidBody.hpp>
#include <rwsim/dynamics/RigidDevice.hpp>
#include <rwsim/dynamics/SuctionCup.hpp>
#include <rwsim/loaders/DynamicWorkCellSaver.hpp>

#include <algorithm>
#include <vector>

#define DEFAULT_WC_PATH "./WorkCell.wc.xml"

using namespace rwsim::dynamics;
using namespace rwsim::loaders;
using namespace rwsim::control;
using namespace rwlibs::control;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::core;

class RigidLink : public Body
{};

namespace {
class ElementCreator
{
  public:
    ElementCreator (DOMElem::Ptr root) : _root (root), _ns (""), _useJoint (false) {}

    template< class T >
    DOMElem::Ptr createElement (T object, DOMElem::Ptr parent, std::string name);
    template< class T > DOMElem::Ptr createElement (T object, DOMElem::Ptr parent);

    bool isProcessed (Body::Ptr);
    std::string correctNS (std::string name);
    void setNS (std::string ns) { _ns = ns; }

    std::string frameOrJoint ()
    {
        if (_useJoint) {
            return "joint";
        }
        else {
            return "frame";
        }
    }

  private:
    DOMElem::Ptr _root;
    std::vector< Body::Ptr > _processedBodies;
    std::string _ns;
    bool _useJoint;
};

template< class T > std::string arrayValue (const T& v, size_t n)
{
    std::ostringstream str;
    str.unsetf (std::ios::floatfield);    // floatfield not set
    str.precision (16);
    for (size_t i = 0; i < n; ++i) {
        str << v[i];
        if (i != n - 1)
            str << " ";
    }
    return str.str ();
}
template< class T > std::string arrayValue (const T& v, size_t n, size_t m)
{
    std::ostringstream str;
    str.unsetf (std::ios::floatfield);    // floatfield not set
    str.precision (16);

    for (size_t i = 0; i < n; ++i) {
        for (size_t j = 0; j < m; ++j) {
            if (j == 0) {
                str << "\n\t\t";
            }
            str << v (i, j);
            str << " ";
        }
    }
    return str.str ();
}

template< class T > std::string arrayValue (const T& v)
{
    return arrayValue< T > (v, v.size ());
}

std::string toString (Constraint::ConstraintType ct)
{
    using CT = Constraint::ConstraintType;
    switch (ct) {
        case CT::Fixed: return "Fixed";
        case CT::Prismatic: return "Prismatic";
        case CT::Revolute: return "Revolute";
        case CT::Universal: return "Universal";
        case CT::Spherical: return "Spherical";
        case CT::Piston: return "Piston";
        case CT::PrismaticRotoid: return "PrismaticRotoid";
        case CT::PrismaticUniversal: return "PrismaticUniversal";
        case CT::Free: return "Free";
        default: return "Unknown";
    }
}
std::string toString (JointController::ControlMode cm)
{
    using CM = JointController::ControlMode;
    switch (cm) {
        case CM::POSITION: return "Position";
        case CM::CNT_POSITION: return "CntPosition";
        case CM::VELOCITY: return "Velocity";
        case CM::FORCE: return "Force";
        case CM::CURRENT: return "Current";
        default: return "Unknown";
    }
}

bool ElementCreator::isProcessed (Body::Ptr body)
{
    if (std::find (_processedBodies.begin (), _processedBodies.end (), body) !=
        _processedBodies.end ()) {
        return true;
    }
    _processedBodies.push_back (body);
    return false;
}

std::string ElementCreator::correctNS (std::string name)
{
    std::size_t pos = name.find (_ns + ".");
    if (pos == std::string::npos)
        return name;
    return name.replace (pos, _ns.length () + 1, "");
}

template<> DOMElem::Ptr ElementCreator::createElement (PropertyMap object, DOMElem::Ptr parent)
{
    DOMElem::Ptr map;
    if (object.getName ().empty ()) {
        map = parent->addChild ("UnNamedPropertyMap");
    }
    else {
        map = parent->addChild (object.getName ());
    }
    for (const rw::core::Ptr< PropertyBase >& p : object.getProperties ()) {
        DOMElem::Ptr pe = map->addChild ("Property");
        pe->addAttribute ("name")->setValue (p->getIdentifier ());
        if (p->getType ().getId () == PropertyType::Types::Float) {
            pe->setValue (object.get< float > (p->getIdentifier ()));
            pe->addAttribute ("type")->setValue ("float");
        }
        else if (p->getType ().getId () == PropertyType::Types::Double) {
            pe->setValue (float (object.get< double > (p->getIdentifier ())));
            pe->addAttribute ("type")->setValue ("float");
        }
        else if (p->getType ().getId () == PropertyType::Types::String) {
            pe->setValue (object.get< std::string > (p->getIdentifier ()));
            pe->addAttribute ("type")->setValue ("string");
        }
        else if (p->getType ().getId () == PropertyType::Types::Int) {
            pe->setValue (object.get< int > (p->getIdentifier ()));
            pe->addAttribute ("type")->setValue ("int");
        }
        else if (p->getType ().getId () == PropertyType::Types::Q) {
            pe->setValue (arrayValue (object.get< rw::math::Q > (p->getIdentifier ())));
            pe->addAttribute ("type")->setValue (DOMBasisTypes::idQ ());
        }
    }
    return map;
}

template<> DOMElem::Ptr ElementCreator::createElement (MaterialDataMap object, DOMElem::Ptr parent)
{
    DOMElem::Ptr map = parent->addChild ("MaterialData");
    for (std::string mat : object.getMaterials ()) {
        DOMElem::Ptr me = map->addChild ("Material");
        me->addAttribute ("id")->setValue (mat);
        me->addChild ("Description")->setValue (object.getDescription (mat));
    }
    DOMElem::Ptr fric = parent->addChild ("FrictionMap");

    for (std::string m1 : object.getMaterials ()) {
        for (std::string m2 : object.getMaterials ()) {
            if (object.hasFrictionData (m1, m2)) {
                DOMElem::Ptr pair = fric->addChild ("Pair");
                pair->addAttribute ("first")->setValue (m1);
                pair->addAttribute ("second")->setValue (m2);
                FrictionData data = object.getFrictionData (m1, m2);
                DOMElem::Ptr fric = pair->addChild ("FrictionData");
                fric->addAttribute ("type")->setValue (data.typeName);
                for (std::pair< std::string, rw::math::Q >& p : data.parameters) {
                    fric->addChild (p.first)->setValue (arrayValue (p.second));
                }
            }
        }
    }

    return parent;
}

template<> DOMElem::Ptr ElementCreator::createElement (ContactDataMap object, DOMElem::Ptr parent)
{
    DOMElem::Ptr map = parent->addChild ("ObjectTypeData");
    for (std::string obj : object.getObjectTypes ()) {
        DOMElem::Ptr me = map->addChild ("ObjectType");
        me->addAttribute ("id")->setValue (obj);
        me->addChild ("Description")->setValue (object.getDesctiption (obj));
    }

    DOMElem::Ptr contact = parent->addChild ("ContactMap");
    for (std::string o1 : object.getObjectTypes ()) {
        for (std::string o2 : object.getObjectTypes ()) {
            bool newton = false;
            try {
                object.getNewtonData (o1, o2);
                newton = true;
            }
            catch (rw::core::Exception& e) {
                newton = false;
            }

            bool chatter = false;
            if (!newton) {
                try {
                    object.getChatterjeeData (o1, o2);
                    chatter = true;
                }
                catch (rw::core::Exception& e) {
                    chatter = false;
                }
            }

            if (newton || chatter) {
                DOMElem::Ptr pair = contact->addChild ("Pair");
                pair->addAttribute ("first")->setValue (o1);
                pair->addAttribute ("second")->setValue (o2);

                if (newton) {
                    ContactDataMap::NewtonData n = object.getNewtonData (o1, o2);
                    DOMElem::Ptr cd              = pair->addChild ("ContactData");
                    cd->addAttribute ("type")->setValue ("Newton");
                    cd->addChild ("cr")->setValue (n.cr);
                }
                else {
                    ContactDataMap::ChatterjeeData n = object.getChatterjeeData (o1, o2);
                    DOMElem::Ptr cd                  = pair->addChild ("ContactData");
                    cd->addAttribute ("type")->setValue ("Chatterjee");
                    cd->addChild ("crN")->setValue (n.crN);
                    cd->addChild ("crT")->setValue (n.crT);
                }
            }
        }
    }

    return parent;
}

template<>
DOMElem::Ptr ElementCreator::createElement (rw::math::Vector3D< double > object,
                                            DOMElem::Ptr parent, std::string name)
{
    DOMElem::Ptr elem = parent->addChild (name);
    elem->setValue (arrayValue (object));

    return elem;
}

template<>
DOMElem::Ptr ElementCreator::createElement (rw::math::Vector3D< double > object,
                                            DOMElem::Ptr parent)
{
    return createElement (object, parent, "Vector3D");
}

template<> DOMElem::Ptr ElementCreator::createElement (Body::Ptr object, DOMElem::Ptr parent)
{
    DOMElem::Ptr elem = parent->addChild ("Body");
    elem->addAttribute (this->frameOrJoint ())
        ->setValue (correctNS (object->getBodyFrame ()->getName ()));
    elem->addChild ("MaterialID")->setValue (object->getInfo ().material);
    elem->addChild ("ObjectID")->setValue (object->getInfo ().objectType);
    return elem;
}

template<> DOMElem::Ptr ElementCreator::createElement (RigidBody::Ptr object, DOMElem::Ptr parent)
{
    DOMElem::Ptr elem = createElement (object.cast< Body > (), parent);
    elem->setName ("RigidBody");
    elem->addChild ("Mass")->setValue (object->getInfo ().mass);
    elem->addChild ("COG")->setValue (arrayValue (object->getInfo ().masscenter));
    elem->addChild ("Inertia")->setValue (arrayValue (object->getInfo ().inertia, 3, 3));
    if (!object->getInfo ().integratorType.empty ()) {
        elem->addChild ("Integrator")->setValue (object->getInfo ().integratorType);
    }

    for (rw::models::Object::Ptr& o : object->getInfo ().objects) {
        if (o->getName () != object->getName ()) {
            std::cout << o->getName () << " != " << object->getName () << std::endl;
            elem->addChild ("Associate")->addAttribute ("object")->setValue (o->getName ());
        }
    }

    return elem;
}

template<>
DOMElem::Ptr ElementCreator::createElement (KinematicBody::Ptr object, DOMElem::Ptr parent)
{
    DOMElem::Ptr elem = createElement ((Body::Ptr) object, parent);
    elem->setName ("KinematicBody");

    return elem;
}

template<> DOMElem::Ptr ElementCreator::createElement (FixedBody::Ptr object, DOMElem::Ptr parent)
{
    DOMElem::Ptr elem = createElement (object.cast< Body > (), parent);
    elem->setName ("FixedBody");

    return elem;
}

template<>
DOMElem::Ptr ElementCreator::createElement (rw::core::Ptr< RigidLink > object, DOMElem::Ptr parent)
{
    DOMElem::Ptr elem = createElement (object.cast< Body > (), parent);
    elem->setName ("RigidLink");
    elem->addChild ("Mass")->setValue (object->getInfo ().mass);
    elem->addChild ("COG")->setValue (arrayValue (object->getInfo ().masscenter));
    elem->addChild ("Inertia")->setValue (arrayValue (object->getInfo ().inertia, 3, 3));
    if (!object->getInfo ().integratorType.empty ()) {
        elem->addChild ("Integrator")->setValue (object->getInfo ().integratorType);
    }

    for (rw::models::Object::Ptr& o : object->getInfo ().objects) {
        if (o->getName () != object->getName ()) {
            std::cout << o->getName () << " != " << object->getName () << std::endl;
            elem->addChild ("Associate")->addAttribute ("object")->setValue (o->getName ());
        }
    }

    return elem;
}

template<> DOMElem::Ptr ElementCreator::createElement (BeamBody::Ptr object, DOMElem::Ptr parent)
{
    DOMElem::Ptr elem = createElement (object.cast< Body > (), parent);
    elem->setName ("BeamBody");

    return elem;
}

template<>
DOMElem::Ptr ElementCreator::createElement (std::vector< Body::Ptr > object, DOMElem::Ptr parent)
{
    for (Body::Ptr& b : object) {
        if (!isProcessed (b)) {
            if (b.cast< FixedBody > ()) {
                createElement (b.cast< FixedBody > (), parent);
            }
            else if (b.cast< KinematicBody > ()) {
                createElement (b.cast< KinematicBody > (), parent);
            }
            else if (b.cast< RigidBody > ()) {
                createElement (b.cast< RigidBody > (), parent);
            }
            else if (b.cast< RigidLink > ()) {
                createElement (b.cast< RigidLink > (), parent);
            }
            else if (b.cast< BeamBody > ()) {
                createElement (b.cast< BeamBody > (), parent);
            }
            else {
                RW_THROW ("Unable to find valid body type for: \"" << b->getName () << "\"");
            }
        }
    }
    return parent;
}

template<>
DOMElem::Ptr ElementCreator::createElement (DynamicDevice::Ptr object, DOMElem::Ptr parent,
                                            std::string name)
{
    DOMElem::Ptr elem = parent->addChild (name);
    elem->addAttribute ("device")->setValue (object->getName ());

    this->setNS (object->getName ());
    Body::Ptr b = object->getBase ();
    if (!isProcessed (b)) {
        if (b.cast< FixedBody > ()) {
            createElement (b.cast< FixedBody > (), elem)->setName ("FixedBase");
        }
        else if (b.cast< KinematicBody > ()) {
            createElement (b.cast< KinematicBody > (), elem)->setName ("KinematicBase");
        }
        else {
            RW_THROW ("Only Fixed body and Kinematic body allowed for Base");
        }
    }
    _useJoint = true;
    for (Body::Ptr l : object->getLinks ()) {
        if (!isProcessed (l)) {
            if (l.cast< RigidBody > ()) {
                createElement (l.cast< RigidBody > (), elem)->setName ("RigidJoint");
            }
            else if (l.cast< KinematicBody > ()) {
                createElement (l.cast< KinematicBody > (), elem)->setName ("KinematicJoint");
            }
            else if (l.cast< RigidLink > ()) {
                createElement (l.cast< RigidLink > (), elem)->setName ("RigidJoint");
            }
            else {
                RW_THROW ("Only RigidBody allowed as link");
            }
        }
    }
    _useJoint = false;
    this->setNS ("");

    return elem;
}

template<>
DOMElem::Ptr ElementCreator::createElement (DynamicDevice::Ptr object, DOMElem::Ptr parent)
{
    return createElement (object, parent, "DynamicDevice");
}

template<>
DOMElem::Ptr ElementCreator::createElement (KinematicDevice::Ptr object, DOMElem::Ptr parent)
{
    DOMElem::Ptr elem = createElement (object.cast< DynamicDevice > (), parent, "KinematicDevice");
    return elem;
}

template<> DOMElem::Ptr ElementCreator::createElement (RigidDevice::Ptr object, DOMElem::Ptr parent)
{
    DOMElem::Ptr elem = createElement (object.cast< DynamicDevice > (), parent, "RigidDevice");

    setNS (object->getName ());
    size_t i = 0;
    for (Body::Ptr l : object->getLinks ()) {
        DOMElem::Ptr fl = elem->addChild ("ForceLimit");
        fl->addAttribute ("joint")->setValue (correctNS (l->getName ()));
        fl->setValue (object->getMotorForceLimits ()[i++]);
    }
    setNS ("");
    return elem;
}

template<> DOMElem::Ptr ElementCreator::createElement (SuctionCup::Ptr object, DOMElem::Ptr parent)
{
    DOMElem::Ptr elem = createElement (object.cast< DynamicDevice > (), parent, "SuctionCup");
    return elem;
}

template<>
DOMElem::Ptr ElementCreator::createElement (std::vector< DynamicDevice::Ptr > object,
                                            DOMElem::Ptr parent)
{
    for (DynamicDevice::Ptr& b : object) {
        if (b.cast< KinematicDevice > ()) {
            createElement (b.cast< KinematicDevice > (), parent);
        }
        else if (b.cast< RigidDevice > ()) {
            createElement (b.cast< RigidDevice > (), parent);
        }
        else if (b.cast< SuctionCup > ()) {
            createElement (b.cast< SuctionCup > (), parent);
        }
        else {
            RW_THROW ("Unknown Device Type");
        }
    }
    return parent;
}

template<> DOMElem::Ptr ElementCreator::createElement (Constraint::Ptr object, DOMElem::Ptr parent)
{
    DOMElem::Ptr elem = parent->addChild ("Constraint");
    elem->addAttribute ("name")->setValue (object->getName ());
    elem->addAttribute ("type")->setValue (toString (object->getType ()));
    elem->addAttribute ("parent")->setValue (object->getBody1 ()->getName ());
    elem->addAttribute ("child")->setValue (object->getBody2 ()->getName ());
    if (object->getTransform () != rw::math::Transform3D< double > ()) {
        DOMElem::Ptr t3d = elem->addChild (DOMBasisTypes::idTransform3D ());
        DOMBasisTypes::write (object->getTransform (), t3d, false);
    }
    if (object->getSpringParams ().enabled) {
        DOMElem::Ptr spring = elem->addChild ("Spring");
        auto com            = object->getSpringParams ().compliance;
        spring->addChild ("Compliance")->setValue (arrayValue (com, com.cols (), com.rows ()));
        auto dam = object->getSpringParams ().damping;
        spring->addChild ("Damping")->setValue (arrayValue (dam, dam.cols (), dam.rows ()));
    }
    return elem;
}

template<>
DOMElem::Ptr ElementCreator::createElement (std::vector< Constraint::Ptr > object,
                                            DOMElem::Ptr parent)
{
    for (Constraint::Ptr& b : object) {
        createElement (b, parent);
    }
    return parent;
}

template<>
DOMElem::Ptr ElementCreator::createElement (rwlibs::simulation::SimulatedController::Ptr object,
                                            DOMElem::Ptr parent, std::string name)
{
    DOMElem::Ptr child = parent->addChild (name);
    child->addAttribute ("name")->setValue (object->getControllerName ());

    return child;
}
template<>
DOMElem::Ptr ElementCreator::createElement (PDController::Ptr object, DOMElem::Ptr parent)
{
    DOMElem::Ptr elem = createElement (
        object.cast< rwlibs::simulation::SimulatedController > (), parent, "PDDeviceController");
    elem->addAttribute ("type")->setValue (
        toString (JointController::ControlMode (object->getControlModes ())));
    elem->addAttribute ("device")->setValue (object->getModel ().getName ());

    elem->addChild ("Sync")->setValue ("False");

    std::vector< double > param;
    for (PDParam& p : object->getParameters ()) {
        param.push_back (p.P);
        param.push_back (p.D);
    }
    elem->addChild ("PDParams")->setValue (arrayValue (param));
    elem->addChild ("TimeStep")->setValue (object->getSampleTime ());
    return elem;
}
template<>
DOMElem::Ptr ElementCreator::createElement (PoseController::Ptr object, DOMElem::Ptr parent)
{
    DOMElem::Ptr elem = createElement (
        object.cast< rwlibs::simulation::SimulatedController > (), parent, "PoseDeviceController");
    elem->addAttribute ("device")->setValue (object->getControlledDevice ()->getName ());
    elem->addChild ("TimeStep")->setValue (object->getSampleTime ());

    return elem;
}
template<>
DOMElem::Ptr ElementCreator::createElement (SerialDeviceController::Ptr object, DOMElem::Ptr parent)
{
    DOMElem::Ptr elem = createElement (object.cast< rwlibs::simulation::SimulatedController > (),
                                       parent,
                                       "SerialDeviceController");
    elem->addAttribute ("device")->setValue (object->getDynamicDevice ()->getName ());

    return elem;
}
template<>
DOMElem::Ptr ElementCreator::createElement (SpringJointController::Ptr object, DOMElem::Ptr parent)
{
    DOMElem::Ptr elem = createElement (
        object.cast< rwlibs::simulation::SimulatedController > (), parent, "SpringJointController");
    elem->addAttribute ("device")->setValue (object->getModel ().getName ());
    elem->addChild ("TimeStep")->setValue (object->getSampleTime ());

    std::vector< double > param;
    for (SpringJointController::SpringParam& p : object->getParameters ()) {
        param.push_back (p.elasticity);
        param.push_back (p.dampening);
        param.push_back (p.offset);
    }
    elem->addChild ("SpringParams")->setValue (arrayValue (param));

    return elem;
}

template<>
DOMElem::Ptr
ElementCreator::createElement (std::vector< rwlibs::simulation::SimulatedController::Ptr > object,
                               DOMElem::Ptr parent)
{
    for (rwlibs::simulation::SimulatedController::Ptr sc : object) {
        if (sc.cast< BeamJointController > ()) {
            RW_THROW ("NOT CURRENTLY SUPPORTED");
        }
        else if (sc.cast< BodyController > ()) {
            RW_THROW ("NOT CURRENTLY SUPPORTED");
        }
        else if (sc.cast< PDController > ()) {
            createElement(sc.cast<PDController>(),parent);
        }
        else if (sc.cast< PoseController > ()) {
            createElement(sc.cast<PoseController>(),parent);
        }
        else if (sc.cast< SerialDeviceController > ()) {
            createElement(sc.cast<SerialDeviceController>(),parent);
        }
        else if (sc.cast< SpringJointController > ()) {
            createElement(sc.cast<SpringJointController>(),parent);
        }
        else if (sc.cast< SyncPDController > ()) {
            RW_THROW ("NOT CURRENTLY SUPPORTED");
        }
        else if (sc.cast< VelRampController > ()) {
            RW_THROW ("NOT CURRENTLY SUPPORTED");
        }
        else {
            RW_THROW ("Unknown CONTROLLER");
        }
    }
    return parent;
}

void createDOMDocument (DOMElem::Ptr rootDoc, rw::core::Ptr< const DynamicWorkCell > dwc,
                        const State state)
{
    DOMElem::Ptr root = rootDoc->addChild ("DynamicWorkCell");

    if (dwc->getWorkcell ()->getFilename () == "") {
        root->addAttribute ("workcell")->setValue (DEFAULT_WC_PATH);
    }
    else {
        root->addAttribute ("workcell")->setValue (dwc->getWorkcell ()->getFilename ());
    }

    ElementCreator c (root);
    c.createElement (dwc->getEngineSettings (), root)->setName ("PhysicsEngine");
    c.createElement (dwc->getGravity (), root, "Gravity");
    c.createElement (dwc->getMaterialData (), root);
    c.createElement (dwc->getContactData (), root);
    c.createElement (dwc->getDynamicDevices (), root);
    c.createElement (dwc->getBodies (), root);
    c.createElement (dwc->getConstraints (), root);
    c.createElement (dwc->getControllers (), root);
}
}    // namespace

void DynamicWorkCellSaver::save (rw::core::Ptr< const DynamicWorkCell > workcell,
                                 const State& state, std::string fileName)
{
    DOMParser::Ptr doc = DOMParser::make ();
    DOMElem::Ptr root  = doc->getRootElement ();

    createDOMDocument (root, workcell, state);

    // save to file
    doc->save (fileName);
}

void DynamicWorkCellSaver::save (rw::core::Ptr< const DynamicWorkCell > workcell,
                                 const State& state, std::ostream& ostream)
{
    DOMParser::Ptr doc = DOMParser::make ();
    DOMElem::Ptr root  = doc->getRootElement ();

    createDOMDocument (root, workcell, state);

    // save to stream
    doc->save (ostream);
}