#include "Object.hpp"

using namespace rw::kinematics;
using namespace rw::geometry;
using namespace rw::models;

Object::Object(rw::core::Ptr<rw::kinematics::Frame> baseframe) :
    _base(baseframe), _frames(std::vector<Frame*>(1, baseframe.get())) {}

Object::Object(std::vector<rw::kinematics::Frame*> frames) : _base(frames[0]), _frames(frames) {}

Object::~Object() {}

rw::kinematics::Frame* Object::getBase() {
    return _base.get();
}
const rw::kinematics::Frame* Object::getBase() const {
    return _base.get();
}
const std::vector<rw::kinematics::Frame*>& Object::getFrames() {
    return _frames;
}

void Object::addFrame(rw::core::Ptr<rw::kinematics::Frame> frame) {
    _frames.push_back(frame.get());
}
