#include <rw/models/WorkCell.hpp>
#include <rw/proximity/BasicFilterStrategy.hpp>
#include <rwsim/contacts/BaseContactDetector.hpp>

using namespace rwsim::contacts;

using namespace rw::proximity;

BaseContactDetector::BaseContactDetector(rw::core::Ptr<rw::models::WorkCell> workcell,
                                         rw::proximity::ProximityFilterStrategy::Ptr filter) :
    _bpFilter(filter == NULL ? rw::core::ownedPtr(new BasicFilterStrategy(workcell)) : filter),
    _wc(workcell), _timer(0.0) {}

BaseContactDetector::~BaseContactDetector() {}

void BaseContactDetector::setProximityFilterStrategy(ProximityFilterStrategy::Ptr filter) {
    _bpFilter = filter;
}

ProximityFilterStrategy::Ptr BaseContactDetector::getProximityFilterStrategy() const {
    return _bpFilter;
}

double BaseContactDetector::getTimer() const {
    return _timer;
}

void BaseContactDetector::setTimer(double value) {
    _timer = value;
}
