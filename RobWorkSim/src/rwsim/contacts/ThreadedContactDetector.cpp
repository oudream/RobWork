
#include <rw/kinematics/FKTable.hpp>
#include <rw/kinematics/Kinematics.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/proximity/ProximityFilter.hpp>
#include <rw/proximity/ProximityStrategyData.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyPQP.hpp>
#include <rwsim/contacts/ThreadedContactDetector.hpp>

using namespace rwsim::contacts;
using namespace rwsim::dynamics;
using namespace rw::proximity;
using namespace rw::kinematics;
using namespace rw::models;
using namespace rw::math;
using namespace rw::geometry;

using namespace rwlibs::proximitystrategies;

ThreadedContactDetector::ThreadedContactDetector(
    rw::core::Ptr<rwsim::dynamics::DynamicWorkCell> dwc, int threads,
    rw::core::Ptr<rw::proximity::ProximityFilterStrategy> filter) :
    BaseContactDetector(dwc->getWorkCell(), filter),
    _dwc(dwc), _tPool(threads < 1 ? std::thread::hardware_concurrency() - 1 : threads) {
    std::vector<Body::Ptr> bodies = _dwc->getBodies();
    _narrowStrat                  = rw::core::ownedPtr(new ProximityStrategyPQP());

    for(Body::Ptr body : _dwc->getBodies()) {
        Object::Ptr obj = body->getObject();
        bool res        = _narrowStrat->addModel(obj);
        if(!res) { RW_WARN("Failed to add Object: " << obj->getName() << " to strategy"); }
    }

    std::vector<Frame*> frames = Kinematics::findAllFrames(_dwc->getWorkcell()->getWorldFrame(),
                                                           _dwc->getWorkCell()->getDefaultState());
    _models                    = frames.size();
    /*for(Frame* frame : frames) {
        //_frame2models[frame] = _narrowStrat->getModel(frame);
        _models++;
    }*/

    if(_tPool.size() > 1) {
        _startLock.lock();
        _kill          = false;
        _activeRunners = 0;
        _activateDone  = true;

        for(std::thread& t : _tPool) {
            t = std::thread(std::bind(&ThreadedContactDetector::threadRunner, this));
        }
    }
    else if(_tPool.size() == 1) { _tPool.clear(); }
}

ThreadedContactDetector::~ThreadedContactDetector() {
    _kill = true;
    _startLock.unlock();
    for(std::thread& t : _tPool) { t.join(); }
}

void ThreadedContactDetector::addContacts(std::vector<Contact>& contacts, MultiDistResult& mRes) {
    MultiDistanceResult* res = &mRes.data.getMultiDistanceData();

    for(size_t i = 0; i < res->distances.size(); i++) {
        Contact c;
        Vector3D<>& p1 = res->p1s[i];
        Vector3D<>& p2 = res->p2s[i];
        c.setPointA(p1);
        c.setPointB(p2);

        c.setFrameA(res->a->getFrame());
        c.setFrameB(res->b->getFrame());

        if(res->distances[i] < 0.00000001) {
            std::pair<Vector3D<>, Vector3D<>> normals =
                mRes.strat->getSurfaceNormals(*res, (int) i);
            // the second is described in b's refframe so convert both to world and
            // combine them
            Vector3D<> a_normal = mRes.wTa.R() * normals.first;
            Vector3D<> b_normal = mRes.wTb.R() * normals.second;

            c.setNormal(-normalize(a_normal - b_normal));
        }
        else { c.setNormal(normalize(p2 - p1)); }
        c.setDepth(-res->distances[i]);
        c.setTransform(inverse(mRes.wTa) * mRes.wTb);
        contacts.push_back(c);
    }
}

std::vector<Contact> ThreadedContactDetector::findContacts(const rw::kinematics::State& state) {
    bool single = (_tPool.size() == 0);

    double maxSepDistance = _dwc->getEngineSettings().get<double>("MaxSepDistance", 0.0005);
    std::vector<Contact> contacts;
    std::vector<std::vector<Contact>> contList;
    FKTable transforms(state);
    std::vector<MultiDistJob> jobs(_models * _models);
    size_t job_i = 0;

    if(!single) {
        _contacts   = &contList;
        _table      = &transforms;
        _jobList    = &jobs;
        _maxSepDist = maxSepDistance;

        _postingJobs  = true;
        _jobs         = 0;
        _jobId        = 0;
        _activateDone = false;
        _startLock.unlock();
    }

    ProximityFilter::Ptr filter = _bpFilter->update(state);
    while(!filter->isEmpty()) {
        const FramePair& pair = filter->frontAndPop();

        FramePair fp = pair;
        if(fp.first < fp.second) {
            Frame* f  = fp.first;
            fp.first  = fp.second;
            fp.second = f;
        }

        if(single) {
            MultiDistResult res;
            res.strat = _narrowStrat;
            res.wTa   = transforms.get(pair.first);
            res.wTb   = transforms.get(pair.second);

            res.strat->distances(pair.first,
                                 res.wTa,
                                 pair.second,
                                 res.wTb,
                                 maxSepDistance,
                                 res.data);

            addContacts(contacts, res);
        }
        else {
            contList.push_back(contacts);
            jobs[job_i++] = MultiDistJob(_frame2models[pair.first],
                                         _frame2models[pair.second],
                                         pair.first,
                                         pair.second);
            _jobs++;
        }
    }

    if(!single) {
        _postingJobs = false;
        // Wait for all jobs to be accepted
        while(_jobs > _jobId && _activeRunners < _tPool.size()) {}
        _startLock.lock();

        // Wait for all Runners to be done
        _activateDone = true;
        while(_activeRunners) {}

        for(std::vector<Contact> l : contList) {
            contacts.insert(contacts.end(), l.begin(), l.end());
        }
    }
    return contacts;
}

void ThreadedContactDetector::threadRunner() {
    while(!_kill) {
        while(!_activateDone) {}

        // Guard to start the Runners
        _startLock.lock();
        _startLock.unlock();

        ++_activeRunners;

        while(_jobs > _jobId || _postingJobs) {
            MultiDistJob* job;
            size_t id;
            {
                std::lock_guard guard(_jobLock);
                if(_jobs == _jobId) {
                    if(_postingJobs) continue;
                    else break;
                }
                id  = _jobId;
                job = &(*_jobList)[id];
                ++_jobId;
            }
            MultiDistResult res;
            res.strat = _narrowStrat;
            res.wTa   = _table->get(job->A);
            res.wTb   = _table->get(job->B);

            res.strat->distances(job->A, res.wTa, job->B, res.wTb, _maxSepDist, res.data);

            addContacts((*_contacts)[id], res);
        }
        --_activeRunners;
    }
}

std::vector<Contact> ThreadedContactDetector::findContacts(const rw::kinematics::State& state,
                                                           ContactDetectorData& data) {
    RW_THROW("Not implemented");
}

std::vector<Contact> ThreadedContactDetector::findContacts(const rw::kinematics::State& state,
                                                           ContactDetectorData& data,
                                                           ContactDetectorTracking& tracking,
                                                           rwsim::log::SimulatorLogScope* log) {
    RW_THROW("Not implemented");
}

std::vector<Contact> ThreadedContactDetector::updateContacts(const rw::kinematics::State& state,
                                                             ContactDetectorData& data,
                                                             ContactDetectorTracking& tracking,
                                                             rwsim::log::SimulatorLogScope* log) {
    RW_THROW("Not implemented");
}