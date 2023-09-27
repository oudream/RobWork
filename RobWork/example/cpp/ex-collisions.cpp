#include <rw/kinematics.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/math.hpp>
#include <rw/models.hpp>
#include <rw/proximity.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rw/core/Exception.hpp>

#include <iostream>
#include <math.h>
#include <omp.h>
#include <string>

using namespace rw::proximity;
using namespace rw::models;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::math;
using namespace rwlibs::proximitystrategies;
using namespace std;

string CDQueryType[4] = {
    "AllContactsFullInfo", "AllContanctsNoInfo", "FirstContactFullInfo", "FirstContactNoInfo"};
string CSQueryType[2] = {"FirstContact","AllContacts"};

void printPSD (ProximityStrategyData data)
{
    std::cout << "    ProximityStrategyData" << std::endl;
    std::cout << "        Abs error  : " << data.abs_err << std::endl;
    std::cout << "        Rel error  : " << data.rel_err << std::endl;
    std::cout << "        QueryType  : " << CSQueryType[data.getCollisionQueryType ()] << std::endl;
    std::cout << "        inCollision: " << data.inCollision () << std::endl;
    CollisionStrategy::Result CSResult = data.getCollisionData ();

    try {
        std::cout << "\n        First Model Geometries: " << std::endl;
        for (string& geo : CSResult.a->getGeometryIDs ()) {
            std::cout << "            Geometry: " << geo << std::endl;
        }
        std::cout << "\n        Second Model Geometries: " << std::endl;
        for (string& geo : CSResult.b->getGeometryIDs ()) {
            std::cout << "            Geometry: " << geo << std::endl;
        }
    }
    catch (rw::core::Exception& e) {
        std::cout << "            " << e.what () << std::endl;
    }

    std::cout << "\n        Transform(aTb): " << CSResult._aTb << std::endl;
    std::cout << "        PrimTests      : " << CSResult.getNrPrimTests () << std::endl;
    std::cout << "        BVTests        : " << CSResult.getNrBVTests () << std::endl;
    std::cout << "        GeoVertex ids  : " << std::endl;
    for (pair< int, int >& ids : CSResult._geomPrimIds) {
        std::cout << "            Vertex: " << ids.first << " and " << ids.second << " colliding"
             << std::endl;
    }
    std::cout << "        CollisionPairs : " << std::endl;
    for (auto& ids : CSResult._collisionPairs) {
        std::cout << "            ColSize: " << ids.size << std::endl;
        std::cout << "            startID: " << ids.startIdx << std::endl;
        std::cout << "            ColPair: " << ids.geoIdxA << " and " << ids.geoIdxB << " colliding"
             << std::endl;
    }
}

void printColInfo (string name, CollisionDetector& detector, WorkCell::Ptr wc, State& state)
{
    std::cout << "#########################################" << std::endl;
    std::cout << "DetectingCollisions using: " << name << "\n" << std::endl;

    // done to pre call to initialize cache
    ProximityData res = ProximityData ();
    bool ret          = detector.inCollision (state, res);

    double start = omp_get_wtime ();
    detector.resetComputationTimeAndCount ();
    ProximityData res1 = ProximityData ();
    bool ret1          = detector.inCollision (state, res1);
    double time1       = detector.getComputationTime ();
    double end         = omp_get_wtime ();
    double time1t      = (end - start) * pow (10, 3);

    start = omp_get_wtime ();
    detector.resetComputationTimeAndCount ();
    ProximityData res2 = ProximityData ();
    res2.setCollisionQueryType (CollisionDetector::QueryType::AllContactsFullInfo);
    bool ret2     = detector.inCollision (state, res2);
    double time2  = detector.getComputationTime ();
    end           = omp_get_wtime ();
    double time2t = (end - start) * pow (10, 3);

    start = omp_get_wtime ();
    detector.resetComputationTimeAndCount ();
    CollisionDetector::QueryResult res3;
    bool ret3     = detector.inCollision (state, &res3);
    double time3  = detector.getComputationTime ();
    end           = omp_get_wtime ();
    double time3t = (end - start) * pow (10, 3);

    std::cout << "Time(ProximityData std)  : " << round (time1 * 1000) / 1000 << "ms - "
         << round (time1t * 10000) / 10000 << "ms" << std::endl;
    std::cout << "Time(ProximityData full) : " << round (time2 * 1000) / 1000 << "ms - "
         << round (time2t * 10000) / 10000 << "ms" << std::endl;
    std::cout << "Time(QueryResult)        : " << round (time3 * 1000) / 1000 << "ms - "
         << round (time3t * 10000) / 10000 << "ms" << std::endl;
    std::cout << std::endl;

    std::cout << "inCollision(ProximityData std)  : " << ret1 << std::endl;
    std::cout << "inCollision(ProximityData full) : " << ret2 << std::endl;
    std::cout << "inCollision(QueryResult)        : " << ret3 << std::endl;
    std::cout << std::endl;

    std::cout << "QueryType(ProximityData std) : " << CDQueryType[res1.getCollisionQueryType ()] << std::endl;
    std::cout << "QueryType(ProximityData full): " << CDQueryType[res2.getCollisionQueryType ()] << std::endl;
    std::cout << "QueryType(QueryResult)       : "
         << "Not Available" << std::endl;

    CollisionDetector::QueryResult res1QR = res1._collisionData;
    CollisionDetector::QueryResult res2QR = res2._collisionData;

    std::cout << "\nColliding Frames (ProximityData std)" << std::endl;
    for (const FramePair& frameP : res1QR.collidingFrames) {
        std::cout << "    " << frameP.first->getName () << " and " << frameP.second->getName () << std::endl;
    }

    std::cout << "\nColliding Frames (ProximityData full)" << std::endl;
    for (const FramePair& frameP : res2QR.collidingFrames) {
        std::cout << "    " << frameP.first->getName () << " and " << frameP.second->getName () << std::endl;
    }
    std::cout << "\nColliding Frames (QueryResult)" << std::endl;
    for (const FramePair& frameP : res3.collidingFrames) {
        std::cout << "    " << frameP.first->getName () << " and " << frameP.second->getName () << std::endl;
    }

    std::cout << "\nProximityStrategyData (ProximityData std)" << std::endl;
    for (ProximityStrategyData& PSD : res1QR._fullInfo) {
        printPSD (PSD);
    }

    std::cout << "\nProximityStrategyData (ProximityDataFULL)" << std::endl;
    for (ProximityStrategyData& PSD : res2QR._fullInfo) {
        printPSD (PSD);
    }

    std::cout << "\nProximityStrategyData (QueryType)" << std::endl;
    for (ProximityStrategyData& PSD : res3._fullInfo) {
        printPSD (PSD);
    }
}

int main (int argc, char** argv)
{
    WorkCell::Ptr wc   = WorkCellFactory::load ("../../ModelData/XMLScenes/RobotOnTable/Scene.xml");
    State state        = wc->getDefaultState ();
    MovableFrame* box1 = wc->findFrame< MovableFrame > ("m_box");
    MovableFrame* box2 = wc->findFrame< MovableFrame > ("m_box2");
    SerialDevice::Ptr ur = wc->findDevice< SerialDevice > ("UR-6-85-5-A");

    Vector3D< double > pos (0.3525, 0.3525, 0.1025);
    RPY< double > rot (0, 0, 3.14159);
    Transform3D< double > trans (pos, rot.toRotation3D ());
    box2->setTransform (trans, state);
    ur->setQ (Q (6, 1.46819, -1.02748, 2.55523, -3.10998, -1.56556, -0.429299), state);

    CollisionDetector rw_detector (wc);
    printColInfo ("Default Collision Detector", rw_detector, wc, state);

    for (string& id : ProximityStrategyFactory::getCollisionStrategyIDs ()) {
        CollisionDetector::Ptr prox_detector =
            new CollisionDetector (wc, ProximityStrategyFactory::makeCollisionStrategy (id));
        printColInfo (id + " Collision Detector", *prox_detector, wc, state);
    }

    return 0;
}