/********************************************************************************
 * Copyright 2013 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
 * Faculty of Engineering, University of Southern Denmark
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************************************************************************/

#ifndef RWSIM_CONTACTS_THREADEDCONTACTDETECTOR_HPP_
#define RWSIM_CONTACTS_THREADEDCONTACTDETECTOR_HPP_

#include <rw/core/Ptr.hpp>
#include <rw/kinematics/FKTable.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/proximity/ProximityStrategyData.hpp>

#include <rwsim/contacts/BaseContactDetector.hpp>
#include <rwsim/dynamics/DynamicWorkCell.hpp>

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>

namespace rwsim { namespace contacts {

    class ThreadedContactDetector : public BaseContactDetector
    {
      public:
        typedef rw::core::Ptr<ThreadedContactDetector> Ptr;

        ThreadedContactDetector(
            rw::core::Ptr<rwsim::dynamics::DynamicWorkCell> dwc, int threads = -1,
            rw::core::Ptr<rw::proximity::ProximityFilterStrategy> filter = NULL);

        ~ThreadedContactDetector();

        /**
         * @brief Find contacts in workcell.
         *
         * @param state [in] The state for which to check for contacts.
         * @return a vector of contacts, some might be subclasses of the Contact class.
         */
        virtual std::vector<Contact> findContacts(const rw::kinematics::State& state);

        /**
         * @brief Find contacts in workcell.
         *
         * Use of this function is encouraged if changes between consecutive calls are expected to
         * be small. This will allow the detection algorithms to do certain speed-ups.
         *
         * @param state [in] The state for which to check for contacts.
         * @param data [in/out] Allows caching between contact detection calls,
         * and makes it possible for detection algorithms to exploit spatial and temporal coherence.
         * @return a vector of contacts, some might be subclasses of the Contact class.
         */
        virtual std::vector<Contact> findContacts(const rw::kinematics::State& state,
                                                  ContactDetectorData& data);

        /**
         * @brief Find contacts in workcell while tracking known contacts.
         *
         * @param state [in] the state to find contacts for.
         * @param data [in/out] allows caching between contact detection calls,
         * and makes it possible for detection algorithms to exploit spatial and temporal coherence.
         * @param tracking [in/out] the tracking data with information about known contacts.
         * @param log [in/out] (optional) store detailed logging information.
         * @return a vector of new contacts.
         */
        virtual std::vector<Contact> findContacts(const rw::kinematics::State& state,
                                                  ContactDetectorData& data,
                                                  ContactDetectorTracking& tracking,
                                                  rwsim::log::SimulatorLogScope* log = NULL);

        /**
         * @brief Updates previously found contacts.
         *
         * @param state [in] the new state to find the updated contacts for.
         * @param data [in/out] allows caching between contact detection calls,
         * and makes it possible for detection algorithms to exploit spatial and temporal coherence.
         * @param tracking [in/out] the tracking data with information about known contacts.
         * @param log [in/out] (optional) store detailed logging information.
         * @return a vector of contacts.
         */
        virtual std::vector<Contact> updateContacts(const rw::kinematics::State& state,
                                                    ContactDetectorData& data,
                                                    ContactDetectorTracking& tracking,
                                                    rwsim::log::SimulatorLogScope* log = NULL);

      private:
        struct MultiDistResult
        {
            rw::proximity::ProximityStrategyData data;
            rw::proximity::DistanceMultiStrategy::Ptr strat;
            rw::math::Transform3D<double> wTa;
            rw::math::Transform3D<double> wTb;
        };

        struct MultiDistJob
        {
            MultiDistJob(rw::proximity::ProximityModel::Ptr& modelA,
                         rw::proximity::ProximityModel::Ptr& modelB, rw::kinematics::Frame* A,
                         rw::kinematics::Frame* B) :
                modelA(modelA),
                modelB(modelB), A(A), B(B) {}
            MultiDistJob() {}

            rw::proximity::ProximityModel::Ptr modelA;
            rw::proximity::ProximityModel::Ptr modelB;
            rw::kinematics::Frame* A;
            rw::kinematics::Frame* B;
        };

        void addContacts(std::vector<Contact>& contacts, MultiDistResult& res);

        using FramePair = std::pair<rw::kinematics::Frame*, rw::kinematics::Frame*>;
        using ProxModelPtrPair =
            std::pair<rw::proximity::ProximityModel::Ptr, rw::proximity::ProximityModel::Ptr>;
        using ProxModelPair =
            std::pair<rw::proximity::ProximityModel*, rw::proximity::ProximityModel*>;

        rwsim::dynamics::DynamicWorkCell::Ptr _dwc;
        std::map<ProxModelPair, rw::proximity::DistanceMultiStrategy::Ptr> _distStrategies;
        std::map<rw::kinematics::Frame*, rw::proximity::ProximityModel::Ptr> _frame2models;
        rw::proximity::DistanceMultiStrategy::Ptr _narrowStrat;

        size_t _models;

        // ##################
        // # Threads
        // ##################

        std::mutex _startLock;

        bool _kill;
        std::mutex _jobLock;
        std::atomic<int> _jobs;
        std::atomic<int> _jobId;
        std::vector<MultiDistJob>* _jobList;

        std::atomic<size_t> _activeRunners;
        std::atomic<bool> _activateDone;
        std::atomic<bool> _postingJobs;
        double _maxSepDist;

        std::vector<std::thread> _tPool;
        rw::kinematics::FKTable* _table;
        std::vector<std::vector<Contact>>* _contacts;

        void threadRunner();
    };
}}    // namespace rwsim::contacts

#endif
