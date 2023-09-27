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

#ifndef RWSIM_CONTACTS_BASECONTACTDETECTOR_HPP_
#define RWSIM_CONTACTS_BASECONTACTDETECTOR_HPP_

/**
 * @file ContactDetector.hpp
 *
 * \copydoc rwsim::contacts::ContactDetector
 */

#include "Contact.hpp"

#include <rw/core/Ptr.hpp>
#include <rw/kinematics/FrameMap.hpp>
#include <rw/proximity/ProximityFilterStrategy.hpp>
#include <rw/proximity/ProximitySetup.hpp>
#include <rw/proximity/ProximitySetupRule.hpp>

namespace rw { namespace models {
    class WorkCell;
}}    // namespace rw::models
namespace rwsim { namespace log {
    class SimulatorLogScope;
}}    // namespace rwsim::log

namespace rwsim { namespace contacts {

    // Forward declarations
    class ContactDetectorData;
    class ContactDetectorTracking;
    class ContactModel;
    class ContactStrategy;

    //! @addtogroup rwsim_contacts

    //! @{
    class BaseContactDetector
    {
      public:
        //! @brief smart pointer type to this class
        typedef rw::core::Ptr<BaseContactDetector> Ptr;

        /**
         * @brief Contact detector for a workcell.
         *
         * If no broad-phase filter is given, a default will be created for the workcell.
         *
         * @param workcell [in] the workcell.
         * @param filter [in] broad-phase filter to remove frames that are obviously not colliding.
         */
        BaseContactDetector(rw::core::Ptr<rw::models::WorkCell> workcell,
                            rw::proximity::ProximityFilterStrategy::Ptr filter = NULL);

        /**
         * @brief Destruct contact detector.
         *
         * The strategy table and stored contact models is cleared.
         */
        virtual ~BaseContactDetector();

        /**
         * @brief Set a new broad-phase filter.
         * @param filter [in] broad-phase filter to remove frames that are obviously not colliding.
         */
        void setProximityFilterStrategy(rw::proximity::ProximityFilterStrategy::Ptr filter);

        /**
         * @brief The broad-phase filter strategy used by the contact detector.
         */
        virtual rw::proximity::ProximityFilterStrategy::Ptr getProximityFilterStrategy() const;

        /**
         * @brief The number of seconds measured used in contact detection.
         * @return the value of the timer in seconds.
         */
        virtual double getTimer() const;

        /**
         * @brief Set the value of a timer that will measure time used during contact detection.
         * @param value [in] the value to set the time to (seconds)
         */
        virtual void setTimer(double value = 0);

        /**
         * @brief Find contacts in workcell.
         *
         * @param state [in] The state for which to check for contacts.
         * @return a vector of contacts, some might be subclasses of the Contact class.
         */
        virtual std::vector<Contact> findContacts(const rw::kinematics::State& state) = 0;

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
                                                  ContactDetectorData& data) = 0;

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
                                                  rwsim::log::SimulatorLogScope* log = NULL) = 0;

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
                                                    rwsim::log::SimulatorLogScope* log = NULL) = 0;


      protected:
        rw::core::Ptr<rw::proximity::ProximityFilterStrategy> _bpFilter;
        rw::core::Ptr<rw::models::WorkCell> _wc;
        double _timer;
    };
    //! @}
}}    // namespace rwsim::contacts
#endif
