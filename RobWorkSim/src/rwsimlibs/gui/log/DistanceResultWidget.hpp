/******************************************************************************
 * Copyright 2019 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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
 ******************************************************************************/

#ifndef RWSIMLIBS_GUI_LOG_DISTANCERESULTWIDGET_HPP_
#define RWSIMLIBS_GUI_LOG_DISTANCERESULTWIDGET_HPP_

/**
 * @file DistanceResultWidget.hpp
 *
 * \copydoc rwsimlibs::gui::DistanceResultWidget
 */

#include "SimulatorLogEntryWidget.hpp"

namespace rwsim { namespace log {
    class LogDistanceResult;
}}    // namespace rwsim::log
namespace rwsim { namespace log {
    class LogPositions;
}}    // namespace rwsim::log

namespace Ui {
class DistanceResultWidget;
}

class QItemSelection;

namespace rwsimlibs { namespace gui {

    //! @addtogroup rwsimlibs_gui

    //! @{
    /**
     * @brief Graphical representation of the log entry
     * rwsim::log::DistanceResultWidget.
     */
    class DistanceResultWidget : public SimulatorLogEntryWidget
    {
        Q_OBJECT
      public:
        /**
         * @brief Construct new widget for a log entry.
         * @param entry [in] a distance result entry.
         * @param parent [in] (optional) the parent Qt widget.
         * Ownership is shared by the caller and the parent widget if given.
         */
        DistanceResultWidget (rw::core::Ptr< const rwsim::log::LogDistanceResult > entry,
                              QWidget* parent = 0);

        //! @brief Destructor.
        virtual ~DistanceResultWidget ();

        //! @copydoc SimulatorLogEntryWidget::setDWC
        virtual void setDWC (rw::core::Ptr< const rwsim::dynamics::DynamicWorkCell > dwc);

        //! @copydoc SimulatorLogEntryWidget::setEntry
        virtual void setEntry (rw::core::Ptr< const rwsim::log::SimulatorLog > entry);

        //! @copydoc SimulatorLogEntryWidget::getEntry
        virtual rw::core::Ptr<const rwsim::log::SimulatorLog> getEntry() const;

        //! @copydoc SimulatorLogEntryWidget::updateEntryWidget
        virtual void updateEntryWidget ();

        //! @copydoc SimulatorLogEntryWidget::showGraphics
        virtual void showGraphics (rw::core::Ptr< rw::graphics::GroupNode > root,
                                   rw::core::Ptr< rw::graphics::SceneGraph > graph);

        //! @copydoc SimulatorLogEntryWidget::getName
        virtual std::string getName () const;

        //! @copydoc SimulatorLogEntryWidget::Dispatcher
        class Dispatcher : public SimulatorLogEntryWidget::Dispatcher
        {
          public:
            //! @brief Constructor.
            Dispatcher ();

            //! @brief Destructor.
            virtual ~Dispatcher ();

            //! @copydoc SimulatorLogEntryWidget::Dispatcher::makeWidget
            SimulatorLogEntryWidget*
            makeWidget (rw::core::Ptr< const rwsim::log::SimulatorLog > entry,
                        QWidget* parent = 0) const;

            //! @copydoc SimulatorLogEntryWidget::Dispatcher::accepts
            bool accepts (rw::core::Ptr< const rwsim::log::SimulatorLog > entry) const;
        };

      private slots:
        void framePairsChanged (const QItemSelection& newSelection,
                                const QItemSelection& oldSelection);

      private:
        Ui::DistanceResultWidget* const _ui;
        rw::core::Ptr< const rwsim::dynamics::DynamicWorkCell > _dwc;
        rw::core::Ptr< const rwsim::log::LogPositions > _positions;
        rw::core::Ptr< const rwsim::log::LogDistanceResult > _result;
        rw::core::Ptr< rw::graphics::GroupNode > _root;
        rw::core::Ptr< rw::graphics::SceneGraph > _graph;
    };
    //! @}

}}    // namespace rwsimlibs::gui

#endif /* RWSIMLIBS_GUI_LOG_DISTANCERESULTWIDGET_HPP_ */
