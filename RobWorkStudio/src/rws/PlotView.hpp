/*****************************************************************************
 * Copyright 2022 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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
 *****************************************************************************/

#ifndef RWS_PLOTVIEW_HPP_
#define RWS_PLOTVIEW_HPP_

/**
 * @file PlotView.hpp
 *
 * \copydoc rws::PlotView
 */

#include <rw/core/Ptr.hpp>
#include <rw/core/ExtensionPoint.hpp>

#include <QGraphicsView>

namespace rws {
//! @addtogroup rws

//! @{
//! @brief GUI Element for showing RobWork plots.
class PlotView
{
    public:
        //! @brief Smart pointer type.
        typedef rw::core::Ptr< PlotView > Ptr;

        //! @brief Constructor.
        PlotView();

        //! @brief Destructor.
        virtual ~PlotView();

        /**
         * @brief Do a ListPlot with the given x- and y-values.
         * @param x [in] the x-values.
         * @param y [in] the y-values.
         * @param title [in] (optional) the plot title.
         * @param xlabel [in] (optional) x axis label.
         * @param ylabel [in] (optional) y axis label.
         */
        virtual void listPlot (const std::vector< double >& x, const std::vector< double >& y,
                       const std::string& title = "", const std::string& xlabel = "",
                       const std::string& ylabel = "") = 0;

        virtual QGraphicsView* getWidget(QWidget* parent) = 0;

        //! @brief Dispatcher for plots.
        class Dispatcher
        {
            protected:
                //! @brief Constructor.
                Dispatcher();

            public:
                //! @brief Smart pointer type.
                typedef rw::core::Ptr< const Dispatcher > Ptr;

                //! @brief Destructor.
                virtual ~Dispatcher();

                /**
                 * @brief Create a new PlotView.
                 *
                 * @return a new PlotView.
                 */
                virtual PlotView::Ptr makePlotView() const = 0;
        };

#if !defined(SWIG)
        /**
         * @addtogroup extensionpoints
         * @extensionpoint{ rws::PlotViewPlugin::Factory,rws::PlotViewPlugin,rws.PlotViewPlugin }
         * \class PlotViewPlugin
         */
#endif
        /**
         * @brief A factory for PlotView::Dispatcher objects. This factory
         * defines an extension point for PointView::Dispatcher (rws.PlotView).
         * Typically this is for plugins that provide widgets capable of
         * showing and interacting with plots.
         * The extensions registered at this extension point must have an
         * extension descriptor with a property called "identifier" giving
         * a unique identifier for that particular type of PlotView.
         */
        class Factory : public rw::core::ExtensionPoint< Dispatcher >
        {
          private:
                Factory ();

          public:
            /**
             * @brief Construct a new PlotView.
             * @param identifier [in] name of the implementation to use.
             * @return a PlotView if PlotView::Dispatcher with identifier is found, nullptr otherwise.
             */
            static rw::core::Ptr< PlotView > makePlotView (const std::string& identifier);

            /**
             * @brief Check if the factory has a specific PlotView::Dispatcher.
             * @param identifier [in] name of the implementation.
             * @return true if implementation was found, false otherwise.
             */
            static bool hasPlotViewDispatcher (const std::string& identifier);

            /**
             * @brief Get a list of identifiers.
             * @return a list of identifiers for PlotView::Dispatcher implementations.
             */
            static std::vector< std::string > getIdentifiers();
        };
};
//! @}

} /* namespace rws */

#endif /* RWS_PLOTVIEW_HPP_ */
