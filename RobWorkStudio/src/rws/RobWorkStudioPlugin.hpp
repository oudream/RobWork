/********************************************************************************
 * Copyright 2009 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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


#ifndef RWS_ROBWORKSTUDIOPLUGIN_HPP
#define RWS_ROBWORKSTUDIOPLUGIN_HPP

#ifdef __WIN32
#include <windows.h>
#endif

#include <QObject>
#include <QDockWidget>
#include <QMenu>
#include <QToolBar>
#include <QIcon>
#include <QtGui>


#include <rw/RobWork.hpp>
#include <rw/common/Message.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/kinematics/Frame.hpp>
#include <rw/proximity/CollisionDetector.hpp>
#include <rwlibs/drawable/WorkCellGLDrawer.hpp>

#ifdef _MSC_VER
#include <RobWorkStudioConfigVS.hpp>
#else
#include <RobWorkStudioConfig.hpp>
#endif

#include <rw/common/Log.hpp>

#include "Convert.hpp"

namespace rws {
	// forward declaration
	class RobWorkStudio;

	/**
	 * @brief abstract interface for RobWork Studio plugins
	 */
	class RobWorkStudioPlugin: public QDockWidget
	{
		Q_OBJECT

	public:
		/**
		 * @brief constructor of the plugin interface
		 * @param name [in] the name of the plugin
		 * @param icon [in] the icon of the plugin
		 */
		RobWorkStudioPlugin(const QString& name, const QIcon& icon);

		//----------------------------------------------------------------------
		// Virtual methods.

		virtual ~RobWorkStudioPlugin();

		/**
		 * @brief is called when RobWorkStudio instance is valid. Can be used
		 * to initialize values in the plugin that depend on RobWorkStudio
		 */
		virtual void initialize();

		/**
		 * @brief called when a workcell is opened
		 * @param workcell [in] that has been loadet
		 */
		virtual void open(rw::models::WorkCell* workcell);

		/**
		 * @brief called when a workcell is being closed.
		 */
		virtual void close();

		/**
		 * @brief name that describe the plugin instance
		 */
		virtual QString name() const;

		/**
		 * @brief sets up the \b menu with this plugin
		 * @param menu [in] the menu wherein the plugin can add its actions
		 */
		virtual void setupMenu(QMenu* menu);

		/**
		 * @brief setsup a \b toolbar with the actions of this plugin
		 * @param toolbar [in] the toolbar wherein the plugin can add its actions
		 */
		virtual void setupToolBar(QToolBar* toolbar);

		/**
		 * @brief sets a convert object that can be used to downcast devices and frames
		 * from the workcell.
		 * @param convert [in]
		 */
		virtual void setConvert(Convert* convert);

		/**
		 * @brief sets the RobWorkStudio instance of the plugin. Normally
		 * only done on construction.
		 */
		virtual void setRobWorkStudio(RobWorkStudio* studio);

		/**
		 * @brief returns a handle to the RobWorkStudio instance
		 */
		virtual RobWorkStudio* getRobWorkStudio();

		/**
		 * @brief Sets the RobWork instance to be used by the plugin
		 * @param robwork [in] RobWork instance
		 */
		virtual void setRobWorkInstance(rw::RobWorkPtr robwork);

		/**
		 * @brief Returns RobWork instance used by the plugin
		 */
		virtual rw::RobWorkPtr getRobWorkInstance();

		/**
		 * @brief returns the RobWorkStudio log instance
		 */
		virtual rw::common::Log& log();

		/**
		 * @brief Sets the log to use
		 * @param log [in] Pointer to the log to use.
		 */
		virtual void setLog(rw::common::LogPtr log);

	public slots:
		/**
		 * @brief toggles the visibility of the plugin
		 */
		void showPlugin();

	private:

	protected:
		///! @brief The show action
		QAction _showAction;

		///! @brief Name of plugin
		QString _name;

		///! @brief Deprecated
		Convert* _convert;

		///! @brief hook back to RobWorkStudio
		RobWorkStudio* _studio;

		///! @brief The RobWork instance to be used
		rw::RobWorkPtr _robwork;

		///! @brief The log instance to be used
		rw::common::LogPtr _log;
	};

}

Q_DECLARE_INTERFACE(rws::RobWorkStudioPlugin, "dk.sdu.mip.Robwork.RobWorkStudioPlugin/0.1");



#endif //#ifndef ROBWORKSTUDIOPLUGIN_HPP
