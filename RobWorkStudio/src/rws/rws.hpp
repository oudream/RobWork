/*
 * rws.hpp
 *
 *  Created on: Apr 10, 2015
 *      Author: jimali
 */

#ifndef RWS_HPP_
#define RWS_HPP_

#include <rws/AboutBox.hpp>
#include <rws/ArcBallController.hpp>
#include <rws/CameraController.hpp>
#include <rws/FixedAxisController.hpp>
#include <rws/HelpAssistant.hpp>
#include <rws/ImageUtil.hpp>
#include <rws/ImageView.hpp>
#include <rws/RWSImageLoaderPlugin.hpp>
#include <rws/RWStudioView3D.hpp>
#include <rws/RobWorkStudio.hpp>
#include <rws/RobWorkStudioPlugin.hpp>
#include <rws/SceneOpenGLViewer.hpp>
#include <rws/SceneViewerWidget.hpp>
#include <rws/propertyview/PropertyViewDialog.hpp>
#include <rws/propertyview/PropertyViewEditor.hpp>
#include <rws/propertyview/VariantManager.hpp>

#define RWS_USE_RWP_NAMESPACE \
    namespace rws {           \
    }                         \
    namespace rwp {           \
    using namespace rws;      \
    }

#endif /* RWS_HPP_ */
