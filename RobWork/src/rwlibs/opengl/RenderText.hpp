#ifndef RWLIBS_OPENGL_RENDERTEXT_HPP
#define RWLIBS_OPENGL_RENDERTEXT_HPP

/**
 * @file RenderText.hpp
 */

#include <rwlibs/os/rwgl.hpp>
#include <rwlibs/opengl/SceneOpenGL.hpp>

#include <rw/graphics/Render.hpp>

#include <RobWorkConfig.hpp>

#include <string>

namespace rwlibs { namespace opengl {

    class RenderText: public rw::graphics::Render
    {
    public:

        /**
         * @brief Constructs a RenderText
         * @param text [in] the text to be rendered
         */
        RenderText(std::string text);
        
        //! @copydoc rw::graphics::Render::draw(const DrawableNode::RenderInfo& info, DrawableNode::DrawType type, double alpha) const
        void draw(const rw::graphics::DrawableNode::RenderInfo& info,
                  rw::graphics::DrawableNode::DrawType type,
                  double alpha) const;
        
    private:

        std::string _text;
        SceneOpenGL::Ptr _scene;
    
    };

}}
#endif // end include guard