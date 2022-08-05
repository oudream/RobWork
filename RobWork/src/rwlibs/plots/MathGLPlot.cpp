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

#include "MathGLPlot.hpp"

#include <rw/sensor/Image.hpp>

#include <mgl2/mgl.h>

using rw::core::ownedPtr;
using rw::graphics::Plot;
using rw::sensor::Image;
using rwlibs::plots::MathGLPlot;

struct MathGLPlot::PlotData {
    mglData x;
    mglData y;
    std::string title;
    std::string xlabel;
    std::string ylabel;
};

MathGLPlot::MathGLPlot():
    Plot(),
    _data(new PlotData())
{
}

MathGLPlot::~MathGLPlot()
{
    delete _data;
}

void MathGLPlot::listPlot (const std::vector< double >& x, const std::vector< double >& y,
        const std::string& title, const std::string& xlabel,
        const std::string& ylabel)
{
    _data->x = mglData(x);
    _data->y = mglData(y);
    _data->title = title;
    _data->xlabel = xlabel;
    _data->ylabel = ylabel;
}

Image::Ptr MathGLPlot::render(unsigned int width, unsigned int height)
{
    mglGraph gr(0, width, height);

    gr.Alpha(true);
    gr.Light(true);
    gr.AddLight(0,mglPoint(1,0,-1));
    gr.Title(_data->title.c_str());
    // gr.FPlot("sin(pi*x)");

    mglData xd(_data->x);
    gr.SetRanges(_data->x.Minimal(), _data->x.Maximal(), _data->y.Minimal(), _data->y.Maximal());
    gr.Mark(_data->x, _data->y, mglData(_data->x.nx, 0.25), "rs");

    gr.Box();
    gr.Label('x', _data->xlabel.c_str(), 0);
    gr.Label('y', _data->ylabel.c_str(), 0);
    gr.Axis("!");

    const unsigned char * const data = gr.GetRGB();
    const Image::Ptr image = ownedPtr(new Image(width, height, Image::RGB, Image::Depth8U));
    for (unsigned int i = 0; i < width; i += 1) {
        for (unsigned int j = 0; j < height; j += 1) {
            const int idx = 3*i + 3*width*j;
            image->setPixel8U(i, j, data[idx + 0], data[idx + 1], data[idx + 2]);
        }
    }
    return image;
}
