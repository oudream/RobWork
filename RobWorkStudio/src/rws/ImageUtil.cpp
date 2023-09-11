/********************************************************************************
 * Copyright 2011 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include "ImageUtil.hpp"

#include <rw/core/Ptr.hpp>

#include <QImage>

using namespace rws;
using namespace rw::sensor;
using namespace rw::core;

rw::sensor::Image::Ptr ImageUtil::toRwImage(const QImage& srcimg) {
    if(srcimg.isNull()) return NULL;
    int w = srcimg.width();
    int h = srcimg.height();

    Image::Ptr dstimg;
    if(srcimg.isGrayscale()) {
        // for now we allways use the 16 bit per pixel for gray images
        dstimg = ownedPtr(new Image(w, h, Image::GRAY, Image::Depth16U));
        for(int y = 0; y < h; y++) {
            for(int x = 0; x < w; x++) {
                QRgb rgb = srcimg.pixel(x, y);
                dstimg->setPixel<Image::Depth16U>(x, y, (uint16_t) qRed(rgb));
            }
        }
    }
    else {
        if(srcimg.hasAlphaChannel()) {
            dstimg = ownedPtr(new Image(w, h, Image::RGBA, Image::Depth8U));
            for(int y = 0; y < h; y++) {
                for(int x = 0; x < w; x++) {
                    QRgb rgb = srcimg.pixel(x, y);
                    // dstimg->setPixel<Image::Depth16U>(x, y, qRed(rgb), qGreen(rgb), qBlue(rgb) );
                    dstimg->setPixel8U(x, y, qRed(rgb), qGreen(rgb), qBlue(rgb), qAlpha(rgb));
                }
            }
        }
        else {
            dstimg = ownedPtr(new Image(w, h, Image::RGB, Image::Depth8U));
            for(int y = 0; y < h; y++) {
                for(int x = 0; x < w; x++) {
                    QRgb rgb = srcimg.pixel(x, y);
                    // dstimg->setPixel<Image::Depth16U>(x, y, qRed(rgb), qGreen(rgb), qBlue(rgb) );
                    dstimg->setPixel8U(x, y, qRed(rgb), qGreen(rgb), qBlue(rgb));
                }
            }
        }
    }

    return dstimg;
}

void ImageUtil::toRwImage(const QImage& srcimg, rw::sensor::Image& dstimg) {
    int w = srcimg.width();
    int h = srcimg.height();

    if(srcimg.isGrayscale()) {
        // for now we allways use the 16 bit per pixel for gray images
        dstimg = Image(w, h, Image::GRAY, Image::Depth16U);
        for(int y = 0; y < h; y++) {
            for(int x = 0; x < w; x++) {
                QRgb rgb = srcimg.pixel(x, y);
                dstimg.setPixel<Image::Depth16U>(x, y, (uint16_t) qRed(rgb));
            }
        }
    }
    else {
        if(srcimg.hasAlphaChannel()) {
            dstimg = Image(w, h, Image::RGBA, Image::Depth8U);
            for(int y = 0; y < h; y++) {
                for(int x = 0; x < w; x++) {
                    QRgb rgb = srcimg.pixel(x, y);
                    // dstimg->setPixel<Image::Depth16U>(x, y, qRed(rgb), qGreen(rgb), qBlue(rgb) );
                    dstimg.setPixel8U(x, y, qRed(rgb), qGreen(rgb), qBlue(rgb), qAlpha(rgb));
                }
            }
        }
        else {
            dstimg = Image(w, h, Image::RGB, Image::Depth8U);
            for(int y = 0; y < h; y++) {
                for(int x = 0; x < w; x++) {
                    QRgb rgb = srcimg.pixel(x, y);
                    // dstimg->setPixel<Image::Depth16U>(x, y, qRed(rgb), qGreen(rgb), qBlue(rgb) );
                    dstimg.setPixel8U(x, y, qRed(rgb), qGreen(rgb), qBlue(rgb));
                }
            }
        }
    }
}

QImage* ImageUtil::toQtImage(const Image& srcimg) {
    QImage* qimage = new QImage(srcimg.getWidth(), srcimg.getHeight(), QImage::Format_RGB32);
    if(srcimg.getColorEncoding() == Image::GRAY) {
        for(size_t i = 0; i < srcimg.getWidth(); i++) {
            for(size_t j = 0; j < srcimg.getHeight(); j++) {
                float val = srcimg.getPixelValue(i, j, 0);
                // std::cout << val << " -- " << (((int)(255.0*val))&0xFF) << "\n";
                int value = 0;
                value += 0xff000000;
                value += (((int) (255 * val)) & 0xFF) << 16;
                value += (((int) (255 * val)) & 0xFF) << 8;
                value += (((int) (255.0 * val)) & 0xFF);

                qimage->setPixel((int) i, (int) j, value);
            }
        }
    }
    else {
        for(size_t i = 0; i < srcimg.getWidth(); i++) {
            for(size_t j = 0; j < srcimg.getHeight(); j++) {
                Pixel4f pixel = srcimg.getPixel(i, j);
                int value     = 0;
                value += 0xff000000;
                value += (((int) (255.0 * pixel.ch[0])) & 0xFF) << 16;
                value += (((int) (255.0 * pixel.ch[1])) & 0xFF) << 8;
                value += ((int) (255.0 * pixel.ch[2]) & 0xFF);

                qimage->setPixel((int) i, (int) j, value);
            }
        }
    }
    return qimage;
}
