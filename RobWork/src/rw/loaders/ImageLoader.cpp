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


#include "ImageLoader.hpp"

#include <rw/common/StringUtil.hpp>
#include <rw/loaders/image/PGMLoader.hpp>
#include <rw/loaders/image/RGBLoader.hpp>
#include <rw/loaders/image/PPMLoader.hpp>
#include <rw/common/macros.hpp>
#include <boost/foreach.hpp>
#include <rw/common/Extension.hpp>

using namespace rw::loaders;
using namespace rw::common;
using namespace rw::sensor;


bool ImageLoader::isImageSupported(const std::string& format){
    std::string format1 = StringUtil::toUpper(format);
    std::vector<std::string> formats = getImageFormats();
    BOOST_FOREACH( std::string format_tmp, formats){
    	std::string format2 = StringUtil::toUpper(format_tmp);
    	if(format2==format1)
    		return true;
    }
    return false;
}


rw::common::Ptr<ImageLoader> ImageLoader::Factory::getImageLoader(const std::string& format){
	using namespace rw::common;
	ImageLoader::Factory ep;
	std::vector<Extension::Ptr> exts = ep.getExtensions();
	BOOST_FOREACH(Extension::Ptr ext, exts){
		if(!ext->getProperties().has(format))
			continue;
		// else try casting to ImageLoader
		ImageLoader::Ptr loader = ext->getObject().cast<ImageLoader>();
		return loader;
	}
	RW_THROW("No loader using that format exists...");
	return NULL;
}

bool ImageLoader::Factory::hasImageLoader(const std::string& format){
	using namespace rw::common;
	ImageLoader::Factory ep;
	std::vector<Extension::Descriptor> exts = ep.getExtensionDescriptors();
	BOOST_FOREACH(Extension::Descriptor& ext, exts){
		if(!ext.getProperties().has(format))
			continue;
		return true;
	}
	return false;
}


rw::sensor::Image::Ptr ImageLoader::Factory::load(const std::string& file)
{
    const std::string ext2 = StringUtil::getFileExtension( file);
    const std::string ext = StringUtil::toUpper(ext2);
	if (ext == ".PGM" ){
        return PGMLoader::load( file );
	} else if (ext == ".PPM" ){
	    return PPMLoader::load( file );
	} else if (ext == ".RGB" ){
	    return RGBLoader::load( file );
	//} else if (ext == ".TGA" ){
	    //return TGALoader::load( file );
    //} else if (ext == ".BMP" ){
        //return BMPLoader::load( file );
    } else {
        // tjeck if any plugins support the file format
        //std::cout << "CHECKING PLUGINS" << std::endl;
    	ImageLoader::Ptr loader = getImageLoader(ext);
    	if(loader!=NULL){
			try {
				Image::Ptr img = loader->loadImage( file );
				return img;
			} catch (...){
				Log::debugLog() << "Tried loading image with extension, but failed!\n";
			}
    	}
    }
	RW_THROW("Image file: " << file << " with extension " << ext << " is not supported!");
	return NULL;
}
