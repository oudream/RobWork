#ifndef __DOG_PLUGIN_HPP
#define __DOG_PLUGIN_HPP

#include <rw/core/Plugin.hpp>



/**
 * Defines a plugin that adds Dog species to AnimalFactory.
 */
class DogPlugin: public rw::core::Plugin {
public:

	DogPlugin();
	
	~DogPlugin();
	
	//! Override Plugin::getExtensionDescriptors
	std::vector<rw::core::Extension::Descriptor> getExtensionDescriptors();
	
	//! Override Plugin::makeExtension
	rw::core::Ptr<rw::core::Extension> makeExtension(const std::string& str);
};

#endif
