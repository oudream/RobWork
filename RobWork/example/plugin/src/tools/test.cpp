#include <path.hpp>

#include <iostream>
#include <rw/core/macros.hpp>
#include <rw/core/Plugin.hpp>
#include <rw/core/RobWork.hpp>

#include <animals.hpp>


using namespace std;
using namespace animals;
using namespace rw::core;



int main(int argc, char* argv[]) {
	/*
	 * Initialize RobWork and let it search for plugins.
	 */
	RobWork::getInstance()->initialize(std::vector<std::string>(1,DOG_PLUGIN_PATH));
	
	/*
	 * List all the plugins used.
	 */
	ExtensionRegistry::Ptr extReg = ExtensionRegistry::getInstance();
	vector<Ptr<Plugin> > plugins = extReg->getPlugins();
	
	std::cout << "List of plugins: " << std::endl;
	for(Ptr<Plugin>& plugin : plugins) {
		std::cout << " + " << plugin->getName() << std::endl;
	}
	
	/*
	 * Construct new Cat, which is available statically.
	 */
	Animal::Ptr cat = AnimalFactory::getAnimal("cat");
	cat->makeSound();
	
	/*
	 * Construct new Dog. This will only work if the dog plugin is compiled and found.
	 * Look up RobWork .cfg.xml file to see where to put the dog plugin .so.
	 */
	Animal::Ptr dog = AnimalFactory::getAnimal("dog");
	if (dog.isNull())
		RW_THROW("Plugin was not found!");
	dog->makeSound();
	
	return 0;
}
