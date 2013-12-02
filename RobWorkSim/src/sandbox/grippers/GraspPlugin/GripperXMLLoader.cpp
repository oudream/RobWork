#include "GripperXMLLoader.hpp"

#include <string>
#include <sstream>
#include <rw/rw.hpp>
#include <rwsim/rwsim.hpp>
#include <iostream>
#include <string>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/optional.hpp>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/filesystem/path.hpp>
#include <rw/loaders/model3d/STLFile.hpp>
#include "Gripper.hpp"
#include "XMLHelpers.hpp"

#define DEBUG cout



using namespace std;
USE_ROBWORK_NAMESPACE;
using namespace robwork;
using namespace rwsim;
using namespace rw::loaders;
using namespace boost::numeric;
using namespace boost::property_tree;
using namespace rwlibs::xml;



/*JawPrimitive::Ptr readJaw(PTree& tree)
{
	Q params = XMLHelpers::readQ(tree);
	
	params[5] *= Deg2Rad;
	if (params[0] == JawPrimitive::Prismatic) params[8] *= Deg2Rad;
	
	cout << params << endl;
	
	return ownedPtr(new JawPrimitive(params));
}*/



void readJaws(PTree& tree, Gripper::Ptr gripper)
{
	boost::optional<PTree&> fileNode = tree.get_child_optional("File");
	if (fileNode) {
		// read jaw geometry from STL file
		string filename = (*fileNode).get_value<string>();
		DEBUG << "Jaw geometry from file: " << filename << endl;
		
		TriMesh::Ptr mesh = STLFile::load(filename);
		gripper->setJawGeometry(ownedPtr(new Geometry(mesh)));
		
		return;
	}
	
	// else use parametrization
	Q params = XMLHelpers::readQ(tree.get_child("Q"));
	params(5) *= Deg2Rad;
	params(8) *= Deg2Rad;
	//cout << "!" << params << endl;
	DEBUG << "Jaw geometry from parameters: " << params << endl;
	gripper->setJawGeometry(params);
}



void readBase(PTree& tree, Gripper::Ptr gripper)
{
	boost::optional<PTree&> fileNode = tree.get_child_optional("File");
	if (fileNode) {
		// read base geometry from STL file
		string filename = (*fileNode).get_value<string>();
		DEBUG << "Base geometry from file: " << filename << endl;
		
		TriMesh::Ptr mesh = STLFile::load(filename);
		gripper->setBaseGeometry(ownedPtr(new Geometry(mesh)));
		
		return;
	}
	
	// else use parametrization
	Q params = XMLHelpers::readQ(tree.get_child("Q"));
	DEBUG << "Base geometry from parameters: " << params << endl;
	gripper->setBaseGeometry(params);
}



void readGeometry(PTree& tree, Gripper::Ptr gripper)
{
	readJaws(tree.get_child("Jaws"), gripper);
	readBase(tree.get_child("Base"), gripper);
}



void readParameters(PTree& tree, Gripper::Ptr gripper)
{
	readGeometry(tree.get_child("Geometry"), gripper);
	
	double offset = XMLHelpers::readDouble(tree.get_child("Offset"));
	gripper->setTCP(Transform3D<>(Vector3D<>(0, 0, offset)));
	
	gripper->setJawdist(XMLHelpers::readDouble(tree.get_child("Jawdist")));
	gripper->setOpening(XMLHelpers::readDouble(tree.get_child("Opening")));
	gripper->setForce(XMLHelpers::readDouble(tree.get_child("Force")));
}



void readResult(PTree& tree, Gripper::Ptr gripper)
{
	GripperQuality::Ptr result = gripper->getQuality();
	
	result->nOfExperiments = XMLHelpers::readInt(tree.get_child("Experiments"));
	result->nOfSuccesses = XMLHelpers::readInt(tree.get_child("Successes"));
	result->nOfSamples = XMLHelpers::readInt(tree.get_child("Samples"));
	//result->shape = XMLHelpers::readDouble(tree.get_child("Shape"));
	result->coverage = XMLHelpers::readDouble(tree.get_child("Coverage"));
	result->success = XMLHelpers::readDouble(tree.get_child("SuccessRatio"));
	result->wrench = XMLHelpers::readDouble(tree.get_child("Wrench"));
	result->topwrench = XMLHelpers::readDouble(tree.get_child("TopWrench"));
	result->quality = XMLHelpers::readDouble(tree.get_child("Quality"));
	
	DEBUG << "Read gripper quality:" << endl;
	DEBUG << *result << endl;
}



Gripper::Ptr readGripper(PTree& tree)
{
	Gripper::Ptr gripper = ownedPtr(new Gripper);
	
	readParameters(tree.get_child("Parameters"), gripper);
	readResult(tree.get_child("Result"), gripper);
	
	return gripper;
}



rw::models::Gripper::Ptr GripperXMLLoader::load(const std::string& filename)
{
	Gripper::Ptr gripper;
	
    try {
        PTree tree;
        read_xml(filename, tree);

        gripper = readGripper(tree.get_child("Gripper"));
        string name = tree.get_child("Gripper").get_child("<xmlattr>.name").get_value<string>();
        gripper->setName(name);        
    } catch (const ptree_error& e) {
        // Convert from parse errors to RobWork errors.
        RW_THROW(e.what());
    }
    
    return gripper;
}



void GripperXMLLoader::save(rw::models::Gripper::Ptr gripper, const std::string& filename)
{
	PTree tree;
	
	tree.put("Gripper.<xmlattr>.name", gripper->getName());
	
	// save jaw geometry
	if (gripper->isJawParametrized()) {
		Q params = gripper->getJawParameters();
		params(5) *= Rad2Deg;
		params(8) *= Rad2Deg;
		tree.put("Gripper.Parameters.Geometry.Jaws.Q", XMLHelpers::QToString(params));
	} else {
		// save STL file
		boost::filesystem::path p(filename);
		string stlfile = p.parent_path().string()+"/jaw.stl";
		STLFile::save(*gripper->_leftGeometry->getGeometryData()->getTriMesh(), stlfile);
		tree.put("Gripper.Parameters.Geometry.Jaws.File", filename);
	}
	
	// save base geometry
	if (gripper->isBaseParametrized()) {
		tree.put("Gripper.Parameters.Geometry.Base.Q", XMLHelpers::QToString(gripper->getBaseParameters()));
	} else {
		// save STL file
		boost::filesystem::path p(filename);
		string stlfile = p.parent_path().string()+"/base.stl";
		STLFile::save(*gripper->_baseGeometry->getGeometryData()->getTriMesh(), stlfile);
		tree.put("Gripper.Parameters.Geometry.Base.File", filename);
	}
	
	tree.put("Gripper.Parameters.Offset", gripper->getTCP().P()[2]);
	tree.put("Gripper.Parameters.Jawdist", gripper->getJawdist());
	tree.put("Gripper.Parameters.Opening", gripper->getOpening());
	tree.put("Gripper.Parameters.Force", gripper->getForce());
	
	GripperQuality::Ptr q = gripper->getQuality();
	tree.put("Gripper.Result.Experiments", q->nOfExperiments);
	tree.put("Gripper.Result.Successes", q->nOfSuccesses);
	tree.put("Gripper.Result.Samples", q->nOfSamples);
	//tree.put("Gripper.Result.Shape", q->shape);
	tree.put("Gripper.Result.Coverage", q->coverage);
	tree.put("Gripper.Result.SuccessRatio", q->success);
	tree.put("Gripper.Result.Wrench", q->wrench);
	tree.put("Gripper.Result.TopWrench", q->topwrench);
	tree.put("Gripper.Result.Quality", q->quality);
	
	try {
		boost::property_tree::xml_writer_settings<char> settings('\t', 1);
        write_xml(filename, tree, std::locale(), settings);
    } catch (const ptree_error& e) {
        // Convert from parse errors to RobWork errors.
        RW_THROW(e.what());
    }
}
