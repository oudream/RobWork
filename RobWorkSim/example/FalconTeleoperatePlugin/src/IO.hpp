#ifndef IO_HPP
#define IO_HPP

#include <rw/math/Q.hpp>
#include <rw/math/Transform3D.hpp>

#include <fstream>
#include <sstream>
#include <string>
#include <vector>

/**
 * @class Pace
 *
 * @brief Describes the robot & gripper configurations and poses of the objects in the scene at
 * specific time step
 */
class Pace
{
    // time
    double time;
    // robot config
    Q robotConfig;
    // gripper config
    Q gripperConfig;
    // object poses
    std::vector<std::pair<int, rw::math::Transform3D<>>> objectPoses;
    // object contacts {[id id #ofPoints points ...]} TODO

  public:
    /// Default constructor
    Pace();

    /// Constructor
    Pace(double _time, Q _robotConfig, Q _gripperConfig,
         std::vector<std::pair<int, rw::math::Transform3D<>>> _objectPoses);

    friend ostream& operator<<(ostream& os, const Pace& pace);
    friend istream& operator>>(istream& is, Pace& pace);
};

/**
 * @class Reader
 * @brief A class to read paces from an ASCII file.
 */
class Reader
{
    ifstream currentFile;

  public:
    /// Convenience function for reading the whole file at once
    std::vector<Pace*> readAllFile(std::string filename);

    /// Constructor
    Reader(std::string filename);

    /// Opens a file for reading
    bool openFile(std::string filename);

    /// Reads next Pace from opened file
    Pace* readNext();

    /// Closes the file
    void closeFile();
};

class Recorder
{
    std::vector<Pace*> paces;

  public:
    /// Stores another Pace by appending it to a vector
    void addPace(Pace* pace);

    /// Saves stored Paces to a specified file
    void save(std::string filename);
};

#endif
