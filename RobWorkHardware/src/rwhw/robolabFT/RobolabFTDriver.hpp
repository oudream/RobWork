/*
 * robotlabFT.hpp
 *
 *  Created on: Nov 18, 2013
 *      Author: trs
 */

#ifndef ROBOTLABFT_HPP_
#define ROBOTLABFT_HPP_

#include "ConvertUtil.hpp"

#include <rw/core/Ptr.hpp>
#include <rw/math/Vector3D.hpp>
#include <rwhw/serialport/SerialPort.hpp>

// Boost
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <cstdlib>
#include <cstring>
#include <stdio.h>

namespace rwhw {
typedef std::pair< rw::math::Vector3D<>, rw::math::Vector3D<> > Wrench3D;

class RobolabFT
{
  public:
    typedef rw::core::Ptr< RobolabFT > Ptr;
    struct RobolabFTData
    {
        // F/T data: {Fx, Fy, Fz, Tx, Ty, Tz}
        Wrench3D data;

        double timestamp;
    };
    RobolabFT ();
    ~RobolabFT ();
    RobolabFTData read ();
    bool write ();

    bool init (const std::string& port, SerialPort::Baudrate baudrate, int sensors);
    void run ();
    void stop ();

  private:
    bool updateData ();
    bool connect (const std::string& port, SerialPort::Baudrate baudrate);
    SerialPort _serialPort;
    char* _dataIn;

    bool _isRunning;
    Wrench3D _data;
    char* _dataOut;
    // Thread function
    void runReceive ();
    double _timestamp;
    int _sensors;

    bool _dataInit;    // if _dataIn and _dataOut is initialized

    // Thread members
    boost::thread _receiveThread;
    bool _threadRunning, _stopThread;
    boost::mutex _mutex;
};
}    // namespace rwhw

#endif /* ROBOTLABFT_HPP_ */
