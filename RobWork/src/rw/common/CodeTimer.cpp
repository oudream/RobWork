#include <rw/common/CodeTimer.hpp>

#include <iomanip>
#include <iostream>
#include <vector>

using namespace rw::common;
using namespace std::chrono;

size_t rw::common::CodeTimer::level = 0;
std::map<std::string, CodeTimer::CodeTimerData> rw::common::CodeTimer::statistics;
std::string rw::common::CodeTimer::current = "";
bool rw::common::CodeTimer::doPrintOuts    = false;

CodeTimer::CodeTimer(std::string name) : _name(name), _parent(CodeTimer::current), _stopped(false) {
    CodeTimer::current = _name;
    CodeTimer::level++;
    _start = high_resolution_clock::now();
}

CodeTimer::~CodeTimer() {
    stop();
}

void CodeTimer::stop() {
    high_resolution_clock::time_point end = high_resolution_clock::now();
    if(!_stopped) {
        auto duration = duration_cast<microseconds>(end - _start);
        if(CodeTimer::doPrintOuts) {
            for(size_t i = 0; i < CodeTimer::level; i++) { std::cout << "  "; }
            std::cout << _name << ": " << duration.count() << " µs" << std::endl;
        }
        CodeTimer::level--;
        CodeTimer::current  = _parent;
        _stopped            = true;
        CodeTimerData& data = CodeTimer::statistics[_name];
        data.parent         = _parent;
        data.duration += duration.count();
        data.calls++;
        auto waste = duration_cast<microseconds>(high_resolution_clock::now() - end);
        data.waste += waste.count();
    }
}

void CodeTimer::operator()(std::string newName) {
    stop();
    _stopped           = false;
    _name              = newName;
    CodeTimer::current = _name;
    CodeTimer::level++;
    _start = high_resolution_clock::now();
}

void CodeTimer::stopAndStartWithName(std::string name) {
    stop();
    _name    = name;
    _stopped = false;
    _start   = high_resolution_clock::now();
}

unsigned long rw::common::CodeTimer::collectWaste(ParentMap& data, std::string current) {
    unsigned long waste                                                  = 0;
    std::vector<std::pair<std::string, CodeTimer::CodeTimerData>> c_data = data[current];
    for(std::pair<std::string, CodeTimer::CodeTimerData>& d : c_data) {
        waste += d.second.waste;
        waste += CodeTimer::collectWaste(data, d.first);
    }
    return waste;
}

void rw::common::CodeTimer::doRepport(size_t level, ParentMap& data, std::string current) {
    std::vector<std::pair<std::string, CodeTimer::CodeTimerData>> c_data = data[current];
    for(std::pair<std::string, CodeTimer::CodeTimerData>& d : c_data) {
        std::string name, time;
        for(size_t i = 0; i < level; i++) { name += "  "; }
        name += d.first + ": ";
        unsigned long waste = CodeTimer::collectWaste(data, d.first);

        time =
            " " +
            std::to_string((waste < d.second.duration ? double(d.second.duration - waste) : 0.0) /
                           d.second.calls) +
            " µs";

        std::cout << std::setw(35) << name << std::setw(20) << time << std::setw(15)
                  << (std::to_string(d.second.duration) + " µs") << d.second.calls << std::endl;
        doRepport(level + 1, data, d.first);
    }
}

void rw::common::CodeTimer::getRepport() {
    ParentMap dataMap;
    std::cout << std::left << std::setprecision(3);
    std::cout << std::setw(35) << "Name: " << std::setw(20) << "avg Time" << std::setw(20)
              << "Total Time:"
              << "Called: " << std::endl;
    for(auto const& values : CodeTimer::statistics) {
        dataMap[values.second.parent].push_back(values);
    }
    doRepport(0, dataMap, "");

    std::cout << std::setprecision(6);
}