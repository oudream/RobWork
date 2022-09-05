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

#ifndef RW_COMMON_CODETIMER_HPP_
#define RW_COMMON_CODETIMER_HPP_

#include <chrono>
#include <map>
#include <string>
#include <vector>

namespace rw { namespace common {
    /**
     * @brief This class can be used to test how much time is used when executing a part of the
     * code.
     *
     */
    class CodeTimer
    {
      public:
        /**
         * @brief Register a new code time ounde the name \b name. if \b name already exists then
         * add the time from this execution to the a label and increment the amount of times this
         * label has been called
         * 
         * OBS this code is not multthread friendly, so only create instances of this class in a single thread
         *
         * @param name name of the code label
         */
        CodeTimer (std::string name = "CodeTimer");

        /**
         * @brief calls stop();
         */
        virtual ~CodeTimer ();

        /**
         * @brief Stops the time and starts it under a new label
         * 
         * @param newName new label
         */
        void operator()(std::string newName);

        /**
         * @brief stop the code timer. if Print outs enabled print the time, else store the result in the static repport.
         */
        void stop ();

        /**
         * @brief Get the Repport object
         */
        static void getRepport ();

        /**
         * @brief enable or disable printouts when stop is called on a global level
         * @param enable 
         */
        static void enablePrintOuts (bool enable);

      private:
        std::string _name;
        std::string _parent;
        bool _stopped;

        std::chrono::high_resolution_clock::time_point _start;

        struct CodeTimerData
        {
            unsigned long duration;    //!< Total execution time for this timer
            unsigned long calls;       //!< Number of times this timer has been called
            unsigned long waste;       //!< Aprox Time this timer has wasted
            std::string parent;        //!< Name of Parent Timer
        };
        using ParentMap =
            std::map< std::string,
                      std::vector< std::pair< std::string, CodeTimer::CodeTimerData > > >;

        static bool doPrintOuts;
        static size_t level;
        static std::string current;
        static std::map< std::string, CodeTimerData > statistics;

        static void doRepport (size_t level, ParentMap& data, std::string current);
        static unsigned long collectWaste (ParentMap& data, std::string current);
    };
}}    // namespace rw::common

#endif