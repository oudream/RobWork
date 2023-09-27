/*****************************************************************************
 * Copyright 2021 The Robotics Group, The Maersk Mc-Kinney Moller Institute,
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

#include <rw/core/os.hpp>

#include <boost/filesystem.hpp>

#ifdef RW_WIN
#include <windows.h>
#endif

std::vector<std::string> OS::InstallPluginLocation(std::string pack) {
#if defined(RW_WIN)
    HKEY hKey          = 0;
    char buf[1024]     = {0};
    DWORD dwType       = 0;
    DWORD dwBufSize    = sizeof(buf);
    std::string subkey = "Software\\Kitware\\CMake\\Packages\\" + pack;
    if(RegOpenKey(HKEY_CURRENT_USER, subkey.c_str(), &hKey) == ERROR_SUCCESS) {
        dwType = REG_SZ;
        if(RegQueryValueEx(hKey, "Location", 0, &dwType, (BYTE*) buf, &dwBufSize) ==
           ERROR_SUCCESS) {
            return {std::string(buf) + "\\lib\\RobWork\\rwplugins"};
        }
        RegCloseKey(hKey);
    }
    return {std::string()};
#else
    std::vector<std::string> rwpluginFolders;

    if(boost::filesystem::exists("/usr/lib/")) {    // Add default plugin location

        boost::filesystem::path p("/usr/lib");
        // Find the architecture dependendt folder containing the rwplugins folder
        // Search all files and folders
        for(boost::filesystem::directory_iterator i(p);
            i != boost::filesystem::directory_iterator();
            i++) {
            // If is directory
            if(boost::filesystem::is_directory(i->path())) {
                std::string rwpluginFolder = "/usr/lib/";
                rwpluginFolder += i->path().filename().string();
                rwpluginFolder += "/RobWork/rwplugins";
                if(boost::filesystem::exists(rwpluginFolder)) {
                    rwpluginFolders.push_back(rwpluginFolder);
                    break;
                }
            }
        }
    }
    if(boost::filesystem::exists(
           "/usr/local/lib/RobWork/rwplugins")) {    // Add default plugin location
        rwpluginFolders.push_back("/usr/local/lib/RobWork/rwplugins");
    }

    return rwpluginFolders;

#endif
}

#if defined(RW_WIN)
std::wstring s2ws(const std::string& s) {
    int len;
    int slength  = (int) s.length() + 1;
    len          = MultiByteToWideChar(CP_ACP, 0, s.c_str(), slength, 0, 0);
    wchar_t* buf = new wchar_t[len];
    MultiByteToWideChar(CP_ACP, 0, s.c_str(), slength, buf, len);
    std::wstring r(buf);
    delete[] buf;
    return r;
}
#endif
