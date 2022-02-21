#include <rw/core/os.hpp>

std::string OS::InstallPluginLocation (std::string pack)
{
#if defined(RW_WIN)
    HKEY hKey          = 0;
    char buf[1024]     = {0};
    DWORD dwType       = 0;
    DWORD dwBufSize    = sizeof (buf);
    std::string subkey = "Software\\Kitware\\CMake\\Packages\\" + pack;
    if (RegOpenKey (HKEY_CURRENT_USER, subkey.c_str (), &hKey) == ERROR_SUCCESS) {
        dwType = REG_SZ;
        if (RegQueryValueEx (hKey, "Location", 0, &dwType, (BYTE*) buf, &dwBufSize) ==
            ERROR_SUCCESS) {
            return std::string (buf) + "\\lib\\RobWork\\rwplugins";
        }
        RegCloseKey (hKey);
    }
    return std::string ();
#else
    if (boost::filesystem::exists ("/usr/lib/")) {    // Add default plugin location

        boost::filesystem::path p ("/usr/lib");
        std::string rwpluginFolder = "";

        // Find the architecture dependendt folder containing the rwplugins folder
        // Search all files and folders
        for (boost::filesystem::directory_iterator i (p);
             i != boost::filesystem::directory_iterator ();
             i++) {
            // If is directory
            if (boost::filesystem::is_directory (i->path ())) {
                rwpluginFolder = "/usr/lib/";
                rwpluginFolder += i->path ().filename ().string ();
                rwpluginFolder += "/RobWork/rwplugins";
                if (boost::filesystem::exists (rwpluginFolder)) {
                    break;
                }
                else {
                    rwpluginFolder = "";
                }
            }
        }
        return rwpluginFolder;
    }
    return "";

#endif
}

#if defined(RW_WIN)
std::wstring s2ws (const std::string& s)
{
    int len;
    int slength  = (int) s.length () + 1;
    len          = MultiByteToWideChar (CP_ACP, 0, s.c_str (), slength, 0, 0);
    wchar_t* buf = new wchar_t[len];
    MultiByteToWideChar (CP_ACP, 0, s.c_str (), slength, buf, len);
    std::wstring r (buf);
    delete[] buf;
    return r;
}
#endif