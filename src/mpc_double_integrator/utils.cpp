#include "utils.h"

std::string GetAbsolutePath(const std::string &path) {
    std::string abs_path = path;
    if (path[0] != '/') {
        std::string current_dir(__FILE__);
        current_dir.erase(current_dir.rfind('/'));
        abs_path = current_dir + "/" + path;
    }
    return abs_path;
}