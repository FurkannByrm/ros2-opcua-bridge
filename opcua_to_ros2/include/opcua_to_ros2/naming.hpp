#pragma once
#include <string>
#include <cctype>

inline std::string make_child_node(const std::string &root, const std::string &field) {
    if (root.empty()) {
        return field; 
    }
    return root + ".\"" + field + "\"";
}

inline std::string make_weld_node(const std::string &root, const std::string &field) {
    if (root.empty()) {
        return field;
    }
    return root + "." + field;
}
