#pragma once
#include <string>
#include <algorithm>
#include <cctype>

inline std::string make_child_node(const std::string &root, const std::string &field) {
  return root + ".\"" + field + "\"";
}