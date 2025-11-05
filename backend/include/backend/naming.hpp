#pragma once
#include <string>
#include <algorithm>
#include <cctype>


inline std::string to_topic(std::string s){

    std::transform(s.begin(),s.end(),s.begin(),[](unsigned char c){return std::tolower(c);});

    for(char& c : s)
     if(!std::isalnum(static_cast<unsigned char>(c)))
        c = '_';

    while(!s.empty() && s.front() == '_')s.erase(s.begin());
    while(!s.empty() && s.back() == '_')s.pop_back();
    return s; 

}