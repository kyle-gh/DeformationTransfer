//
//  Correspondence.cpp
//  Deform
//
//  Created by Kyle on 12/8/18.
//  Copyright © 2018 Kyle. All rights reserved.
//

#include "Correspondence.h"

#include <iostream>
#include <fstream>

std::vector<std::string> splitLine(const std::string& line, char sep = ',')
{
    std::vector<std::string> parts;
    
    std::string::size_type prev_pos = 0, pos = 0;
    
    while((pos = line.find(sep, pos)) != std::string::npos)
    {
        parts.push_back(line.substr(prev_pos, pos - prev_pos));
        
        prev_pos = ++pos;
    }
    
    parts.push_back(line.substr(prev_pos, pos-prev_pos));
    
    return parts;
}

bool Correspondence::read(const std::string& path)
{
    std::ifstream file(path);
    if (!file.is_open())
        return false;
    
    std::string line;
    
    std::getline(file, line);
    
    setSize(std::stoi(line));
    
    while(std::getline(file, line))
    {
        auto parts = splitLine(line);
        
        int s = std::stoi(parts[0]);
        
        for (int i = 1; i < parts.size(); i++)
        {
            add(s, std::stoi(parts[i]));
        }
    }
    
    return true;
}

bool Correspondence::write(const std::string& path) const
{
    return false;
}

bool Correspondence::writeSize(std::ofstream& out, size_t size) const
{
    out << size << std::endl;
    
    return true;
}

bool Correspondence::writeLine(std::ofstream& out, int s, const List& t) const
{
    if (t.empty())
        return true;
    
    out << s;
    
    for (int i = 0; i < t.size(); i++)
        out << "," << t[i];
    
    out << std::endl;
    
    return true;
}
