//
//  SingleDenseCorrespondence.cpp
//  Deform
//
//  Created by Kyle on 12/10/18.
//  Copyright © 2018 Kyle. All rights reserved.
//

#include "SingleDenseCorrespondence.h"

Correspondence::List __temp(1);
Correspondence::List __empty;

void SingleDenseCorrespondence::setSize(size_t size)
{
    _correspondences.resize(size);
    
    clear();
}

void SingleDenseCorrespondence::clear()
{
    for (size_t i = 0; i < _correspondences.size(); i++)
        _correspondences[i] = -1;
    
    _numPairs = 0;
}

bool SingleDenseCorrespondence::add(int s, int t)
{
    if (_correspondences[s] == -1)
        _numPairs++;
    
    _correspondences[s] = t;
    
    return true;
}

bool SingleDenseCorrespondence::has(int s) const
{
    return _correspondences[s] != -1;
}

Correspondence::List& SingleDenseCorrespondence::get(int s)
{
    if (_correspondences[s] == -1)
        return __empty;
    
    __temp[0] = _correspondences[s];
    return __temp;
}

const Correspondence::List& SingleDenseCorrespondence::get(int s) const
{
    if (_correspondences[s] == -1)
        return __empty;
    
    __temp[0] = _correspondences[s];
    return __temp;
}

bool SingleDenseCorrespondence::write(const std::string& path) const
{
    std::ofstream file(path);
    if (!file.is_open())
        return false;
    
    writeSize(file, _correspondences.size());
    
    List temp(1);
    
    for (int i = 0; i < _correspondences.size(); i++)
    {
        temp[0] = _correspondences[i];
        
        writeLine(file, i, temp);
    }
    
    file.close();
    
    return true;
}

void SingleDenseCorrespondence::getPairs(std::vector<std::pair<int, int>>& pairs)
{
    for (auto i = 0; i < _correspondences.size(); i++)
    {
        pairs.push_back(std::make_pair(i, _correspondences[i]));
    }
}
