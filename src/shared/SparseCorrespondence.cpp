//
//  SparseCorrespondence.cpp
//  Deform
//
//  Created by Kyle on 12/8/18.
//  Copyright © 2018 Kyle. All rights reserved.
//

#include "SparseCorrespondence.h"

void SparseCorrespondence::clear()
{
    for(auto c : _correspondences)
    {
        c.second.clear();
    }
    
    _correspondences.clear();
    
    _numPairs = 0;
}

bool SparseCorrespondence::add(int s, int t)
{
    auto iter = _correspondences.find(s);
    if (iter != _correspondences.end())
    {
        auto& list = iter->second;
        
        if (std::find(list.begin(), list.end(), t) == list.end())
        {
            list.push_back(t);
            _numPairs++;
        }
    }
    else
    {
        _correspondences[s].push_back(t);
        _numPairs++;
    }
    
    return true;
}

bool SparseCorrespondence::has(int s) const
{
    return _correspondences.find(s) != _correspondences.end();
}

Correspondence::List& SparseCorrespondence::get(int s)
{
    auto iter = _correspondences.find(s);
    if (iter == _correspondences.end())
    {
        _empty.clear();
        return _empty;
    }
    else
    {
        return iter->second;
    }
}

const Correspondence::List& SparseCorrespondence::get(int s) const
{
    auto iter = _correspondences.find(s);
    if (iter == _correspondences.end())
    {
        return _empty;
    }
    else
    {
        return iter->second;
    }
}

bool SparseCorrespondence::write(const std::string& path) const
{
    std::ofstream file(path);
    if (!file.is_open())
        return false;
    
    writeSize(file, _correspondences.size());
    
    for (auto c : _correspondences)
    {
        writeLine(file, c.first, c.second);
    }
    
    file.close();
    
    return true;
}

void SparseCorrespondence::getPairs(std::vector<std::pair<int, int>>& pairs)
{
    for (auto c : _correspondences)
    {
        for (auto t : c.second)
        {
            pairs.push_back(std::make_pair(c.first, t));
        }
    }
}
