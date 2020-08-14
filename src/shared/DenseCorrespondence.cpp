//
//  DenseCorrespondence.cpp
//  Deform
//
//  Created by Kyle on 12/7/18.
//  Copyright © 2018 Kyle. All rights reserved.
//

#include "DenseCorrespondence.h"

void DenseCorrespondence::setSize(size_t size)
{
    _correspondences.resize(size);
    
    clear();
}

void DenseCorrespondence::clear()
{
    for(auto c : _correspondences)
    {
        c.clear();
    }
    
    _numPairs = 0;
}

bool DenseCorrespondence::add(int s, int t)
{
    auto& corr = _correspondences[s];
    if (std::find(corr.begin(), corr.end(), t) == corr.end())
    {
        corr.push_back(t);
        _numPairs++;
    }
    
    return true;
}

bool DenseCorrespondence::has(int s) const
{
    return !get(s).empty();
}

Correspondence::List& DenseCorrespondence::get(int s)
{
    return _correspondences[s];
}

const Correspondence::List& DenseCorrespondence::get(int s) const
{
    return _correspondences[s];
}

bool DenseCorrespondence::write(const std::string& path) const
{
    std::ofstream file(path);
    if (!file.is_open())
        return false;
    
    writeSize(file, _correspondences.size());
    
    for (int i = 0; i < _correspondences.size(); i++)
    {
        writeLine(file, i, _correspondences[i]);
    }
    
    file.close();
    
    return true;
}

void DenseCorrespondence::getPairs(std::vector<std::pair<int, int>>& pairs)
{
    for (auto i = 0; i < _correspondences.size(); i++)
    {
        const auto& c = _correspondences[i];
        for (auto j = 0; j < c.size(); j++)
        {
            pairs.push_back(std::make_pair(i, c[j]));
        }
    }
}
