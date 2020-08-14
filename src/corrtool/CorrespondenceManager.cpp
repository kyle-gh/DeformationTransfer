//
//  CorrespondenceManager.cpp
//  CorrTool
//
//  Created by Kyle on 12/20/18.
//  Copyright © 2018 Kyle. All rights reserved.
//

#include "CorrespondenceManager.h"

#include "../shared/SparseCorrespondence.h"

#define INVALID ((unsigned int)-1)

bool CorrespondenceManager::hasCorrespondence(unsigned int f)
{
    for (auto i = 0; i < _pairs.size(); i++)
    {
        const auto& pair = _pairs[i];
        if (pair.first == f || pair.second == f)
            return true;
    }
    
    return false;
}

bool CorrespondenceManager::hasCorrespondence(Target t, unsigned int f)
{
    for (auto i = 0; i < _pairs.size(); i++)
    {
        const auto& pair = _pairs[i];
        
        if (t == TARGET_A)
        {
            if (pair.first == f)
                return true;
        }
        else
        {
            if (pair.second == f)
                return true;
        }
    }
    
    return false;
}

bool CorrespondenceManager::isActive(unsigned int f)
{
    return _active.first == f || _active.second == f;
}

bool CorrespondenceManager::isActive(Target t, unsigned int f)
{
    if (t == TARGET_A)
        return _active.first == f;
    else
        return _active.second == f;
}

bool CorrespondenceManager::setActiveA(unsigned int f)
{
    _active.first = f;
    
    return true;
}

bool CorrespondenceManager::setActiveB(unsigned int f)
{
    _active.second = f;
    
    return true;
}

bool CorrespondenceManager::setActive(Target t, unsigned int f)
{
    if (t == TARGET_A)
        return setActiveA(f);
    else
        return setActiveB(f);
}

bool CorrespondenceManager::pushActive()
{
    if (_active.first != INVALID && _active.second != INVALID)
    {
        _pairs.push_back(_active);
        _active.first = INVALID;
        _active.second = INVALID;
        
        return true;
    }
    
    return false;
}

bool CorrespondenceManager::popActive()
{
    if (_pairs.empty())
        return false;
    
    _active = _pairs[_pairs.size() - 1];
    _pairs.pop_back();
    
    return true;
}

bool CorrespondenceManager::read(const std::string& path)
{
    _path = path;
    
    SparseCorrespondence corr;
    if (corr.read(_path))
    {
        _pairs.clear();
        corr.getPairs(_pairs);
        
        return true;
    }
    
    return false;
}

bool CorrespondenceManager::write()
{
    SparseCorrespondence corr;
    
    for (auto i = 0; i < _pairs.size(); i++)
    {
        corr.add(_pairs[i].first, _pairs[i].second);
    }
    
    corr.write(_path);
    
    return true;
}
