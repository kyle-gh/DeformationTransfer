//
//  CorrespondenceManager.hpp
//  CorrTool
//
//  Created by Kyle on 12/20/18.
//  Copyright © 2018 Kyle. All rights reserved.
//

#ifndef CorrespondenceManager_hpp
#define CorrespondenceManager_hpp

#include <vector>
#include <memory>
#include <string>

class CorrespondenceManager
{
public:
    enum Target
    {
        TARGET_A,
        TARGET_B
    };
    
    bool read(const std::string& path);
    bool write();
    
    bool hasCorrespondence(unsigned int f);
    bool hasCorrespondence(Target t, unsigned int f);
    bool isActive(unsigned int f);
    bool isActive(Target t, unsigned int f);
    
    bool setActiveA(unsigned int f);
    bool setActiveB(unsigned int f);
    bool setActive(Target t, unsigned int f);
    
    bool pushActive();
    
    bool popActive();
    
private:
    typedef std::pair<int, int> Pair;
    
    std::string _path;
    
    std::vector<Pair> _pairs;
    
    Pair _active;
    
};

typedef std::shared_ptr<CorrespondenceManager> CorrespondenceManagerPtr;

inline CorrespondenceManagerPtr MakeCorrespondenceManager() { return std::make_shared<CorrespondenceManager>(); }

#endif /* CorrespondenceManager_hpp */
