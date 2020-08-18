//
//  Correspondence.hpp
//  Deform
//
//  Created by Kyle on 12/8/18.
//  Copyright © 2018 Kyle. All rights reserved.
//

#ifndef Correspondence_hpp
#define Correspondence_hpp

#include <vector>
#include <memory>
#include <fstream>

#include "Mesh.h"
#include "Search.h"

class Correspondence
{
public:
    typedef std::vector<int> List;
    typedef std::vector<std::pair<int, int>> PairsList;
    
    virtual bool read(const std::string& path);
    virtual bool write(const std::string& path) const;
    
    virtual void setSize(size_t size) = 0;
    virtual size_t size() const = 0;
    
    virtual void clear() = 0;
    
    virtual bool add(int s, int t) = 0;
    
    virtual bool has(int s) const = 0;
    
    virtual List& get(int s) = 0;
    virtual const List& get(int s) const = 0;
    
    virtual size_t numPairs() const = 0;
    virtual void getPairs(PairsList& pairs) = 0;
    
protected:
    bool writeSize(std::ofstream& out, size_t size) const;
    bool writeLine(std::ofstream& out, int s, const List& t) const;
};

typedef std::shared_ptr<Correspondence> CorrespondencePtr;

#endif /* Correspondence_hpp */
