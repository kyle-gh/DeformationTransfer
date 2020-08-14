//
//  SparseCorrespondence.hpp
//  Deform
//
//  Created by Kyle on 12/8/18.
//  Copyright © 2018 Kyle. All rights reserved.
//

#ifndef SparseCorrespondence_hpp
#define SparseCorrespondence_hpp

#include "Correspondence.h"

#include <map>

class SparseCorrespondence : public Correspondence
{
public:
    virtual void setSize(size_t size) {}
    virtual size_t size() const { return _correspondences.size(); }
    
    virtual bool write(const std::string& path) const;
    
    virtual void clear();
    
    virtual bool add(int s, int t);
    
    virtual bool has(int s) const;
    
    virtual List& get(int s);
    virtual const List& get(int s) const;
    
    virtual size_t numPairs() const { return _numPairs; }
    virtual void getPairs(std::vector<std::pair<int, int>>& pairs);
    
private:
    std::map<int, List> _correspondences;
    
    List _empty;
    
    int _numPairs;
};

#endif /* SparseCorrespondence_hpp */
