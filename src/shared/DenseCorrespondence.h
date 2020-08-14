//
//  Correspondence.hpp
//  Deform
//
//  Created by Kyle on 12/7/18.
//  Copyright © 2018 Kyle. All rights reserved.
//

#ifndef DenseCorrespondence_h
#define DenseCorrespondence_h

#include "Correspondence.h"

class DenseCorrespondence : public Correspondence
{
public:
    virtual bool write(const std::string& path) const;
    
    virtual void setSize(size_t size);
    virtual size_t size() const { return _correspondences.size(); }
    
    virtual void clear();
    
    virtual bool add(int s, int t);
    
    virtual bool has(int s) const;
    
    virtual List& get(int s);
    virtual const List& get(int s) const;
    
    virtual size_t numPairs() const { return _numPairs; }
    virtual void getPairs(std::vector<std::pair<int, int>>& pairs);
    
private:
    std::vector<List> _correspondences;
    
    int _numPairs;
};

#endif /* DenseCorrespondence_h */
