//
// Created by User on 2019/6/5.
//

#ifndef RANSAC_RANGE_HPP
#define RANSAC_RANGE_HPP

template<class t>
class range_t {
    t _begin, _end;

public:
    class iterator {
        t value;
    
    public:
        explicit iterator(t value) : value(value) {}
        
        bool operator!=(const iterator others) const {
            return value != others.value;
        }
        
        iterator &operator++() {
            ++value;
            return *this;
        }
        
        t operator*() const {
            return value;
        }
    };
    
    range_t(t begin, t end) : _begin(begin), _end(end + 1) {}
    
    iterator begin() const {
        return iterator(_begin);
    }
    
    iterator end() const {
        return iterator(_end);
    }
};


#endif //RANSAC_RANGE_HPP
