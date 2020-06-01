#pragma once

//  (C) Copyright Greg Colvin and Beman Dawes 1998, 1999.
//  Copyright (c) 2001, 2002 Peter Dimov
//
//  Distributed under the Boost Software License, Version 1.0. (See
//  accompanying file LICENSE_1_0.txt or copy at
//  http://www.boost.org/LICENSE_1_0.txt)
//
//  http://www.boost.org/libs/smart_ptr/scoped_ptr.htm
//

#include <memory>               // for std::auto_ptr

//  scoped_ptr mimics a built-in pointer except that it guarantees deletion
//  of the object pointed to, either on destruction of the scoped_ptr or via
//  an explicit reset(). scoped_ptr is a simple solution for simple needs;
//  use boost::shared_ptr or std::auto_ptr if your needs are more complex.
template<class T> class scoped_ptr // noncopyable
{
private:
    T * ptr;

    scoped_ptr(scoped_ptr const &);
    scoped_ptr & operator=(scoped_ptr const &);

    typedef scoped_ptr<T> this_type;

public:

    typedef T element_type;

    explicit scoped_ptr(T * p = 0): ptr(p) // never throws
    {
    }

#ifndef BOOST_NO_AUTO_PTR
    explicit scoped_ptr(std::auto_ptr<T> p): ptr(p.release()) // never throws
    {
    }
#endif

    ~scoped_ptr() // never throws
    {
		delete ptr;
    }

    void reset(T * p = 0) // never throws
    {
        ASSERT(p == 0 || p != ptr); // catch self-reset errors
        this_type(p).swap(*this);
    }

    T & operator*() const // never throws
    {
        ASSERT(ptr != 0);
        return *ptr;
    }

    T * operator->() const // never throws
    {
        ASSERT(ptr != 0);
        return ptr;
    }

    T * get() const // never throws
    {
        return ptr;
    }

    // implicit conversion to "bool"
    typedef T * this_type::*unspecified_bool_type;

    operator unspecified_bool_type() const // never throws
    {
        return ptr == 0? 0: &this_type::ptr;
    }

    bool operator! () const // never throws
    {
        return ptr == 0;
    }

    void swap(scoped_ptr & b) // never throws
    {
        T * tmp = b.ptr;
        b.ptr = ptr;
        ptr = tmp;
    }
};

template<class T> inline void swap(scoped_ptr<T> & a, scoped_ptr<T> & b) // never throws
{
    a.swap(b);
}

// get_pointer(p) is a generic way to say p.get()

template<class T> inline T * get_pointer(scoped_ptr<T> const & p)
{
    return p.get();
}

