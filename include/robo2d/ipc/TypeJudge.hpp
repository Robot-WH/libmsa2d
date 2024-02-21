
#pragma once 
#include <iostream>
namespace  ipc {

template<typename _T>
class is_const {
public:
    static bool is() {
        return false;  
    }
};

template<typename _T>
class is_const<const _T> {
public:
    static bool is() {
        return true;  
    }
};

template<typename _T>
class is_const<const _T&> {
public:
    static bool is() {
        return true;  
    }
};

enum class ref_type {non_ref = 0, lvalue_ref, rvalue_ref};

template<typename _T>
class is_ref {
public:
    static ref_type is() {
        return ref_type::non_ref;  
    }
};

template<typename _T>
class is_ref<_T&> {
public:
    static ref_type is() {
        return ref_type::lvalue_ref;  
    }
};

template<typename _T>
class is_ref<_T&&> {
public:
    static ref_type is() {
        return ref_type::rvalue_ref;  
    }
};
}