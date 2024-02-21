#pragma once 
#include "TypeJudge.hpp"
namespace ipc {
/**
 * @brief:  BinderWrapper 基类  
 * @details:  在 function中只依赖这个基类 ， 对外的 函数为Call  设计为虚函数 供外界调用，BinderWrapperImpl 为它的子类 ，在 BinderWrapperImpl  中
 *                        完成对于binder 的保存以及Call的实现  
 * @param _return_type 函数返回型别
 * @param _args_type 函数参数型别 
 */    
template<typename _return_type, typename... _args_type>
class BinderWrapperBase {
public:
    virtual ~BinderWrapperBase() {}
    /**
     * @brief:  调用绑定的函数 
     * @param _args_type 函数的输入参数类型
     */            
    virtual _return_type Call(_args_type... args) = 0;  
};  // class BinderWrapperBase

template<typename _binder_type, typename _return_type, typename... _args_type>
class BinderWrapperImpl : public BinderWrapperBase<_return_type, _args_type...> {
public:
    BinderWrapperImpl(_binder_type binder) : binder_(binder) {}
    /**
     * @brief:  调用绑定的函数    重写   
     * @details:  _args_type 被设置为和绑定callback相同的参数
     */            
    _return_type Call(_args_type... args) override {
        return binder_(std::forward<_args_type>(args)...);
    }
private:
    _binder_type binder_;  
}; // class BinderWrapperImpl

/**
 * @brief: 单参数函数基类   即只有一个参数的函数 
 */
class SingleParamFunctionBase {
public:
    SingleParamFunctionBase(bool is_const, ref_type ref_type) 
            : is_const_param_(is_const), ref_type_(ref_type) {
    }
    virtual ~SingleParamFunctionBase() {
        // std::cout << "~SingleParamFunctionBase()" <<std::endl;
    }
    const bool& is_const_param() const {
        return is_const_param_;  
    }
    const ref_type& get_ref_type() const {
        return ref_type_;  
    }
private:
    // 保存的param的特性 - 引用? const? 
    bool is_const_param_ = false;  
    ref_type ref_type_ = ref_type::non_ref;  
};

/**
 * @brief: 主模板 
 */    
template<typename _return_type, typename... _args_type>
class SingleParamFunction : public SingleParamFunctionBase {}; // class Function

/**
 * @brief:  std::SingleParamFunction 单参数的偏特例化
 */    
template<typename _return_type, typename _args_type>
class SingleParamFunction<_return_type(_args_type)> : public SingleParamFunctionBase {
public:
    /**
     * @brief: 构造    同时传入绑定的函数，并且绑定的函数对象可以是任意类型  
     */    
    template<typename _binder_type>
    SingleParamFunction(_binder_type binder) 
            : SingleParamFunctionBase(is_const<_args_type>::is(), is_ref<_args_type>::is()) {
        binder_wrapper_ = 
            new BinderWrapperImpl<_binder_type, _return_type, _args_type>{binder};  
    }
    // 析构函数  
    virtual ~SingleParamFunction() {
        // std::cout << "~SingleParamFunction()" <<std::endl;
        delete binder_wrapper_;   
    }
    // 仿函数  
    template<typename _T>
    _return_type operator()(_T&& args) {
        return binder_wrapper_->Call(std::forward<_T>(args)); 
    }
private:
    BinderWrapperBase<_return_type, _args_type>* binder_wrapper_ = nullptr;  
}; // class SingleParamFunction
}