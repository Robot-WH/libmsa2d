#pragma once 
#include <set>
#include <list>
#include <deque>
#include <functional>
#include <mutex>
#include <shared_mutex>
#include <thread>
#include <condition_variable>
#include <typeindex>
#include <type_traits>
#include <atomic>
#include "Function.hpp"
namespace ipc {
/**
 * @brief: 数据管理器对外接口类  
 * @details:  非模板的万能类  
 */    
class DataManagerBase {
public:
    DataManagerBase(uint16_t const& capacity) : capacity_(capacity) {}
    virtual ~DataManagerBase() {}
    virtual inline std::type_index GetDataType() const = 0; 
    virtual inline uint16_t GetDataSize() const = 0; 
    virtual inline void DeleteFrontData() = 0; 
    virtual bool Callback() = 0;  
    virtual inline bool IsEmpty() const = 0; 
    inline uint16_t GetCapacity() const {
        return capacity_;  
    }
protected:
    uint16_t capacity_;
};

/**
 * @brief: 数据管理器 实现
 */
template<typename _DataT>
class DataManagerImpl : public DataManagerBase {
public:
    DataManagerImpl(uint16_t const& capacity, SingleParamFunctionBase* p_callback_wrapper) 
    : DataManagerBase(capacity), p_callback_wrapper_(p_callback_wrapper), 
        type_info_(typeid(_DataT)) {}    // typeid 不区分const和&  也就是 const int& 和 int 是一样的
    
    ~DataManagerImpl() {
        // std::cout << "~DataManagerImpl()" <<std::endl;
        delete p_callback_wrapper_;  
    }

    /**
     * @brief: 传入左值引用
     * @details: 此时SingleParamFunction 参数能为 const _DataT&和_DataT和_DataT&
     */    
    void Call(_DataT& data) {
        // std::cout << "Call(_DataT& data)" << std::endl;
        if (p_callback_wrapper_->get_ref_type() == ref_type::non_ref) {
            // 若回调函数参数是非引用，那么回调只能是值传递   
            SingleParamFunction<void(_DataT)>* p_callback =  
                dynamic_cast<SingleParamFunction<void(_DataT)>*>(p_callback_wrapper_);
            (*p_callback)(data);  
        } else {
            // 若回调函数参数是引用，那么只能是左值引用  
            if (p_callback_wrapper_->is_const_param()) {
                // 常量左值引用 
                SingleParamFunction<void(const _DataT&)>* p_callback =  
                    dynamic_cast<SingleParamFunction<void(const _DataT&)>*>(p_callback_wrapper_);
                (*p_callback)(data);  
            } else {
                // 左值引用 
                SingleParamFunction<void(_DataT&)>* p_callback =  
                    dynamic_cast<SingleParamFunction<void(_DataT&)>*>(p_callback_wrapper_);
                (*p_callback)(data);  
            }
        }
    }

    /**
     * @brief: 传入常量左值引用
     * @details: 此时SingleParamFunction 参数只能为 const _DataT&和_DataT
     */    
    void Call(const _DataT& data) {
        // std::cout << "Call(const _DataT& data )" << std::endl;
        if (p_callback_wrapper_->get_ref_type() == ref_type::non_ref) {
            // 若回调函数参数是非引用，那么回调只能是值传递   
            SingleParamFunction<void(_DataT)>* p_callback =  
                dynamic_cast<SingleParamFunction<void(_DataT)>*>(p_callback_wrapper_);
            (*p_callback)(data);  
        } else {
            // 若回调函数参数是引用，那么只能是常量左值引用  
            SingleParamFunction<void(const _DataT&)>* p_callback =  
                dynamic_cast<SingleParamFunction<void(const _DataT&)>*>(p_callback_wrapper_);
            (*p_callback)(data);  
        }
    }

     /**
     * @brief: 传入右值引用
     * @details: 此时SingleParamFunction 参数能为 const _DataT&和_DataT和_DataT&&
     */    
    void Call(_DataT&& data) {
        // std::cout << "Call(_DataT&& data )" << std::endl;
        if (p_callback_wrapper_->get_ref_type() == ref_type::rvalue_ref) {
            // 若回调函数参数是右值引用
            SingleParamFunction<void(_DataT&&)>* p_callback =  
                dynamic_cast<SingleParamFunction<void(_DataT&&)>*>(p_callback_wrapper_);
            (*p_callback)(std::move(data));  
        } else {
            if (p_callback_wrapper_->is_const_param()) {
                // 常量左值引用 
                SingleParamFunction<void(const _DataT&)>* p_callback =  
                    dynamic_cast<SingleParamFunction<void(const _DataT&)>*>(p_callback_wrapper_);
                (*p_callback)(std::move(data));  
            } else {
                // 只能是值传递
                SingleParamFunction<void(_DataT)>* p_callback =  
                    dynamic_cast<SingleParamFunction<void(_DataT)>*>(p_callback_wrapper_);
                (*p_callback)(std::move(data));  
            }
        }
    }

    /**
     * @brief: 执行回调函数
     * @return 执行回调是否成功
     */        
    virtual bool Callback() override {
        if (data_buffer_.empty()) return false;   
        // 如果回调是右值引用参数 那么需要将数据转为右值 
        if (p_callback_wrapper_->get_ref_type() == ref_type::rvalue_ref) {
            Call(std::move(data_buffer_.front()));  
        } else {
            Call(data_buffer_.front());
        }
        DeleteFrontData();  
        return true;  
    }

    /**
     * @brief: 在队列最末尾添加数据 
     */             
    template<typename _DataType>
    bool AddData(_DataType&& data) {
        if (type_info_ != std::type_index(typeid(_DataType))) {
            throw std::bad_cast();  
        }
        data_m_.lock();  
        if (data_buffer_.size() >= capacity_) {
            data_buffer_.pop_front();   
        }
        data_buffer_.push_back(std::forward<_DataType>(data));
        // std::cout << "data_buffer_ add :" << data << std::endl;
        data_m_.unlock();  
        return true;  
    }
    
    /**
     * @brief: 删除头数据
     */            
    inline void DeleteFrontData() override {
        data_m_.lock();
        data_buffer_.pop_front();   
        data_m_.unlock();  
    }

    virtual inline uint16_t GetDataSize() const override {
        return data_buffer_.size();  
    }

    // 获取数据类型  
    inline std::type_index GetDataType() const override {
        return type_info_;  
    }
    
    virtual inline bool IsEmpty() const override {
        return data_buffer_.size() == 0;   
    }

private:
    std::mutex data_m_;   // 加了mutex后变成了只移型别   (禁止了拷贝构造)
    std::deque<_DataT> data_buffer_;       // 容器的型别不支持 const/&，_DataT 必须是非引用和非const的  
    std::type_index type_info_;  // 数据类型信息 
    SingleParamFunctionBase* p_callback_wrapper_; 
};

/**
 * @brief: 订阅者类   
 * @details 包含订阅回调函数和订阅数据缓存
 *                      由于在DataDispatcher类中通过一个容器管理所有Topic的Subscriber，因此该类必须是非模板基类
 */    
class Subscriber {
public:
    Subscriber() : data_cache_(nullptr) {}

    /**
     * @brief: 构造函数
     * @param callback 类内成员函数地址
     * @param class_addr 类对象地址
     * @param  cache_capacity 缓存容量  
     * @param high_priority 是否为高优先级
     */        
    template<typename _InputT, typename _Ctype>
    Subscriber(void (_Ctype::*callback)(_InputT), _Ctype* class_addr, int cache_capacity = 1,
            bool high_priority = false) : high_priority_(high_priority) {
        using pure_type = typename std::remove_const<std::remove_reference_t<_InputT>>::type; 
        data_cache_ = new DataManagerImpl<pure_type>(cache_capacity, 
            new SingleParamFunction<void(_InputT)>(std::bind(callback, class_addr, std::placeholders::_1)));
    }

    /**
     * @brief: 构造函数
     * @param callback 普通成员函数地址
     * @param  cache_capacity 缓存容量  
     * @param high_priority 是否为高优先级
     */      
    template<typename _InputT>
    Subscriber(void (*callback)(_InputT), int cache_capacity = 1,
            bool high_priority = false) : high_priority_(high_priority) {
        using no_ref_type = std::remove_reference_t<_InputT>; 
        using pure_type = typename std::remove_const<no_ref_type>::type; 
        data_cache_ = new DataManagerImpl<pure_type>(cache_capacity, 
            new SingleParamFunction<void(_InputT)>(callback));
    }

    virtual ~Subscriber() {
        // std::cout << "~Subscriber" <<std::endl;
        delete data_cache_; 
    }
private:
    /**
     * @brief: 回调  调度线程通过该函数触发该订阅者的回调
     */        
    bool callback() {
        return data_cache_->Callback();  
    }

    /**
     * @brief: 检查是否所有缓存数据都处理完毕
     */        
    bool cacheDataEmpty() {
        return data_cache_->IsEmpty();  
    }

    /**
     * @brief: 在DataDispatcher类的Publish函数中被调用
     */    
    template<typename _DataT>
    void addData(_DataT&& data) {
        // 判断输入数据与数据容器的类型是否一致     const和& 不会影响  type_index的结果  
        if (data_cache_->GetDataType() != std::type_index(typeid(_DataT))) {
            std::cerr<<"DataDispatcher addData() Type error !!!"<<std::endl;
            throw std::bad_cast();  
        }

        using pure_type = typename std::remove_const<std::remove_reference_t<_DataT>>::type; 
        DataManagerImpl<pure_type>* data_manager_ptr =  
            dynamic_cast<DataManagerImpl<pure_type>*>(data_cache_);

        if (high_priority_) {
            data_manager_ptr->Call(std::forward<_DataT>(data));  // 高优先级的直接调用 
        } else {
            data_manager_ptr->AddData(std::forward<_DataT>(data));     // 线程安全的
        }
    }

    friend class DataDispatcher; 
    // 数据管理 (数据缓存与回调调度)    要求能处理任何的数据，因此必须是非模板泛化基类
    DataManagerBase* data_cache_ = nullptr;  
    bool high_priority_ = false; 
};

/**
 * @brief: 数据调度器 
 * @details:  负责系统各种类型数据的保存，读取   
 */    
class DataDispatcher {
public:
    /**
     * @brief: 单例的创建函数  
     */            
    static DataDispatcher& GetInstance() {
        static DataDispatcher data_dispatcher;
        return data_dispatcher; 
    }

    virtual ~DataDispatcher() {
        for (const auto& name_set : subscriber_container_) {
            for (const auto& pt : name_set.second) {
                delete pt;  
            }
        }
    }
    
    /**
     * @brief: 订阅某个数据容器，回调函数为类内成员的重载 
     * @details: 订阅动作，告知DataDispatcher，_Ctype类对象的callback函数要订阅名字为name的数据容器
     * @param _DataT 回调函数的输入类型  
     * @param name 数据容器名
     * @param callback 注册的回调函数
     * @param class_addr 类对象地址
     * @param cache_capacity 缓存容量 
     * @param high_priority true 高优先级的订阅者在数据发布时会直接调用回调
     *                                                false 低优先级的订阅者在数据发布后由回调线程统一进行调度
     */        
    template<typename _InputT, typename _Ctype>
    Subscriber& Subscribe(std::string const& name, 
                                                        void (_Ctype::*callback)(_InputT), 
                                                        _Ctype* class_addr,
                                                        int cache_capacity = 1,
                                                        bool high_priority = false) {
        Subscriber* p_subscriber = new Subscriber(callback, class_addr, cache_capacity, high_priority);
        substriber_container_m_.lock();
        active_data_container_m_.lock();  
        active_data_container_[name] = false;  
        subscriber_container_[name].insert(p_subscriber); 
        active_data_container_m_.unlock();  
        substriber_container_m_.unlock();  
        return *p_subscriber;  
    }

    /**
     * @brief: 订阅某个数据容器，回调函数为普通函数 
     * @param name 数据容器名
     * @param callback 注册的回调函数
     * @param cache_capacity 缓存容量 
     */        
    template<typename _InputT, typename _Ctype>
    Subscriber& Subscribe(std::string const& name, 
                                                    void (*callback)(_InputT), 
                                                    int cache_capacity = 1,
                                                    bool high_priority = false) {
        Subscriber* p_subscriber = new Subscriber(callback, cache_capacity, high_priority);
        substriber_container_m_.lock();
        active_data_container_m_.lock();  
        active_data_container_[name] = false;  
        subscriber_container_[name].insert(p_subscriber); 
        active_data_container_m_.unlock();  
        substriber_container_m_.unlock();  
        return *p_subscriber;  
    }

    /**
     * @brief: 发布数据   
     * @details 向数据容器发布数据 done 
     * @param[in] name 数据的标识名
     * @param[in] data 加入的数据 
     */            
    template<typename _T>
    void Publish(std::string const& name, _T&& data) {
        // 如果有订阅者  则将数据传送到各个订阅者
        std::shared_lock<std::shared_mutex> m_l(substriber_container_m_);  // 禁止subscriber_container_ 写数据
        if (subscriber_container_[name].size()) {
            ///////////////////////////////////////////////////
            // 遍历该topic的所有订阅者，并将数据发送给他们
            data_m_.lock();  
            for (const auto& pt : subscriber_container_[name]) {
                pt->addData(std::forward<_T>(data));     // addData是线程安全的
            }
            data_m_.unlock();
            ///////////////////////////////////////////////////
            m_l.unlock();     // = unlock_shared(), 可以对 subscriber_container_ 进行更新了
            ///////////////////////////////////////////////////
            active_data_container_m_.lock_shared();
            if (!active_data_container_.at(name)) {
                active_data_container_.at(name) = true;    // 将name 设置为活跃 ， 表示有新数据 
                ++active_num_;
            }
            active_data_container_m_.unlock_shared();  
            ///////////////////////////////////////////////////
            // std::cout << "active_data_container_ name: " << name << "=true" << std::endl;
            con_.notify_one();  
        }
        return;
    }

    std::thread& GetThread() {
        return callback_thread_; 
    }

protected:
    DataDispatcher() {
        callback_thread_ = std::thread(&DataDispatcher::process, this);
    }
    DataDispatcher(DataDispatcher const& object) = default;
    DataDispatcher(DataDispatcher&& object) = default; 

    // 处理线程   
    void process() {
        while (true) {
            std::unique_lock<std::mutex> lock_m(process_m_);
            con_.wait(lock_m,[&] {
                return active_num_ > 0;
            });
            // 遍历每一个topic，并找到活跃的(接受到了新数据)
            std::shared_lock<std::shared_mutex> s_l(active_data_container_m_); // 禁止写，防止active_data_container_遍历失败
            for (const auto& obj : active_data_container_) {
                if (!obj.second) continue;   // 如果没有新数据 则跳过 
                // 该容器的全部订阅者都执行回调函数
                for (const auto& pt : subscriber_container_[obj.first]) {
                    pt->callback();   // callback() 是线程安全的
                }
                // 检查该数据容器的全部订阅者是否都处理完了自己的数据
                data_m_.lock(); 
                int finish = true; 
                for (const auto& pt : subscriber_container_[obj.first]) {
                    if (!pt->cacheDataEmpty()) {
                        finish = false;
                        break;
                    }
                }
                // 如果所有订阅者的数据都回调完毕  则失能
                if (finish) {
                    active_data_container_.at(obj.first) = false;  
                    --active_num_; 
                    // std::cout << "active_data_container_ erase: " << obj.first <<std::endl;
                    // std::cout << "----------------------------------------------" <<std::endl;
                }
                data_m_.unlock();  
            }
        };
    }

private:
    // 线程相关
    std::mutex data_m_; 
    std::mutex process_m_; 
    std::mutex data_container_m_; 
    std::shared_mutex substriber_container_m_; 
    std::shared_mutex active_data_container_m_;  
    std::atomic<int> active_num_{0}; 
    std::condition_variable con_;   
    std::thread callback_thread_;   
    // 核心数据
    std::unordered_map<std::string, std::set<Subscriber*>> subscriber_container_;    // 管理每个Topic 的 Subscriber 
    std::unordered_map<std::string, bool> active_data_container_; 
}; // class 
} // namespace 