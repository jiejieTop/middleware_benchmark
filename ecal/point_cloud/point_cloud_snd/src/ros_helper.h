/*
 * @Author       : jiejie
 * @GitHub       : https://github.com/jiejieTop
 * @Date         : 2022-03-29 17:51:37
 * @LastEditors  : jiejie
 * @LastEditTime : 2022-10-28 17:50:50
 * @FilePath     : /ecal/samples/cpp/image/image_snd/src/ros_helper.h
 * Copyright (c) 2022 jiejie, All Rights Reserved. Please keep the author information and source code according to the
 * license.
 */

#include <ros/rate.h>
#include <ros/ros.h>

#include <functional>
#include <iostream>
#include <string>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <map>

namespace ros {

#define SINGLETON_POTION(T) T::potion()
#define SINGLETON_INSTANCE(T) T::instance()
#define SINGLETON_DELETE_INSTANCE(T) T::delete_instance()

#define DECLARE_SINGLETON_INSTANCE_API(T)                            \
public:                                                              \
    static T& instance_operating(bool create_if_needed = true)       \
    {                                                                \
        static std::mutex singleton_mutex_;                          \
        static T* instance_ = nullptr;                               \
        if (create_if_needed) {                                      \
            if (instance_) {                                         \
                return *instance_;                                   \
            } else {                                                 \
                std::unique_lock<std::mutex> lock(singleton_mutex_); \
                if (!instance_) {                                    \
                    instance_ = new (std::nothrow) T;                \
                }                                                    \
            }                                                        \
        } else {                                                     \
            if (instance_) {                                         \
                std::unique_lock<std::mutex> lock(singleton_mutex_); \
                if (instance_) {                                     \
                    delete instance_;                                \
                    instance_ = nullptr;                             \
                }                                                    \
            }                                                        \
        }                                                            \
        return *instance_;                                           \
    }                                                                \
                                                                     \
    static T& instance() { return instance_operating(true); }        \
                                                                     \
    static T* potion() { return &instance_operating(true); }         \
                                                                     \
    static void delete_instance() { instance_operating(false); }

#define DECLARE_PUBLIC_DEFAULT_CONSTRUCTOR(T) \
public:                                       \
    T() = default;

#define DECLARE_PRIVATE_DEFAULT_CONSTRUCTOR(T) \
private:                                       \
    T() = default;

#define DECLARE_PRIVATE_CONSTRUCTOR(T) \
public:                                \
    T() = delete;

#define DECLARE_PRIVATE_COPY_CONSTRUCTOR(T) \
private:                                    \
    T(const T&);                            \
    T& operator=(const T&);

#define DECLARE_NO_COPY_CLASS(T) \
private:                         \
    T(const T&) = delete;        \
    T& operator=(const T&) = delete;

/* 声明单例模式：私有构造函数 */
#define DECLARE_SINGLETON_IMPLEMENT_PRIVATE_CONSTRUCTOR(T) DECLARE_SINGLETON_INSTANCE_API(T)

/* 声明单例模式：私有默认构造函数 */
#define DECLARE_SINGLETON_IMPLEMENT_PRIVATE_DEFAULT_CONSTRUCTOR(T) \
    DECLARE_SINGLETON_INSTANCE_API(T)                              \
    DECLARE_PRIVATE_DEFAULT_CONSTRUCTOR(T)

/* 声明单例模式：私有构造函数，禁止拷贝 */
#define DECLARE_SINGLETON_IMPLEMENT_PRIVATE_CONSTRUCTOR_NO_COPY(T) \
    DECLARE_SINGLETON_IMPLEMENT_PRIVATE_CONSTRUCTOR(T)             \
    DECLARE_NO_COPY_CLASS(T)

#define DECLARE_SINGLETON_IMPLEMENT(T) DECLARE_SINGLETON_IMPLEMENT_PRIVATE_CONSTRUCTOR_NO_COPY(T)

template <class T>
class singleton {
    DECLARE_SINGLETON_INSTANCE_API(T)
};



namespace private_check_type {

    template <typename T, bool IsBase = false>
    struct check;

    /*
        Output state management
    */

    class output {
        bool is_compact_ = true;

        template <typename T>
        bool check_empty(const T&)
        {
            return false;
        }
        bool check_empty(const char* val) { return (!val) || (val[0] == 0); }

        template <typename T>
        void out(const T& val)
        {
            if (check_empty(val))
                return;
            if (!is_compact_)
                sr_ += " ";
            using ss_t = std::ostringstream;
            sr_ += static_cast<ss_t&>(ss_t() << val).str();
            is_compact_ = false;
        }

        std::string& sr_;

    public:
        output(std::string& sr) : sr_(sr) {}

        output& operator()(void) { return (*this); }

        template <typename T1, typename... T>
        output& operator()(const T1& val, const T&... args)
        {
            out(val);
            return operator()(args...);
        }

        output& compact(void)
        {
            is_compact_ = true;
            return (*this);
        }
    };

    // ()

    template <bool>
    struct bracket {
        output& out_;

        bracket(output& out, const char* = nullptr) : out_(out) { out_("(").compact(); }

        ~bracket(void) { out_.compact()(")"); }
    };

    template <>
    struct bracket<false> {
        bracket(output& out, const char* str = nullptr) { out(str); }
    };

    // [N]

    template <size_t N = 0>
    struct bound {
        output& out_;

        bound(output& out) : out_(out) {}
        ~bound(void)
        {
            if (N == 0)
                out_("[]");
            else
                out_("[").compact()(N).compact()("]");
        }
    };

    // (P1, P2, ...)

    template <bool, typename... P>
    struct parameter;

    template <bool IsStart>
    struct parameter<IsStart> {
        output& out_;

        parameter(output& out) : out_(out) {}
        ~parameter(void) { bracket<IsStart>{ out_ }; }
    };

    template <bool IsStart, typename P1, typename... P>
    struct parameter<IsStart, P1, P...> {
        output& out_;

        parameter(output& out) : out_(out) {}
        ~parameter(void)
        {
            [this](bracket<IsStart>&&) {
                check<P1>{ out_ };
                parameter<false, P...>{ out_.compact() };
            }(bracket<IsStart>{ out_, "," });
        }
    };

    // Do output at destruct

    struct at_destruct {
        output& out_;
        const char* str_;

        at_destruct(output& out, const char* str = nullptr) : out_(out), str_(str) {}
        ~at_destruct(void) { out_(str_); }

        void set_str(const char* str = nullptr) { str_ = str; }
    };

    /*
        CV-qualifiers, references, pointers
    */

    template <typename T, bool IsBase>
    struct check {
        output out_;
        check(const output& out) : out_(out)
        {
#if defined(__GNUC__)
            const char* typeid_name = typeid(T).name();
            auto deleter = [](char* p) {
                if (p)
                    free(p);
            };
            std::unique_ptr<char, decltype(deleter)> real_name{
                abi::__cxa_demangle(typeid_name, nullptr, nullptr, nullptr), deleter
            };
            out_(real_name ? real_name.get() : typeid_name);
#else
            out_(typeid(T).name());
#endif
        }
    };

#define CHECK_TYPE__(OPT)                                      \
    template <typename T, bool IsBase>                         \
    struct check<T OPT, IsBase> : check<T, true> {             \
        using base_t = check<T, true>;                         \
        using base_t::out_;                                    \
        check(const output& out) : base_t(out) { out_(#OPT); } \
    };

    CHECK_TYPE__(const)
    CHECK_TYPE__(volatile)
    CHECK_TYPE__(const volatile)
    CHECK_TYPE__(&)
    CHECK_TYPE__(&&)
    CHECK_TYPE__(*)

#undef CHECK_TYPE__

    /*
        Arrays
    */

#define CHECK_TYPE_ARRAY__(CV_OPT, BOUND_OPT, ...)                                          \
    template <typename T, bool IsBase __VA_ARGS__>                                          \
    struct check<T CV_OPT[BOUND_OPT], IsBase> : check<T CV_OPT, !std::is_array<T>::value> { \
        using base_t = check<T CV_OPT, !std::is_array<T>::value>;                           \
        using base_t::out_;                                                                 \
                                                                                            \
        bound<BOUND_OPT> bound_ = out_;                                                     \
        bracket<IsBase> bracket_ = out_;                                                    \
                                                                                            \
        check(const output& out) : base_t(out) {}                                           \
    };

#define CHECK_TYPE_ARRAY_CV__(BOUND_OPT, ...)                \
    CHECK_TYPE_ARRAY__(, BOUND_OPT, , ##__VA_ARGS__)         \
    CHECK_TYPE_ARRAY__(const, BOUND_OPT, , ##__VA_ARGS__)    \
    CHECK_TYPE_ARRAY__(volatile, BOUND_OPT, , ##__VA_ARGS__) \
    CHECK_TYPE_ARRAY__(const volatile, BOUND_OPT, , ##__VA_ARGS__)

#define CHECK_TYPE_PLACEHOLDER__
    CHECK_TYPE_ARRAY_CV__(CHECK_TYPE_PLACEHOLDER__)
#if defined(__GNUC__)
    CHECK_TYPE_ARRAY_CV__(0)
#endif
    CHECK_TYPE_ARRAY_CV__(N, size_t N)

#undef CHECK_TYPE_PLACEHOLDER__
#undef CHECK_TYPE_ARRAY_CV__
#undef CHECK_TYPE_ARRAY__

    /*
        Functions
    */

    template <typename T, bool IsBase, typename... P>
    struct check<T(P...), IsBase> : check<T, true> {
        using base_t = check<T, true>;
        using base_t::out_;

        parameter<true, P...> parameter_ = out_;
        bracket<IsBase> bracket_ = out_;

        check(const output& out) : base_t(out) {}
    };

    /*
        Pointers to members
    */

    template <typename T, bool IsBase, typename C>
    struct check<T C::*, IsBase> : check<T, true> {
        using base_t = check<T, true>;
        using base_t::out_;

        check(const output& out) : base_t(out)
        {
            check<C>{ out_ };
            out_.compact()("::*");
        }
    };

    /*
        Pointers to member functions
    */

#define CHECK_TYPE_MEM_FUNC__(...)                                \
    template <typename T, bool IsBase, typename C, typename... P> \
    struct check<T (C::*)(P...) __VA_ARGS__, IsBase> {            \
        at_destruct cv_ = base_.out_;                             \
        check<T(P...), true> base_;                               \
        output& out_ = base_.out_;                                \
                                                                  \
        check(const output& out) : base_(out)                     \
        {                                                         \
            cv_.set_str(#__VA_ARGS__);                            \
            check<C>{ out_ };                                     \
            out_.compact()("::*");                                \
        }                                                         \
    };

    CHECK_TYPE_MEM_FUNC__()
    CHECK_TYPE_MEM_FUNC__(const)
    CHECK_TYPE_MEM_FUNC__(volatile)
    CHECK_TYPE_MEM_FUNC__(const volatile)

#undef CHECK_TYPE_MEM_FUNC__

}  // namespace private_check_type

/*
    Get the name of the given type

    check_type<const volatile void *>()
    -->
    void const volatile *
*/

template <typename T>
inline std::string check_type(void)
{
    std::string str;
    private_check_type::check<T>{ str };
    return str;
}



class ros_helper {
    DECLARE_SINGLETON_IMPLEMENT(ros_helper);

public:
    template <class M>
    void subscribe(const std::string& topic, uint32_t queue_size, void (*fp)(M))
    {
        auto sub = nh_->subscribe<M>(topic, queue_size, fp);
        std::unique_lock<std::mutex> lock(sub_lock_);
        subscribes_.push_back(std::move(sub));
    }

    template <class M>
    void subscribe(const std::string& topic, uint32_t queue_size, void (*fp)(const boost::shared_ptr<M const>&))
    {
        auto sub = nh_->subscribe<M>(topic, queue_size, fp);
        std::unique_lock<std::mutex> lock(sub_lock_);
        subscribes_.push_back(std::move(sub));
    }

    template <class M, class T>
    void subscribe(const std::string& topic, uint32_t queue_size, void (T::*fp)(M), T* obj)
    {
        auto sub = nh_->subscribe<M>(topic, queue_size, fp, obj);
        std::unique_lock<std::mutex> lock(sub_lock_);
        subscribes_.push_back(std::move(sub));
    }

    template <class M, class T>
    void subscribe(const std::string& topic, uint32_t queue_size, void (T::*fp)(M) const, T* obj)
    {
        auto sub = nh_->subscribe<M>(topic, queue_size, fp, obj);
        std::unique_lock<std::mutex> lock(sub_lock_);
        subscribes_.push_back(std::move(sub));
    }

    template <class M, class T>
    void subscribe(const std::string& topic, uint32_t queue_size, void (T::*fp)(const boost::shared_ptr<M const>&),
                   T* obj)
    {
        auto sub = nh_->subscribe<M>(topic, queue_size, fp, obj);
        std::unique_lock<std::mutex> lock(sub_lock_);
        subscribes_.push_back(std::move(sub));
    }

    template <class M, class T>
    void subscribe(const std::string& topic, uint32_t queue_size,
                   void (T::*fp)(const boost::shared_ptr<M const>&) const, T* obj)
    {
        auto sub = nh_->subscribe<M>(topic, queue_size, fp, obj);
        std::unique_lock<std::mutex> lock(sub_lock_);
        subscribes_.push_back(std::move(sub));
    }

    template <class M, class C>
    void subscribe(const std::string& topic, uint32_t queue_size, const boost::function<void(C)>& callback)
    {
        auto sub = nh_->subscribe<M>(topic, queue_size, callback);
        std::unique_lock<std::mutex> lock(sub_lock_);
        subscribes_.push_back(std::move(sub));
    }

    template <class M>
    void publish(const std::string& topic, const M& msg)
    {
        if (shutdown_)
            return;
        static std::string type = check_type<M>();
        auto it = publisher_.find(type + topic);
        if (it == publisher_.end()) {
            ros::Publisher publisher = nh_->advertise<M>(topic, 10);
            std::unique_lock<std::mutex> lock(pub_lock_);
            publisher_.insert(std::make_pair(type + topic, std::move(publisher)));
            publisher.publish(msg);
        } else {
            auto publisher = it->second;
            publisher.publish(msg);
        }
    }

private:
    ros_helper();
    ~ros_helper();

private:
    bool shutdown_;
    std::thread thread_;
    std::mutex pub_lock_;
    std::mutex sub_lock_;
    std::vector<ros::Subscriber> subscribes_;
    std::map<std::string, ros::Publisher> publisher_;
    ros::NodeHandle* nh_ = nullptr;
};

}  // namespace ros
