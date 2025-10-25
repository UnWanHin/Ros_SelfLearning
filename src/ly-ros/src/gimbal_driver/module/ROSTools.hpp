#pragma once

#include <map>
#include <memory>
#include <functional>
#include <mutex>
#include <string>

#include <fmt/format.h>
#include <fmt/chrono.h>

// ROS2 ?文件
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/executors.hpp>

namespace LangYa::roslog
{
#define LY_DEF_ROS_LOG(name, rosName)\
    template<typename ...Args> \
    void name(const char* fmt, Args &&... args) \
    { \
        RCLCPP_##rosName(rclcpp::get_logger("default"), fmt, std::forward<Args>(args)...); \
    } \

    LY_DEF_ROS_LOG(error, ERROR);
    LY_DEF_ROS_LOG(warn, WARN);
    LY_DEF_ROS_LOG(info, INFO);
    LY_DEF_ROS_LOG(debug, DEBUG);
#undef LY_DEF_ROS_LOG
}

namespace LangYa
{
    template<typename TGlobal>
    class MultiCallback
    {
        std::function<void(const TGlobal&)> PostEvent;
        std::mutex Mutex{};
        TGlobal Global{};

    public:
        MultiCallback(std::function<void(const TGlobal&)> postEvent) noexcept : PostEvent(std::move(postEvent)) {}

        template<typename TTopic>
        std::function<void(typename TTopic::CallbackArg)> Generate(
                std::function<void(TGlobal&, const typename TTopic::Msg&)> modifier
        ) noexcept
        {
            return [this, modifier](typename TTopic::CallbackArg arg)
            {
                std::lock_guard lock{ Mutex };
                modifier(Global, *arg);
                PostEvent(Global);
            };
        }
    };

    template<const char* TName, typename TMessage>
    struct ROSTopic
    {
        static constexpr auto Name = TName;
        using Msg = TMessage;
        // ROS2 使用 shared_ptr<const Msg> 作?回???
        using CallbackArg = std::shared_ptr<const TMessage>;
    };

#define LY_DEF_ROS_TOPIC(varname, topic, type)\
    inline static constexpr const char varname##_topic_name[] = topic;\
    using varname = LangYa::ROSTopic<varname##_topic_name, type>;

    template<const char* TName>
    class ROSNode
    {
        std::shared_ptr<rclcpp::Node> NodePtr{};
        std::map<std::string, rclcpp::PublisherBase::SharedPtr> Publishers{};
        std::map<std::string, rclcpp::SubscriptionBase::SharedPtr> Subscribers{};

    public:
        ROSNode() noexcept = default;

        bool Initialize(int argc, char** argv) noexcept try
        {
            if (!rclcpp::ok()) {
                rclcpp::init(argc, argv);
            }
            NodePtr = std::make_shared<rclcpp::Node>(TName);
            return true;
        }
        catch (const std::exception& ex)
        {
            RCLCPP_ERROR(rclcpp::get_logger("default"), "ROSNode::Initialize: %s", ex.what());
            return false;
        }

        template<typename TTopic>
        std::shared_ptr<rclcpp::Publisher<typename TTopic::Msg>> Publisher()
        {
            std::string name{ TTopic::Name };
            if (!Publishers.contains(name))
            {
                // ?建具体的?布者
                auto publisher = NodePtr->create_publisher<typename TTopic::Msg>(name, rclcpp::QoS(10));
                Publishers[name] = publisher;
                return publisher;
            }
            // ?行安全的?型??
            return std::static_pointer_cast<rclcpp::Publisher<typename TTopic::Msg>>(Publishers[name]);
        }

        template<typename TTopic>
        void GenSubscriber(auto callback)
        {
            std::string name{ TTopic::Name };
            if (Subscribers.contains(name)) return;
            
            // ?建??者
            auto subscriber = NodePtr->create_subscription<typename TTopic::Msg>(
                name, 
                rclcpp::QoS(10), 
                callback
            );
            Subscribers[name] = subscriber;
        }

        template<typename T>
        bool GetParam(const std::string& name, T& value, const T& defaultValue) noexcept
        {
            try {
                // ?取??
                value = NodePtr->declare_parameter<T>(name, defaultValue);
                return true;
            } catch (const std::exception& ex) {
                RCLCPP_ERROR(rclcpp::get_logger("default"), "ROSNode::GetParam: %s", ex.what());
                return false;
            }
        }

        // ?取??指?的方法，用于 spin
        std::shared_ptr<rclcpp::Node> GetNode() const
        {
            return NodePtr;
        }

        // ?查是否有效的方法
        bool IsOK() const
        {
            return rclcpp::ok() && NodePtr != nullptr;
        }
    };
}

// #pragma once

// #include <map>
// #include <memory>
// #include <functional>
// #include <mutex>
// #include <string>

// #include <fmt/format.h>
// #include <fmt/chrono.h>

// // ROS2 ?文件
// #include <rclcpp/rclcpp.hpp>
// #include <rclcpp/logger.hpp>
// #include <rclcpp/parameter.hpp>
// #include <rclcpp/executors.hpp>

// namespace LangYa::roslog
// {
// #define LY_DEF_ROS_LOG(name, rosName)\
//     template<typename ...Args> \
//     void name(const char* fmt, Args &&... args) \
//     { \
//         RCLCPP_##rosName(rclcpp::get_logger("default"), fmt, std::forward<Args>(args)...); \
//     } \

//     LY_DEF_ROS_LOG(error, ERROR);
//     LY_DEF_ROS_LOG(warn, WARN);
//     LY_DEF_ROS_LOG(info, INFO);
//     LY_DEF_ROS_LOG(debug, DEBUG);
// #undef LY_DEF_ROS_LOG
// }

// namespace LangYa
// {
//     template<typename TGlobal>
//     class MultiCallback
//     {
//         std::function<void(const TGlobal&)> PostEvent;
//         std::mutex Mutex{};
//         TGlobal Global{};

//     public:
//         MultiCallback(std::function<void(const TGlobal&)> postEvent) noexcept : PostEvent(std::move(postEvent)) {}

//         template<typename TTopic>
//         std::function<void(typename TTopic::CallbackArg)> Generate(
//                 std::function<void(TGlobal&, const typename TTopic::Msg&)> modifier
//         ) noexcept
//         {
//             return [this, modifier](typename TTopic::CallbackArg arg)
//             {
//                 std::lock_guard lock{ Mutex };
//                 modifier(Global, *arg);
//                 PostEvent(Global);
//             };
//         }
//     };

//     template<const char* TName, typename TMessage>
//     struct ROSTopic
//     {
//         static constexpr auto Name = TName;
//         using Msg = TMessage;
//         // ROS2 使用 shared_ptr<const Msg> 作?回???
//         using CallbackArg = std::shared_ptr<const TMessage>;
//     };

// #define LY_DEF_ROS_TOPIC(varname, topic, type)\
//     inline static constexpr const char varname##_topic_name[] = topic;\
//     using varname = LangYa::ROSTopic<varname##_topic_name, type>;

//     template<const char* TName>
//     class ROSNode
//     {
//         std::shared_ptr<rclcpp::Node> NodePtr{};
//         std::map<std::string, rclcpp::PublisherBase::SharedPtr> Publishers{};
//         std::map<std::string, rclcpp::SubscriptionBase::SharedPtr> Subscribers{};

//     public:
//         ROSNode() noexcept = default;

//         bool Initialize(int argc, char** argv) noexcept try
//         {
//             if (!rclcpp::ok()) {
//                 rclcpp::init(argc, argv);
//             }
//             NodePtr = std::make_shared<rclcpp::Node>(TName);
//             return true;
//         }
//         catch (const std::exception& ex)
//         {
//             RCLCPP_ERROR(rclcpp::get_logger("default"), "ROSNode::Initialize: %s", ex.what());
//             return false;
//         }

//         template<typename TTopic>
//         auto Publisher()
//         {
//             std::string name{ TTopic::Name };
//             if (!Publishers.contains(name))
//             {
//                 // 使用 ROS2 的方式?建?布者
//                 auto publisher = NodePtr->create_publisher<typename TTopic::Msg>(name, rclcpp::QoS(10));
//                 Publishers[name] = publisher;
//                 return publisher;
//             }
//             return Publishers[name];  // 直接返回共享指?
//         }

//         template<typename TTopic>
//         void GenSubscriber(auto callback)
//         {
//             std::string name{ TTopic::Name };
//             if (Subscribers.contains(name)) return;
            
//             // 使用 ROS2 的方式?建??者
//             auto subscriber = NodePtr->create_subscription<typename TTopic::Msg>(
//                 name, 
//                 rclcpp::QoS(10), 
//                 callback
//             );
//             Subscribers[name] = subscriber;
//         }

//         template<typename T>
//         bool GetParam(const std::string& name, T& value, const T& defaultValue) noexcept
//         {
//             try {
//                 // 使用 ROS2 的方式?取??
//                 value = NodePtr->declare_parameter<T>(name, defaultValue);
//                 return true;
//             } catch (const std::exception& ex) {
//                 RCLCPP_ERROR(rclcpp::get_logger("default"), "ROSNode::GetParam: %s", ex.what());
//                 return false;
//             }
//         }

//         // 添加?取??的方法，用于 spin
//         std::shared_ptr<rclcpp::Node> GetNode() const
//         {
//             return NodePtr;
//         }

//         // 添加?查??是否有效的方法
//         bool IsOK() const
//         {
//             return rclcpp::ok() && NodePtr != nullptr;
//         }
//     };
// }


// #pragma once

// #include <map>
// #include <memory>
// #include <functional>
// #include <mutex>

// #include <fmt/format.h>
// #include <fmt/chrono.h>

// #include <ros/ros.h>

// namespace LangYa::roslog
// {
// #define LY_DEF_ROS_LOG(name, rosName)\
//     template<typename ...Args> \
//     void name(const auto fmt, Args &&... args) \
//     { \
//         ROS_##rosName##_STREAM(fmt::format(fmt, std::forward<Args>(args)...)); \
//     } \

//     LY_DEF_ROS_LOG(error, ERROR);
//     LY_DEF_ROS_LOG(warn, WARN);
//     LY_DEF_ROS_LOG(info, INFO);
//     LY_DEF_ROS_LOG(debug, DEBUG);
// #undef LY_DEF_ROS_LOG

// }

// namespace LangYa
// {
//     template<typename TGlobal>
//     class MultiCallback
//     {
//         std::function<void(const TGlobal&)> PostEvent;
//         std::mutex Mutex{};
//         TGlobal Global{};

//     public:
//         MultiCallback(std::function<void(const TGlobal&)> postEvent) noexcept : PostEvent(std::move(postEvent)) {}

//         template<typename TTopic>
//         std::function<void(typename TTopic::CallbackArg)> Generate(
//                 std::function<void(TGlobal&, typename TTopic::Msg const&)> modifier
//         ) noexcept
//         {
//             return [this, modifier](typename TTopic::CallbackArg arg)
//             {
//                 std::lock_guard lock{ Mutex };
//                 modifier(Global, *arg);
//                 PostEvent(Global);
//             };
//         }
//     };

//     template<const char* TName, typename TMessage>
//     struct ROSTopic
//     {
//         static constexpr auto Name = TName;
//         using Msg = TMessage;
//         using CallbackArg = typename Msg::ConstPtr;
//     };

// #define LY_DEF_ROS_TOPIC(varname, topic, type)\
//     inline static constexpr const char varname##_topic_name[] = topic;\
//     using varname = LangYa::ROSTopic<varname##_topic_name, type>;

//     template<const char* TName>
//     class ROSNode
//     {
//         std::unique_ptr<ros::NodeHandle> NodePtr{};
//         std::map<std::string, ros::Publisher> Publishers{};
//         std::map<std::string, ros::Subscriber> Subscribers{};

//     public:
//         ROSNode() noexcept = default;

//         bool Initialize(int argc, char** argv) noexcept try
//         {
//             if (!ros::isInitialized()) ros::init(argc, argv, TName);
//             NodePtr = std::make_unique<ros::NodeHandle>();
//             return true;
//         }
//         catch (const std::exception& ex)
//         {
//             ROS_ERROR("ROSNode::Initialize: %s", ex.what());
//             return false;
//         }

//         template<typename TTopic>
//         ros::Publisher& Publisher()
//         {
//             std::string name{ TTopic::Name };
//             if (!Publishers.contains(name))
//                 Publishers.insert({ name, NodePtr.get()->advertise<typename TTopic::Msg>(name, 3) });
//             return Publishers[name];
//         }

//         template<typename TTopic>
//         void GenSubscriber(auto callback)
//         {
//             std::string name{ TTopic::Name };
//             if (Subscribers.contains(name)) return;
//             Subscribers.insert({ name, NodePtr.get()->subscribe<typename TTopic::Msg>(name, 3, callback) });
//         }

//         template<typename T>
//         bool GetParam(const std::string& name, T& value, const T& defaultValue) noexcept
//         {
//             if(NodePtr.get()->param(name, value, defaultValue)) return true;
//             roslog::error("ROSNode::GetParam: cannot get param({})", name);
//             return false;
//         }
//     };


// }

