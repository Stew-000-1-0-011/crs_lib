#pragma once

#include <cstdint>
#ifdef __cpp_lib_concepts
#include <concepts>
#endif

#include <mutex>

#include <can_plugins/Frame.h>

namespace CRSLib
{
    struct EmptyCallback
    {
        void callback(auto) noexcept{}
    };

    template<class T>
    concept IsCanBuffer = requires(T buffer, const can_plugins::FrameConstPtr& frame_p)
    {
        {buffer.push(frame_p)} noexcept -> std::convertible_to<bool>;
        {buffer.get_value()} noexcept;
        {buffer.clear()} noexcept;
    };

    template<class Tag_, class Callback, class CanBuffer>
#ifdef __cpp_concepts
    requires requires(Callback callback, CanBuffer buffer, const can_plugins::FrameConstPtr& frame_p)
    {
        {buffer.push(frame_p)} noexcept -> std::convertible_to<bool>;
        {buffer.get_value()} noexcept;
        {buffer.clear()} noexcept;
        requires
        requires(decltype(buffer.get_value()) value)
        {
            {callback.callback(value)} noexcept;
        };
    }
#endif
    struct CanSubscriber final
    {
        using Tag = Tag_;
        std::uint32_t can_id{};
        CanBuffer buffer{};
        Callback callback{};
        std::mutex mtx{};

    private:
        bool is_active{true};

    public:
        constexpr CanSubscriber(const std::uint32_t can_id, const CanBuffer& buffer, const Callback& callback) noexcept:
            can_id{can_id},
            buffer{buffer},
            callback{callback}
        {}

        CanSubscriber(const CanSubscriber& obj):
            can_id{obj.can_id},
            buffer{obj.buffer},
            callback{obj.callback}
        {}

        CanSubscriber& operator=(const CanSubscriber& obj)
        {
            if(&obj == this) return *this;
            can_id = obj.can_id;
            buffer = obj.buffer;
            callback = obj.callbck;
            return *this;
        }

        CanSubscriber(CanSubscriber&& obj):
            can_id{obj.can_id},
            buffer{std::move(obj.buffer)},
            callback{std::move(obj.callback)}
        {}

        CanSubscriber& operator=(CanSubscriber&& obj)
        {
            can_id = obj.can_id;
            buffer = std::move(obj.buffer);
            callback = std::move(obj.callbck);
            return *this;
        }

        void push_and_call(const can_plugins::Frame::ConstPtr& frame_p) noexcept
        {
            if(frame_p->id != can_id || !is_active) return;

            if constexpr(!std::same_as<Callback, EmptyCallback>)
            {
                std::lock_guard lck{mtx};
                if(buffer.push(frame_p))
                {
                    callback.callback(buffer.get_value());
                }
            }
            else
            {
                std::lock_guard lck{mtx};
                buffer.push(frame_p);
            }
        }

        void activate() noexcept
        {
            std::lock_guard lck{mtx};
            is_active = true;
            buffer.clear();
        }

        void deactivate() noexcept
        {
            std::lock_guard lck{mtx};
            is_active = false;
            buffer.clear();
        }
    };

    namespace Implement::CanSubscriberImp
    {
        template<class T>
        struct is_can_subscriber_t : std::false_type
        {};

        template<class Tag, class CanCallback, class CanBuffer>
        struct is_can_subscriber_t<CRSLib::CanSubscriber<Tag, CanCallback, CanBuffer>> : std::true_type
        {};
    }

    template<class T>
    concept IsCanSubscriber = Implement::CanSubscriberImp::is_can_subscriber_t<T>::value;
}