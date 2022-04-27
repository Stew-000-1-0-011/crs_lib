#pragma once

#include <cstdint>
#ifdef __cpp_lib_concepts
#include <concepts>
#endif

#include <can_plugins/Frame.h>

namespace CRSLib
{
    namespace
    {
        template<class Tag_, class Callback, class CanBuffer>
#ifdef __cpp_concepts
        requires requires(Callback callback, CanBuffer buffer, const can_plugins::Frame::ConstPtr& frame_p)
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

        private:
            bool is_active{true};

        public:
            void push_and_call(const can_plugins::Frame::ConstPtr& frame_p) noexcept
            {
                if(frame_p->id != can_id || !is_active) return;

                if(buffer.push(frame_p))
                {
                    callback.callback(buffer.get_value());
                }
            }

            void activate() noexcept
            {
                is_active = true;
                buffer.clear();
            }

            void deactivate() noexcept
            {
                is_active = false;
                buffer.clear();
            }
        };

        namespace Implement::CanSubscriber
        {
            template<class T>
            struct is_can_subscriber_t : std::false_type
            {};

            template<class Tag, class CanCallback, class CanBuffer>
            struct is_can_subscriber_t<CRSLib::CanSubscriber<Tag, CanCallback, CanBuffer>> : std::true_type
            {};
        }

        namespace Concept
        {
            template<class T>
            concept CanSubscriber = Implement::CanSubscriber::is_can_subscriber_t<T>::value;
        }
    }
}