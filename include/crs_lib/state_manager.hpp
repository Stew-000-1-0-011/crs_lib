#include <cstdint>
#include <atomic>
#include <type_traits>

#include <ros/ros.h>

#ifndef __cpp_lib_concepts
#include "StewLib/has_member.hpp"
#endif

#include "std_msgs_type_convertor.hpp"

#ifndef __cpp_lib_concepts
Stew_define_has_member(callback, callback)
#endif

namespace CRSLib
{
    namespace
    {
        template<class T>
        struct StateManagerCallback;

        template<typename StateEnum, class T>
#ifdef __cpp_concepts
        requires requires
        {
            requires std::is_enum_v<StateEnum>;
            StateEnum::disable;
        }
#endif
        class StateManager final
        {
            using StateMsg = StdMsgsTypeConvertor<std::underlying_type_t<StateEnum>>;
            inline static ros::Publisher pub{};
            inline static ros::Subscriber sub{};

            std::atomic<StateEnum> state{StateEnum::disable};
            T * this_p;

        public:
            StateManager(ros::NodeHandle& nh, T *const this_p) noexcept:
                this_p{this_p}
            {
                if(!pub) pub = nh.advertise<StateMsg>("crs_lib/stew_state", 10);
                if(!sub) sub = nh.subscribe<StateMsg>("crs_lib/stew_state", 10, &StateManager::callback, this);
            }

            StateEnum get_state() const noexcept
            {
                return state;
            }

            void set_state(const StateEnum state) noexcept
            {
                this->state = state;
                StateMsg state_msg;
                state_msg.data = static_cast<std::uint8_t>(state);
                pub.publish(state_msg);
            }

        private:
            void callback(const StateMsg::ConstPtr& msg_p) noexcept
            {
                state = static_cast<StateEnum>(msg_p->data);

#ifdef __cpp_lib_concepts
                if constexpr(requires{StateManagerCallback<T>::callback(this_p, state);})
                {
                    StateManagerCallback<T>::callback(this_p, state);
                }
#else
                if constexpr(StewLib::HasMember::has_callback_v<StateManagerCallback<T>>)
                {
                    StateManagerCallback<T>::callback(this_p, state);
                }
#endif
            }
        };
    }
}