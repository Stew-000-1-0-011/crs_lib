/*
・なんでXmlRpcValueは内部のasStringやらasArrayやらを直接触らせてくれないんだろう。
・なんでtime.hをインクルードしているんだ？ctimeじゃダメなのか？？struct tmとか書きたくないんだが。
てかC++ならもっと高級なのあったような...遅いから？メモリ食うから？
*/

#pragma once

#include <string>
#include <vector>
#include <optional>

#ifdef __cpp_lib_concepts
#include <concepts>
#endif

#include <ros/ros.h>

#include "StewLib/judge_lower_cost_than_ref.hpp"

namespace CRSLib
{
    namespace RosparamUtil
    {
        namespace Implement::RosparamUtilImp
        {
            template<class T>
#ifdef __cpp_lib_concepts
            concept is_string_like = std::convertible_to<T, const char *const> || std::convertible_to<T, const std::string&>;
#else
            inline constexpr bool is_string_like = std::is_convertible_v<T, const char *const> || std::is_convertible_v<T, const std::string&>;
#endif

#ifdef __cpp_lib_concepts
            template<class T>
            concept is_xml_rpc_type =
                std::same_as<T, bool> || std::same_as<T, int>
                || std::same_as<T, double> || std::same_as<T, std::string>
                || std::same_as<T, struct tm> || std::same_as<T, std::vector<char>>;
#endif
            // arrayとstructはキャストできないので無い。
            template<class T>
#ifdef __cpp_lib_concepts
            requires is_xml_rpc_type<T>
#endif
            inline constexpr XmlRpc::XmlRpcValue::Type type_enum = XmlRpc::XmlRpcValue::Type::TypeInvalid;

            template<>
            inline constexpr XmlRpc::XmlRpcValue::Type type_enum<bool> = XmlRpc::XmlRpcValue::Type::TypeBoolean;
            template<>
            inline constexpr XmlRpc::XmlRpcValue::Type type_enum<int> = XmlRpc::XmlRpcValue::Type::TypeInt;
            template<>
            inline constexpr XmlRpc::XmlRpcValue::Type type_enum<double> = XmlRpc::XmlRpcValue::Type::TypeDouble;
            template<>
            inline constexpr XmlRpc::XmlRpcValue::Type type_enum<std::string> = XmlRpc::XmlRpcValue::Type::TypeString;
            template<>
            inline constexpr XmlRpc::XmlRpcValue::Type type_enum<struct tm> = XmlRpc::XmlRpcValue::Type::TypeDateTime;
            template<>
            inline constexpr XmlRpc::XmlRpcValue::Type type_enum<std::vector<char>> = XmlRpc::XmlRpcValue::Type::TypeBase64;
        }

        template<class Variable, class DefaultValue>
        inline bool assert_param(Variable& obj, bool is_valid, const DefaultValue& default_value, const char *const var_name = "") noexcept
        {
            if(!is_valid)
            {
                ROS_ERROR_STREAM(
                    "Stew: rosparam error. The variable " << var_name << " does not satisfy predicate. "
                    << "This variable is set to the default value of " << default_value << '.'
                );
                obj = default_value;
                return false;
            }
            return true;
        }

        struct StewXmlRpc final
        {
            XmlRpc::XmlRpcValue xml;
            std::string param_name{};
        };

        inline std::optional<StewXmlRpc> get_param(const ros::NodeHandle& nh, const std::string& key) noexcept
        {
            XmlRpc::XmlRpcValue xml{};
            try
            {
                nh.getParam(key, xml);
            }
            catch(ros::InvalidNameException& except)
            {
                ROS_ERROR("Stew: %s is invalid key.", key.c_str());
                return std::nullopt;
            }
            return StewXmlRpc{xml, key};
        }

        // std::size_tじゃなくてintでいいの？ -> そもそもoperator[]がintになってる
        inline std::optional<StewXmlRpc> get_param(const std::optional<StewXmlRpc>& xml_opt, const int index) noexcept
        {
            if(!xml_opt.has_value())
            {
                return std::nullopt;
            }
            if(xml_opt->xml.getType() != XmlRpc::XmlRpcValue::TypeArray)
            {
                ROS_ERROR("Stew: %s is not array.", xml_opt->param_name.c_str());
                return std::nullopt;
            }
            if(index >= xml_opt->xml.size())
            {
                ROS_ERROR("Stew: out range access of %s.", xml_opt->param_name.c_str());
            }

            return StewXmlRpc{xml_opt->xml[index], xml_opt->param_name + "[" + std::to_string(index) + "]"};
        }

        inline std::optional<StewXmlRpc> get_param(const std::optional<StewXmlRpc>& xml_opt, const auto& str) noexcept
        {
            static_assert(Implement::RosparamUtilImp::is_string_like<decltype(str)>);

            if(!xml_opt.has_value())
            {
                return std::nullopt;
            }
            if(xml_opt->xml.getType() != XmlRpc::XmlRpcValue::TypeStruct)
            {
                ROS_ERROR("Stew: %s is not struct.", xml_opt->param_name.c_str());
                return std::nullopt;
            }
            if(const auto tmp_xml = xml_opt->xml; !tmp_xml[str].valid())
            {
                ROS_ERROR("Stew: invalid key %s for %s, or The value of this key is none.", std::string(std::move(str)).c_str(), xml_opt->param_name.c_str());
                return std::nullopt;
            }
            
            return StewXmlRpc{xml_opt->xml[str], xml_opt->param_name + "[" + str + "]"};
        }

        // XmlRpcValue::Typeとそのクラスを紐づける変数テンプレートなりが欲しかった。作った。
        template<class T>
#ifdef __cpp_lib_concepts
        requires Implement::RosparamUtilImp::is_xml_rpc_type<T>
#endif
        inline T xml_rpc_cast(const std::optional<StewXmlRpc>& xml_opt) noexcept
        {
            if(!xml_opt.has_value())
            {
                return {};
            }
            else if(xml_opt->xml.getType() != Implement::RosparamUtilImp::type_enum<T>)
            {
                ROS_ERROR("Stew: xml_rpc_cast is invaild. This type is different from %s.", xml_opt->param_name.c_str());
                return {};
            }
            
            return static_cast<T>(xml_opt->xml);
        }

        template<class T>
#ifdef __cpp_lib_concepts
        requires Implement::RosparamUtilImp::is_xml_rpc_type<T>
#endif
        inline T read_param(const std::optional<StewXmlRpc>& value, const auto& key) noexcept
        {
            return xml_rpc_cast<T>(get_param(value, key));
        }
    }
}