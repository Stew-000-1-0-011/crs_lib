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

// namespace CRSLib
// {
//     namespace
//     {
//         namespace RosparamUtil
//         {
//             template<class Variable, class Functor, class DefaultValue>
// #ifdef __cpp_concepts
//             requires requires (Variable obj, Functor pred, DefaultValue default_value)
//             {
//                 {pred.template operator()<Variable>(obj)} noexcept -> std::convertible_to<bool>;
//                 obj = default_value;
//             }
// #endif
//             bool assert_param(Variable& obj, Functor pred, const DefaultValue default_value, const char *const obj_name = "") noexcept
//             {
//                 if(!(pred.template operator()<Variable>(obj)))
//                 {
//                     ROS_ERROR_STREAM(
//                         "Stew: rosparam error. The value " << obj_name << " does not satisfy predicate " << pred.pred_name << ". "
//                         << "This variable is set to the default value of " << default_value << '.'
//                     );
//                     obj = default_value;
//                     return false;
//                 }
//                 return true;
//             }

//             // std::size_tじゃなくてintでいいの？ -> そもそもoperator[]がintになってる
//             std::optional<XmlRpc::XmlRpcValue> get_param(const std::optional<XmlRpc::XmlRpcValue>& value, const int index) noexcept
//             {
//                 if(!value.has_value())
//                 {
//                     return std::nullopt;
//                 }
//                 else if(value->getType() != XmlRpc::XmlRpcValue::TypeArray)
//                 {
//                     ROS_ERROR("Stew: XmlRpcValue is not array.");
//                     return std::nullopt;
//                 }
//                 else if(index >= value->size())
//                 {
//                     ROS_ERROR("Stew: out range access of XmlRpcValue.");
//                 }

//                 return (*value)[index];
//             }

//             std::optional<XmlRpc::XmlRpcValue> get_param(const std::optional<XmlRpc::XmlRpcValue>& value, const char *const str) noexcept
//             {
//                 if(!value.has_value())
//                 {
//                     return std::nullopt;
//                 }
//                 else if(value->getType() != XmlRpc::XmlRpcValue::TypeStruct)
//                 {
//                     ROS_ERROR("Stew: XmlRpcValue is not struct.");
//                     return std::nullopt;
//                 }
//                 else if(const auto ret_value = (*value)[str]; !ret_value.valid())
//                 {
//                     ROS_ERROR("Stew: invalid key for XmlRpcValue, or This key has no value.");
//                     return std::nullopt;
//                 }
                
//                 return (*value)[str];
//             }

//             std::optional<XmlRpc::XmlRpcValue> get_param(const std::optional<XmlRpc::XmlRpcValue>& value, const std::string& str) noexcept
//             {
//                 if(!value.has_value())
//                 {
//                     return std::nullopt;
//                 }
//                 else if(value->getType() != XmlRpc::XmlRpcValue::TypeStruct)
//                 {
//                     ROS_ERROR("Stew: XmlRpcValue is not struct.");
//                     return std::nullopt;
//                 }
//                 else if(const auto ret_value = (*value)[str]; !ret_value.valid())
//                 {
//                     ROS_ERROR("Stew: invalid key for XmlRpcValue, or This key has no value.");
//                     return std::nullopt;
//                 }

//                 return (*value)[str];
//             }

//             namespace Implement
//             {
// #ifdef __cpp_concepts
//                 template<class T>
//                 concept is_xml_rpc_type =
//                     std::same_as<T, bool> || std::same_as<T, int>
//                     || std::same_as<T, double> || std::same_as<T, std::string>
//                     || std::same_as<T, struct tm> || std::same_as<T, std::vector<char>>;
// #endif
//                 // arrayとstructはキャストできないので無い。
//                 template<class T>
// #ifdef __cpp_concepts
//                 requires is_xml_rpc_type<T>
// #endif
//                 inline constexpr XmlRpc::XmlRpcValue::Type type_enum = XmlRpc::XmlRpcValue::Type::TypeInvalid;

//                 template<>
//                 inline constexpr XmlRpc::XmlRpcValue::Type type_enum<bool> = XmlRpc::XmlRpcValue::Type::TypeBoolean;
//                 template<>
//                 inline constexpr XmlRpc::XmlRpcValue::Type type_enum<int> = XmlRpc::XmlRpcValue::Type::TypeInt;
//                 template<>
//                 inline constexpr XmlRpc::XmlRpcValue::Type type_enum<double> = XmlRpc::XmlRpcValue::Type::TypeDouble;
//                 template<>
//                 inline constexpr XmlRpc::XmlRpcValue::Type type_enum<std::string> = XmlRpc::XmlRpcValue::Type::TypeString;
//                 template<>
//                 inline constexpr XmlRpc::XmlRpcValue::Type type_enum<struct tm> = XmlRpc::XmlRpcValue::Type::TypeDateTime;
//                 template<>
//                 inline constexpr XmlRpc::XmlRpcValue::Type type_enum<std::vector<char>> = XmlRpc::XmlRpcValue::Type::TypeBase64;
//             }

//             // XmlRpcValue::Typeとそのクラスを紐づける変数テンプレートなりが欲しかった。作った。
//             template<class T>
// #ifdef __cpp_concepts
//             requires Implement::is_xml_rpc_type<T>
// #endif
//             T xml_rpc_cast(const std::optional<XmlRpc::XmlRpcValue>& value) noexcept
//             {
//                 if(!value.has_value())
//                 {
//                     return {};
//                 }
//                 else if(value->getType() != Implement::type_enum<T>)
//                 {
//                     ROS_ERROR("Stew: xml_rpc_cast is invaild.");
//                     return {};
//                 }
                
//                 return static_cast<T>(*value);
//             }

//             template<class T>
// #ifdef __cpp_concepts
//             requires Implement::is_xml_rpc_type<T>
// #endif
//             T read_param(const std::optional<XmlRpc::XmlRpcValue>& value, const int key) noexcept
//             {
//                 return xml_rpc_cast<T>(get_param(value, key));
//             }

//             template<class T>
// #ifdef __cpp_concepts
//             requires Implement::is_xml_rpc_type<T>
// #endif
//             T read_param(const std::optional<XmlRpc::XmlRpcValue>& value, const char *const key) noexcept
//             {
//                 return xml_rpc_cast<T>(get_param(value, key));
//             }

//             template<class T>
// #ifdef __cpp_concepts
//             requires Implement::is_xml_rpc_type<T>
// #endif
//             T read_param(const std::optional<XmlRpc::XmlRpcValue>& value, const std::string& key) noexcept
//             {
//                 return xml_rpc_cast<T>(get_param(value, key));
//             }
//         }
//     }
// }

namespace CRSLib
{
    namespace
    {
        namespace RosparamUtil
        {
            namespace Implement
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


#define stew_define_predicate(pred_name_, pred_body) \
            inline constexpr struct\
            {\
                const char *const pred_name = #pred_name_;\
                template<typename T>\
                constexpr bool operator()(const StewLib::low_cost_ref_val_t<T> obj) const noexcept\
                    pred_body\
            } pred_name_

            stew_define_predicate(is_positive, {return obj > 0;});
            stew_define_predicate(is_not_negative, {return obj >= 0;});
            stew_define_predicate(is_nonzero, {return static_cast<bool>(obj);});

#undef stew_define_predicate


            template<class Variable, class Functor, class DefaultValue>
#ifdef __cpp_lib_concepts
            requires requires (Variable obj, Functor pred, DefaultValue default_value)
            {
                {pred.template operator()<Variable>(obj)} noexcept -> std::convertible_to<bool>;
                obj = default_value;
            }
#endif
            inline bool assert_param(Variable& obj, Functor pred, const DefaultValue default_value, const char *const obj_name = "") noexcept
            {
                if(!(pred.template operator()<Variable>(obj)))
                {
                    ROS_ERROR_STREAM(
                        "Stew: rosparam error. The value " << obj_name << " does not satisfy predicate " << pred.pred_name << ". "
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
                static_assert(Implement::is_string_like<decltype(str)>);

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

            // std::optional<StewXmlRpc> get_param(const std::optional<StewXmlRpc>& xml_opt, const std::string& str) noexcept
            // {
            //     if(!xml_opt.has_value())
            //     {
            //         return std::nullopt;
            //     }
            //     else if(xml_opt->xml.getType() != XmlRpc::XmlRpcValue::TypeStruct)
            //     {
            //         ROS_ERROR("Stew: XmlRpcValue is not struct.");
            //         return std::nullopt;
            //     }
            //     else if(const auto tmp_xml = xml_opt->xml; !tmp_xml[str].valid())
            //     {
            //         ROS_ERROR("Stew: invalid key for XmlRpcValue, or This key has no value.");
            //         return std::nullopt;
            //     }

            //     return StewXmlRpc{xml_opt->xml[str], xml_opt->param_name + "[" + str + "]"};
            // }

            // XmlRpcValue::Typeとそのクラスを紐づける変数テンプレートなりが欲しかった。作った。
            template<class T>
#ifdef __cpp_lib_concepts
            requires Implement::is_xml_rpc_type<T>
#endif
            inline T xml_rpc_cast(const std::optional<StewXmlRpc>& xml_opt) noexcept
            {
                if(!xml_opt.has_value())
                {
                    return {};
                }
                else if(xml_opt->xml.getType() != Implement::type_enum<T>)
                {
                    ROS_ERROR("Stew: xml_rpc_cast is invaild. This type is different from %s.", xml_opt->param_name.c_str());
                    return {};
                }
                
                return static_cast<T>(xml_opt->xml);
            }

            template<class T>
#ifdef __cpp_lib_concepts
            requires Implement::is_xml_rpc_type<T>
#endif
            inline T read_param(const std::optional<StewXmlRpc>& value, const auto& key) noexcept
            {
                return xml_rpc_cast<T>(get_param(value, key));
            }

//             template<class T>
// #ifdef __cpp_concepts
//             requires Implement::is_xml_rpc_type<T>
// #endif
//             T read_param(const std::optional<XmlRpc::XmlRpcValue>& value, const char *const key) noexcept
//             {
//                 return xml_rpc_cast<T>(get_param(value, key));
//             }

//             template<class T>
// #ifdef __cpp_concepts
//             requires Implement::is_xml_rpc_type<T>
// #endif
//             T read_param(const std::optional<XmlRpc::XmlRpcValue>& value, const std::string& key) noexcept
//             {
//                 return xml_rpc_cast<T>(get_param(value, key));
//             }
        }
    }
}