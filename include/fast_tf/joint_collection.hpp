#pragma once

#include <tuple>
#include <type_traits>
#include <utility>

#include <eigen3/Eigen/Dense>

#include "joint.hpp"

namespace fast_tf {

namespace internal {

// https://stackoverflow.com/a/57528226
template <typename T, typename... Ts>
struct unique : std::type_identity<T> {};

template <typename... Ts, typename U, typename... Us>
struct unique<std::tuple<Ts...>, U, Us...>
    : std::conditional_t<
          (std::is_same_v<U, Ts> || ...), unique<std::tuple<Ts...>, Us...>,
          unique<std::tuple<Ts..., U>, Us...>> {};

template <typename... Ts>
using unique_tuple = typename unique<std::tuple<>, Ts...>::type;

// https://stackoverflow.com/a/41171291
template <typename T, typename U>
struct has_type;

template <typename T, typename... Us>
struct has_type<T, std::tuple<Us...>> : std::disjunction<std::is_same<T, Us>...> {};

} // namespace internal

template <internal::is_non_root_link... ChildLinkTs>
class JointCollection {
public:
    using TupleT = internal::unique_tuple<Joint<ChildLinkTs>...>;

public:
    JointCollection() = default;

    [[deprecated("Link types passed in as template parameter is duplicated.")]]
    JointCollection() requires(!std::is_same_v<TupleT, std::tuple<Joint<ChildLinkTs>...>>)
    = default;

    template <typename ChildT>
    struct contains_joint {
        static constexpr bool value = internal::has_type<Joint<ChildT>, TupleT>::value;
    };

    template <typename ChildT>
    static constexpr bool contains_joint_v = contains_joint<ChildT>::value;

    template <internal::is_link From, internal::is_link To, internal::is_transform TransformT>
    requires(internal::has_transform_joint<From, To> && contains_joint_v<To>)
    void set_transform(TransformT&& transform) {
        std::get<fast_tf::Joint<To>>(collection_).transform = std::forward<TransformT>(transform);
    }

    template <internal::is_link From, internal::is_link To, internal::is_translation TranslationT>
    requires(internal::has_translation_joint<From, To> && contains_joint_v<To>)
    void set_transform(TranslationT&& translation) {
        std::get<fast_tf::Joint<To>>(collection_).transform =
            std::forward<TranslationT>(translation);
    }

    template <internal::is_link From, internal::is_link To, internal::is_translation TranslationT>
    requires(internal::has_transform_joint<From, To> && contains_joint_v<To>)
    void set_transform(TranslationT&& translation) {
        std::get<fast_tf::Joint<To>>(collection_).transform.translation() =
            std::forward<TranslationT>(translation).translation();
    }

    template <internal::is_link From, internal::is_link To, internal::is_rotation RotationT>
    requires(internal::has_rotation_joint<From, To> && contains_joint_v<To>)
    void set_transform(RotationT&& rotation) {
        std::get<fast_tf::Joint<To>>(collection_).transform = std::forward<RotationT>(rotation);
    }

    template <internal::is_link From, internal::is_link To, internal::is_rotation RotationT>
    requires(internal::has_transform_joint<From, To> && contains_joint_v<To>)
    void set_transform(RotationT&& rotation) {
        std::get<fast_tf::Joint<To>>(collection_).transform.linear() =
            std::forward<RotationT>(rotation).matrix();
    }

    template <internal::is_link From, internal::is_link To, typename... Ts>
    requires(internal::has_joint<From, To> && contains_joint_v<To>) void set_state(Ts&&... value) {
        std::get<fast_tf::Joint<To>>(collection_).set_state(std::forward<Ts>(value)...);
    }

    template <internal::is_link From, internal::is_link To>
    requires(internal::has_joint<From, To> && contains_joint_v<To>) auto& get_transform() {
        return std::get<fast_tf::Joint<To>>(collection_).transform;
    }

    template <internal::is_link From, internal::is_link To>
    requires(internal::has_joint<From, To> && contains_joint_v<To>)
    const auto& get_transform() const {
        return std::get<fast_tf::Joint<To>>(collection_).transform;
    }

    // private:
    TupleT collection_;
};

} // namespace fast_tf