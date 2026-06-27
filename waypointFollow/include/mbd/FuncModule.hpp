#pragma once

/// @file FuncModule.hpp
/// @brief MBD FuncModule 模板基类, 所有控制模块的统一抽象

#include <utility>

namespace waypoint_follow_mbd {

/// FuncModule 模板基类
/// @tparam Traits 包含 Input/Output/Param/State/Sub 五元结构体的特征类型
template<typename Traits>
class FuncModule {
public:
  using Input  = typename Traits::Input;
  using Output = typename Traits::Output;
  using Param  = typename Traits::Param;
  using State  = typename Traits::State;
  using Sub    = typename Traits::Sub;

  FuncModule() = default;
  explicit FuncModule(const Param& param) : param_(param) {}
  FuncModule(const Param& param, Sub sub)
    : param_(param), sub_(std::move(sub)) {}

  virtual ~FuncModule() = default;

  FuncModule(const FuncModule&) = delete;
  FuncModule& operator=(const FuncModule&) = delete;
  FuncModule(FuncModule&&) = default;
  FuncModule& operator=(FuncModule&&) = default;

  /// 核心计算入口, 子类必须实现
  virtual void run(const Input& input, Output& output) = 0;

  void setParam(const Param& param) { param_ = param; }
  void setState(const State& state) { state_ = state; }
  const Param& getParam() const { return param_; }
  const State& getState() const { return state_; }

protected:
  Param param_{};  ///< 配置参数
  State state_{};  ///< 时序状态
  Sub   sub_{};    ///< 子模块实体
};

} // namespace waypoint_follow_mbd
