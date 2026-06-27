#pragma once
/**
 * @file FuncModule.hpp
 * @brief MBD FuncModule 基类模板（值语义、Traits 五元结构）
 *
 * Traits 必须包含：Input / Output / Param / State / Sub 五个嵌套类型。
 * 派生类通过 `using FuncModule::FuncModule;` 引入构造函数；
 * 必须覆写 `void run(const Input&, Output&)`。
 */

#include <utility>

namespace cbf_mbd {

template <typename Traits>
class FuncModule {
public:
  using Input = typename Traits::Input;
  using Output = typename Traits::Output;
  using Param = typename Traits::Param;
  using State = typename Traits::State;
  using Sub = typename Traits::Sub;

  FuncModule() = default;

  explicit FuncModule(Param param) : param_(std::move(param)) {}

  FuncModule(Param param, Sub sub)
      : param_(std::move(param)), sub_(std::move(sub)) {}

  virtual ~FuncModule() = default;

  void setParam(Param param) { param_ = std::move(param); }
  void setState(State state) { state_ = std::move(state); }

  const Param& getParam() const { return param_; }
  const State& getState() const { return state_; }
  const Sub& getSub() const { return sub_; }
  Sub& mutableSub() { return sub_; }

  virtual void run(const Input& input, Output& output) = 0;

protected:
  Param param_{};
  State state_{};
  Sub sub_{};
};

}  // namespace cbf_mbd
