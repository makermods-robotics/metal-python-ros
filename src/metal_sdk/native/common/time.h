#include <atomic>
#include <chrono>
#include <thread>

namespace makermods {
namespace metal {

class LoopRate {
 public:
  // hz: 要求频率
  explicit LoopRate(double hz)
      : period_double_(1.0 / hz),
        period_ns_(std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(period_double_))),
        last_time_(std::chrono::steady_clock::now()) {}

  void sleep() {
    auto now = std::chrono::steady_clock::now();
    auto elapsed = now - last_time_;

    if (elapsed < period_ns_) {
      auto sleep_duration = period_ns_ - elapsed;
      std::this_thread::sleep_for(sleep_duration);
      // 更新为上一次 +一个周期
      last_time_ += period_ns_;
    } else {
      // 如果超时（处理比周期慢），就把 last_time_ 设为当前时间
      last_time_ = now;
    }
  }

  void reset() { last_time_ = std::chrono::steady_clock::now(); }

 private:
  double period_double_;                // 周期的浮点秒表示
  std::chrono::nanoseconds period_ns_;  // 周期的整数纳秒表示
  std::chrono::time_point<std::chrono::steady_clock> last_time_;
};

}  // namespace metal
}  // namespace makermods