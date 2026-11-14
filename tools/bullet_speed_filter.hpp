#ifndef TOOLS__BULLET_SPEED_FILTER_HPP
#define TOOLS__BULLET_SPEED_FILTER_HPP

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace tools
{
// 固定窗口 + epsilon 去重的弹速采样器。
// 只有新样本与上一个样本的差值超过 eps 时才计入窗口，避免下位机重复发送同一个
// 弹速值时被反复计入均值。是否要求窗口"满"才使用均值（而不是原始读数）由调用方
// 自己决定（见 empty()/full()），这里只统一"存哪些样本、怎么去重"这部分逻辑，
// 不改变 CBoard/Gimbal 各自原有的"窗口未满时怎么办"的行为。
class BulletSpeedFilter
{
public:
  explicit BulletSpeedFilter(size_t window_size = 3, float eps = 1e-3F)
  : window_size_(window_size), eps_(eps), samples_(window_size, 0.0F)
  {
  }

  void push(float value)
  {
    if (has_last_sample_ && std::fabs(value - last_sample_) <= eps_) return;

    samples_[index_] = value;
    index_ = (index_ + 1) % window_size_;
    if (count_ < window_size_) count_++;
    last_sample_ = value;
    has_last_sample_ = true;
  }

  bool empty() const { return count_ == 0; }
  bool full() const { return count_ == window_size_; }
  size_t size() const { return count_; }

  // 当前已有样本（不要求窗口满）的平均值，调用前应先用 empty() 判断。
  float average() const
  {
    float sum = 0.0F;
    for (size_t i = 0; i < count_; i++) sum += samples_[i];
    return sum / static_cast<float>(count_);
  }

  void reset()
  {
    std::fill(samples_.begin(), samples_.end(), 0.0F);
    index_ = 0;
    count_ = 0;
    has_last_sample_ = false;
  }

private:
  size_t window_size_;
  float eps_;
  std::vector<float> samples_;
  size_t index_ = 0;
  size_t count_ = 0;
  float last_sample_ = 0.0F;
  bool has_last_sample_ = false;
};

}  // namespace tools

#endif  // TOOLS__BULLET_SPEED_FILTER_HPP
