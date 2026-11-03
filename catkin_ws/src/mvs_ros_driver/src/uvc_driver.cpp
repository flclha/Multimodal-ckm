#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
#include <opencv2/opencv.hpp>
#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <linux/uvcvideo.h>
#include <linux/usb/video.h>
#include <signal.h>
#include <cstring>
#include <vector>
#include <chrono>
#include <thread>
#include <atomic>
#include <algorithm>
#include <cmath>
#include <cerrno>
#include <ctime>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <deque>
#include <limits>
#include <mutex>
#include <condition_variable>

using namespace std;

struct time_stamp {
  int64_t high;
  int64_t low;
};
volatile time_stamp *pointt = (volatile time_stamp *)MAP_FAILED;
volatile time_stamp *clock_offset_pointt = (volatile time_stamp *)MAP_FAILED;
volatile time_stamp *lidar_timeshare_pointt = (volatile time_stamp *)MAP_FAILED;

bool exit_flag = false;
int width = 640, height = 480; // 默认分辨率，可从参数调整
image_transport::Publisher pub;
ros::Publisher v4l2_timing_pub;
float image_scale = 1.0;
int trigger_enable = 1;
int trigger_backlight_mode = 2;
string device_name = "/dev/video0"; // 默认设备
string pixel_format = "YUYV";
int frame_rate = 0;
int timestamp_wait_ms = 30;
string timestamp_source = "v4l2_soe_lidar_clock";
string stm32_trigger_path = "/home/nuc/stm32_trigger_time";
string clock_offset_path = "/home/nuc/mid360_clock_offset";
string lidar_timeshare_path = "/home/nuc/timeshare";
int64_t stm32_trigger_period_us = 100000;
int64_t stm32_trigger_to_exposure_center_us = 4760;
int64_t stm32_trigger_phase_us = 0;
int64_t stm32_pps_minus_mid360_integer_ns = 0;
uint32_t stm32_sequence_reset_threshold = 100;
int64_t camera_calibration_offset_us = 0;
int64_t v4l2_soe_to_exposure_center_us = 3000;
int stm32_v4l2_init_frames = 20;
int64_t stm32_v4l2_init_max_residual_ns = 20000000LL;
string v4l2_timing_topic = "/uvc_camera/v4l2_timing";
bool pts_metadata_enable = false;
string pts_metadata_device = "/dev/video1";
double pts_clock_hz = 48000000.0;
int64_t pts_match_max_delta_us = 20000;
double camera_clock_scale = 1.0;
int64_t camera_clock_offset_ns = 0;
bool camera_clock_calibration_valid = false;
int64_t exposure_center_offset_ns = 0;
int64_t camera_calibration_offset_ns = 0;
int session_anchor_warmup_ms = 500;
int session_anchor_collect_ms = 3000;
int session_anchor_min_pairs = 20;
double session_anchor_max_abs_mean_ms = 1.0;
double session_anchor_max_p95_ms = 3.0;
double session_anchor_max_abs_drift_ms_per_s = 0.01;
int session_anchor_pts_reset_missing_frames = 5;
int64_t v4l2_monitor_reference_residual_ns = 0;
int64_t last_trigger_seq = 0;
int64_t last_predicted_trigger_time_ns = 0;
int64_t last_mid360_pps_epoch_ns = 0;
int64_t last_stm32_pps_epoch_ns = 0;
int64_t last_trigger_index_in_second = -1;
uint32_t last_camera_sequence = 0;
bool camera_sequence_initialized = false;
int64_t last_camera_stamp_ns = 0;
uint64_t hardware_stamp_count = 0;
uint64_t fallback_stamp_count = 0;
uint64_t duplicate_frame_drop_count = 0;
bool v4l2_fixed_anchor_initialized = false;
int64_t v4l2_anchor_ns = 0;
int64_t mid360_anchor_ns = 0;
int64_t last_raw_v4l2_soe_ns = 0;
int64_t last_fixed_mapped_soe_ns = 0;
uint64_t last_pts_unwrapped_ticks = 0;
int64_t last_pts_mapped_ns = 0;
int64_t last_diagnostic_clock_offset_ns = 0;
int64_t last_diagnostic_clock_offset_seq = 0;
int64_t v4l2_stats_window_start_ns = 0;
uint64_t v4l2_stats_header_duplicate_count = 0;
uint64_t v4l2_stats_header_rollback_count = 0;
uint64_t v4l2_stats_sequence_jump_count = 0;
std::vector<double> v4l2_stats_raw_period_ms;
std::vector<double> v4l2_stats_mapped_period_ms;
std::ofstream time_diag_csv;

uint16_t ReadLe16(const uint8_t *p) {
  return static_cast<uint16_t>(p[0]) |
         (static_cast<uint16_t>(p[1]) << 8);
}

uint32_t ReadLe32(const uint8_t *p) {
  return static_cast<uint32_t>(p[0]) |
         (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) |
         (static_cast<uint32_t>(p[3]) << 24);
}

struct PtsFrameMetadata {
  uint32_t raw_pts = 0;
  uint64_t unwrapped_pts = 0;
  bool unwrap_valid = false;
  uint32_t first_scr_stc = 0;
  uint16_t first_scr_sof = 0;
  uint16_t first_host_sof = 0;
  uint64_t first_payload_monotonic_ns = 0;
  uint32_t first_meta_sequence = 0;
  int fid = -1;
  bool consumed = false;
};

class PtsMetadataReader {
 public:
  ~PtsMetadataReader() { Close(); }

  bool Open(const std::string &device) {
    fd_ = open(device.c_str(), O_RDWR | O_NONBLOCK);
    if (fd_ < 0) {
      ROS_ERROR("Unable to open UVC metadata device %s: %s", device.c_str(),
                std::strerror(errno));
      return false;
    }

    v4l2_requestbuffers request = {};
    request.count = 8;
    request.type = V4L2_BUF_TYPE_META_CAPTURE;
    request.memory = V4L2_MEMORY_MMAP;
    if (ioctl(fd_, VIDIOC_REQBUFS, &request) < 0 || request.count < 2) {
      ROS_ERROR("Unable to request UVC metadata buffers on %s: %s",
                device.c_str(), std::strerror(errno));
      Close();
      return false;
    }

    mappings_.resize(request.count);
    for (uint32_t index = 0; index < request.count; ++index) {
      v4l2_buffer buffer = {};
      buffer.type = request.type;
      buffer.memory = request.memory;
      buffer.index = index;
      if (ioctl(fd_, VIDIOC_QUERYBUF, &buffer) < 0) {
        ROS_ERROR("VIDIOC_QUERYBUF failed for UVC metadata: %s",
                  std::strerror(errno));
        Close();
        return false;
      }
      mappings_[index].length = buffer.length;
      mappings_[index].address =
          mmap(nullptr, buffer.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_,
               buffer.m.offset);
      if (mappings_[index].address == MAP_FAILED ||
          ioctl(fd_, VIDIOC_QBUF, &buffer) < 0) {
        ROS_ERROR("Unable to map/queue UVC metadata buffer: %s",
                  std::strerror(errno));
        Close();
        return false;
      }
    }

    v4l2_buf_type type = V4L2_BUF_TYPE_META_CAPTURE;
    if (ioctl(fd_, VIDIOC_STREAMON, &type) < 0) {
      ROS_ERROR("VIDIOC_STREAMON failed for UVC metadata device %s: %s",
                device.c_str(), std::strerror(errno));
      Close();
      return false;
    }
    streaming_ = true;
    drain_running_.store(true);
    drain_thread_ = std::thread(&PtsMetadataReader::DrainLoop, this);
    ROS_INFO("Capturing 48 MHz UVC PTS/SCR metadata from %s", device.c_str());
    return true;
  }

  bool Match(uint32_t video_sequence, uint64_t video_monotonic_ns,
             int64_t max_delta_ns,
             PtsFrameMetadata &matched, int64_t &signed_delta_ns) {
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::nanoseconds(
                              std::max<int64_t>(0, max_delta_ns));
    std::unique_lock<std::mutex> lock(state_mutex_);
    while (true) {
      for (size_t index = 0; index < ready_.size(); ++index) {
        if (ready_[index].first_meta_sequence == video_sequence &&
            ready_[index].unwrap_valid) {
          matched = ready_[index];
          ready_.erase(ready_.begin() + index);
          signed_delta_ns =
              static_cast<int64_t>(matched.first_payload_monotonic_ns) -
              static_cast<int64_t>(video_monotonic_ns);
          return true;
        }
      }
      if (current_valid_ && !current_.consumed && current_.unwrap_valid &&
          current_.first_meta_sequence == video_sequence) {
        matched = current_;
        current_.consumed = true;
        signed_delta_ns =
            static_cast<int64_t>(matched.first_payload_monotonic_ns) -
            static_cast<int64_t>(video_monotonic_ns);
        return true;
      }
      if (std::chrono::steady_clock::now() >= deadline) return false;
      metadata_ready_.wait_until(lock, deadline);
    }
  }

  uint64_t duplicate_count() const { return duplicate_count_.load(); }
  uint64_t rollback_count() const { return rollback_count_.load(); }

  void ResetUnwrapForNewSession() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    ready_.clear();
    current_ = PtsFrameMetadata();
    current_valid_ = false;
    unwrap_initialized_ = false;
    last_raw_pts_ = 0;
    last_unwrapped_pts_ = 0;
    ROS_WARN("Reset UVC PTS unwrap state; a new session anchor is required");
  }

 private:
  void DrainLoop() {
    while (drain_running_.load()) {
      bool drained_any = false;
      while (drain_running_.load()) {
        v4l2_buffer buffer = {};
        buffer.type = V4L2_BUF_TYPE_META_CAPTURE;
        buffer.memory = V4L2_MEMORY_MMAP;
        if (ioctl(fd_, VIDIOC_DQBUF, &buffer) < 0) {
          if (errno != EAGAIN && errno != EINTR) {
            ROS_WARN_THROTTLE(2.0, "VIDIOC_DQBUF UVC metadata failed: %s",
                              std::strerror(errno));
          }
          break;
        }
        drained_any = true;
        if (buffer.index < mappings_.size()) {
          {
            std::lock_guard<std::mutex> lock(state_mutex_);
            Parse(static_cast<const uint8_t *>(mappings_[buffer.index].address),
                  buffer.bytesused, buffer.sequence);
          }
          metadata_ready_.notify_all();
        }
        if (ioctl(fd_, VIDIOC_QBUF, &buffer) < 0) {
          ROS_WARN("Unable to recycle UVC metadata buffer: %s",
                   std::strerror(errno));
          break;
        }
      }
      if (!drained_any) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    }
  }

  struct Mapping {
    void *address = MAP_FAILED;
    size_t length = 0;
  };

  static void Consider(uint64_t reference, const PtsFrameMetadata &candidate,
                       size_t index, uint64_t &best_abs_delta,
                       size_t &best_index) {
    if (!candidate.unwrap_valid || candidate.first_payload_monotonic_ns == 0)
      return;
    const uint64_t delta = candidate.first_payload_monotonic_ns > reference
                               ? candidate.first_payload_monotonic_ns - reference
                               : reference - candidate.first_payload_monotonic_ns;
    if (delta < best_abs_delta) {
      best_abs_delta = delta;
      best_index = index;
    }
  }

  void StartFrame(uint32_t raw_pts, uint32_t meta_sequence) {
    if (current_valid_ && !current_.consumed) {
      ready_.push_back(current_);
      while (ready_.size() > 64) ready_.pop_front();
    }
    PtsFrameMetadata next;
    next.raw_pts = raw_pts;
    next.first_meta_sequence = meta_sequence;
    if (!unwrap_initialized_) {
      next.unwrapped_pts = raw_pts;
      next.unwrap_valid = true;
      unwrap_initialized_ = true;
      last_raw_pts_ = raw_pts;
      last_unwrapped_pts_ = next.unwrapped_pts;
    } else {
      const int32_t signed_delta = static_cast<int32_t>(raw_pts - last_raw_pts_);
      if (signed_delta > 0) {
        next.unwrapped_pts =
            last_unwrapped_pts_ + static_cast<uint32_t>(raw_pts - last_raw_pts_);
        next.unwrap_valid = true;
        last_raw_pts_ = raw_pts;
        last_unwrapped_pts_ = next.unwrapped_pts;
      } else {
        if (signed_delta == 0) {
          ++duplicate_count_;
        } else {
          ++rollback_count_;
        }
        ROS_WARN("UVC PTS rollback/duplicate across frames: previous=%u current=%u",
                 last_raw_pts_, raw_pts);
        // Ignore stale or out-of-order metadata. Replacing current_ here would
        // discard the last valid frame and poison all subsequent matching.
        return;
      }
    }
    current_ = next;
    current_valid_ = true;
  }

  void Parse(const uint8_t *data, size_t size, uint32_t meta_sequence) {
    size_t offset = 0;
    while (offset + sizeof(uvc_meta_buf) <= size) {
      const auto *meta = reinterpret_cast<const uvc_meta_buf *>(data + offset);
      const size_t object_size = sizeof(meta->ns) + sizeof(meta->sof) + meta->length;
      if (meta->length < 2 || object_size < sizeof(uvc_meta_buf) ||
          offset + object_size > size) {
        break;
      }
      const uint8_t *payload = meta->buf;
      const size_t payload_size = meta->length - 2;
      size_t payload_offset = 0;
      if (!(meta->flags & UVC_STREAM_PTS) || payload_size < 4) {
        offset += object_size;
        continue;
      }
      const uint32_t raw_pts = ReadLe32(payload);
      payload_offset += 4;
      // This camera emits auxiliary metadata records with the PTS-present bit
      // set but a zero PTS. They are not image frame timestamps.
      if (raw_pts == 0) {
        offset += object_size;
        continue;
      }
      const int fid = (meta->flags & UVC_STREAM_FID) ? 1 : 0;
      // Some UVC devices flip FID between payload records that still belong to
      // the same image. PTS is the authoritative frame identity here.
      if (!current_valid_ || current_.raw_pts != raw_pts) {
        StartFrame(raw_pts, meta_sequence);
      }
      if (current_.fid < 0) current_.fid = fid;
      if (current_.first_payload_monotonic_ns == 0) {
        current_.first_payload_monotonic_ns = meta->ns;
        current_.first_host_sof = meta->sof;
      }
      if ((meta->flags & UVC_STREAM_SCR) &&
          payload_size >= payload_offset + 6 && current_.first_scr_stc == 0) {
        current_.first_scr_stc = ReadLe32(payload + payload_offset);
        current_.first_scr_sof = ReadLe16(payload + payload_offset + 4);
      }
      offset += object_size;
    }
  }

  void Close() {
    drain_running_.store(false);
    metadata_ready_.notify_all();
    if (drain_thread_.joinable()) drain_thread_.join();
    if (fd_ >= 0 && streaming_) {
      v4l2_buf_type type = V4L2_BUF_TYPE_META_CAPTURE;
      ioctl(fd_, VIDIOC_STREAMOFF, &type);
    }
    for (const Mapping &mapping : mappings_) {
      if (mapping.address != MAP_FAILED) munmap(mapping.address, mapping.length);
    }
    mappings_.clear();
    if (fd_ >= 0) close(fd_);
    fd_ = -1;
    streaming_ = false;
  }

  int fd_ = -1;
  bool streaming_ = false;
  std::atomic<bool> drain_running_{false};
  std::thread drain_thread_;
  mutable std::mutex state_mutex_;
  std::condition_variable metadata_ready_;
  std::vector<Mapping> mappings_;
  std::deque<PtsFrameMetadata> ready_;
  PtsFrameMetadata current_;
  bool current_valid_ = false;
  bool unwrap_initialized_ = false;
  uint32_t last_raw_pts_ = 0;
  uint64_t last_unwrapped_pts_ = 0;
  std::atomic<uint64_t> duplicate_count_{0};
  std::atomic<uint64_t> rollback_count_{0};
};

struct PtsSessionAnchorSample {
  uint64_t ticks = 0;
  int64_t mid360_observation_ns = 0;
  uint32_t sequence = 0;
};

class PtsSessionAnchor {
 public:
  enum class Result { kWaiting, kLocked, kFailed };

  void Start(uint64_t streamon_monotonic_ns) {
    streamon_monotonic_ns_ = streamon_monotonic_ns;
    first_valid_pair_monotonic_ns_ = 0;
    collect_start_monotonic_ns_ = 0;
    samples_.clear();
    locked_ = false;
    failed_ = false;
    failure_reason_.clear();
    b_session_ns_ = 0;
    pts_anchor_ticks_ = 0;
    mid_anchor_ns_ = 0;
    residual_mean_ms_ = 0.0;
    residual_std_ms_ = 0.0;
    residual_p95_ms_ = 0.0;
    residual_max_ms_ = 0.0;
    residual_drift_ms_per_s_ = 0.0;
    duplicate_count_ = 0;
    rollback_count_ = 0;
    sequence_jump_count_ = 0;
  }

  Result Observe(uint64_t now_monotonic_ns,
                 const PtsFrameMetadata *metadata,
                 int64_t mid360_observation_ns) {
    if (locked_) return Result::kLocked;
    if (failed_) return Result::kFailed;
    if (first_valid_pair_monotonic_ns_ == 0) {
      if (!metadata || !metadata->unwrap_valid || mid360_observation_ns <= 0)
        return Result::kWaiting;
      first_valid_pair_monotonic_ns_ = now_monotonic_ns;
      collect_start_monotonic_ns_ =
          first_valid_pair_monotonic_ns_ +
          static_cast<uint64_t>(session_anchor_warmup_ms) * 1000000ULL;
      return Result::kWaiting;
    }
    if (now_monotonic_ns < collect_start_monotonic_ns_)
      return Result::kWaiting;

    if (metadata && metadata->unwrap_valid && mid360_observation_ns > 0) {
      if (!samples_.empty()) {
        if (metadata->unwrapped_pts == samples_.back().ticks) {
          ++duplicate_count_;
        } else if (metadata->unwrapped_pts < samples_.back().ticks) {
          ++rollback_count_;
        }
        if (metadata->first_meta_sequence != samples_.back().sequence + 1)
          ++sequence_jump_count_;
      }
      samples_.push_back({metadata->unwrapped_pts, mid360_observation_ns,
                          metadata->first_meta_sequence});
    }

    const uint64_t collect_ns =
        static_cast<uint64_t>(session_anchor_collect_ms) * 1000000ULL;
    if (now_monotonic_ns < collect_start_monotonic_ns_ + collect_ns)
      return Result::kWaiting;
    return Finalize();
  }

  int64_t Map(uint64_t ticks) const {
    const long double delta_ticks =
        static_cast<long double>(ticks) -
        static_cast<long double>(pts_anchor_ticks_);
    const int64_t delta_ns = static_cast<int64_t>(std::llround(
        static_cast<long double>(camera_clock_scale) * delta_ticks *
        1000000000.0L / static_cast<long double>(pts_clock_hz)));
    return mid_anchor_ns_ + delta_ns;
  }

  bool locked() const { return locked_; }
  int64_t b_session_ns() const { return b_session_ns_; }
  uint64_t pts_anchor_ticks() const { return pts_anchor_ticks_; }
  int64_t mid_anchor_ns() const { return mid_anchor_ns_; }
  size_t valid_pairs() const { return samples_.size(); }
  double residual_mean_ms() const { return residual_mean_ms_; }
  double residual_std_ms() const { return residual_std_ms_; }
  double residual_p95_ms() const { return residual_p95_ms_; }
  double residual_max_ms() const { return residual_max_ms_; }
  double residual_drift_ms_per_s() const {
    return residual_drift_ms_per_s_;
  }
  uint64_t duplicate_count() const { return duplicate_count_; }
  uint64_t rollback_count() const { return rollback_count_; }
  uint64_t sequence_jump_count() const { return sequence_jump_count_; }
  const std::string &failure_reason() const { return failure_reason_; }
  double elapsed_ms(uint64_t now_monotonic_ns) const {
    return (now_monotonic_ns - streamon_monotonic_ns_) / 1000000.0;
  }

 private:
  Result Finalize() {
    if (samples_.size() < static_cast<size_t>(session_anchor_min_pairs)) {
      failure_reason_ = "valid_pairs_below_minimum";
      failed_ = true;
      return Result::kFailed;
    }

    std::vector<int64_t> offsets;
    offsets.reserve(samples_.size());
    for (const auto &sample : samples_) {
      const int64_t scaled_pts_ns = static_cast<int64_t>(std::llround(
          static_cast<long double>(camera_clock_scale) *
          static_cast<long double>(sample.ticks) * 1000000000.0L /
          static_cast<long double>(pts_clock_hz)));
      offsets.push_back(sample.mid360_observation_ns - scaled_pts_ns);
    }
    std::sort(offsets.begin(), offsets.end());
    if (offsets.size() & 1U) {
      b_session_ns_ = offsets[offsets.size() / 2];
    } else {
      b_session_ns_ = static_cast<int64_t>(std::llround(
          (static_cast<long double>(offsets[offsets.size() / 2 - 1]) +
           static_cast<long double>(offsets[offsets.size() / 2])) /
          2.0L));
    }

    std::vector<double> residual_ms;
    residual_ms.reserve(samples_.size());
    for (const auto &sample : samples_) {
      const int64_t mapped_ns = static_cast<int64_t>(std::llround(
          static_cast<long double>(camera_clock_scale) *
          static_cast<long double>(sample.ticks) * 1000000000.0L /
          static_cast<long double>(pts_clock_hz))) + b_session_ns_;
      residual_ms.push_back(
          (mapped_ns - sample.mid360_observation_ns) / 1000000.0);
    }

    long double mean = 0.0L;
    for (double value : residual_ms) mean += value;
    mean /= static_cast<long double>(residual_ms.size());
    residual_mean_ms_ = static_cast<double>(mean);
    long double variance = 0.0L;
    for (double value : residual_ms) {
      const long double delta = value - mean;
      variance += delta * delta;
    }
    residual_std_ms_ = std::sqrt(static_cast<double>(
        variance / static_cast<long double>(residual_ms.size())));
    std::vector<double> absolute_residual_ms;
    absolute_residual_ms.reserve(residual_ms.size());
    for (double value : residual_ms)
      absolute_residual_ms.push_back(std::abs(value));
    std::sort(absolute_residual_ms.begin(), absolute_residual_ms.end());
    const size_t p95_index = std::min(
        absolute_residual_ms.size() - 1,
        static_cast<size_t>(std::ceil(0.95 * absolute_residual_ms.size())) - 1);
    residual_p95_ms_ = absolute_residual_ms[p95_index];
    residual_max_ms_ = absolute_residual_ms.back();

    const uint64_t ticks_0 = samples_.front().ticks;
    long double elapsed_mean_s = 0.0L;
    for (const auto &sample : samples_) {
      elapsed_mean_s +=
          (sample.ticks - ticks_0) / static_cast<long double>(pts_clock_hz);
    }
    elapsed_mean_s /= static_cast<long double>(samples_.size());
    long double covariance = 0.0L;
    long double denominator = 0.0L;
    for (size_t index = 0; index < samples_.size(); ++index) {
      const long double elapsed_s =
          (samples_[index].ticks - ticks_0) /
          static_cast<long double>(pts_clock_hz);
      const long double dx = elapsed_s - elapsed_mean_s;
      covariance += dx * (residual_ms[index] - mean);
      denominator += dx * dx;
    }
    residual_drift_ms_per_s_ = denominator > 0.0L
        ? static_cast<double>(covariance / denominator)
        : std::numeric_limits<double>::infinity();

    if (std::abs(residual_mean_ms_) >= session_anchor_max_abs_mean_ms) {
      failure_reason_ = "absolute_residual_mean_exceeds_limit";
    } else if (residual_p95_ms_ >= session_anchor_max_p95_ms) {
      failure_reason_ = "absolute_residual_p95_exceeds_limit";
    } else if (std::abs(residual_drift_ms_per_s_) >=
               session_anchor_max_abs_drift_ms_per_s) {
      failure_reason_ = "absolute_residual_drift_exceeds_limit";
    } else if (duplicate_count_ || rollback_count_ || sequence_jump_count_) {
      failure_reason_ = "timestamp_or_sequence_anomaly";
    }
    if (!failure_reason_.empty()) {
      failed_ = true;
      return Result::kFailed;
    }

    const auto &anchor_sample = samples_[samples_.size() / 2];
    pts_anchor_ticks_ = anchor_sample.ticks;
    mid_anchor_ns_ = MapAbsolute(pts_anchor_ticks_);
    locked_ = true;
    return Result::kLocked;
  }

  int64_t MapAbsolute(uint64_t ticks) const {
    return static_cast<int64_t>(std::llround(
               static_cast<long double>(camera_clock_scale) *
               static_cast<long double>(ticks) * 1000000000.0L /
               static_cast<long double>(pts_clock_hz))) +
           b_session_ns_;
  }

  uint64_t streamon_monotonic_ns_ = 0;
  uint64_t first_valid_pair_monotonic_ns_ = 0;
  uint64_t collect_start_monotonic_ns_ = 0;
  std::vector<PtsSessionAnchorSample> samples_;
  bool locked_ = false;
  bool failed_ = false;
  std::string failure_reason_;
  int64_t b_session_ns_ = 0;
  uint64_t pts_anchor_ticks_ = 0;
  int64_t mid_anchor_ns_ = 0;
  double residual_mean_ms_ = 0.0;
  double residual_std_ms_ = 0.0;
  double residual_p95_ms_ = 0.0;
  double residual_max_ms_ = 0.0;
  double residual_drift_ms_per_s_ = 0.0;
  uint64_t duplicate_count_ = 0;
  uint64_t rollback_count_ = 0;
  uint64_t sequence_jump_count_ = 0;
};

struct V4L2BootstrapSample {
  uint32_t sequence = 0;
  int64_t mapped_mid360_time_ns = 0;
};
std::vector<V4L2BootstrapSample> v4l2_bootstrap_samples;

bool ReadCommittedValue(volatile time_stamp *source, int64_t &seq,
                        int64_t &value) {
  if (source == MAP_FAILED) return false;
  for (int attempt = 0; attempt < 4; ++attempt) {
    const int64_t seq_before = source->high;
    std::atomic_thread_fence(std::memory_order_acquire);
    const int64_t committed_value = source->low;
    std::atomic_thread_fence(std::memory_order_acquire);
    const int64_t seq_after = source->high;
    if (seq_before == seq_after && seq_after > 0 &&
        (seq_after & 1) == 0) {
      seq = seq_after;
      value = committed_value;
      return true;
    }
    std::this_thread::yield();
  }
  return false;
}

bool ReadCommittedTimestamp(int64_t &seq, int64_t &stamp_ns) {
  return ReadCommittedValue(pointt, seq, stamp_ns) && stamp_ns > 0;
}

bool ReadLidarTimeShare(int64_t &seq, int64_t &stamp_ns) {
  return ReadCommittedValue(lidar_timeshare_pointt, seq, stamp_ns) &&
         stamp_ns > 0;
}

int64_t TimespecToNs(const struct timespec &ts) {
  return static_cast<int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}

int64_t V4L2BufferTimestampNs(const struct v4l2_buffer &buf) {
  return static_cast<int64_t>(buf.timestamp.tv_sec) * 1000000000LL +
         static_cast<int64_t>(buf.timestamp.tv_usec) * 1000LL;
}

bool V4L2CaptureHostRealtimeNs(const struct v4l2_buffer &buf,
                               int64_t &host_realtime_ns) {
  const int64_t buffer_ns = V4L2BufferTimestampNs(buf);
  if (buffer_ns <= 0) return false;

  if ((buf.flags & V4L2_BUF_FLAG_TIMESTAMP_MASK) ==
      V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC) {
    struct timespec realtime_now, monotonic_now;
    if (clock_gettime(CLOCK_REALTIME, &realtime_now) != 0 ||
        clock_gettime(CLOCK_MONOTONIC, &monotonic_now) != 0) {
      return false;
    }
    host_realtime_ns = buffer_ns + TimespecToNs(realtime_now) -
                       TimespecToNs(monotonic_now);
  } else {
    host_realtime_ns = buffer_ns;
  }
  return true;
}

bool V4L2MappedMid360TimeNs(const struct v4l2_buffer &buf,
                            int64_t &mid360_time_ns) {
  int64_t host_capture_ns = 0;
  int64_t offset_seq = 0;
  int64_t host_minus_lidar_ns = 0;
  if (!V4L2CaptureHostRealtimeNs(buf, host_capture_ns) ||
      !ReadCommittedValue(clock_offset_pointt, offset_seq,
                          host_minus_lidar_ns)) {
    return false;
  }
  mid360_time_ns = host_capture_ns - host_minus_lidar_ns;
  return mid360_time_ns > 0;
}

bool V4L2CaptureAndMapMid360TimeNs(const struct v4l2_buffer &buf,
                                   int64_t &host_capture_ns,
                                   int64_t &mid360_time_ns) {
  int64_t offset_seq = 0;
  int64_t host_minus_lidar_ns = 0;
  if (!V4L2CaptureHostRealtimeNs(buf, host_capture_ns) ||
      !ReadCommittedValue(clock_offset_pointt, offset_seq,
                          host_minus_lidar_ns)) {
    return false;
  }
  mid360_time_ns = host_capture_ns - host_minus_lidar_ns;
  return mid360_time_ns > 0;
}

const char *V4L2TimestampReference(const struct v4l2_buffer &buf) {
#ifdef V4L2_BUF_FLAG_TSTAMP_SRC_SOE
  if ((buf.flags & V4L2_BUF_FLAG_TSTAMP_SRC_MASK) ==
      V4L2_BUF_FLAG_TSTAMP_SRC_SOE) {
    return "SOE";
  }
#endif
  return "EOF_or_unspecified";
}

bool V4L2TimestampIsSoe(const struct v4l2_buffer &buf) {
#ifdef V4L2_BUF_FLAG_TSTAMP_SRC_SOE
  return (buf.flags & V4L2_BUF_FLAG_TSTAMP_SRC_MASK) ==
         V4L2_BUF_FLAG_TSTAMP_SRC_SOE;
#else
  return false;
#endif
}

void LogV4L2FixedAnchorStatsIfReady(int64_t mapped_soe_ns) {
  if (v4l2_stats_window_start_ns == 0) {
    v4l2_stats_window_start_ns = mapped_soe_ns;
    return;
  }
  if (mapped_soe_ns - v4l2_stats_window_start_ns < 120000000000LL) return;

  auto summarize = [](const std::vector<double> &periods, double &mean,
                      double &stddev, double &minimum, double &maximum,
                      double &frequency, uint64_t &outside_97_103) {
    mean = stddev = minimum = maximum = frequency = 0.0;
    outside_97_103 = 0;
    if (periods.empty()) return;
    minimum = *std::min_element(periods.begin(), periods.end());
    maximum = *std::max_element(periods.begin(), periods.end());
    double sum = 0.0;
    for (double period : periods) {
      sum += period;
      if (period < 97.0 || period > 103.0) ++outside_97_103;
    }
    mean = sum / periods.size();
    double squared_error_sum = 0.0;
    for (double period : periods) {
      const double error = period - mean;
      squared_error_sum += error * error;
    }
    stddev = std::sqrt(squared_error_sum / periods.size());
    frequency = sum > 0.0 ? periods.size() * 1000.0 / sum : 0.0;
  };

  double raw_mean, raw_stddev, raw_minimum, raw_maximum, raw_frequency;
  double mapped_mean, mapped_stddev, mapped_minimum, mapped_maximum;
  double mapped_frequency;
  uint64_t raw_outside, mapped_outside;
  summarize(v4l2_stats_raw_period_ms, raw_mean, raw_stddev, raw_minimum,
            raw_maximum, raw_frequency, raw_outside);
  summarize(v4l2_stats_mapped_period_ms, mapped_mean, mapped_stddev,
            mapped_minimum, mapped_maximum, mapped_frequency, mapped_outside);

  ROS_INFO("V4L2_FIXED_ANCHOR_120S raw_period_ms(mean/std/min/max)="
           "%.6f/%.6f/%.6f/%.6f raw_hz=%.6f raw_outside_97_103=%lu "
           "mapped_period_ms(mean/std/min/max)=%.6f/%.6f/%.6f/%.6f "
           "mapped_hz=%.6f mapped_outside_97_103=%lu duplicates=%lu "
           "rollbacks=%lu sequence_jumps=%lu samples=%zu",
           raw_mean, raw_stddev, raw_minimum, raw_maximum, raw_frequency,
           raw_outside, mapped_mean, mapped_stddev, mapped_minimum,
           mapped_maximum, mapped_frequency, mapped_outside,
           v4l2_stats_header_duplicate_count,
           v4l2_stats_header_rollback_count,
           v4l2_stats_sequence_jump_count,
           v4l2_stats_mapped_period_ms.size());

  v4l2_stats_window_start_ns = mapped_soe_ns;
  v4l2_stats_header_duplicate_count = 0;
  v4l2_stats_header_rollback_count = 0;
  v4l2_stats_sequence_jump_count = 0;
  v4l2_stats_raw_period_ms.clear();
  v4l2_stats_mapped_period_ms.clear();
}

int64_t FallbackNowInLidarClockDomainNs() {
  const int64_t now_ns = ros::Time::now().toNSec();
  int64_t offset_seq = 0;
  int64_t host_minus_lidar_ns = 0;
  if (ReadCommittedValue(clock_offset_pointt, offset_seq, host_minus_lidar_ns)) {
    return now_ns - host_minus_lidar_ns;
  }
  ROS_WARN_THROTTLE(
      2.0,
      "MID360 clock offset is unavailable at %s; using raw ros::Time::now() "
      "fallback, which may be outside the LiDAR/IMU clock domain",
      clock_offset_path.c_str());
  return now_ns;
}

int64_t FloorDiv(int64_t value, int64_t divisor) {
  if (divisor <= 0) return 0;
  int64_t quotient = value / divisor;
  const int64_t remainder = value % divisor;
  if (remainder < 0) --quotient;
  return quotient;
}

int64_t NormalizedTriggerPhaseNs() {
  const int64_t period_ns = stm32_trigger_period_us * 1000LL;
  if (period_ns <= 0) return 0;
  int64_t phase_ns = (stm32_trigger_phase_us * 1000LL) % period_ns;
  if (phase_ns < 0) phase_ns += period_ns;
  return phase_ns;
}

int64_t AlignToNearestTriggerGridNs(int64_t reference_ns) {
  const int64_t period_ns = stm32_trigger_period_us * 1000LL;
  const int64_t phase_ns = NormalizedTriggerPhaseNs();
  if (period_ns <= 0) return reference_ns;
  const int64_t shifted = reference_ns - phase_ns;
  int64_t cycles = FloorDiv(shifted, period_ns);
  const int64_t remainder = shifted - cycles * period_ns;
  if (remainder >= period_ns / 2) ++cycles;
  return cycles * period_ns + phase_ns;
}

int64_t AlignToPreviousTriggerGridNs(int64_t reference_ns) {
  const int64_t period_ns = stm32_trigger_period_us * 1000LL;
  const int64_t phase_ns = NormalizedTriggerPhaseNs();
  if (period_ns <= 0) return reference_ns;
  return FloorDiv(reference_ns - phase_ns, period_ns) * period_ns + phase_ns;
}

int64_t FloorToSecondNs(int64_t timestamp_ns) {
  constexpr int64_t kSecondNs = 1000000000LL;
  return FloorDiv(timestamp_ns, kSecondNs) * kSecondNs;
}

bool BootstrapTriggerGridFromV4L2(const struct v4l2_buffer &buf,
                                  int64_t &trigger_seq,
                                  int64_t &trigger_time_ns) {
  constexpr int64_t kSecondNs = 1000000000LL;
  const int64_t period_ns = stm32_trigger_period_us * 1000LL;
  if (period_ns <= 0 || kSecondNs % period_ns != 0) return false;

  int64_t mapped_mid360_time_ns = 0;
  if (!V4L2MappedMid360TimeNs(buf, mapped_mid360_time_ns)) {
    ROS_WARN_THROTTLE(1.0, "Waiting for V4L2-to-MID360 clock mapping before "
                      "initial Trigger matching");
    return false;
  }

  if (!v4l2_bootstrap_samples.empty() &&
      buf.sequence <= v4l2_bootstrap_samples.back().sequence) {
    ROS_WARN("Restarting V4L2 bootstrap because Camera sequence is not "
             "strictly increasing: prev=%u current=%u",
             v4l2_bootstrap_samples.back().sequence, buf.sequence);
    v4l2_bootstrap_samples.clear();
  }
  v4l2_bootstrap_samples.push_back({buf.sequence, mapped_mid360_time_ns});
  if (static_cast<int>(v4l2_bootstrap_samples.size()) <
      stm32_v4l2_init_frames) {
    ROS_INFO("Collecting V4L2 bootstrap samples: %zu/%d",
             v4l2_bootstrap_samples.size(), stm32_v4l2_init_frames);
    return false;
  }

  const auto &first = v4l2_bootstrap_samples.front();
  const auto &last = v4l2_bootstrap_samples.back();
  const int64_t phase_ns = NormalizedTriggerPhaseNs();
  const int64_t effective_phase_ns =
      phase_ns + stm32_pps_minus_mid360_integer_ns;
  const int64_t camera_offset_ns =
      (stm32_trigger_to_exposure_center_us + camera_calibration_offset_us) *
      1000LL;
  const int64_t first_second_ns = FloorToSecondNs(first.mapped_mid360_time_ns);
  const int64_t triggers_per_second = kSecondNs / period_ns;
  bool found = false;
  int64_t best_first_trigger_ns = 0;
  int64_t best_sum_abs_residual_ns = 0;
  int64_t best_max_residual_ns = 0;

  // The only V4L2 use in the timestamp path: choose the one initial Trigger
  // phase whose whole sequence best agrees with the version4 MID360 mapping.
  for (int second_offset = -1; second_offset <= 1; ++second_offset) {
    const int64_t pps_epoch_ns = first_second_ns + second_offset * kSecondNs;
    for (int k = 0; k < triggers_per_second; ++k) {
      const int64_t first_trigger_ns =
          pps_epoch_ns + effective_phase_ns + k * period_ns;
      int64_t sum_abs_residual_ns = 0;
      int64_t max_residual_ns = 0;
      for (const auto &sample : v4l2_bootstrap_samples) {
        const uint32_t frame_delta = sample.sequence - first.sequence;
        const int64_t predicted_camera_ns =
            first_trigger_ns + static_cast<int64_t>(frame_delta) * period_ns +
            camera_offset_ns;
        const int64_t residual_ns =
            std::llabs(sample.mapped_mid360_time_ns - predicted_camera_ns);
        sum_abs_residual_ns += residual_ns;
        max_residual_ns = std::max(max_residual_ns, residual_ns);
      }
      if (!found || sum_abs_residual_ns < best_sum_abs_residual_ns) {
        found = true;
        best_first_trigger_ns = first_trigger_ns;
        best_sum_abs_residual_ns = sum_abs_residual_ns;
        best_max_residual_ns = max_residual_ns;
      }
    }
  }

  const double mean_residual_ms =
      best_sum_abs_residual_ns / 1000000.0 / v4l2_bootstrap_samples.size();
  if (!found || best_max_residual_ns > stm32_v4l2_init_max_residual_ns) {
    ROS_WARN("V4L2 bootstrap rejected: mean_residual=%.3f ms max_residual=%.3f "
             "ms threshold=%.3f ms. Retrying with a new 20-frame window.",
             mean_residual_ms, best_max_residual_ns / 1000000.0,
             stm32_v4l2_init_max_residual_ns / 1000000.0);
    v4l2_bootstrap_samples.clear();
    return false;
  }

  const uint32_t sequence_delta = last.sequence - first.sequence;
  trigger_time_ns =
      best_first_trigger_ns + static_cast<int64_t>(sequence_delta) * period_ns;
  trigger_seq = FloorDiv(trigger_time_ns - effective_phase_ns, period_ns);
  int64_t index_in_second = trigger_seq % triggers_per_second;
  if (index_in_second < 0) index_in_second += triggers_per_second;

  int64_t lidar_seq = 0;
  int64_t lidar_stamp_ns = 0;
  if (!ReadLidarTimeShare(lidar_seq, lidar_stamp_ns)) return false;
  last_trigger_seq = trigger_seq;
  last_predicted_trigger_time_ns = trigger_time_ns;
  last_camera_sequence = last.sequence;
  last_mid360_pps_epoch_ns = FloorToSecondNs(lidar_stamp_ns);
  const int64_t trigger_mid360_second_ns =
      FloorToSecondNs(trigger_time_ns - effective_phase_ns);
  last_stm32_pps_epoch_ns =
      trigger_mid360_second_ns + stm32_pps_minus_mid360_integer_ns;
  last_trigger_index_in_second = index_in_second;
  int64_t signed_residual_sum_ns = 0;
  for (const auto &sample : v4l2_bootstrap_samples) {
    const uint32_t frame_delta = sample.sequence - first.sequence;
    const int64_t predicted_camera_ns =
        best_first_trigger_ns + static_cast<int64_t>(frame_delta) * period_ns +
        camera_offset_ns;
    signed_residual_sum_ns += sample.mapped_mid360_time_ns - predicted_camera_ns;
  }
  v4l2_monitor_reference_residual_ns =
      signed_residual_sum_ns / static_cast<int64_t>(v4l2_bootstrap_samples.size());
  camera_sequence_initialized = true;
  ROS_INFO("Locked STM32 Trigger grid from %zu V4L2 bootstrap frames: "
           "index_in_second=%ld mean_residual=%.3f ms max_residual=%.3f ms. "
           "V4L2 now only monitors alignment.",
           v4l2_bootstrap_samples.size(), index_in_second, mean_residual_ms,
           best_max_residual_ns / 1000000.0);
  v4l2_bootstrap_samples.clear();
  return true;
}

void MonitorV4L2TriggerAlignment(const struct v4l2_buffer &buf,
                                 int64_t camera_time_ns) {
  int64_t mapped_mid360_time_ns = 0;
  if (!V4L2MappedMid360TimeNs(buf, mapped_mid360_time_ns)) return;
  const int64_t period_ns = stm32_trigger_period_us * 1000LL;
  if (period_ns <= 0) return;
  const int64_t residual_change_ns =
      (mapped_mid360_time_ns - camera_time_ns) -
      v4l2_monitor_reference_residual_ns;
  const int64_t abs_change_ns = std::llabs(residual_change_ns);
  const int64_t trigger_steps =
      residual_change_ns >= 0
          ? (abs_change_ns + period_ns / 2) / period_ns
          : -((abs_change_ns + period_ns / 2) / period_ns);
  const int64_t trigger_step_error_ns =
      std::llabs(residual_change_ns - trigger_steps * period_ns);
  if (trigger_steps != 0 &&
      trigger_step_error_ns <= stm32_v4l2_init_max_residual_ns) {
    ROS_WARN_THROTTLE(
        1.0,
        "V4L2 monitor: possible %ld-frame Trigger mismatch, mapped-time "
        "change=%.3f ms from bootstrap reference; no automatic adjustment",
        trigger_steps, residual_change_ns / 1000000.0);
  }
}

bool PredictTriggerFromMid360Grid(uint32_t frame_sequence, int64_t &trigger_seq,
                                  int64_t &trigger_time_ns) {
  const int64_t period_ns = stm32_trigger_period_us * 1000LL;
  int64_t lidar_seq = 0;
  int64_t lidar_stamp_ns = 0;
  if (period_ns <= 0 || !ReadLidarTimeShare(lidar_seq, lidar_stamp_ns)) {
    return false;
  }

  if (!camera_sequence_initialized || last_predicted_trigger_time_ns <= 0) {
    // MID360 is already disciplined by the STM32 PPS/GPRMC signal. Use its
    // hardware time as the common STM32 time-axis anchor, then place the first
    // camera frame on the nearest 10 Hz trigger grid.
    trigger_time_ns = AlignToNearestTriggerGridNs(lidar_stamp_ns);
  } else {
    const uint32_t frame_delta = frame_sequence - last_camera_sequence;
    trigger_time_ns =
        last_predicted_trigger_time_ns + static_cast<int64_t>(frame_delta) * period_ns;
  }

  if (trigger_time_ns <= 0) return false;
  trigger_seq = trigger_time_ns / period_ns;
  return true;
}

bool PredictTriggerFromStm32PpsGrid(uint32_t frame_sequence,
                                    int64_t &trigger_seq,
                                    int64_t &trigger_time_ns) {
  constexpr int64_t kSecondNs = 1000000000LL;
  const int64_t period_ns = stm32_trigger_period_us * 1000LL;
  const int64_t phase_ns = NormalizedTriggerPhaseNs();
  int64_t lidar_seq = 0;
  int64_t lidar_stamp_ns = 0;
  if (period_ns <= 0) {
    return false;
  }

  if (kSecondNs % period_ns != 0) {
    ROS_ERROR_THROTTLE(
        2.0,
        "STM32 Trigger period must divide one second exactly: period_ns=%ld",
        period_ns);
    return false;
  }
  const int64_t triggers_per_second = kSecondNs / period_ns;
  const int64_t effective_phase_ns =
      phase_ns + stm32_pps_minus_mid360_integer_ns;

  if (!camera_sequence_initialized || last_predicted_trigger_time_ns <= 0) {
    return false;
  }
  const uint32_t frame_delta = frame_sequence - last_camera_sequence;
  trigger_time_ns = last_predicted_trigger_time_ns +
                    static_cast<int64_t>(frame_delta) * period_ns;

  // Once the first Camera/Trigger association is locked, a transient failure
  // to read timeshare must not switch the timestamp to a different clock. The
  // sequence-derived Trigger remains valid until a fresh PPS observation is
  // available. ros::Time::now() remains the fallback before initialization.
  if (!ReadLidarTimeShare(lidar_seq, lidar_stamp_ns)) {
    last_trigger_index_in_second =
        (last_trigger_index_in_second + static_cast<int64_t>(frame_delta)) %
        triggers_per_second;
    if (last_trigger_index_in_second < 0) {
      last_trigger_index_in_second += triggers_per_second;
    }
    trigger_seq = FloorDiv(trigger_time_ns - effective_phase_ns, period_ns);
    ROS_WARN_THROTTLE(
        2.0,
        "MID360 timeshare is temporarily unavailable; preserving the locked "
        "STM32 Trigger grid (frame_seq=%u trigger_time=%.9f)",
        frame_sequence, trigger_time_ns / 1e9);
    return trigger_time_ns > 0;
  }

  const int64_t observed_pps_epoch_ns = FloorToSecondNs(lidar_stamp_ns);
  const int64_t corrected_stm32_pps_epoch_ns =
      observed_pps_epoch_ns + stm32_pps_minus_mid360_integer_ns;

  if (observed_pps_epoch_ns > last_mid360_pps_epoch_ns) {
    // Preserve the Camera frame association established by V4L2 sequence.
    // Crossing a MID360/GPRMC integer second only corrects the theoretical
    // STM32 Trigger grid epoch; it never performs a nearest-frame rematch.
    int64_t index_in_second =
        (last_trigger_index_in_second + static_cast<int64_t>(frame_delta)) %
        triggers_per_second;
    if (index_in_second < 0) index_in_second += triggers_per_second;

    const int64_t predicted_trigger_time_ns = trigger_time_ns;
    const int64_t anchored_trigger_time_ns =
        corrected_stm32_pps_epoch_ns + phase_ns +
        index_in_second * period_ns;
    const double correction_ms =
        (anchored_trigger_time_ns - predicted_trigger_time_ns) / 1000000.0;

    // A newly observed integer second can itself be stale when the LiDAR
    // producer or shared-memory update was delayed. Accepting that epoch would
    // move Camera time backward by whole seconds. Only a nearby PPS candidate
    // is allowed to discipline the already locked Trigger grid.
    if (std::llabs(anchored_trigger_time_ns - predicted_trigger_time_ns) <=
        period_ns / 2) {
      trigger_time_ns = anchored_trigger_time_ns;
      last_stm32_pps_epoch_ns = corrected_stm32_pps_epoch_ns;
      ROS_INFO("MID360 PPS epoch advanced: old=%.9f new=%.9f "
               "index_in_second=%ld trigger_grid_correction=%.3f ms",
               last_mid360_pps_epoch_ns / 1e9,
               observed_pps_epoch_ns / 1e9,
               index_in_second, correction_ms);
    } else {
      // Consume the observation so it is not reconsidered on every frame, but
      // preserve the monotonic sequence-derived Trigger time.
      last_stm32_pps_epoch_ns =
          predicted_trigger_time_ns - phase_ns -
          index_in_second * period_ns;
      ROS_WARN("Ignoring stale MID360 PPS epoch: observed=%.9f "
               "predicted_trigger=%.9f candidate_trigger=%.9f "
               "correction=%.3f ms",
               observed_pps_epoch_ns / 1e9,
               predicted_trigger_time_ns / 1e9,
               anchored_trigger_time_ns / 1e9, correction_ms);
    }
    last_mid360_pps_epoch_ns = observed_pps_epoch_ns;
    last_trigger_index_in_second = index_in_second;
  } else if (observed_pps_epoch_ns < last_mid360_pps_epoch_ns) {
    ROS_WARN_THROTTLE(
        2.0,
        "MID360/GPRMC integer-second epoch moved backward: current=%.9f "
        "last=%.9f; keeping the existing Camera Trigger grid",
        observed_pps_epoch_ns / 1e9, last_mid360_pps_epoch_ns / 1e9);
    last_trigger_index_in_second =
        (last_trigger_index_in_second + static_cast<int64_t>(frame_delta)) %
        triggers_per_second;
    if (last_trigger_index_in_second < 0) {
      last_trigger_index_in_second += triggers_per_second;
    }
  } else {
    last_trigger_index_in_second =
        (last_trigger_index_in_second + static_cast<int64_t>(frame_delta)) %
        triggers_per_second;
    if (last_trigger_index_in_second < 0) {
      last_trigger_index_in_second += triggers_per_second;
    }
  }

  if (trigger_time_ns <= 0) return false;
  trigger_seq = FloorDiv(trigger_time_ns - effective_phase_ns, period_ns);

  const int64_t latest_expected_trigger_ns =
      FloorDiv(lidar_stamp_ns - effective_phase_ns, period_ns) * period_ns +
      effective_phase_ns;
  const double anchor_error_ms =
      (trigger_time_ns - latest_expected_trigger_ns) / 1000000.0;
  if (std::abs(anchor_error_ms) > 200.0) {
    ROS_WARN_THROTTLE(
        2.0,
        "STM32 PPS-grid model is more than two periods from the latest "
        "PPS-disciplined LiDAR clock observation: error=%.3f ms",
        anchor_error_ms);
  }
  return true;
}

void OpenTimeDiagCsvIfRequested() {
  const char *path = std::getenv("FAST_LIVO_TIME_DIAG_CSV");
  if (!path || !*path || time_diag_csv.is_open()) return;
  time_diag_csv.open(path, std::ios::out | std::ios::trunc);
  if (!time_diag_csv.is_open()) {
    ROS_WARN("Failed to open FAST_LIVO_TIME_DIAG_CSV=%s", path);
    return;
  }
  time_diag_csv
      << "frame_index,frame_seq,pps_time_ns,trigger_index,trigger_time_ns,"
         "camera_time_ns,camera_period_ms,camera_calibration_offset_us,"
         "raw_v4l2_soe_ns,v4l2_mapped_soe_ns,raw_soe_period_ms,"
         "mapped_soe_period_ms,v4l2_soe_to_exposure_center_us,"
         "diagnostic_clock_offset_ns,diagnostic_offset_delta_ns,"
         "sequence_discontinuity,v4l2_timestamp_reference,"
         "v4l2_flags,buf_sequence,buf_bytesused,dequeue_monotonic_ns,"
         "dequeue_realtime_ns,current_mid360_time_ns,"
         "current_mid360_time_seq,pts_valid,pts_raw,pts_unwrapped_ticks,"
         "pts_time_s,pts_scr_stc,pts_scr_sof,pts_host_sof,"
         "pts_payload_monotonic_ns,pts_meta_sequence,"
         "pts_video_match_delta_ns,pts_mid360_observation_ns,"
          "camera_clock_scale,camera_clock_offset_ns,"
          "exposure_center_offset_ns,camera_calibration_offset_ns,"
          "camera_mid360_residual_ns,timestamp_source,"
          "pts_metadata_duplicate_count,"
          "pts_metadata_rollback_count,pts_match_drop_count\n";
}

void SignalHandler(int signal) {
  if (signal == SIGINT) {
    fprintf(stderr, "\nReceived Ctrl+C, exiting...\n");
    exit_flag = true;
  }
}

void SetupSignalHandler() {
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = SignalHandler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);
}

bool set_v4l2_control(int fd, int id, int value) {
  struct v4l2_control ctrl;
  ctrl.id = id;
  ctrl.value = value;
  if (ioctl(fd, VIDIOC_S_CTRL, &ctrl) < 0) {
    perror("VIDIOC_S_CTRL");
    return false;
  }
  return true;
}

bool get_v4l2_control(int fd, int id, int &value) {
  struct v4l2_control ctrl;
  ctrl.id = id;
  if (ioctl(fd, VIDIOC_G_CTRL, &ctrl) < 0) {
    perror("VIDIOC_G_CTRL");
    return false;
  }
  value = ctrl.value;
  return true;
}

bool verify_external_trigger_mode(int fd, const char *stage) {
  const int expected = trigger_enable ? trigger_backlight_mode : 0;
  int actual = -1;

  if (!get_v4l2_control(fd, V4L2_CID_BACKLIGHT_COMPENSATION, actual)) {
    ROS_ERROR("Cannot read Camera trigger mode at %s", stage);
    return false;
  }

  if (actual != expected) {
    ROS_ERROR("Camera trigger verification failed at %s: "
              "expected Backlight=%d, read Backlight=%d",
              stage, expected, actual);
    return false;
  }

  ROS_INFO("Camera trigger verified at %s: TriggerEnable=%d, "
           "Backlight=%d, TriggerSource=physical SYNC input "
           "(fixed by Camera firmware; no V4L2 Line0 selector)",
           stage, trigger_enable, actual);
  return true;
}

bool configure_external_trigger_mode(int fd) {
  const int requested = trigger_enable ? trigger_backlight_mode : 0;

  if (!set_v4l2_control(fd, V4L2_CID_BACKLIGHT_COMPENSATION, requested)) {
    ROS_ERROR("Failed to configure Camera trigger: Backlight=%d", requested);
    return false;
  }
  return verify_external_trigger_mode(fd, "before STREAMON");
}

uint32_t pixelFormatFromString(const string &format) {
  if (format == "MJPG" || format == "MJPEG") return V4L2_PIX_FMT_MJPEG;
  if (format == "YUYV") return V4L2_PIX_FMT_YUYV;

  ROS_WARN("Unsupported PixelFormat '%s'. Falling back to YUYV.", format.c_str());
  return V4L2_PIX_FMT_YUYV;
}

bool ReadInt64Parameter(const cv::FileNode &node, int64_t &value) {
  if (node.empty()) return false;
  try {
    if (node.isString()) {
      value = std::stoll(static_cast<std::string>(node));
    } else {
      value = static_cast<int64_t>(std::llround(static_cast<double>(node)));
    }
    return true;
  } catch (const std::exception &error) {
    ROS_ERROR("Invalid int64 camera parameter: %s", error.what());
    return false;
  }
}

bool set_v4l2_frame_rate(int fd, int fps) {
  if (fps <= 0) return true;

  struct v4l2_streamparm parm;
  memset(&parm, 0, sizeof(parm));
  parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  parm.parm.capture.timeperframe.numerator = 1;
  parm.parm.capture.timeperframe.denominator = fps;

  if (ioctl(fd, VIDIOC_S_PARM, &parm) < 0) {
    perror("VIDIOC_S_PARM");
    return false;
  }

  ROS_INFO("Set camera frame rate to %.3f fps",
           static_cast<double>(parm.parm.capture.timeperframe.denominator) /
               parm.parm.capture.timeperframe.numerator);
  return true;
}

void setParams(int fd, const string &params_file) {
  cv::FileStorage Params(params_file, cv::FileStorage::READ);
  if (!Params.isOpened()) {
    string msg = "Failed to open settings file at:" + params_file;
    ROS_ERROR_STREAM(msg.c_str());
    exit(-1);
  }
  image_scale = Params["image_scale"];
  if (image_scale < 0.1) image_scale = 1.0;

  width = Params["Width"];
  height = Params["Height"];
  if (!Params["PixelFormat"].empty()) pixel_format = (string)Params["PixelFormat"];
  if (!Params["FrameRate"].empty()) frame_rate = (int)Params["FrameRate"];
  if (!Params["TimestampWaitMs"].empty())
    timestamp_wait_ms = std::max(0, (int)Params["TimestampWaitMs"]);
  if (!Params["TimestampSource"].empty())
    timestamp_source = (string)Params["TimestampSource"];
  if (!Params["Stm32TriggerPath"].empty())
    stm32_trigger_path = (string)Params["Stm32TriggerPath"];
  if (!Params["ClockOffsetPath"].empty())
    clock_offset_path = (string)Params["ClockOffsetPath"];
  if (!Params["LidarTimeSharePath"].empty())
    lidar_timeshare_path = (string)Params["LidarTimeSharePath"];
  if (!Params["PtsMetadataEnable"].empty())
    pts_metadata_enable = (int)Params["PtsMetadataEnable"] != 0;
  if (!Params["PtsMetadataDevice"].empty())
    pts_metadata_device = (string)Params["PtsMetadataDevice"];
  if (!Params["PtsClockHz"].empty())
    pts_clock_hz = (double)Params["PtsClockHz"];
  if (!Params["PtsMatchMaxDeltaUs"].empty())
    pts_match_max_delta_us =
        std::max<int64_t>(0, (int64_t)(int)Params["PtsMatchMaxDeltaUs"]);
  if (!Params["CameraClockScale"].empty())
    camera_clock_scale = (double)Params["CameraClockScale"];
  ReadInt64Parameter(Params["CameraClockOffsetNs"], camera_clock_offset_ns);
  if (!Params["CameraClockCalibrationValid"].empty())
    camera_clock_calibration_valid =
        (int)Params["CameraClockCalibrationValid"] != 0;
  if (!Params["SessionAnchorWarmupMs"].empty())
    session_anchor_warmup_ms =
        std::max(0, (int)Params["SessionAnchorWarmupMs"]);
  if (!Params["SessionAnchorCollectMs"].empty())
    session_anchor_collect_ms =
        std::max(1, (int)Params["SessionAnchorCollectMs"]);
  if (!Params["SessionAnchorMinPairs"].empty())
    session_anchor_min_pairs =
        std::max(2, (int)Params["SessionAnchorMinPairs"]);
  if (!Params["SessionAnchorMaxAbsMeanMs"].empty())
    session_anchor_max_abs_mean_ms =
        (double)Params["SessionAnchorMaxAbsMeanMs"];
  if (!Params["SessionAnchorMaxP95Ms"].empty())
    session_anchor_max_p95_ms =
        (double)Params["SessionAnchorMaxP95Ms"];
  if (!Params["SessionAnchorMaxAbsDriftMsPerS"].empty())
    session_anchor_max_abs_drift_ms_per_s =
        (double)Params["SessionAnchorMaxAbsDriftMsPerS"];
  if (!Params["SessionAnchorPtsResetMissingFrames"].empty())
    session_anchor_pts_reset_missing_frames =
        std::max(2, (int)Params["SessionAnchorPtsResetMissingFrames"]);
  ReadInt64Parameter(Params["ExposureCenterOffsetNs"],
                     exposure_center_offset_ns);
  ReadInt64Parameter(Params["CameraCalibrationOffsetNs"],
                     camera_calibration_offset_ns);
  if (!Params["Stm32TriggerPeriodUs"].empty())
    stm32_trigger_period_us = (int64_t)(int)Params["Stm32TriggerPeriodUs"];
  if (!Params["Stm32TriggerPhaseUs"].empty())
    stm32_trigger_phase_us = (int64_t)(int)Params["Stm32TriggerPhaseUs"];
  if (!Params["Stm32PpsMinusMid360IntegerNs"].empty())
    stm32_pps_minus_mid360_integer_ns =
        (int64_t)(int)Params["Stm32PpsMinusMid360IntegerNs"];
  if (!Params["Stm32SequenceResetThreshold"].empty())
    stm32_sequence_reset_threshold =
        std::max(1, (int)Params["Stm32SequenceResetThreshold"]);
  if (!Params["Stm32TriggerToExposureCenterUs"].empty())
    stm32_trigger_to_exposure_center_us =
        (int64_t)(int)Params["Stm32TriggerToExposureCenterUs"];
  if (!Params["CameraCalibrationOffsetUs"].empty())
    camera_calibration_offset_us =
        (int64_t)(int)Params["CameraCalibrationOffsetUs"];
  if (!Params["V4l2SoeToExposureCenterUs"].empty())
    v4l2_soe_to_exposure_center_us =
        (int64_t)(int)Params["V4l2SoeToExposureCenterUs"];
  if (!Params["Stm32V4l2InitFrames"].empty())
    stm32_v4l2_init_frames = std::max(3, (int)Params["Stm32V4l2InitFrames"]);
  if (!Params["Stm32V4l2InitMaxResidualUs"].empty())
    stm32_v4l2_init_max_residual_ns =
        std::max<int64_t>(1000, (int64_t)(int)Params["Stm32V4l2InitMaxResidualUs"]) *
        1000LL;
  if (!Params["Backlight"].empty())
    trigger_backlight_mode = (int)Params["Backlight"];
  if (timestamp_source == "pts_mid360_session_anchor") {
    if (!pts_metadata_enable || !camera_clock_calibration_valid ||
        !std::isfinite(camera_clock_scale) || pts_clock_hz <= 0.0 ||
        camera_clock_scale <= 0.0 || exposure_center_offset_ns != 0 ||
        camera_calibration_offset_ns != 0) {
      ROS_FATAL("PTS session-anchor configuration is invalid: "
                "PtsMetadataEnable=%d CameraClockCalibrationValid=%d "
                "CameraClockScale=%.17g PtsClockHz=%.3f "
                "ExposureCenterOffsetNs=%ld CameraCalibrationOffsetNs=%ld",
                pts_metadata_enable, camera_clock_calibration_valid,
                camera_clock_scale, pts_clock_hz,
                exposure_center_offset_ns, camera_calibration_offset_ns);
      exit(-1);
    }
    // A configured absolute offset is deliberately ignored and erased. Every
    // STREAMON must prove a new b_session before any image is published.
    camera_clock_offset_ns = 0;
    ROS_INFO("PTS session-anchor mode: fixed_a=%.17g PtsClockHz=%.3f "
             "warmup_ms=%d collect_ms=%d min_pairs=%d "
             "max_abs_mean_ms=%.3f max_p95_ms=%.3f "
             "max_abs_drift_ms_per_s=%.6f; old b/fallback disabled",
             camera_clock_scale, pts_clock_hz, session_anchor_warmup_ms,
             session_anchor_collect_ms, session_anchor_min_pairs,
             session_anchor_max_abs_mean_ms, session_anchor_max_p95_ms,
             session_anchor_max_abs_drift_ms_per_s);
  } else if (timestamp_source == "pts_mid360_fixed") {
    ROS_FATAL("TimestampSource=pts_mid360_fixed is disabled because its "
              "absolute b is invalid after Camera STREAMON; use "
              "pts_mid360_session_anchor");
    exit(-1);
  }
  int exposure_auto_mode = Params["ExposureAutoMode"];
  int exposure = Params["Exposure"];
  int exposure_time_lower = Params["ExposureTimeLower"];
  int exposure_time_upper = Params["ExposureTimeUpper"];
  int gain = Params["Gain"];
  int gain_auto = Params["GainAuto"];

  // 设置自动曝光模式
  int exposure_mode_value;
  std::string exposure_mode_str;
  switch (exposure_auto_mode) {
    case 0: // Manual
      exposure_mode_value = 1; // Manual Mode
      exposure_mode_str = "Manual";
      break;
    case 1: // Auto (使用Aperture Priority作为自动模式)
      exposure_mode_value = 3; // Aperture Priority Mode
      exposure_mode_str = "Aperture Priority (Auto)";
      break;
    case 2: // Aperture Priority
      exposure_mode_value = 3; // Aperture Priority Mode
      exposure_mode_str = "Aperture Priority";
      break;
    default:
      exposure_mode_value = 1; // Manual Mode
      exposure_mode_str = "Manual (default)";
      break;
  }

  if (!set_v4l2_control(fd, V4L2_CID_EXPOSURE_AUTO, exposure_mode_value)) {
    ROS_WARN("Failed to set auto exposure mode to %s", exposure_mode_str.c_str());
  } else {
    ROS_INFO("Set auto exposure mode to %s", exposure_mode_str.c_str());
  }

  // Generic V4L2 auto exposure does not provide controls for these bounds.
  // Keep the YAML values informational instead of claiming they were applied.
  if (exposure_auto_mode == 1) { // Auto mode (Aperture Priority)
    ROS_WARN("Auto exposure enabled; ExposureTimeLower/Upper (%d/%d) are not applied by this UVC driver",
             exposure_time_lower, exposure_time_upper);
  }

  // 如果是手动曝光，设置曝光时间
  if (exposure_auto_mode == 0) { // Manual mode
      if (!set_v4l2_control(fd, V4L2_CID_EXPOSURE_ABSOLUTE, exposure)) {
        ROS_ERROR("Failed to set exposure time to %d", exposure);
      } else {
        ROS_INFO("Set manual exposure preset to %d (camera manual: 156 = 6 ms)",
                 exposure);
      }
    }

  // Trigger mode is applied after format/frame-rate negotiation.

  // 设置增益（UVC摄像头通常只支持手动增益）
  if (!set_v4l2_control(fd, V4L2_CID_GAIN, gain)) {
    ROS_ERROR("Failed to set gain to %d", gain);
  } else {
    ROS_INFO("Set gain to %d", gain);
  }

  if (gain_auto == 1) {
    ROS_WARN("Auto gain requested but not supported by this UVC camera. Using manual gain: %d", gain);
  }

  // Apply optional color controls from the version-local YAML. Keeping these
  // explicit makes live RGB clouds repeatable instead of depending on whatever
  // values the device retained from a previous process.
  auto apply_optional_control = [&](const char *name, uint32_t id) {
    if (Params[name].empty()) return;
    const int requested = (int)Params[name];
    if (!set_v4l2_control(fd, id, requested)) {
      ROS_WARN("Failed to set camera %s to %d", name, requested);
      return;
    }
    int actual = requested;
    if (get_v4l2_control(fd, id, actual)) {
      ROS_INFO("Set camera %s=%d (readback=%d)", name, requested, actual);
    } else {
      ROS_INFO("Set camera %s=%d", name, requested);
    }
  };

  apply_optional_control("Brightness", V4L2_CID_BRIGHTNESS);
  apply_optional_control("Contrast", V4L2_CID_CONTRAST);
  apply_optional_control("Saturation", V4L2_CID_SATURATION);
  apply_optional_control("Gamma", V4L2_CID_GAMMA);
  apply_optional_control("WhiteBalanceAuto", V4L2_CID_AUTO_WHITE_BALANCE);
}

void *WorkThread(void *arg) {
  int fd = *(int *)arg;
  PtsMetadataReader pts_reader;
  PtsSessionAnchor pts_session_anchor;
  bool pts_reader_available = false;
  uint64_t pts_match_drop_count = 0;
  int consecutive_pts_match_drops = 0;
  struct v4l2_format fmt;
  memset(&fmt, 0, sizeof(fmt));
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = width;
  fmt.fmt.pix.height = height;
  fmt.fmt.pix.pixelformat = pixelFormatFromString(pixel_format);
  fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;

  if (ioctl(fd, VIDIOC_S_FMT, &fmt) < 0) {
    perror("VIDIOC_S_FMT");
    return NULL;
  }
  set_v4l2_frame_rate(fd, frame_rate);

  // Format and frame-rate negotiation may restore device defaults. Apply the
  // vendor trigger control only after both have completed.
  if (!configure_external_trigger_mode(fd)) {
    return NULL;
  }

  char fourcc[5] = {
      static_cast<char>(fmt.fmt.pix.pixelformat & 0xff),
      static_cast<char>((fmt.fmt.pix.pixelformat >> 8) & 0xff),
      static_cast<char>((fmt.fmt.pix.pixelformat >> 16) & 0xff),
      static_cast<char>((fmt.fmt.pix.pixelformat >> 24) & 0xff),
      '\0'};
  ROS_INFO("Camera capture format: %ux%u %s",
           fmt.fmt.pix.width, fmt.fmt.pix.height, fourcc);

  // 请求缓冲区
  struct v4l2_requestbuffers req;
  memset(&req, 0, sizeof(req));
  req.count = 4;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;

  if (ioctl(fd, VIDIOC_REQBUFS, &req) < 0) {
    perror("VIDIOC_REQBUFS");
    return NULL;
  }

  vector<void *> buffers(req.count);
  vector<size_t> buffer_lengths(req.count);

  for (int i = 0; i < req.count; ++i) {
    struct v4l2_buffer buf;
    memset(&buf, 0, sizeof(buf));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;

    if (ioctl(fd, VIDIOC_QUERYBUF, &buf) < 0) {
      perror("VIDIOC_QUERYBUF");
      return NULL;
    }

    buffers[i] = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.offset);
    if (buffers[i] == MAP_FAILED) {
      perror("mmap");
      return NULL;
    }
    buffer_lengths[i] = buf.length;

    if (ioctl(fd, VIDIOC_QBUF, &buf) < 0) {
      perror("VIDIOC_QBUF");
      return NULL;
    }
  }

  if (pts_metadata_enable ||
      timestamp_source == "pts_mid360_session_anchor") {
    pts_reader_available = pts_reader.Open(pts_metadata_device);
    if (!pts_reader_available &&
        timestamp_source == "pts_mid360_session_anchor") {
      ROS_FATAL("PTS session-anchor source requires metadata device %s",
                pts_metadata_device.c_str());
      return NULL;
    }
  }

  // 启动捕获
  enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(fd, VIDIOC_STREAMON, &type) < 0) {
    perror("VIDIOC_STREAMON");
    return NULL;
  }

  if (!verify_external_trigger_mode(fd, "after STREAMON")) {
    ioctl(fd, VIDIOC_STREAMOFF, &type);
    return NULL;
  }

  struct timespec streamon_monotonic_ts = {};
  clock_gettime(CLOCK_MONOTONIC, &streamon_monotonic_ts);
  const uint64_t streamon_monotonic_ns = static_cast<uint64_t>(
      TimespecToNs(streamon_monotonic_ts));
  if (timestamp_source == "pts_mid360_session_anchor") {
    pts_session_anchor.Start(streamon_monotonic_ns);
    ros::param::set("/uvc_camera/session_anchor_locked", false);
    ros::param::set("/uvc_camera/session_anchor_b_ns", std::string("0"));
    ROS_INFO("PTS_SESSION_ANCHOR_START fixed_a=%.17g "
             "streamon_monotonic_ns=%lu warmup_ms=%d collect_ms=%d; "
             "camera publication blocked until validation passes",
             camera_clock_scale, streamon_monotonic_ns,
             session_anchor_warmup_ms, session_anchor_collect_ms);
  }

  while (!exit_flag && ros::ok()) {
    struct v4l2_buffer buf;
    memset(&buf, 0, sizeof(buf));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    if (ioctl(fd, VIDIOC_DQBUF, &buf) < 0) {
      perror("VIDIOC_DQBUF");
      continue;
    }

    // Capture host dequeue timing immediately after DQBUF for diagnostics only.
    struct timespec dequeue_monotonic_ts = {};
    struct timespec dequeue_realtime_ts = {};
    clock_gettime(CLOCK_MONOTONIC, &dequeue_monotonic_ts);
    clock_gettime(CLOCK_REALTIME, &dequeue_realtime_ts);
    const int64_t dequeue_monotonic_ns = TimespecToNs(dequeue_monotonic_ts);
    const int64_t dequeue_realtime_ns = TimespecToNs(dequeue_realtime_ts);
    int64_t current_mid360_time_seq = 0;
    int64_t current_mid360_time_ns = 0;
    const bool current_mid360_time_valid = ReadLidarTimeShare(
        current_mid360_time_seq, current_mid360_time_ns);

    // Preserve the original V4L2 timing evidence alongside the image. In the
    // SOE source mode, its MID360-domain mapping also drives image timing.
    const int64_t v4l2_buffer_timestamp_ns = V4L2BufferTimestampNs(buf);
    int64_t v4l2_host_capture_ns = 0;
    int64_t v4l2_mapped_mid360_ns = 0;
    int64_t diagnostic_offset_seq = 0;
    int64_t diagnostic_clock_offset_ns = 0;
    int64_t diagnostic_offset_delta_ns = 0;
    const bool v4l2_host_time_valid =
        V4L2CaptureHostRealtimeNs(buf, v4l2_host_capture_ns);
    const bool diagnostic_offset_valid = ReadCommittedValue(
        clock_offset_pointt, diagnostic_offset_seq, diagnostic_clock_offset_ns);
    if (diagnostic_offset_valid &&
        diagnostic_offset_seq != last_diagnostic_clock_offset_seq) {
      if (last_diagnostic_clock_offset_seq > 0) {
        diagnostic_offset_delta_ns =
            diagnostic_clock_offset_ns - last_diagnostic_clock_offset_ns;
      }
      last_diagnostic_clock_offset_ns = diagnostic_clock_offset_ns;
      last_diagnostic_clock_offset_seq = diagnostic_offset_seq;
    }
    bool v4l2_mapping_valid = false;
    PtsFrameMetadata pts_metadata;
    int64_t pts_video_match_delta_ns = 0;
    bool pts_valid = false;
    if (pts_reader_available &&
        (buf.flags & V4L2_BUF_FLAG_TIMESTAMP_MASK) ==
            V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC) {
      pts_valid = pts_reader.Match(
          buf.sequence, static_cast<uint64_t>(v4l2_buffer_timestamp_ns),
          pts_match_max_delta_us * 1000LL, pts_metadata,
          pts_video_match_delta_ns);
    }
    const int64_t pts_mid360_observation_ns =
        pts_valid && v4l2_host_time_valid && diagnostic_offset_valid
            ? v4l2_host_capture_ns - diagnostic_clock_offset_ns
            : 0;
    int64_t camera_mid360_residual_ns = 0;

    ros::Time rcv_time;
    int64_t timestamp_ns = 0;
    int64_t trigger_seq = 0;
    int64_t trigger_time_ns = 0;
    int64_t pps_time_ns = 0;
    int trigger_index = -1;
    double camera_period_ms = 0.0;
    double raw_soe_period_ms = 0.0;
    double mapped_soe_period_ms = 0.0;
    bool sequence_discontinuity = false;
    bool used_fallback_this_frame = false;
    if (timestamp_source == "pts_mid360_session_anchor") {
      const bool anchor_was_locked = pts_session_anchor.locked();
      if (!pts_valid) {
        ++pts_match_drop_count;
        ++duplicate_frame_drop_count;
        ++consecutive_pts_match_drops;
        if (anchor_was_locked &&
            consecutive_pts_match_drops >=
                session_anchor_pts_reset_missing_frames) {
          ROS_ERROR("PTS_SESSION_RESET detected after %d consecutive "
                    "unmatched frames; invalidating b_session and blocking "
                    "camera publication",
                    consecutive_pts_match_drops);
          pts_reader.ResetUnwrapForNewSession();
          pts_session_anchor.Start(
              static_cast<uint64_t>(dequeue_monotonic_ns));
          camera_clock_offset_ns = 0;
          camera_sequence_initialized = false;
          last_pts_unwrapped_ticks = 0;
          last_pts_mapped_ns = 0;
          last_camera_stamp_ns = 0;
          consecutive_pts_match_drops = 0;
          ros::param::set("/uvc_camera/session_anchor_locked", false);
          ros::param::set("/uvc_camera/session_anchor_b_ns",
                          std::string("0"));
        }
        const auto anchor_result = pts_session_anchor.Observe(
            static_cast<uint64_t>(dequeue_monotonic_ns), nullptr, 0);
        if (anchor_result == PtsSessionAnchor::Result::kFailed) {
          ROS_FATAL("PTS_SESSION_ANCHOR_FAILED reason=%s valid_pairs=%lu "
                    "duplicates=%lu rollbacks=%lu sequence_jumps=%lu; "
                    "no timestamp fallback is permitted",
                    pts_session_anchor.failure_reason().c_str(),
                    pts_session_anchor.valid_pairs(),
                    pts_session_anchor.duplicate_count(),
                    pts_session_anchor.rollback_count(),
                    pts_session_anchor.sequence_jump_count());
          ros::param::set("/uvc_camera/session_anchor_failure",
                          pts_session_anchor.failure_reason());
          exit_flag = true;
          ioctl(fd, VIDIOC_QBUF, &buf);
          break;
        }
        ROS_WARN_THROTTLE(
            2.0,
            "Camera publication blocked: waiting for matched PTS and "
            "session-anchor lock (frame_seq=%u consecutive_missing=%d)",
            buf.sequence, consecutive_pts_match_drops);
        ioctl(fd, VIDIOC_QBUF, &buf);
        continue;
      }
      consecutive_pts_match_drops = 0;

      const auto anchor_result = pts_session_anchor.Observe(
          static_cast<uint64_t>(dequeue_monotonic_ns), &pts_metadata,
          pts_mid360_observation_ns);
      if (anchor_result == PtsSessionAnchor::Result::kFailed) {
        ROS_FATAL("PTS_SESSION_ANCHOR_FAILED reason=%s valid_pairs=%lu "
                  "mean_ms=%.6f p95_ms=%.6f drift_ms_per_s=%.9f "
                  "duplicates=%lu rollbacks=%lu sequence_jumps=%lu; "
                  "no timestamp fallback is permitted",
                  pts_session_anchor.failure_reason().c_str(),
                  pts_session_anchor.valid_pairs(),
                  pts_session_anchor.residual_mean_ms(),
                  pts_session_anchor.residual_p95_ms(),
                  pts_session_anchor.residual_drift_ms_per_s(),
                  pts_session_anchor.duplicate_count(),
                  pts_session_anchor.rollback_count(),
                  pts_session_anchor.sequence_jump_count());
        ros::param::set("/uvc_camera/session_anchor_failure",
                        pts_session_anchor.failure_reason());
        exit_flag = true;
        ioctl(fd, VIDIOC_QBUF, &buf);
        break;
      }
      if (anchor_result == PtsSessionAnchor::Result::kWaiting) {
        ROS_INFO_THROTTLE(
            1.0,
            "PTS_SESSION_ANCHOR_WAIT elapsed_ms=%.1f valid_pairs=%lu; "
            "camera publication remains blocked",
            pts_session_anchor.elapsed_ms(
                static_cast<uint64_t>(dequeue_monotonic_ns)),
            pts_session_anchor.valid_pairs());
        ioctl(fd, VIDIOC_QBUF, &buf);
        continue;
      }

      if (!anchor_was_locked) {
        camera_clock_offset_ns = pts_session_anchor.b_session_ns();
        ros::param::set("/uvc_camera/session_anchor_locked", true);
        ros::param::set("/uvc_camera/session_anchor_b_ns",
                        std::to_string(camera_clock_offset_ns));
        ros::param::set("/uvc_camera/session_anchor_elapsed_ms",
                        pts_session_anchor.elapsed_ms(
                            static_cast<uint64_t>(dequeue_monotonic_ns)));
        ros::param::set("/uvc_camera/session_anchor_valid_pairs",
                        static_cast<int>(pts_session_anchor.valid_pairs()));
        ros::param::set("/uvc_camera/session_anchor_residual_mean_ms",
                        pts_session_anchor.residual_mean_ms());
        ros::param::set("/uvc_camera/session_anchor_residual_p95_ms",
                        pts_session_anchor.residual_p95_ms());
        ros::param::set("/uvc_camera/session_anchor_residual_drift_ms_per_s",
                        pts_session_anchor.residual_drift_ms_per_s());
        ROS_INFO("PTS_SESSION_ANCHOR_LOCKED fixed_a=%.17g "
                 "b_session_ns=%ld pts_anchor_ticks=%lu mid_anchor_ns=%ld "
                 "elapsed_ms=%.3f warmup_ms=%d collect_ms=%d "
                 "valid_pairs=%lu residual_mean_ms=%.6f "
                 "residual_std_ms=%.6f residual_p95_ms=%.6f "
                 "residual_max_ms=%.6f drift_ms_per_s=%.9f "
                 "duplicates=%lu rollbacks=%lu sequence_jumps=%lu; "
                 "a,b_session frozen until PTS reset/STREAMOFF",
                 camera_clock_scale, camera_clock_offset_ns,
                 pts_session_anchor.pts_anchor_ticks(),
                 pts_session_anchor.mid_anchor_ns(),
                 pts_session_anchor.elapsed_ms(
                     static_cast<uint64_t>(dequeue_monotonic_ns)),
                 session_anchor_warmup_ms, session_anchor_collect_ms,
                 pts_session_anchor.valid_pairs(),
                 pts_session_anchor.residual_mean_ms(),
                 pts_session_anchor.residual_std_ms(),
                 pts_session_anchor.residual_p95_ms(),
                 pts_session_anchor.residual_max_ms(),
                 pts_session_anchor.residual_drift_ms_per_s(),
                 pts_session_anchor.duplicate_count(),
                 pts_session_anchor.rollback_count(),
                 pts_session_anchor.sequence_jump_count());
      }

      if (camera_sequence_initialized) {
        const uint32_t sequence_delta = buf.sequence - last_camera_sequence;
        sequence_discontinuity = sequence_delta != 1;
        if (sequence_discontinuity) {
          ++v4l2_stats_sequence_jump_count;
          ROS_WARN("PTS camera sequence discontinuity: previous=%u current=%u "
                   "delta=%u; session clock parameters remain frozen",
                   last_camera_sequence, buf.sequence, sequence_delta);
        }
      }

      // Centered form avoids multiplying two large absolute time values:
      // mid_anchor + a * (pts_ticks - pts_anchor_ticks) / PTS_CLOCK_HZ.
      v4l2_mapped_mid360_ns =
          pts_session_anchor.Map(pts_metadata.unwrapped_pts);
      timestamp_ns = v4l2_mapped_mid360_ns;
      v4l2_mapping_valid = true;
      if (pts_mid360_observation_ns > 0) {
        camera_mid360_residual_ns =
            timestamp_ns - pts_mid360_observation_ns;
      }
      if (last_pts_unwrapped_ticks > 0) {
        raw_soe_period_ms =
            (pts_metadata.unwrapped_pts - last_pts_unwrapped_ticks) /
            pts_clock_hz * 1000.0;
        mapped_soe_period_ms =
            (v4l2_mapped_mid360_ns - last_pts_mapped_ns) / 1000000.0;
        camera_period_ms =
            (timestamp_ns - last_camera_stamp_ns) / 1000000.0;
        if (raw_soe_period_ms == 0.0) {
          ++v4l2_stats_header_duplicate_count;
        } else if (raw_soe_period_ms < 0.0) {
          ++v4l2_stats_header_rollback_count;
        } else {
          v4l2_stats_raw_period_ms.push_back(raw_soe_period_ms);
        }
        if (mapped_soe_period_ms > 0.0) {
          v4l2_stats_mapped_period_ms.push_back(mapped_soe_period_ms);
        }
      }
      last_pts_unwrapped_ticks = pts_metadata.unwrapped_pts;
      last_pts_mapped_ns = v4l2_mapped_mid360_ns;
      LogV4L2FixedAnchorStatsIfReady(v4l2_mapped_mid360_ns);
      last_camera_sequence = buf.sequence;
      camera_sequence_initialized = true;
      ++hardware_stamp_count;
    } else if (timestamp_source == "v4l2_soe_lidar_clock") {
      if (camera_sequence_initialized) {
        const uint32_t sequence_delta = buf.sequence - last_camera_sequence;
        sequence_discontinuity = sequence_delta != 1;
        if (sequence_discontinuity) {
          ++v4l2_stats_sequence_jump_count;
          ROS_WARN("V4L2 sequence discontinuity: previous=%u current=%u "
                   "delta=%u; timestamp mapping remains SOE-delta based",
                   last_camera_sequence, buf.sequence, sequence_delta);
        }
      }

      const bool anchor_can_initialize =
          v4l2_buffer_timestamp_ns > 0 && v4l2_host_time_valid &&
          diagnostic_offset_valid && V4L2TimestampIsSoe(buf);
      if (!v4l2_fixed_anchor_initialized && anchor_can_initialize) {
        v4l2_anchor_ns = v4l2_buffer_timestamp_ns;
        mid360_anchor_ns =
            v4l2_host_capture_ns - diagnostic_clock_offset_ns;
        v4l2_fixed_anchor_initialized = true;
        ROS_INFO("V4L2 fixed anchor initialized: v4l2_anchor_ns=%ld "
                 "mid360_anchor_ns=%ld diagnostic_clock_offset_ns=%ld "
                 "diagnostic_offset_seq=%ld",
                 v4l2_anchor_ns, mid360_anchor_ns,
                 diagnostic_clock_offset_ns, diagnostic_offset_seq);
      }

      if (v4l2_fixed_anchor_initialized &&
          v4l2_buffer_timestamp_ns > 0 && V4L2TimestampIsSoe(buf)) {
        v4l2_mapped_mid360_ns =
            mid360_anchor_ns +
            (v4l2_buffer_timestamp_ns - v4l2_anchor_ns);
        v4l2_mapping_valid = true;
        timestamp_ns = v4l2_mapped_mid360_ns +
            (v4l2_soe_to_exposure_center_us +
             camera_calibration_offset_us) *
                1000LL;

        if (last_raw_v4l2_soe_ns > 0) {
          raw_soe_period_ms =
              (v4l2_buffer_timestamp_ns - last_raw_v4l2_soe_ns) /
              1000000.0;
          mapped_soe_period_ms =
              (v4l2_mapped_mid360_ns - last_fixed_mapped_soe_ns) /
              1000000.0;
          camera_period_ms = mapped_soe_period_ms;
          if (raw_soe_period_ms == 0.0) {
            ++v4l2_stats_header_duplicate_count;
          } else if (raw_soe_period_ms < 0.0) {
            ++v4l2_stats_header_rollback_count;
          } else {
            v4l2_stats_raw_period_ms.push_back(raw_soe_period_ms);
          }
          if (mapped_soe_period_ms > 0.0) {
            v4l2_stats_mapped_period_ms.push_back(mapped_soe_period_ms);
          }
        }
        if (v4l2_buffer_timestamp_ns > last_raw_v4l2_soe_ns &&
            v4l2_mapped_mid360_ns > last_fixed_mapped_soe_ns) {
          last_raw_v4l2_soe_ns = v4l2_buffer_timestamp_ns;
          last_fixed_mapped_soe_ns = v4l2_mapped_mid360_ns;
          LogV4L2FixedAnchorStatsIfReady(v4l2_mapped_mid360_ns);
        }
        ++hardware_stamp_count;
      } else {
        ++fallback_stamp_count;
        used_fallback_this_frame = true;
        timestamp_ns = FallbackNowInLidarClockDomainNs();
        ROS_WARN_THROTTLE(
            2.0,
            "V4L2 SOE timestamp or fixed MID360 anchor is unavailable "
            "(reference=%s anchor=%d); falling back to ros::Time::now() "
            "mapped into the MID360 clock domain (fallback=%lu, frame_seq=%u)",
            V4L2TimestampReference(buf), v4l2_fixed_anchor_initialized,
            fallback_stamp_count, buf.sequence);
      }
      last_camera_sequence = buf.sequence;
      camera_sequence_initialized = true;
    } else if (timestamp_source == "stm32_trigger_shared" ||
        timestamp_source == "stm32_pps_phase_grid" ||
        timestamp_source == "mid360_trigger_grid") {
      bool got_trigger = false;
      if (timestamp_source == "stm32_trigger_shared") {
        got_trigger = ReadCommittedTimestamp(trigger_seq, trigger_time_ns);
      } else if (timestamp_source == "stm32_pps_phase_grid") {
        if (camera_sequence_initialized) {
          const uint32_t frame_delta = buf.sequence - last_camera_sequence;
          if (frame_delta > stm32_sequence_reset_threshold) {
            ROS_WARN("Camera sequence reset/large jump: prev=%u current=%u "
                     "delta=%u; restarting V4L2 bootstrap matching",
                     last_camera_sequence, buf.sequence, frame_delta);
            camera_sequence_initialized = false;
            last_predicted_trigger_time_ns = 0;
            last_trigger_index_in_second = -1;
            v4l2_bootstrap_samples.clear();
          }
        }
        if (!camera_sequence_initialized) {
          got_trigger = BootstrapTriggerGridFromV4L2(
              buf, trigger_seq, trigger_time_ns);
          if (!got_trigger) {
            ioctl(fd, VIDIOC_QBUF, &buf);
            continue;
          }
        } else {
          got_trigger = PredictTriggerFromStm32PpsGrid(
              buf.sequence, trigger_seq, trigger_time_ns);
        }
      } else {
        got_trigger = PredictTriggerFromMid360Grid(
            buf.sequence, trigger_seq, trigger_time_ns);
      }
      if (!got_trigger) {
        ++fallback_stamp_count;
        used_fallback_this_frame = true;
        timestamp_ns = FallbackNowInLidarClockDomainNs();
        ROS_WARN_THROTTLE(
            2.0,
            "Camera trigger timestamp is unavailable for source=%s; falling back to "
            "ros::Time::now() mapped into the MID360 clock domain "
            "(fallback=%lu, frame_seq=%u)",
            timestamp_source.c_str(), fallback_stamp_count, buf.sequence);
      } else if (timestamp_source == "stm32_trigger_shared" &&
                 trigger_seq == last_trigger_seq) {
        ++fallback_stamp_count;
        used_fallback_this_frame = true;
        timestamp_ns = FallbackNowInLidarClockDomainNs();
        ROS_WARN_THROTTLE(
            2.0,
            "STM32 trigger timestamp was not updated; falling back to "
            "ros::Time::now() mapped into the MID360 clock domain "
            "(seq=%ld, fallback=%lu, frame_seq=%u)",
            trigger_seq, fallback_stamp_count, buf.sequence);
      } else {
        if (camera_sequence_initialized) {
          const uint32_t frame_delta = buf.sequence - last_camera_sequence;
          const int64_t trigger_delta = trigger_seq - last_trigger_seq;
          if (frame_delta > 1) {
            ROS_WARN(
                "Camera sequence skipped: prev=%u current=%u missed=%u",
                last_camera_sequence, buf.sequence, frame_delta - 1);
          }
          if (trigger_delta != static_cast<int64_t>(frame_delta)) {
            ROS_WARN(
                "STM32 trigger/frame sequence mismatch: prev_frame=%u "
                "frame=%u frame_delta=%u prev_trigger=%ld trigger=%ld "
                "trigger_delta=%ld",
                last_camera_sequence, buf.sequence, frame_delta,
                last_trigger_seq, trigger_seq, trigger_delta);
          }
        }

        const int64_t period_ns = stm32_trigger_period_us * 1000LL;
        if (timestamp_source == "stm32_pps_phase_grid" &&
            last_stm32_pps_epoch_ns > 0 &&
            last_trigger_index_in_second >= 0) {
          pps_time_ns = last_stm32_pps_epoch_ns;
          trigger_index = static_cast<int>(last_trigger_index_in_second);
        } else {
          pps_time_ns =
              (trigger_time_ns / 1000000000LL) * 1000000000LL;
        }
        if (period_ns > 0 && trigger_index < 0) {
          trigger_index =
              static_cast<int>((trigger_time_ns - pps_time_ns) / period_ns);
          if (trigger_index < 0 || trigger_index > 9) {
            ROS_WARN("STM32 trigger index outside 0..9: index=%d "
                     "trigger_time=%.9f pps_time=%.9f",
                     trigger_index, trigger_time_ns / 1e9,
                     pps_time_ns / 1e9);
          }
        }
        timestamp_ns = trigger_time_ns +
                       (stm32_trigger_to_exposure_center_us +
                        camera_calibration_offset_us) *
                           1000LL;
        if (timestamp_source == "stm32_pps_phase_grid") {
          MonitorV4L2TriggerAlignment(buf, timestamp_ns);
        }
        if (last_camera_stamp_ns > 0) {
          camera_period_ms =
              (timestamp_ns - last_camera_stamp_ns) / 1000000.0;
          if (std::abs(camera_period_ms -
                       stm32_trigger_period_us / 1000.0) > 5.0) {
            ROS_WARN(
                "Camera timestamp period abnormal: %.3f ms "
                "(frame_seq=%u trigger_seq=%ld)",
                camera_period_ms, buf.sequence, trigger_seq);
          }
        }
        last_trigger_seq = trigger_seq;
        last_predicted_trigger_time_ns = trigger_time_ns;
        ++hardware_stamp_count;
      }
      last_camera_sequence = buf.sequence;
      camera_sequence_initialized = true;
    } else {
      ++fallback_stamp_count;
      used_fallback_this_frame = true;
      timestamp_ns = FallbackNowInLidarClockDomainNs();
      ROS_WARN_THROTTLE(
          2.0,
          "Unsupported TimestampSource='%s'. Version5 expects "
          "'pts_mid360_session_anchor', "
          "'v4l2_soe_lidar_clock', "
          "'stm32_trigger_shared', "
          "'stm32_pps_phase_grid', or "
          "'mid360_trigger_grid'; falling back to ros::Time::now() mapped "
          "into the MID360 clock domain (fallback=%lu, frame_seq=%u)",
          timestamp_source.c_str(), fallback_stamp_count, buf.sequence);
    }

    if (timestamp_ns <= last_camera_stamp_ns) {
      ++duplicate_frame_drop_count;
      ROS_WARN_THROTTLE(
          5.0,
          "Dropping non-monotonic camera timestamp instead of synthesizing a "
          "100 ms step: current=%ld last=%ld fallback=%d",
          timestamp_ns, last_camera_stamp_ns, used_fallback_this_frame);
      ioctl(fd, VIDIOC_QBUF, &buf);
      continue;
    }
    last_camera_stamp_ns = timestamp_ns;
    rcv_time.fromNSec(timestamp_ns);
    OpenTimeDiagCsvIfRequested();
    if (time_diag_csv.is_open()) {
      time_diag_csv << hardware_stamp_count << "," << buf.sequence << ","
                    << pps_time_ns << "," << trigger_index << ","
                    << trigger_time_ns << "," << timestamp_ns << ","
                    << camera_period_ms << "," << camera_calibration_offset_us << ","
                    << v4l2_buffer_timestamp_ns << ","
                    << (v4l2_mapping_valid ? v4l2_mapped_mid360_ns : 0) << ","
                    << raw_soe_period_ms << "," << mapped_soe_period_ms << ","
                    << v4l2_soe_to_exposure_center_us << ","
                    << (diagnostic_offset_valid ? diagnostic_clock_offset_ns : 0)
                    << "," << diagnostic_offset_delta_ns << ","
                    << (sequence_discontinuity ? 1 : 0) << ","
                    << V4L2TimestampReference(buf) << ","
                    << buf.flags << ","
                    << buf.sequence << "," << buf.bytesused << ","
                    << dequeue_monotonic_ns << "," << dequeue_realtime_ns << ","
                    << (current_mid360_time_valid ? current_mid360_time_ns : 0)
                    << ","
                    << (current_mid360_time_valid ? current_mid360_time_seq : 0)
                    << "," << (pts_valid ? 1 : 0) << ","
                    << (pts_valid ? pts_metadata.raw_pts : 0) << ","
                    << (pts_valid ? pts_metadata.unwrapped_pts : 0) << ",";
      if (pts_valid) {
        time_diag_csv << std::fixed << std::setprecision(12)
                      << (pts_metadata.unwrapped_pts / pts_clock_hz)
                      << std::defaultfloat;
      }
      time_diag_csv << "," << (pts_valid ? pts_metadata.first_scr_stc : 0)
                    << "," << (pts_valid ? pts_metadata.first_scr_sof : 0)
                    << "," << (pts_valid ? pts_metadata.first_host_sof : 0)
                    << ","
                    << (pts_valid ? pts_metadata.first_payload_monotonic_ns : 0)
                    << "," << (pts_valid ? pts_metadata.first_meta_sequence : 0)
                    << "," << (pts_valid ? pts_video_match_delta_ns : 0)
                    << "," << pts_mid360_observation_ns
                    << "," << std::setprecision(17) << camera_clock_scale
                    << "," << camera_clock_offset_ns
                    << "," << exposure_center_offset_ns
                    << "," << camera_calibration_offset_ns
                    << "," << camera_mid360_residual_ns
                    << "," << timestamp_source
                    << "," << pts_reader.duplicate_count()
                    << "," << pts_reader.rollback_count()
                    << "," << pts_match_drop_count << "\n";
      time_diag_csv.flush();
    }

    cv::Mat rgb;
    if (fmt.fmt.pix.pixelformat == V4L2_PIX_FMT_MJPEG) {
      cv::Mat mjpeg(1, buf.bytesused, CV_8UC1, buffers[buf.index]);
      cv::Mat bgr = cv::imdecode(mjpeg, cv::IMREAD_COLOR);
      if (bgr.empty()) {
        ROS_WARN_THROTTLE(2.0, "Failed to decode MJPG frame");
        ioctl(fd, VIDIOC_QBUF, &buf);
        continue;
      }
      cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
    } else {
      cv::Mat yuyv(height, width, CV_8UC2, buffers[buf.index]);
      cv::cvtColor(yuyv, rgb, cv::COLOR_YUV2RGB_YUYV);
    }

    if (image_scale > 0.0 && image_scale != 1.0) {
      cv::resize(rgb, rgb, cv::Size(rgb.cols * image_scale, rgb.rows * image_scale), cv::INTER_LINEAR);
    }

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", rgb).toImageMsg();
    msg->header.seq = buf.sequence;
    msg->header.stamp = rcv_time;
    pub.publish(msg);
    std_msgs::String timing_msg;
    std::ostringstream timing;
    timing << "sequence=" << buf.sequence
           << ";v4l2_buffer_timestamp_ns=" << v4l2_buffer_timestamp_ns
           << ";v4l2_clock="
           << (((buf.flags & V4L2_BUF_FLAG_TIMESTAMP_MASK) ==
                V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC)
                   ? "monotonic"
                   : "realtime")
           << ";v4l2_reference=" << V4L2TimestampReference(buf)
           << ";host_capture_realtime_ns=" << v4l2_host_capture_ns
           << ";mapped_mid360_time_ns="
           << (v4l2_mapping_valid ? v4l2_mapped_mid360_ns : 0)
           << ";raw_soe_period_ms=" << raw_soe_period_ms
           << ";mapped_soe_period_ms=" << mapped_soe_period_ms
           << ";diagnostic_clock_offset_ns="
           << (diagnostic_offset_valid ? diagnostic_clock_offset_ns : 0)
           << ";diagnostic_offset_delta_ns=" << diagnostic_offset_delta_ns
           << ";sequence_discontinuity="
           << (sequence_discontinuity ? 1 : 0)
           << ";image_header_stamp_ns=" << timestamp_ns
           << ";v4l2_flags=" << buf.flags
           << ";dequeue_monotonic_ns=" << dequeue_monotonic_ns
           << ";dequeue_realtime_ns=" << dequeue_realtime_ns
           << ";current_mid360_time_ns="
           << (current_mid360_time_valid ? current_mid360_time_ns : 0)
           << ";current_mid360_time_seq="
           << (current_mid360_time_valid ? current_mid360_time_seq : 0)
           << ";pts_valid=" << (pts_valid ? 1 : 0)
           << ";pts_raw=" << (pts_valid ? pts_metadata.raw_pts : 0)
           << ";pts_unwrapped_ticks="
           << (pts_valid ? pts_metadata.unwrapped_pts : 0)
           << ";pts_scr_stc=" << (pts_valid ? pts_metadata.first_scr_stc : 0)
           << ";pts_scr_sof=" << (pts_valid ? pts_metadata.first_scr_sof : 0)
           << ";pts_payload_monotonic_ns="
           << (pts_valid ? pts_metadata.first_payload_monotonic_ns : 0)
           << ";pts_video_match_delta_ns="
           << (pts_valid ? pts_video_match_delta_ns : 0)
           << ";pts_mid360_observation_ns=" << pts_mid360_observation_ns
           << ";camera_mid360_residual_ns=" << camera_mid360_residual_ns
           << ";timestamp_source=" << timestamp_source;
    timing_msg.data = timing.str();
    v4l2_timing_pub.publish(timing_msg);
    if (timestamp_source == "pts_mid360_session_anchor") {
      ROS_INFO("Camera time sync: source=%s frame_seq=%u pts_raw=%u "
               "pts_unwrapped=%lu scale=%.17g offset_ns=%ld "
               "camera_time=%.9f residual_ms=%.6f period_ms=%.6f",
               timestamp_source.c_str(), buf.sequence, pts_metadata.raw_pts,
               pts_metadata.unwrapped_pts, camera_clock_scale,
               camera_clock_offset_ns, timestamp_ns / 1e9,
               camera_mid360_residual_ns / 1e6, camera_period_ms);
    } else if (timestamp_source == "v4l2_soe_lidar_clock") {
      ROS_INFO("Camera time sync: source=%s frame_seq=%u "
               "v4l2_reference=%s raw_soe_ns=%ld mapped_soe_ns=%ld "
               "raw_period_ms=%.6f mapped_period_ms=%.6f "
               "diagnostic_offset_ns=%ld diagnostic_offset_delta_ns=%ld "
               "sequence_discontinuity=%d exposure_center_us=%ld "
               "calibration_offset_us=%ld camera_time=%.9f "
               "image.header.stamp=%.9f",
               timestamp_source.c_str(), buf.sequence,
               V4L2TimestampReference(buf), v4l2_buffer_timestamp_ns,
               v4l2_mapped_mid360_ns, raw_soe_period_ms,
               mapped_soe_period_ms,
               diagnostic_offset_valid ? diagnostic_clock_offset_ns : 0,
               diagnostic_offset_delta_ns,
               sequence_discontinuity ? 1 : 0,
               v4l2_soe_to_exposure_center_us,
               camera_calibration_offset_us, timestamp_ns / 1e9,
               msg->header.stamp.toSec());
    } else {
      ROS_INFO("Camera time sync: source=%s frame_seq=%u pps_time=%.9f "
               "trigger_index=%d trigger_time=%.9f camera_time=%.9f "
               "image.header.stamp=%.9f period_ms=%.3f",
               timestamp_source.c_str(), buf.sequence, pps_time_ns / 1e9,
               trigger_index, trigger_time_ns / 1e9, timestamp_ns / 1e9,
               msg->header.stamp.toSec(), camera_period_ms);
    }
    ROS_INFO_STREAM_THROTTLE(
        10.0, "Camera timestamp counters: hardware=" << hardware_stamp_count
               << " fallback=" << fallback_stamp_count
               << " dropped=" << duplicate_frame_drop_count
              << " sequence_jumps=" << v4l2_stats_sequence_jump_count
              << " fixed_anchor=" << v4l2_fixed_anchor_initialized
               << " source=" << timestamp_source
              << " v4l2_flags=0x" << std::hex << buf.flags << std::dec);

    if (ioctl(fd, VIDIOC_QBUF, &buf) < 0) {
      perror("VIDIOC_QBUF");
    }
  }

  // 停止捕获
  if (ioctl(fd, VIDIOC_STREAMOFF, &type) < 0) {
    perror("VIDIOC_STREAMOFF");
  }

  for (int i = 0; i < req.count; ++i) {
    munmap(buffers[i], buffer_lengths[i]);
  }

  return NULL;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "uvc_driver");
  string params_file = string(argv[1]);
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  ros::Rate loop_rate(10);

  cv::FileStorage Params(params_file, cv::FileStorage::READ);
  if (!Params.isOpened()) {
    string msg = "Failed to open settings file at:" + params_file;
    ROS_ERROR_STREAM(msg.c_str());
    exit(-1);
  }

  trigger_enable = Params["TriggerEnable"];
  if (!Params["TimestampSource"].empty())
    timestamp_source = (string)Params["TimestampSource"];
  if (!Params["Stm32TriggerPath"].empty())
    stm32_trigger_path = (string)Params["Stm32TriggerPath"];
  if (!Params["ClockOffsetPath"].empty())
    clock_offset_path = (string)Params["ClockOffsetPath"];
  if (!Params["LidarTimeSharePath"].empty())
    lidar_timeshare_path = (string)Params["LidarTimeSharePath"];
  if (!Params["V4l2TimingTopic"].empty())
    v4l2_timing_topic = (string)Params["V4l2TimingTopic"];
  string pub_topic = Params["TopicName"];
  device_name = (string)Params["DeviceName"]; // 从参数文件读取设备名

  pub = it.advertise(pub_topic, 1);
  v4l2_timing_pub = nh.advertise<std_msgs::String>(v4l2_timing_topic, 200);
  ROS_INFO("Recording-friendly V4L2 timing diagnostics on %s",
           v4l2_timing_topic.c_str());

  pointt = (volatile time_stamp *)MAP_FAILED;
  if (timestamp_source == "stm32_trigger_shared") {
    const char *shared_file_name = stm32_trigger_path.c_str();
    int fd_time = open(shared_file_name, O_RDONLY);
    if (fd_time < 0) {
      ROS_ERROR("Unable to open STM32 trigger timestamp file %s; "
                "version5 will use ros::Time::now() fallback",
                shared_file_name);
    } else {
      pointt = (volatile time_stamp *)mmap(
          NULL, sizeof(time_stamp), PROT_READ, MAP_SHARED, fd_time, 0);
      if (pointt == MAP_FAILED) {
        ROS_ERROR("Unable to mmap STM32 trigger timestamp file %s; "
                  "version5 will use ros::Time::now() fallback",
                  shared_file_name);
      }
      close(fd_time);
    }
  }

  int fd_offset = open(clock_offset_path.c_str(), O_RDONLY);
  if (fd_offset < 0) {
    ROS_WARN("Unable to open MID360 clock offset file %s; fallback will use "
             "raw ros::Time::now()",
             clock_offset_path.c_str());
    clock_offset_pointt = (volatile time_stamp *)MAP_FAILED;
  } else {
    clock_offset_pointt = (volatile time_stamp *)mmap(
        NULL, sizeof(time_stamp), PROT_READ, MAP_SHARED, fd_offset, 0);
    if (clock_offset_pointt == MAP_FAILED) {
      ROS_WARN("Unable to mmap MID360 clock offset file %s; fallback will use "
               "raw ros::Time::now()",
               clock_offset_path.c_str());
    }
    close(fd_offset);
  }

  int fd_lidar_timeshare = open(lidar_timeshare_path.c_str(), O_RDONLY);
  if (fd_lidar_timeshare < 0) {
    ROS_WARN("Unable to open LiDAR timeshare file %s; mid360_trigger_grid "
             "will use the clock-offset fallback until it is available",
             lidar_timeshare_path.c_str());
    lidar_timeshare_pointt = (volatile time_stamp *)MAP_FAILED;
  } else {
    lidar_timeshare_pointt = (volatile time_stamp *)mmap(
        NULL, sizeof(time_stamp), PROT_READ, MAP_SHARED, fd_lidar_timeshare, 0);
    if (lidar_timeshare_pointt == MAP_FAILED) {
      ROS_WARN("Unable to mmap LiDAR timeshare file %s; mid360_trigger_grid "
               "will use the clock-offset fallback until it is available",
               lidar_timeshare_path.c_str());
    }
    close(fd_lidar_timeshare);
  }

  SetupSignalHandler();

  int fd = open(device_name.c_str(), O_RDWR);
  if (fd < 0) {
    perror("open device");
    ROS_ERROR("Failed to open device %s. Please check permissions or run with sudo.", device_name.c_str());
    return -1;
  }

  ROS_INFO("Successfully opened device: %s", device_name.c_str());

  setParams(fd, params_file);

  ROS_INFO("Finish all params set! Start grabbing...");

  pthread_t nThreadID;
  int ret = pthread_create(&nThreadID, NULL, WorkThread, &fd);
  if (ret != 0) {
    printf("thread create failed.ret = %d\n", ret);
    return -1;
  }

  while (!exit_flag && ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  if (nThreadID) {
    pthread_join(nThreadID, NULL);
    ROS_INFO_STREAM("Worker thread joined.");
  }

  close(fd);
  if (pointt != MAP_FAILED) {
    munmap((void *)pointt, sizeof(time_stamp));
  }

  return 0;
}
