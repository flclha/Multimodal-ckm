#include <fcntl.h>
#include <linux/uvcvideo.h>
#include <linux/usb/video.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace {

volatile sig_atomic_t stop_requested = 0;

void HandleSignal(int) { stop_requested = 1; }

uint64_t ClockNs(clockid_t clock_id) {
  timespec ts{};
  clock_gettime(clock_id, &ts);
  return static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL + ts.tv_nsec;
}

uint16_t ReadLe16(const uint8_t* p) {
  return static_cast<uint16_t>(p[0]) |
         (static_cast<uint16_t>(p[1]) << 8);
}

uint32_t ReadLe32(const uint8_t* p) {
  return static_cast<uint32_t>(p[0]) |
         (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) |
         (static_cast<uint32_t>(p[3]) << 24);
}

struct Mapping {
  void* address = MAP_FAILED;
  size_t length = 0;
};

struct Summary {
  uint32_t record_count = 0;
  uint32_t malformed_count = 0;
  uint32_t pts_record_count = 0;
  uint32_t scr_record_count = 0;
  uint32_t eof_count = 0;
  uint32_t pts_unique_count = 0;
  uint32_t dominant_pts = 0;
  uint32_t first_scr_stc = 0;
  uint32_t last_scr_stc = 0;
  uint16_t first_scr_sof = 0;
  uint16_t last_scr_sof = 0;
  uint16_t first_host_sof = 0;
  uint16_t last_host_sof = 0;
  uint64_t first_payload_host_ns = 0;
  uint64_t last_payload_host_ns = 0;
  uint8_t flags_or = 0;
  int fid = -1;
};

struct FrameAccumulator {
  bool active = false;
  uint64_t frame_index = 0;
  uint32_t pts = 0;
  uint32_t first_meta_buf_sequence = 0;
  uint32_t last_meta_buf_sequence = 0;
  uint32_t record_count = 0;
  uint32_t scr_record_count = 0;
  uint32_t eof_count = 0;
  uint32_t first_scr_stc = 0;
  uint32_t last_scr_stc = 0;
  uint16_t first_scr_sof = 0;
  uint16_t last_scr_sof = 0;
  uint16_t first_host_sof = 0;
  uint16_t last_host_sof = 0;
  uint64_t first_payload_host_ns = 0;
  uint64_t last_payload_host_ns = 0;
  uint8_t flags_or = 0;
  int fid = -1;

  void Flush(std::ofstream& csv) {
    if (!active) return;
    csv << frame_index++ << ',' << first_meta_buf_sequence << ','
        << last_meta_buf_sequence << ',' << pts << ',' << record_count << ','
        << scr_record_count << ',' << eof_count << ',' << first_scr_stc << ','
        << last_scr_stc << ',' << first_scr_sof << ',' << last_scr_sof << ','
        << first_host_sof << ',' << last_host_sof << ','
        << first_payload_host_ns << ',' << last_payload_host_ns << ','
        << static_cast<unsigned>(flags_or) << ',' << fid << '\n';
    csv.flush();
    active = false;
  }

  void Start(uint32_t new_pts, uint32_t meta_buf_sequence) {
    active = true;
    pts = new_pts;
    first_meta_buf_sequence = meta_buf_sequence;
    last_meta_buf_sequence = meta_buf_sequence;
    record_count = scr_record_count = eof_count = 0;
    first_scr_stc = last_scr_stc = 0;
    first_scr_sof = last_scr_sof = 0;
    first_host_sof = last_host_sof = 0;
    first_payload_host_ns = last_payload_host_ns = 0;
    flags_or = 0;
    fid = -1;
  }
};

Summary ParseMetadata(const uint8_t* data, size_t size) {
  Summary result;
  std::map<uint32_t, uint32_t> pts_counts;
  size_t offset = 0;

  while (offset + sizeof(uvc_meta_buf) <= size) {
    const auto* meta = reinterpret_cast<const uvc_meta_buf*>(data + offset);
    const size_t object_size = sizeof(meta->ns) + sizeof(meta->sof) + meta->length;
    if (meta->length < 2 || object_size < sizeof(uvc_meta_buf) ||
        offset + object_size > size) {
      ++result.malformed_count;
      break;
    }

    ++result.record_count;
    result.flags_or |= meta->flags;
    result.eof_count += (meta->flags & UVC_STREAM_EOF) != 0;
    const int fid = (meta->flags & UVC_STREAM_FID) ? 1 : 0;
    if (result.fid < 0) result.fid = fid;
    if (result.first_payload_host_ns == 0) {
      result.first_payload_host_ns = meta->ns;
      result.first_host_sof = meta->sof;
    }
    result.last_payload_host_ns = meta->ns;
    result.last_host_sof = meta->sof;

    const uint8_t* payload = meta->buf;
    size_t payload_size = meta->length - 2;
    size_t payload_offset = 0;
    if ((meta->flags & UVC_STREAM_PTS) && payload_size >= 4) {
      const uint32_t pts = ReadLe32(payload);
      ++pts_counts[pts];
      ++result.pts_record_count;
      payload_offset += 4;
    }
    if ((meta->flags & UVC_STREAM_SCR) &&
        payload_size >= payload_offset + 6) {
      const uint32_t scr_stc = ReadLe32(payload + payload_offset);
      const uint16_t scr_sof = ReadLe16(payload + payload_offset + 4);
      if (result.scr_record_count == 0) {
        result.first_scr_stc = scr_stc;
        result.first_scr_sof = scr_sof;
      }
      result.last_scr_stc = scr_stc;
      result.last_scr_sof = scr_sof;
      ++result.scr_record_count;
    }

    offset += object_size;
  }

  result.pts_unique_count = pts_counts.size();
  uint32_t dominant_count = 0;
  for (const auto& item : pts_counts) {
    if (item.second > dominant_count) {
      result.dominant_pts = item.first;
      dominant_count = item.second;
    }
  }
  return result;
}

void AccumulateMetadataFrames(const uint8_t* data, size_t size,
                              uint32_t meta_buf_sequence,
                              FrameAccumulator& frame,
                              std::ofstream& frame_csv) {
  size_t offset = 0;
  while (offset + sizeof(uvc_meta_buf) <= size) {
    const auto* meta = reinterpret_cast<const uvc_meta_buf*>(data + offset);
    const size_t object_size = sizeof(meta->ns) + sizeof(meta->sof) + meta->length;
    if (meta->length < 2 || object_size < sizeof(uvc_meta_buf) ||
        offset + object_size > size) {
      break;
    }

    const uint8_t* payload = meta->buf;
    const size_t payload_size = meta->length - 2;
    size_t payload_offset = 0;
    if (!(meta->flags & UVC_STREAM_PTS) || payload_size < 4) {
      offset += object_size;
      continue;
    }
    const uint32_t pts = ReadLe32(payload);
    payload_offset += 4;
    if (frame.active && frame.pts != pts) frame.Flush(frame_csv);
    if (!frame.active) frame.Start(pts, meta_buf_sequence);

    frame.last_meta_buf_sequence = meta_buf_sequence;
    ++frame.record_count;
    frame.flags_or |= meta->flags;
    frame.eof_count += (meta->flags & UVC_STREAM_EOF) != 0;
    const int fid = (meta->flags & UVC_STREAM_FID) ? 1 : 0;
    if (frame.fid < 0) frame.fid = fid;
    if (frame.first_payload_host_ns == 0) {
      frame.first_payload_host_ns = meta->ns;
      frame.first_host_sof = meta->sof;
    }
    frame.last_payload_host_ns = meta->ns;
    frame.last_host_sof = meta->sof;

    if ((meta->flags & UVC_STREAM_SCR) &&
        payload_size >= payload_offset + 6) {
      const uint32_t scr_stc = ReadLe32(payload + payload_offset);
      const uint16_t scr_sof = ReadLe16(payload + payload_offset + 4);
      if (frame.scr_record_count == 0) {
        frame.first_scr_stc = scr_stc;
        frame.first_scr_sof = scr_sof;
      }
      frame.last_scr_stc = scr_stc;
      frame.last_scr_sof = scr_sof;
      ++frame.scr_record_count;
    }
    offset += object_size;
  }
}

}  // namespace

int main(int argc, char** argv) {
  const std::string device = argc > 1 ? argv[1] : "/dev/video1";
  const std::string output = argc > 2 ? argv[2] : "uvc_metadata_frames.csv";
  const int duration_seconds = argc > 3 ? std::max(1, std::stoi(argv[3])) : 125;

  signal(SIGINT, HandleSignal);
  signal(SIGTERM, HandleSignal);

  const int fd = open(device.c_str(), O_RDWR | O_NONBLOCK);
  if (fd < 0) {
    perror("open metadata device");
    return 1;
  }

  v4l2_requestbuffers request{};
  request.count = 8;
  request.type = V4L2_BUF_TYPE_META_CAPTURE;
  request.memory = V4L2_MEMORY_MMAP;
  if (ioctl(fd, VIDIOC_REQBUFS, &request) < 0 || request.count < 2) {
    perror("VIDIOC_REQBUFS metadata");
    close(fd);
    return 1;
  }

  std::vector<Mapping> mappings(request.count);
  for (uint32_t i = 0; i < request.count; ++i) {
    v4l2_buffer buffer{};
    buffer.type = request.type;
    buffer.memory = request.memory;
    buffer.index = i;
    if (ioctl(fd, VIDIOC_QUERYBUF, &buffer) < 0) {
      perror("VIDIOC_QUERYBUF metadata");
      return 1;
    }
    mappings[i].length = buffer.length;
    mappings[i].address = mmap(nullptr, buffer.length, PROT_READ | PROT_WRITE,
                               MAP_SHARED, fd, buffer.m.offset);
    if (mappings[i].address == MAP_FAILED) {
      perror("mmap metadata");
      return 1;
    }
    if (ioctl(fd, VIDIOC_QBUF, &buffer) < 0) {
      perror("VIDIOC_QBUF metadata");
      return 1;
    }
  }

  std::ofstream frame_csv(output, std::ios::out | std::ios::trunc);
  std::ofstream buffer_csv(output + ".buffers.csv",
                           std::ios::out | std::ios::trunc);
  if (!frame_csv || !buffer_csv) {
    std::cerr << "Unable to open " << output << "\n";
    return 1;
  }
  frame_csv
      << "metadata_frame_index,first_meta_buf_sequence,last_meta_buf_sequence,"
         "dominant_pts,record_count,scr_record_count,eof_count,first_scr_stc,"
         "last_scr_stc,first_scr_sof,last_scr_sof,first_host_sof,"
         "last_host_sof,first_payload_host_ns,last_payload_host_ns,"
         "payload_flags_or,fid\n";
  buffer_csv
      << "meta_buf_sequence,dq_monotonic_ns,dq_realtime_ns,bytesused,"
         "v4l2_flags,record_count,malformed_count,pts_record_count,"
         "scr_record_count,eof_count,pts_unique_count,dominant_pts,"
         "first_scr_stc,last_scr_stc,first_scr_sof,last_scr_sof,"
         "first_host_sof,last_host_sof,first_payload_host_ns,"
         "last_payload_host_ns,payload_flags_or,fid\n";

  v4l2_buf_type type = V4L2_BUF_TYPE_META_CAPTURE;
  if (ioctl(fd, VIDIOC_STREAMON, &type) < 0) {
    perror("VIDIOC_STREAMON metadata");
    return 1;
  }

  const uint64_t deadline = ClockNs(CLOCK_MONOTONIC) +
                            static_cast<uint64_t>(duration_seconds) * 1000000000ULL;
  uint64_t buffer_count = 0;
  FrameAccumulator frame;
  while (!stop_requested && ClockNs(CLOCK_MONOTONIC) < deadline) {
    pollfd poll_fd{};
    poll_fd.fd = fd;
    poll_fd.events = POLLIN | POLLPRI;
    const int poll_result = poll(&poll_fd, 1, 1000);
    if (poll_result < 0) {
      if (errno == EINTR) continue;
      perror("poll metadata");
      break;
    }
    if (poll_result == 0) continue;

    v4l2_buffer buffer{};
    buffer.type = V4L2_BUF_TYPE_META_CAPTURE;
    buffer.memory = V4L2_MEMORY_MMAP;
    if (ioctl(fd, VIDIOC_DQBUF, &buffer) < 0) {
      if (errno == EAGAIN || errno == EINTR) continue;
      perror("VIDIOC_DQBUF metadata");
      break;
    }

    const uint64_t dq_monotonic_ns = ClockNs(CLOCK_MONOTONIC);
    const uint64_t dq_realtime_ns = ClockNs(CLOCK_REALTIME);
    const Summary summary = ParseMetadata(
        static_cast<const uint8_t*>(mappings[buffer.index].address),
        buffer.bytesused);
    buffer_csv << buffer.sequence << ',' << dq_monotonic_ns << ','
               << dq_realtime_ns << ',' << buffer.bytesused << ','
               << buffer.flags << ',' << summary.record_count << ','
               << summary.malformed_count << ',' << summary.pts_record_count
               << ',' << summary.scr_record_count << ',' << summary.eof_count
               << ',' << summary.pts_unique_count << ',' << summary.dominant_pts
               << ',' << summary.first_scr_stc << ',' << summary.last_scr_stc
               << ',' << summary.first_scr_sof << ',' << summary.last_scr_sof
               << ',' << summary.first_host_sof << ',' << summary.last_host_sof
               << ',' << summary.first_payload_host_ns << ','
               << summary.last_payload_host_ns << ','
               << static_cast<unsigned>(summary.flags_or) << ',' << summary.fid
               << '\n';
    buffer_csv.flush();
    AccumulateMetadataFrames(
        static_cast<const uint8_t*>(mappings[buffer.index].address),
        buffer.bytesused, buffer.sequence, frame, frame_csv);
    ++buffer_count;

    if (ioctl(fd, VIDIOC_QBUF, &buffer) < 0) {
      perror("VIDIOC_QBUF metadata recycle");
      break;
    }
  }

  frame.Flush(frame_csv);
  ioctl(fd, VIDIOC_STREAMOFF, &type);
  for (const Mapping& mapping : mappings) {
    if (mapping.address != MAP_FAILED) munmap(mapping.address, mapping.length);
  }
  close(fd);
  std::cerr << "Captured " << buffer_count << " metadata buffers into "
            << output << "\n";
  return buffer_count > 0 ? 0 : 2;
}
