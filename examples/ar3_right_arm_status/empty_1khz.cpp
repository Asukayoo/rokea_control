/**
 * 空 1 kHz（规范写法）：
 *   SCHED_FIFO + 绑核 + mlockall
 *   clock_nanosleep(ABS) 睡到 deadline-margin，再 spin 到 deadline
 *
 * 用法：sudo ./empty_1khz [duration_s=5] [cpu=4] [prio=80]
 * 建议先：sudo bash ../../scripts/rt_performance.sh
 * 且 grub 已 isolcpus=2,4 并重启。
 */
#include <algorithm>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#include <sys/prctl.h>
#include <time.h>
#include <unistd.h>
#include <vector>

namespace {

constexpr int64_t kPeriodNs = 1000000;     // 1 ms
constexpr int64_t kSpinMarginNs = 100000;  // 最后 100 µs 忙等

int64_t now_ns() {
  timespec ts{};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}

timespec ns_to_ts(int64_t ns) {
  timespec ts{};
  ts.tv_sec = ns / 1000000000LL;
  ts.tv_nsec = ns % 1000000000LL;
  return ts;
}

void apply_rt(int cpu, int prio) {
  prctl(PR_SET_TIMERSLACK, 1);
  if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0)
    perror("mlockall");

  // 本进程再握一次 dma latency（脚本也会握）
  int fd = open("/dev/cpu_dma_latency", O_RDWR);
  if (fd >= 0) {
    int32_t lat = 0;
    if (write(fd, &lat, sizeof(lat)) != sizeof(lat))
      perror("cpu_dma_latency");
    // 故意不 close，保持到进程结束
    (void)fd;
  }

  cpu_set_t set;
  CPU_ZERO(&set);
  CPU_SET(cpu, &set);
  if (pthread_setaffinity_np(pthread_self(), sizeof(set), &set) != 0) {
    perror("affinity");
    std::exit(1);
  }
  sched_param sp{};
  sp.sched_priority = prio;
  if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0) {
    perror("SCHED_FIFO");
    std::exit(1);
  }
  int pol = 0;
  sched_param got{};
  pthread_getschedparam(pthread_self(), &pol, &got);
  std::cout << "empty_1khz(hybrid) pid=" << getpid()
            << " policy=" << (pol == SCHED_FIFO ? "FIFO" : "?")
            << " prio=" << got.sched_priority << " cpu=" << cpu << "\n";
}

}  // namespace

int main(int argc, char **argv) {
  const double dur = (argc >= 2) ? std::stod(argv[1]) : 5.0;
  const int cpu = (argc >= 3) ? std::stoi(argv[2]) : 4;
  const int prio = (argc >= 4) ? std::stoi(argv[3]) : 80;
  apply_rt(cpu, prio);

  const int warmup = 500;
  const int nmax = std::max(1, static_cast<int>(std::lround(dur * 1000.0)));
  std::vector<double> dts;
  dts.reserve(static_cast<size_t>(nmax));

  int64_t next = now_ns() + kPeriodNs;
  int64_t prev = 0;
  for (int i = 0; i < nmax + warmup; ++i) {
    const int64_t sleep_until = next - kSpinMarginNs;
    if (sleep_until > now_ns()) {
      timespec ts = ns_to_ts(sleep_until);
      clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr);
    }
    while (now_ns() < next) {
    }
    const int64_t t = now_ns();
    if (i >= warmup && prev > 0)
      dts.push_back((t - prev) / 1e6);
    prev = t;
    next += kPeriodNs;
    if (t > next + kPeriodNs)
      next = t + kPeriodNs;
  }

  auto s = dts;
  std::sort(s.begin(), s.end());
  const double avg = std::accumulate(dts.begin(), dts.end(), 0.0) / dts.size();
  size_t o105 = 0, o12 = 0, o20 = 0;
  for (double d : dts) {
    if (d > 1.05) ++o105;
    if (d > 1.2) ++o12;
    if (d > 2.0) ++o20;
  }
  const double p99 = s[static_cast<size_t>((s.size() - 1) * 0.99)];
  const double p999 = s[static_cast<size_t>((s.size() - 1) * 0.999)];
  std::cout << std::fixed << std::setprecision(3) << "samples=" << dts.size()
            << " avg=" << avg << " p99=" << p99 << " p999=" << p999
            << " max=" << s.back() << " ms\n"
            << ">1.05ms=" << o105 << " >1.2ms=" << o12 << " >2.0ms=" << o20 << "\n"
            << ((s.back() <= 1.05 && o20 == 0) ? "PASS\n" : "CHECK\n");
  return (s.back() <= 1.05) ? 0 : 1;
}
