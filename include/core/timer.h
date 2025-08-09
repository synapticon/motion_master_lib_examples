#pragma once

#include <atomic>
#include <chrono>
#include <mutex>

#ifdef _WIN32
#include <windows.h>  // Must be included before mmsystem.h for Windows multimedia timer
#endif
#ifdef __linux__
#include <signal.h>  // POSIX signal handling
#include <time.h>    // POSIX timer and time functions
#endif

#ifdef _WIN32
#include <mmsystem.h>  // Windows multimedia system for timeSetEvent
#endif

#include <condition_variable>

namespace mm::core::timer {

/**
 * @class MainTimer
 * @brief Cross-platform high-resolution periodic timer.
 *
 * The MainTimer class implements a singleton timer that generates periodic
 * high-resolution timer events. It is designed to be thread-safe and allows
 * other components in the application to synchronize with or wait for timer
 * ticks.
 *
 * This class supports both Windows and POSIX-compliant systems, providing a
 * consistent interface for time-based operations across platforms.
 */
class MainTimer {
 public:
  /**
   * @brief Static delay in milliseconds between each timer tick.
   */
  static unsigned int DELAY_MS;

  /**
   * @brief Static mutex protecting the static condition variable for notifying
   * all listener threads.
   *
   * This mutex must be locked whenever waiting on or notifying the static
   * condition variable to ensure thread-safe synchronization across all
   * listeners.
   */
  static std::mutex listenerMx;

  /**
   * @brief Static condition variable used to notify all listener threads of
   * timer ticks.
   *
   * This condition variable works together with the static mutex `listenerMx`
   * to safely synchronize notification and waiting of multiple listener
   * threads.
   */
  static std::condition_variable listenerCv;

  /**
   * @brief Returns the singleton instance of the MainTimer.
   *
   * Ensures only one instance of MainTimer exists throughout the application's
   * lifetime. Initializes the timer on the first call.
   *
   * @return Reference to the singleton MainTimer instance.
   */
  static MainTimer& getInstance();

  /**
   * @brief Notifies all listeners (waiting threads) about a timer tick.
   *
   * Broadcasts a signal using the condition variable to wake any threads
   * waiting for a tick.
   */
  static void notifyAllListeners();

  /**
   * @brief Waits for the next timer tick to occur.
   *
   * Blocks the current thread until a tick is received. Tracks missed ticks
   * and updates internal state accordingly.
   */
  void wait();

  /**
   * @brief Returns the current tick count.
   *
   * Tick count increments with each timer tick and reflects the total number
   * of ticks since the timer was started.
   *
   * @return The current tick count as an unsigned 64-bit integer.
   */
  uint64_t getTickCount() const;

  /**
   * @brief Returns the number of completed wait cycles.
   *
   * A cycle is recorded each time `wait()` completes successfully.
   *
   * @return Completed cycle count as an unsigned 64-bit integer.
   */
  uint64_t getCycleCount() const;

  /**
   * @brief Returns the number of missed timer ticks in the current cycle.
   *
   * Indicates how many ticks were missed due to delays in processing.
   *
   * @return Number of missed ticks as an unsigned 64-bit integer.
   */
  uint64_t getCycleMissedTickCount() const;

  /**
   * @brief Returns the total elapsed time since the timer started.
   *
   * Uses a high-resolution clock to calculate the elapsed time in microseconds.
   *
   * @return Elapsed time in microseconds as a signed 64-bit integer.
   */
  int64_t getElapsedTime() const;

  /**
   * @brief Returns the interval in microseconds between two consecutive
   * `wait()` calls.
   *
   * @return The interval in microseconds as a signed 64-bit integer.
   */
  int64_t getWaitIntervalUs() const;

 private:
  /** @brief Mutex for synchronizing access to internal state. */
  std::mutex mx_;

  /** @brief Condition variable used to block/wake waiting threads. */
  std::condition_variable cv_;

  /** @brief Timer ID used on Windows. */
  unsigned int timerID_;

  /** @brief Flag indicating if the timer has ticked. */
  bool timerTick_;

  /** @brief Atomic count of the total number of timer ticks. */
  std::atomic<uint64_t> tickCount_;

  /** @brief Atomic count of completed wait cycles. */
  std::atomic<uint64_t> cycleCount_;

  /** @brief Atomic count of the previous tick for cycle computation. */
  std::atomic<uint64_t> prevTickCount_;

  /** @brief Atomic count of missed ticks during the current cycle. */
  std::atomic<uint64_t> cycleMissedTickCount_;

  /** @brief Start time in microseconds since epoch. */
  int64_t start_;

  /** @brief Timestamp of the previous wait() call. */
  std::chrono::steady_clock::time_point lastWaitTime_{};

  /** @brief Interval in microseconds between two wait() calls. */
  std::atomic<int64_t> waitIntervalUs_{0};

  /**
   * @brief Constructs and initializes the timer.
   *
   * Private to enforce singleton pattern. Initializes internal state and sets
   * up the platform-specific timer.
   */
  MainTimer();

  /**
   * @brief Cleans up timer resources.
   */
  ~MainTimer();

  /** @brief Deleted copy constructor to prevent copying. */
  MainTimer(const MainTimer&) = delete;

  /** @brief Deleted assignment operator to prevent copying. */
  MainTimer& operator=(const MainTimer&) = delete;

#ifdef _WIN32
  /**
   * @brief Windows-specific timer callback function.
   *
   * Triggered by the multimedia timer and invokes `onTimerTick`.
   *
   * @param uID Timer identifier.
   * @param uMsg Message identifier (unused).
   * @param dwUser User data pointer (this pointer).
   * @param dw1 Reserved (unused).
   * @param dw2 Reserved (unused).
   */
  static void CALLBACK TimerCallback(unsigned int uID, unsigned int uMsg,
                                     DWORD_PTR dwUser, DWORD_PTR dw1,
                                     DWORD_PTR dw2);
#endif
#ifdef __linux__
  /** @brief POSIX timer identifier used on Linux systems. */
  timer_t timerid_;

  /**
   * @brief Signal handler triggered on each POSIX timer tick.
   *
   * Invoked when SIGALRM is raised by the timer and delegates to `onTimerTick`.
   *
   * @param sig Signal number (usually SIGALRM).
   */
  static void timer_handler(int sig);
#endif

  /**
   * @brief Handles internal logic for a timer tick.
   *
   * Increments tick counters and signals waiting threads.
   */
  void onTimerTick();
};

/**
 * @class MainTimerListener
 * @brief Listener for periodic ticks from a MainTimer.
 *
 * The MainTimerListener class allows a thread to wait for periodic ticks
 * generated by a MainTimer instance. It tracks tick counts, completed wait
 * cycles, and elapsed time since its creation. This is useful for applications
 * that require precise, per-thread timing synchronization based on a shared
 * timer.
 */
class MainTimerListener {
 public:
  /**
   * @brief Constructs a new MainTimerListener instance.
   *
   * Initializes counters for ticks, reference tick, and cycles. Also records
   * the start time using the system's high-resolution clock.
   */
  MainTimerListener();

  /**
   * @brief Waits for the next timer tick and updates internal state.
   *
   * Blocks the current thread until a tick is received from the MainTimer.
   * Increments tick and cycle counters. Each successful call represents a
   * completed timing cycle.
   */
  void wait();

  /**
   * @brief Returns the number of ticks since the first tick received.
   *
   * The tick count is adjusted to be relative to the first tick seen by this
   * listener instance.
   *
   * @return Relative tick count as an unsigned 64-bit integer.
   */
  uint64_t getTickCount() const;

  /**
   * @brief Returns the number of completed wait cycles.
   *
   * Each call to `wait()` increases the cycle count. This provides a measure
   * of how many complete wait-tick sequences have occurred.
   *
   * @return Completed cycle count as an unsigned 64-bit integer.
   */
  uint64_t getCycleCount() const;

  /**
   * @brief Returns the total elapsed time since the listener was created.
   *
   * Calculates the elapsed time in microseconds using a high-resolution clock.
   *
   * @return Elapsed time in microseconds as a signed 64-bit integer.
   */
  int64_t getElapsedTime() const;

 private:
  /** @brief Current tick count since the listener began receiving ticks. */
  std::atomic<uint64_t> tickCount_;

  /** @brief Reference tick count recorded at the first tick received. */
  std::atomic<uint64_t> refTickCount_;

  /** @brief Count of completed wait cycles. */
  std::atomic<uint64_t> cycleCount_;

  /** @brief Timestamp in microseconds when the listener was constructed. */
  int64_t start_;
};

}  // namespace mm::core::timer
