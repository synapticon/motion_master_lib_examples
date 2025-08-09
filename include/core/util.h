#pragma once

#include <algorithm>
#include <bit>
#include <charconv>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <iterator>
#include <map>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <optional>
#include <queue>
#include <set>
#include <sstream>
#include <string>
#include <string_view>
#include <thread>
#include <type_traits>
#include <typeindex>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

namespace mm::core::util {

/**
 * @brief Parses a hexadecimal string into an unsigned integer of type T.
 *
 * This function converts a hex string (e.g., "0x1A3F" or "1A3F") to an
 * unsigned integer of the specified type T. It supports any unsigned
 * integer type such as uint16_t, uint32_t, uint64_t, etc.
 *
 * The function uses `std::stoul` for types up to the size of `unsigned long`,
 * and `std::stoull` for larger types (e.g., `uint64_t`).
 *
 * @tparam T The unsigned integer type to parse to. Must be an unsigned integral
 * type.
 * @param s The hexadecimal string to parse.
 * @return Parsed value as type T.
 *
 * @throws std::invalid_argument if the string is not a valid hex number.
 * @throws std::out_of_range if the parsed value is out of range for type T.
 */
template <typename T>
T parseHex(const std::string& s) {
  static_assert(std::is_unsigned<T>::value,
                "parseHex only supports unsigned integer types");

  // Use the appropriate std::stoul or std::stoull depending on size
  if constexpr (sizeof(T) <= sizeof(unsigned long)) {
    // Use std::stoul
    return static_cast<T>(std::stoul(s, nullptr, 16));
  } else {
    // Use std::stoull for larger types like uint64_t
    return static_cast<T>(std::stoull(s, nullptr, 16));
  }
}

/**
 * @brief Represents the PDO mapping for RX and TX channels.
 *
 * Contains two ordered maps:
 * - `rx`: Maps 16-bit keys to vectors of 32-bit unsigned integers for receive
 * PDO entries.
 * - `tx`: Maps 16-bit keys to vectors of 32-bit unsigned integers for transmit
 * PDO entries.
 */
struct UiPdoMapping {
  std::map<std::uint16_t, std::vector<std::uint32_t>> rx;
  std::map<std::uint16_t, std::vector<std::uint32_t>> tx;
};

/**
 * @brief Top-level UI configuration JSON structure.
 *
 * Contains the PDO mappings under the `pdoMapping` member.
 */
struct UiConfigJson {
  UiPdoMapping pdoMapping;
};

/**
 * @brief Deserialize a JSON object into a UiPdoMapping structure.
 *
 * This function parses the JSON object representing the PDO mappings,
 * extracting the "rx" and "tx" maps. The JSON keys are hex strings
 * (e.g., "0x1600") which are converted to uint16_t keys in the maps.
 * The values are arrays of hex strings representing uint32_t entries.
 *
 * @param j JSON object expected to contain "rx" and "tx" mappings.
 * @param p Reference to UiPdoMapping struct to populate with parsed data.
 */
void from_json(const nlohmann::json& j, UiPdoMapping& p);

/**
 * @brief Deserialize a JSON object into a UiConfigJson structure.
 *
 * Extracts the "pdoMapping" field from the JSON object and converts it
 * into a UiPdoMapping instance, which is assigned to the `pdoMapping` member.
 *
 * @param j JSON object expected to contain a "pdoMapping" field.
 * @param r Reference to UiConfigJson struct to populate.
 */
void from_json(const nlohmann::json& j, UiConfigJson& r);

/**
 * @brief Specifies the byte order used when converting a byte sequence to an
 * integer.
 *
 * This enum is used to indicate whether the byte sequence should be interpreted
 * in little-endian or big-endian format.
 *
 * - Little: Least significant byte comes first.
 * - Big: Most significant byte comes first.
 */
enum class Endianness { LITTLE, BIG };

/**
 * @brief Converts a byte sequence to an integer of type T with optional offset
 * and endianness.
 *
 * This function reads up to sizeof(T) bytes from the given data vector starting
 * at the specified offset and interprets them as an integer of type T. If there
 * are fewer than sizeof(T) bytes available, the missing high-order bytes are
 * zero-padded.
 *
 * Supported integer types include signed and unsigned types like int16_t,
 * uint32_t, etc.
 *
 * @tparam T        The target integral type to convert to (e.g., int32_t,
 * uint16_t).
 * @param data      A vector of bytes to extract the integer value from.
 * @param offset    The starting index in the vector (default is 0).
 * @param endian    The byte order used for conversion (default is
 * Endianness::Little).
 * @return T        The resulting integer value of type T.
 *
 * @note This function uses static_assert to ensure T is an integral type.
 * @note If the offset is beyond the end of the vector, the result will be 0.
 */
template <typename T>
T toInteger(const std::vector<std::uint8_t>& data, std::size_t offset = 0,
            Endianness endian = Endianness::LITTLE) {
  // Ensure T is an integral type
  static_assert(std::is_integral<T>::value, "T must be an integral type");

  // Use unsigned type for accumulation to avoid sign extension issues
  using U = typename std::make_unsigned<T>::type;
  U value = 0;

  // Check if offset is within bounds
  // If offset is greater than the size of data, return 0
  std::size_t remaining = (data.size() > offset) ? (data.size() - offset) : 0;
  std::size_t byteCount = (sizeof(T) < remaining) ? sizeof(T) : remaining;

  if (endian == Endianness::LITTLE) {
    for (std::size_t i = 0; i < byteCount; ++i) {
      value |= static_cast<U>(data[offset + i]) << (8 * i);
    }
  } else {
    for (std::size_t i = 0; i < byteCount; ++i) {
      value |= static_cast<U>(data[offset + i]) << (8 * (sizeof(T) - 1 - i));
    }
  }

  return static_cast<T>(value);
}

/**
 * @brief Splits a byte vector into parts of a specified maximum size.
 *
 * @param input The input vector of bytes.
 * @param partSize The maximum size (in bytes) for each part.
 * @return std::vector<std::vector<uint8_t>> The resulting vector of parts.
 */
std::vector<std::vector<uint8_t>> splitIntoParts(
    const std::vector<uint8_t>& input, size_t partSize);

/**
 * @brief Compresses a block of data into a ZIP archive containing a single
 * file.
 *
 * This function creates an in-memory ZIP archive and adds a single entry (file)
 * to it, containing the provided raw data. It returns the entire ZIP archive as
 * a byte vector.
 *
 * @param filename The name of the file entry to store inside the ZIP archive.
 * @param data The raw data to be compressed and stored in the ZIP archive.
 * @return std::vector<uint8_t> The resulting ZIP archive as a vector of bytes.
 *
 * @throws std::runtime_error If any step of the ZIP creation fails (e.g., entry
 * open, write, or stream copy).
 *
 * @note This function requires a ZIP library that provides zip_stream_*
 * functions, such as miniz or a compatible wrapper.
 */
std::vector<uint8_t> zipData(const std::string& filename,
                             const std::vector<uint8_t>& data);

/**
 * @brief Unzips a ZIP archive from an in-memory byte buffer.
 *
 * This function opens a ZIP archive from the provided raw data buffer,
 * extracts all files inside it, and returns a mapping from filenames
 * to their respective uncompressed file data.
 *
 * @param data A vector of bytes containing the ZIP archive data.
 *
 * @return An unordered_map where each key is a filename (string) and
 *         the corresponding value is a vector<uint8_t> with the uncompressed
 *         file contents.
 *
 * @throws std::runtime_error If the ZIP archive cannot be opened, is empty,
 *         contains invalid entries, or any error occurs during extraction.
 */
std::unordered_map<std::string, std::vector<uint8_t>> unzipData(
    const std::vector<uint8_t>& data);

/**
 * @brief Converts a string_view to a numeric type.
 *
 * This function uses std::from_chars to convert a given std::string_view into a
 * numeric value of type T. The conversion is done without allocating memory,
 * and the function returns true if the conversion succeeds, false otherwise.
 *
 * @tparam T The numeric type to convert the string_view to (e.g., int, long,
 * double).
 * @param sv The string_view representing the numeric string to be converted.
 * @param result The variable where the result of the conversion will be stored.
 * @return True if the conversion was successful, false otherwise.
 *
 * @note This function only supports arithmetic types (integers and
 * floating-point types).
 */
template <typename T>
bool stringViewToNumber(std::string_view sv, T& result) {
  static_assert(std::is_arithmetic_v<T>, "T must be an arithmetic type");

  auto [ptr, ec] = std::from_chars(sv.data(), sv.data() + sv.size(), result);
  return ec == std::errc();
}

/**
 * @brief Converts a trivially copyable value to a byte vector.
 *
 * Serializes the given value into a `std::vector<uint8_t>` by copying its
 * memory representation. Optionally converts to big-endian byte order.
 *
 * @tparam T The type of the value to convert. Must be trivially copyable.
 * @param value The value to convert to bytes.
 * @param bigEndian If true, the resulting byte vector will be in big-endian
 * order. If false (default), little-endian order is used.
 * @return A vector of bytes representing the memory of the value.
 *
 * @note The function performs a shallow copy of the memory representation.
 *       Use with caution on types with padding or platform-dependent layout.
 */
template <typename T>
std::vector<uint8_t> toBytes(T value, bool bigEndian = false) {
  static_assert(std::is_trivially_copyable<T>::value,
                "T must be trivially copyable");

  std::vector<uint8_t> bytes(sizeof(T));
  std::memcpy(bytes.data(), &value, sizeof(T));

  if (bigEndian) {
    std::reverse(bytes.begin(), bytes.end());
  }

  return bytes;
}

/**
 * @brief Reconstructs a value of type T from a byte vector.
 *
 * Deserializes a value of trivially copyable type T from a
 * `std::vector<uint8_t>` containing its memory representation. Optionally
 * interprets the byte vector as big-endian.
 *
 * @tparam T The type of the value to reconstruct. Must be trivially copyable.
 * @param bytes The byte vector representing the serialized value.
 * @param bigEndian If true, the byte vector is interpreted as big-endian.
 *                  If false (default), little-endian order is assumed.
 * @return The deserialized value of type T.
 *
 * @throws std::invalid_argument If the size of the byte vector does not match
 * sizeof(T).
 *
 * @note The function performs a shallow memory copy. Ensure that the byte
 * vector was created with a compatible layout (e.g., using `toBytes<T>`). Be
 * cautious with types containing padding or platform-specific layout.
 */
template <typename T>
T fromBytes(const std::vector<uint8_t>& bytes, bool bigEndian = false) {
  static_assert(std::is_trivially_copyable<T>::value,
                "T must be trivially copyable");

  if (bytes.size() != sizeof(T)) {
    throw std::invalid_argument(
        "Byte vector size does not match target type size");
  }

  std::vector<uint8_t> temp = bytes;

  if (bigEndian) {
    std::reverse(temp.begin(), temp.end());
  }

  T value;
  std::memcpy(&value, temp.data(), sizeof(T));
  return value;
}

/**
 * @brief Formats the given index and subindex into a parameter identifier
 * string.
 *
 * This function takes an index and a subindex and formats them into a string
 * of the form "0xINDEX:SUBINDEX". The index is formatted as a 4-digit
 * hexadecimal number, and the subindex is formatted as a 2-digit hexadecimal
 * number.
 *
 * @param index The 16-bit index value (e.g., 0x2030).
 * @param subindex The 8-bit subindex value (e.g., 0x01).
 * @return std::string The formatted parameter ID string in the format
 * "0xINDEX:SUBINDEX".
 */
inline std::string makeParameterId(int index, int subindex) {
  std::stringstream oss;
  // Format index and subindex into "0xINDEX:SUBINDEX"
  oss << "0x" << std::setw(4) << std::setfill('0') << std::hex << std::uppercase
      << index << ":" << std::setw(2) << std::setfill('0') << std::hex
      << std::uppercase << subindex;

  return oss.str();
}

/**
 * @brief Converts a vector of bytes to a space-separated hexadecimal string.
 *
 * Formats each byte in the input vector as a two-digit hexadecimal value
 * with a `0x` prefix. Bytes are separated by a single space.
 *
 * Example: a vector containing {0xAB, 0xCD} will be formatted as "0xAB 0xCD".
 *
 * @param data The vector of bytes to format.
 * @return A string containing the hexadecimal representation of the bytes.
 */
inline std::string toHex(const std::vector<uint8_t>& data) {
  std::ostringstream oss;
  oss << std::uppercase;  // Ensure hex letters are uppercase
  for (size_t i = 0; i < data.size(); ++i) {
    oss << "0x" << std::hex << std::setw(2) << std::setfill('0')
        << static_cast<int>(data[i]);
    if (i != data.size() - 1) {
      oss << " ";
    }
  }
  return oss.str();
}

/**
 * @brief Converts an integer value to its hexadecimal string representation.
 *
 * This function template formats any integral value as a hexadecimal string,
 * including a `0x` prefix, uppercase letters, and leading zeros based on the
 * size of the type.
 *
 * @tparam T An integral type (e.g., int, uint32_t).
 * @param value The integer value to convert.
 * @return A string containing the hexadecimal representation of the input.
 *
 * @note If the input type is not integral, a compile-time error will occur.
 */
template <typename T>
inline std::string toHex(T value) {
  static_assert(std::is_integral<T>::value, "toHex requires an integer type");
  constexpr size_t hexWidth = sizeof(T) * 2;

  std::ostringstream oss;
  oss << "0x" << std::hex << std::uppercase << std::setw(hexWidth)
      << std::setfill('0') << static_cast<uint64_t>(value);
  return oss.str();
}

/**
 * @brief Reads the entire contents of a binary file into a byte vector.
 *
 * Opens the file in binary mode and reads all bytes into a
 * std::vector<uint8_t>.
 *
 * @param filePath The path to the binary file to read.
 * @return A std::vector<uint8_t> containing the file's raw bytes.
 *
 * @throws std::runtime_error If the file cannot be opened.
 */
std::vector<uint8_t> readBinaryFile(const std::string& filename);

/**
 * @brief Reads the entire content of a text file into a std::string.
 *
 * @param filePath The path to the text file to be read.
 * @return A std::string containing the full content of the file.
 *
 * @throws std::runtime_error If the file cannot be opened.
 */
std::string readTextFile(const std::string& filename);

/**
 * @brief Joins a list of strings into a single string with a delimiter.
 *
 * Concatenates all elements in the given vector of strings, inserting the
 * specified delimiter between each element. If the list is empty, returns
 * an empty string.
 *
 * @param list The vector of strings to join.
 * @param delimiter The delimiter to insert between each string.
 * @return A single string composed of the input strings separated by the
 * delimiter.
 */
std::string joinStrings(const std::vector<std::string>& list,
                        const std::string& delimiter);

/**
 * @brief Formats a MAC address string to ensure each component is two digits
 * and uppercase.
 *
 * This function takes a MAC address string in formats such as "a-b-c-d-e-f" or
 * "a:b:c:d:e:f", and returns a standardized format like "0A:0B:0C:0D:0E:0F". It
 * ensures that each component is two characters long (adding leading zeros if
 * necessary), uses ':' as the delimiter, and converts all letters to uppercase.
 *
 * @param originalMacAddress The input MAC address string, using either ':' or
 * '-' as delimiter.
 * @return A formatted MAC address string with two-digit uppercase hexadecimal
 * components, separated by colons.
 */
std::string formatMacAddress(const std::string& originalMacAddress);

/**
 * @brief Extracts the IP address from a socket address string.
 *
 * Parses a string of the form "IP:port" and returns the IP portion.
 *
 * @param socketAddress A socket address in the format "IP:port".
 * @return The extracted IP address.
 * @throws std::invalid_argument If the input string does not contain a colon.
 */
std::string extractIpAddress(const std::string& socketAddress);

/**
 * @brief Extracts the port number from a socket address string.
 *
 * Parses a string of the form "IP:port" and returns the port portion as an
 * unsigned short.
 *
 * @param socketAddress A socket address in the format "IP:port".
 * @return The extracted port number.
 * @throws std::invalid_argument If the input string does not contain a valid
 * port.
 */
unsigned short extractPort(const std::string& socketAddress);

/**
 * @brief Checks whether a string ends with the given suffix.
 *
 * @param str The full string to check.
 * @param suffix The suffix to compare against.
 * @return true if str ends with suffix, false otherwise.
 */
inline bool endsWith(const std::string& str, const std::string& suffix) {
  return str.size() >= suffix.size() &&
         std::equal(suffix.rbegin(), suffix.rend(), str.rbegin());
}

/**
 * @brief Count how many strings in the list start with the given prefix.
 *
 * @param strings The vector of strings to search.
 * @param prefix The prefix to match.
 * @return Number of strings starting with the given prefix.
 */
inline size_t countStringsStartingWith(const std::vector<std::string>& strings,
                                       const std::string& prefix) {
  return std::count_if(
      strings.begin(), strings.end(), [&](const std::string& s) {
        return s.rfind(prefix, 0) == 0;  // checks if s starts with prefix
      });
}

/**
 * @brief Splits a string into substrings based on a delimiter character.
 *
 * Parses the input string, splitting it at each occurrence of the delimiter,
 * and returns a vector containing the resulting substrings in order.
 *
 * @param input The input string to split.
 * @param delimiter The character used as the delimiter to split the string.
 * @return std::vector<std::string> A vector of substrings obtained by splitting
 * the input.
 */
inline std::vector<std::string> split(const std::string& input,
                                      char delimiter) {
  std::vector<std::string> result;
  std::istringstream stream(input);
  std::string token;

  while (std::getline(stream, token, delimiter)) {
    result.push_back(token);
  }

  return result;
}

/**
 * @brief A simple thread-safe FIFO queue.
 *
 * Provides push, blocking pop, and non-blocking pop operations
 * with thread safety using a mutex and condition variable.
 *
 * @tparam T Type of elements stored in the queue.
 */
template <typename T>
class ThreadSafeQueue {
 private:
  /// @brief Underlying standard queue holding the data.
  std::queue<T> queue_;

  /// @brief Mutex to protect access to the queue.
  mutable std::mutex mtx_;

  /// @brief Condition variable to block waiting pops until data is available.
  std::condition_variable cv_;

 public:
  /**
   * @brief Construct a new empty ThreadSafeQueue.
   */
  ThreadSafeQueue() = default;

  /**
   * @brief Deleted copy constructor.
   */
  ThreadSafeQueue(const ThreadSafeQueue&) = delete;

  /**
   * @brief Deleted copy assignment operator.
   */
  ThreadSafeQueue& operator=(const ThreadSafeQueue&) = delete;

  /**
   * @brief Push a new value into the queue.
   *
   * @param value Value to push.
   */
  void push(T value) {
    {
      std::lock_guard<std::mutex> lock(mtx_);
      queue_.push(std::move(value));
    }
    cv_.notify_one();
  }

  /**
   * @brief Try to pop a value from the queue without blocking.
   *
   * @return An optional containing the popped value if available,
   *         or std::nullopt if the queue was empty.
   */
  std::optional<T> try_pop() {
    std::lock_guard<std::mutex> lock(mtx_);
    if (queue_.empty()) return std::nullopt;
    T value = std::move(queue_.front());
    queue_.pop();
    return value;
  }

  /**
   * @brief Wait for an item to be available and pop it.
   *
   * Blocks until an item is available.
   *
   * @return The popped value.
   */
  T wait_and_pop() {
    std::unique_lock<std::mutex> lock(mtx_);
    cv_.wait(lock, [this] { return !queue_.empty(); });
    T value = std::move(queue_.front());
    queue_.pop();
    return value;
  }

  /**
   * @brief Check whether the queue is empty.
   *
   * @return true if the queue is empty, false otherwise.
   */
  bool empty() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return queue_.empty();
  }

  /**
   * @brief Get the current number of items in the queue.
   *
   * Note: size may change immediately after this call in concurrent use.
   *
   * @return The number of items currently in the queue.
   */
  size_t size() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return queue_.size();
  }
};

/**
 * @brief Gets the current system time in milliseconds since the Unix epoch.
 *
 * Uses std::chrono to obtain the current time point from the system clock,
 * converts it to milliseconds duration since epoch, and returns the count
 * as a 64-bit integer.
 *
 * @return int64_t Current time in milliseconds since 1970-01-01 00:00:00 UTC.
 */
inline int64_t currentTimeMillis() {
  // Get current time point from system clock
  auto now = std::chrono::system_clock::now();

  // Convert to duration since epoch in milliseconds
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
      now.time_since_epoch());

  // Return the count as an integer (int64_t)
  return duration.count();
}

/**
 * Maps a value from one range to another using linear interpolation.
 *
 * @tparam T Type of input and output values (int, float, double, etc.)
 * @param value Input value to be mapped
 * @param inMin Lower bound of input range
 * @param inMax Upper bound of input range
 * @param outMin Lower bound of output range
 * @param outMax Upper bound of output range
 * @return Mapped value in the output range
 */
template <typename T>
inline constexpr T linearScale(T value, T inMin, T inMax, T outMin, T outMax) {
  static_assert(std::is_arithmetic<T>::value,
                "linearScale requires a numeric type");
  return outMin + (value - inMin) * (outMax - outMin) / (inMax - inMin);
}

}  // namespace mm::core::util

/**
 * @brief Specialization of std::hash for std::pair<uint16_t, uint8_t>.
 *
 * This specialization allows `std::unordered_map` to use `std::pair<uint16_t,
 * uint8_t>` as a key type. It combines the hash values of the `first` and
 * `second` elements of the pair to create a unique hash value.
 *
 * The hash is computed by XORing the hash values of the two elements, with the
 * second element's hash shifted to reduce collisions.
 */
namespace std {
template <>
struct hash<std::pair<uint16_t, uint8_t>> {
  /**
   * @brief Computes a hash for a given std::pair<uint16_t, uint8_t>.
   *
   * This function combines the hash values of the `first` and `second` members
   * of the pair to generate a hash suitable for use in unordered containers
   * like `std::unordered_map`.
   *
   * @param p The pair to be hashed.
   * @return The computed hash value.
   */
  size_t operator()(const std::pair<uint16_t, uint8_t>& p) const {
    return hash<uint16_t>()(p.first) ^ (hash<uint8_t>()(p.second) << 1);
  }
};

}  // namespace std
