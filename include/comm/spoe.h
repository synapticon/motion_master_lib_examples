#pragma once

#include <atomic>
#include <boost/asio.hpp>
#include <chrono>
#include <nlohmann/json.hpp>
#include <string>

#include "base.h"
#include "core/util.h"

namespace mm::comm::spoe {

/**
 * @brief Enumeration of SPoE message types used in the protocol.
 *
 * Each message type corresponds to a specific operation or request within the
 * SPoE communication protocol.
 *
 * @note This is specified in the "Ethernet interface definition" document.
 */
enum class SpoeMessageType : uint8_t {
  SDO_READ = 0x01,   ///< Read a Service Data Object (SDO) value.
  SDO_WRITE = 0x02,  ///< Write a value to a Service Data Object (SDO).
  PDO_RXTX_FRAME =
      0x03,  ///< Transmit or receive a Process Data Object (PDO) frame.
  PDO_CONTROL = 0x04,      ///< Control or configure PDO behavior.
  SDO_BATCH_READ = 0x06,   ///< Read multiple SDOs in a single request.
  FIRMWARE_UPDATE = 0x0B,  ///< Perform firmware update operation.
  FILE_READ = 0x0C,        ///< Read a file from the device.
  FILE_WRITE = 0x0D,       ///< Write a file to the device.
  STATE_CONTROL =
      0x0E,           ///< Control the state of the device (e.g., INIT, PREOP).
  STATE_READ = 0x0F,  ///< Read the current state of the device.
  //   PARAM_INDEXES_LIST = 0x10,  ///< Request a list of parameter indexes.
  PARAM_LIST = 0x13,  ///< Request a full list of parameters.
  SERVER_INFO = 0x20  ///< Request information about the server or device.
};

/**
 * @brief Status codes representing the state of an SPoE request message.
 *
 * This enum indicates whether a message is complete, part of a segmented
 * sequence, or contains an error.
 */
enum class SpoeMessageRequestStatus : uint8_t {
  OK = 0x00,      ///< Message is complete and valid.
  FIRST = 0x80,   ///< First segment of a multi-part message.
  MIDDLE = 0xC0,  ///< Middle segment of a multi-part message.
  LAST = 0x40,    ///< Last segment of a multi-part message.
};

/**
 * @brief Error status codes returned when reading the parameter list via
 * SPoE.
 */
enum class SpoeMessageParamListErrorStatus : uint8_t {
  SUCCESS_ACK = 0x58,  ///< Success Acknowledgement

  ERR = 0x63,
  BUSY_INDICATION = 0x28,
};

/**
 * @brief Status codes returned during SDO (Service Data Object) read and write
 * operations.
 *
 * These codes represent the result of accessing parameters on the SPoE device.
 */
enum class SpoeMessageSdoStatus : uint16_t {
  NO_ERR = 0x0000,  ///< Success Acknowledgement
  GENERIC = 0x0001,
  NOT_FOUND = 0x0002,
  READ_ONLY = 0x0003,
  WRITE_ONLY = 0x0004,
  WRONG_TYPE = 0x0005,
  INVALID_LIST = 0x0006,
  INSUFFICIENT_BUFFER = 0x0007,
  VALUEINFO_UNAVAILABLE = 0x0008,
  UNKNOWN_OR_UNSUPPORTED = 0x0009,
  LOCAL_TRANSFER = 0x000A,
  UNSUPPORTED_ACCESS = 0x000B,
  SI0_NOT_ZERO = 0x000C,
  SUB_NOT_FOUND = 0x000D,
};

// enum class SpoeMessageSdoStatus : uint16_t {
//   NO_ERROR = 0x0000,
// };

/**
 * @brief Error status codes returned when reading or writing a file via SPoE
 * message.
 */
enum class SpoeMessageFileErrorStatus : uint8_t {
  SUCCESS_ACK = 0x58,  ///< Success Acknowledgement

  UNDEFINED = 0x00,
  NOT_FOUND = 0x01,
  ACCESS_DENIED = 0x02,
  STORAGE_FULL = 0x03,
  ILLEGAL_REQ = 0x04,
  PACKET_NUMBER = 0x05,
  ALREADY_EXISTS = 0x06,
  NO_USER = 0x07,
  BOOTSTRAP_ONLY = 0x08,
  NOT_BOOTSTRAP = 0x09,
  NO_RIGHTS = 0x0A,
  PROGRAM_ERROR = 0x0B,
  BUSY = 0x0C,
  FILENAME_LEN = 0x0D,
  TIMEOUT = 0x0E,
  FLASH_BUSY = 0x28,
  COMMUNICATION_BRIDGE_ERROR = 0x63
};

/**
 * @brief Packet status codes for SPoE messages.
 *
 * This enum defines various status flags used to indicate the completeness and
 * validity of a SPoE message, including support for segmented transfers.
 */
enum class SpoeMessagePacketStatus : uint8_t {
  OK = 0x00,      ///< Message is complete and valid.
  FIRST = 0x80,   ///< First segment of a multi-part message.
  MIDDLE = 0xC0,  ///< Middle segment of a multi-part message.
  LAST = 0x40,    ///< Last segment of a multi-part message.
};

/**
 * @enum PdoMode
 * @brief Defines the modes for Process Data Object (PDO) communication over
 * SPoE.
 *
 * @details
 * - NONE: No process or cyclic data exchange is available.
 * - MONITOR: The device will only send RxPDO values; requests in messages will
 * be ignored.
 * - CONTROL: Both RxPDO and TxPDO data exchange is enabled.
 */
enum class PdoMode : uint8_t {
  NONE = 0,  ///< No process or cyclic data exchange available.
  MONITOR =
      1,  ///< Device sends RxPDO values only; incoming requests are ignored.
  CONTROL = 2  ///< Both RxPDO and TxPDO data exchange enabled.
};

/**
 * @struct SpoeMessage
 * @brief Structure representing a parsed SPoE response message.
 *
 * This structure is used to hold the parsed information from a SPoE response
 * message, including its type, sequence ID, status, size, and payload data.
 */
struct Message {
  /** The size of the message header. */
  static constexpr size_t kHeaderSize = 7;

  /** The buffer size used for communication, excluding the message header. */
  static constexpr size_t kBufferSize = 1500 - kHeaderSize;

  /**
   * @brief The type of the response message.
   * @details This is a one-byte field representing the type of the response.
   */
  SpoeMessageType type;

  /**
   * @brief The sequence ID of the response message.
   * @details This is a two-byte field that contains the unique sequence ID
   *          of the response message.
   */
  uint16_t id;

  /**
   * @brief The status of the response message.
   * @details This is a two-byte field that contains the status code of the
   *          response message.
   */
  uint16_t status;

  /**
   * @brief The size of the buffer in the response message.
   * @details This is a two-byte field that specifies the size of the data
   *          buffer in the response message.
   */
  uint16_t size;

  /**
   * @brief The payload data of the response message.
   * @details This vector contains the raw data of the response message,
   *          excluding the header fields.
   */
  std::vector<uint8_t> data;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Message, type, id, status, size, data);

/**
 * @brief Parses a raw SPoE message buffer into a structured SpoeMessage.
 *
 * This function interprets the first 7 bytes of the buffer as the message
 * header, extracting the type, sequence ID, status, and payload size. If the
 * size field is greater than zero, it also extracts the payload data.
 *
 * @param buffer The raw byte buffer containing the SPoE message.
 *               Must be at least 7 bytes long to contain the header.
 * @return A structured representation of the parsed SPoE message.
 *
 * @throws std::runtime_error If the buffer is smaller than the required header
 * size.
 *
 * @see SpoeMessage, SpoeMessageType
 */
Message parseMessage(const std::vector<uint8_t>& buffer);

/**
 * @brief Serializes an SpoeMessage object into a byte buffer.
 *
 * This function converts the provided SpoeMessage into a byte array,
 * which can be sent over a network. The SpoeMessage type, ID, status,
 * size, and any associated data are serialized into the buffer in a specific
 * format expected by the receiving device.
 *
 * @param message The SpoeMessage object to serialize.
 * @return A std::vector<uint8_t> containing the serialized byte buffer.
 *
 * @see SpoeMessage, SpoeMessageType
 */
std::vector<uint8_t> serializeSpoeMessage(const Message& message);

/**
 * @struct ServerInfo
 * @brief Contains information about the server's protocol version and
 * monitoring mode.
 */
struct ServerInfo {
  /**
   * @brief Version of the communication protocol used by the server.
   */
  uint16_t protocolVersion;

  /**
   * @brief Indicates the monitoring mode configuration.
   */
  uint8_t monitoringMode;
};

/**
 * @brief Concrete refresher that uploads parameters from a device.
 *
 * This class derives from `mm::comm::DeviceParameterRefresher` and implements
 * the `refresh()` method to handle parameter uploads in bulk over SPoE.
 *
 * The `refresh()` method is called by the base class's worker thread.
 * It receives a snapshot of the current parameters and performs a single
 * bulk upload operation, which is more efficient for SPoE-based devices than
 * uploading parameters one by one.
 */
class DeviceParameterRefresher
    : public mm::comm::base::DeviceParameterRefresher {
 public:
  // Forward constructor to base class constructor
  DeviceParameterRefresher(mm::comm::base::Device& device,
                           std::chrono::milliseconds interval);

 protected:
  /**
   * @brief Rereads the specified device parameters in a single message
   * exchange.
   *
   * This method is automatically called by the base class in a background
   * thread. The implementation should handle uploading all parameters
   * using the SPoE-specific protocol (readSdoBatch).
   *
   * The refresh operation is performed only if the device is in a state that
   * permits parameter reading—namely:
   * - PRE-OPERATIONAL (2)
   * - SAFE-OPERATIONAL (4)
   * - OPERATIONAL (8)
   *
   * @param addresses A snapshot of the current parameter addresses (index and
   * subindex pairs) to be uploaded.
   */
  void refresh(const std::vector<mm::comm::base::Parameter::Address>& addresses)
      override;
};

/**
 * @class Device
 * @brief Handles TCP communication with SOMANET devices over SPoE.
 *
 * This class provides methods for connecting to a remote server, sending
 * messages, and receiving responses over a TCP connection using Boost.Asio.
 * It manages the underlying socket, connection, and I/O operations required
 * for client-server communication.
 */
class Device : public mm::comm::base::Device {
 public:
  /**
   * @brief Constructs a SpoeDevice object with the specified IP address,
   * port, and optionally position.
   *
   * Initializes the client with the target server's IP address and port
   * number.
   *
   * @param ip The IP address of the server to connect to.
   * @param port The port number to use for the connection.
   * @param position Device position in the network chain.
   * @param refresherInterval The interval at which the device parameters are
   * refreshed. Defaults to 3000 milliseconds.
   */
  Device(const std::string& ip, uint16_t port, uint16_t position = 0,
         std::chrono::milliseconds refresherInterval =
             std::chrono::milliseconds(3000));

  /**
   * @brief Destructor for the Device class.
   *
   * Closes the socket and disconnects from the server if the socket is open.
   */
  ~Device();

  /**
   * @brief Returns the socket address as a string in the format "IP:port".
   *
   * This function retrieves the IP address and port number from the configured
   * endpoint and formats them as a single string (e.g., "192.168.1.10:8080").
   *
   * @return A string representation of the socket address.
   */
  std::string getSocketAddress() const;

  /**
   * @brief Increments the sequence ID atomically and wraps it around at the
   *        maximum value.
   *
   * This function atomically increments the `seq_id` (a 16-bit unsigned
   * integer). When the value reaches the maximum value of `uint16_t` (0xFFFF),
   * it wraps around to 0. It ensures thread safety when used in multithreaded
   * environments.
   *
   * @return The updated sequence ID after incrementing, wrapped around if
   *         necessary.
   */
  uint16_t incrementSeqId();

  /**
   * @brief Attempts to connect the SPoE device to the specified endpoint.
   *
   * This function starts an asynchronous connection attempt and waits for it
   * to complete within the given expiry time. If the connection attempt
   * exceeds the timeout, it is canceled.
   *
   * @param expiryTime The maximum duration to wait for the connection attempt.
   * @return true if the connection was successful, false otherwise.
   */
  bool connect(std::chrono::seconds expiryTime = std::chrono::seconds(3));

  /**
   * @brief Checks if the SPoE socket is currently open.
   *
   * Returns true if the underlying socket is open, indicating an active
   * or initialized connection. Returns false otherwise.
   *
   * @return true if the socket is open, false otherwise.
   */
  bool isConnected();

  /**
   * @brief Gracefully disconnects the SPoE device.
   *
   * Cancels any outstanding asynchronous operations, performs a TCP shutdown
   * for both send and receive directions, and closes the underlying socket.
   * If the socket is already closed, this function does nothing.
   *
   * @return true if the socket was closed successfully or was already closed,
   * false if an error occurred during shutdown or close.
   *
   * @note This method safely handles any exceptions and logs all socket errors.
   * Use this before attempting to reconnect with the same socket instance.
   */
  bool disconnect();

  /**
   * @brief Exchanges a message with a remote server and waits for a response
   * with a timeout.
   *
   * This method serializes the request message, sends it to the server using an
   * asynchronous write operation, and waits for a response with a timeout. If
   * the operation takes longer than the specified expiry time, the operation
   * will be canceled and an error will be thrown.
   *
   * @param requestMessage The message to send to the remote server.
   * @param expiryTime The duration to wait for the socket operation
   * (read/write) before timing out.
   * @return SpoeMessage The parsed response message from the server.
   *
   * @throws std::runtime_error If the write or read operation fails, or if the
   * operation times out.
   */
  Message exchangeWithTimeout(
      const Message& request,
      const std::chrono::steady_clock::duration expiryTime);

  uint16_t getPosition() const override;

  /**
   * @brief Sends a request to read the state of the device and returns the
   * state value.
   *
   * This function sends a request to the device using the
   * `SpoeMessageType::STATE_READ` message type. It increments the sequence
   * ID, constructs the request message, serializes it, and sends it to the
   * device. After sending the request, the function waits for the response,
   * parses the response message, and returns the state value as a `uint8_t`.
   *
   * The returned state corresponds to the EtherCAT state machine states,
   * where:
   * - INIT: 1
   * - PREOP: 2
   * - SAFEOP: 4
   * - OP: 8
   * - BOOT: 3
   *
   * @param refresh If true, forces a refresh of the device state by sending a
   *                `STATE_READ` request even if the state is already cached.
   * @param expiryTime The duration to wait for the socket operation
   * (read/write) before timing out. Defaults to 5000 milliseconds.
   * @return The state value of the device as a uint8_t.
   *
   * @throws boost::system::system_error if sending or receiving fails.
   * @throws std::runtime_error if the response buffer is too small to parse.
   */
  uint8_t getState(bool refresh = false,
                   const std::chrono::steady_clock::duration expiryTime =
                       std::chrono::milliseconds(5000)) override;

  /**
   * @brief Sends a state control command to the SPoE device and checks the
   * response.
   *
   * This method constructs a `STATE_CONTROL` type `SpoeMessage` with the
   * given state, sends it over the socket, and waits for a response. It then
   * parses the received message and returns `true` if the operation was
   * acknowledged with `OK` status.
   *
   * @param state The new state to set on the remote device.
   * @param expiryTime The duration to wait for the socket operation
   * (read/write) before timing out. Defaults to 5000 milliseconds.
   * @return `true` if the response status is `OK`, `false` otherwise.
   *
   * @throws boost::system::system_error if sending or receiving fails.
   * @throws std::runtime_error if the response buffer is too small to parse.
   */
  bool setState(uint8_t state,
                const std::chrono::steady_clock::duration expiryTime =
                    std::chrono::milliseconds(5000)) override;

  /**
   * @brief Reads a file from the device using segmented SPoE messages.
   *
   * This function sends a `FILE_READ` request to the device and reads the file
   * in segments until the last segment is received. It validates the content
   * and returns either the file data or an appropriate error.
   *
   * @param filename The name of the file to read from the device.
   * @param expiryTime Timeout duration for communication with the device.
   * @return std::variant<std::vector<uint8_t>, mm::comm::DeviceFileError>
   *         - On success: a vector containing the file content.
   *         - On failure: a DeviceFileError indicating the error type.
   *
   * Possible errors:
   * - DeviceFileError::NullTerminator if the file starts with a null byte.
   * - DeviceFileError::EmptyContent if the content is empty and the file
   *   is not found in the file list.
   */

  /**
   * @brief Reads the contents of a file over SPoE from the device.
   *
   * This function requests the file in multiple segments by sending SPoE
   * messages with increasing sequence IDs. It accumulates the received data
   * until the entire file is read.
   *
   * The request starts with the filename in the first message, then subsequent
   * messages request the next segments until the last segment is received.
   *
   * Error handling:
   * - If the device responds with an error status, a DeviceResponseException is
   *   thrown containing the device position and error code.
   * - If the received content is empty, the function checks if the file exists
   *   on the device by calling `readFileList(true)`.
   *     - If the file is found, the empty content is considered valid and
   *       returned.
   *     - If not found, a DeviceResponseException is thrown indicating the file
   *       was not found.
   *
   * @param filename The name of the file to read.
   * @param expiryTime Timeout duration for each socket read/write operation.
   *                   Defaults to 5000 milliseconds.
   * @return A vector of bytes containing the file content. May be empty if the
   *         file exists but contains no data.
   *
   * @throws DeviceResponseException When the device returns an error status, or
   *         the file does not exist on the device.
   * @throws boost::system::system_error If socket communication fails during
   *         the request or response.
   */
  std::vector<uint8_t> readFile(
      const std::string& filename,
      const std::chrono::steady_clock::duration expiryTime =
          std::chrono::milliseconds(5000)) override;

  std::vector<std::string> readFileList(
      const bool stripSizeSuffix = true,
      const std::chrono::steady_clock::duration expiryTime =
          std::chrono::milliseconds(5000)) override;

  /**
   * @brief Remove a file from the SPoE device.
   *
   * This function removes a file on the device by reading a special file named
   * "fs-remove=&lt;filename&gt;". The device signals successful removal by
   * returning a response message starting with a specific confirmation prefix.
   *
   * If the response does not contain the expected prefix or if the operation
   * fails, a DeviceResponseException is thrown.
   *
   * @param filename Name of the file to remove.
   * @param expiryTime Maximum duration to wait for the response before timing
   * out. Defaults to 5000 milliseconds.
   *
   * @throws mm::comm::DeviceResponseException if the removal fails or the
   * device returns an unexpected response.
   */
  void removeFile(const std::string& filename,
                  const std::chrono::steady_clock::duration expiryTime =
                      std::chrono::milliseconds(5000)) override;

  /**
   * @brief Updates the firmware on this SPoE device.
   *
   * Puts the device into the INIT and BOOT states, writes the provided firmware
   * package (optionally skipping specified files), triggers the firmware update
   * process on the device, and then disconnects and reconnects the device.
   *
   * @param data The firmware package as a vector of bytes (typically a zip
   * archive).
   * @param skipFiles A list of file names to skip when writing the firmware to
   * the device.
   * @param progressCallback Optional callback to report update progress
   * (accepts a percentage [0–100]).
   * @param expiryTime The duration to wait for the socket operation
   * (read/write) before timing out. Defaults to 60000 milliseconds.
   *
   * @throws std::runtime_error If triggering the firmware update fails.
   *
   * @note This function disconnects and reconnects the device automatically
   * after the update.
   */
  void updateFirmware(
      const std::vector<std::uint8_t>& data,
      const std::vector<std::string>& skipFiles = {"SOMANET_CiA_402.xml.zip",
                                                   "stack_image.svg.zip"},
      std::function<void(uint8_t, std::string)> progressCallback = nullptr,
      const std::chrono::steady_clock::duration expiryTime =
          std::chrono::milliseconds(60000)) override;

  /**
   * @brief Write a file to the SPoE device in segments.
   *
   * This function writes the specified file to the device by first sending the
   * filename and then transmitting the file data in segments. The transfer uses
   * a maximum segment size for each write operation and ensures that each
   * segment is acknowledged by the device before proceeding.
   *
   * If any part of the transfer fails, a DeviceResponseException is thrown with
   * details about the error.
   *
   * @param filename Name of the file to be written on the device.
   * @param data File contents to write.
   * @param expiryTime The duration to wait for the socket operation
   * (read/write) before timing out. Defaults to 5000 milliseconds.
   *
   * @throws mm::comm::DeviceResponseException if the device responds with an
   * error during any segment write.
   * @throws boost::system::system_error If socket communication fails during
   * the request or response.
   */
  void writeFile(const std::string& filename, const std::vector<uint8_t>& data,
                 const std::chrono::steady_clock::duration expiryTime =
                     std::chrono::milliseconds(5000)) override;

  /**
   * @brief Sends a firmware update request to the connected Integro device.
   *
   * Constructs and transmits a firmware update request using the custom SPoE
   * communication protocol. Waits for the device's response and verifies
   * whether the request was successfully acknowledged.
   *
   * This request must be issued after uploading one or both of the following
   * files:
   * - **app_firmware.bin**: Contains the SoC firmware.
   * - **com_firmware.bin**: Contains the communication chip firmware.
   *
   * @param expiryTime The duration to wait for the socket operation
   * (read/write) before timing out. Defaults to 2000 milliseconds.
   * @return true if the device responds with an OK status; false otherwise.
   */
  bool triggerFirmwareUpdate(const std::chrono::steady_clock::duration
                                 expiryTime = std::chrono::milliseconds(2000));

  /**
   * @brief Retrieves the parameter list from the device.
   *
   * Sends a `PARAM_LIST` request to the device and reads the response in
   * multiple segments using a sequence ID for each segment. The response is
   * validated after each exchange. If the status is not
   * `SpoeMessageParamListErrorStatus::SUCCESS_ACK`, a
   * `DeviceResponseException` is thrown.
   *
   * Parses each 68-byte entry in the response into a `mm::comm::Parameter`
   * object, extracting index, subindex, data type, code, flags, bit length, and
   * name. Optionally performs an SDO batch read to update parameter values if
   * @p readValues is true.
   *
   * Uses a lock guard with a recursive mutex to ensure thread safety.
   *
   * @param readValues If true, performs an additional SDO batch read to
   * retrieve current parameter values from the device.
   * @param expiryTime The duration to wait for the socket operation
   * (read/write) before timing out. Defaults to 1000 milliseconds.
   * @return A vector of parsed `mm::comm::Parameter` objects containing the
   * parameters retrieved from the device.
   *
   * @throws boost::system::system_error if sending or receiving fails.
   * @throws DeviceResponseException if any response status indicates an error.
   */
  std::vector<mm::comm::base::Parameter> getParametersFromDevice(
      bool readValues = false,
      const std::chrono::steady_clock::duration expiryTime =
          std::chrono::milliseconds(1000));

  /**
   * @brief Reads an SDO (Service Data Object) from the device.
   *
   * Constructs and sends an SDO read request for the specified index and
   * subindex, waits for a response within the given expiry time, and returns
   * the received data. If the device responds with an error status, an
   * DeviceResponseException is thrown.
   *
   * @param index The index of the SDO to read.
   * @param subindex The subindex of the SDO to read.
   * @param expiryTime The duration to wait for the socket operation
   * (read/write) before timing out. Defaults to 1000 milliseconds.
   * @return A vector containing the data from the SDO response.
   *         An empty vector is returned if the operation fails.
   *
   * @throws boost::system::system_error if sending or receiving fails.
   * @throws DeviceResponseException If the response contains an unexpected
   * status code.
   */
  std::vector<uint8_t> readSdo(
      uint16_t index, uint8_t subindex,
      const std::chrono::steady_clock::duration expiryTime =
          std::chrono::milliseconds(1000));

  /**
   * @brief Reads a batch of SDOs from the device in a single request.
   *
   * Constructs a batch read message containing the specified SDO indices and
   * subindices, sends it to the device, and waits for a response within the
   * given timeout. The response buffer contains the raw SDO values in sequence,
   * each prefixed with a 2-byte little-endian length.
   *
   * The method validates the response and splits it into individual SDO value
   * blocks matching the requested addresses.
   *
   * @param addresses A vector of (index, subindex) pairs representing the SDOs
   * to read.
   * @param expiryTime The duration to wait for the socket operation
   * (read/write) before timing out. Defaults to 1000 milliseconds.
   * @return A vector of SDO value blocks. Each entry holds the raw bytes for
   * one SDO value, in the same order as the input `addresses`.
   *
   * The returned buffer layout is:
   *   [Len1 (2 bytes)] [Data1 (Len1 bytes)] [Len2 (2 bytes)] [Data2 (Len2
   * bytes)] ...
   *
   * @throws boost::system::system_error if sending or receiving fails.
   * @throws DeviceResponseException If the response indicates an unexpected
   * status code.
   * @throws std::runtime_error If the response data format is invalid or
   * incomplete.
   */
  std::vector<std::vector<uint8_t>> readSdoBatch(
      const std::vector<mm::comm::base::Parameter::Address>& addresses,
      const std::chrono::steady_clock::duration expiryTime =
          std::chrono::milliseconds(1000));

  /**
   * @brief Reads a batch of SDOs for the specified parameters in a single
   * request.
   *
   * Converts the given list of parameters into (index, subindex) address pairs
   * and performs a single batch read using the SPoE protocol.
   *
   * @param parameters List of parameters to read.
   * @param expiryTime Timeout duration for the read operation. Defaults to 1000
   * ms.
   * @return A vector of raw data buffers, one per parameter read.
   */
  std::vector<std::vector<uint8_t>> readSdoBatch(
      const std::vector<mm::comm::base::Parameter>& parameters,
      const std::chrono::steady_clock::duration expiryTime =
          std::chrono::milliseconds(1000));

  /**
   * @brief Reads the specified parameters in batches and updates their values.
   *
   * Splits the given list of parameters into batches of the specified size,
   * performs a batch SDO read for each batch, and updates each original
   * parameter with the retrieved value.
   *
   * @param parameters The parameters to read and update.
   * @param batchSize The maximum number of parameters to read per batch.
   * @param expiryTime The timeout for each batch read operation.
   */
  void readSdoBatchAndUpdateParameters(
      std::vector<mm::comm::base::Parameter>& parameters,
      std::size_t batchSize = 50,
      std::chrono::steady_clock::duration expiryTime =
          std::chrono::milliseconds(1000));

  /**
   * @brief Writes a Service Data Object (SDO) to the device.
   *
   * Constructs and sends an SDO write request for the specified index and
   * subindex, including the provided value bytes. Waits for a response within
   * the given expiry time. If the device responds with an error status, an
   * DeviceResponseException is thrown.
   *
   * @param index The index of the SDO to write.
   * @param subindex The subindex of the SDO to write.
   * @param value A vector containing the data to be written to the SDO.
   * @param expiryTime The duration to wait for the socket operation
   * (read/write) before timing out. Defaults to 1000 milliseconds.
   * @return `true` if the write operation was successful, `false` otherwise.
   *
   * @throws boost::system::system_error if sending or receiving fails.
   * @throws DeviceResponseException If the response indicates an unexpected
   * status code.
   */
  bool writeSdo(uint16_t index, uint8_t subindex,
                const std::vector<uint8_t>& data,
                const std::chrono::steady_clock::duration expiryTime =
                    std::chrono::milliseconds(1000));

  /**
   * @brief Loads parameters from the device and stores them locally.
   *
   * This function retrieves a list of parameters using the `getParameters`
   * method. It then stores each parameter in a map for later use, keyed by
   * a pair consisting of the parameter's index and subindex.
   *
   * @param readValues If true, the values of the parameters are read from the
   * device; otherwise, only the parameter metadata is retrieved.
   * @param expiryTime The duration to wait for the socket operation
   * (read/write) before timing out. Defaults to 9000 milliseconds.
   * @return The number of loaded parameters.
   */
  size_t loadParameters(bool readValues = false,
                        const std::chrono::steady_clock::duration expiryTime =
                            std::chrono::milliseconds(9000)) override;

  void clearParameters() override;

  /**
   * @brief Get a vector of references to parameters.
   *
   * This function returns a vector containing reference wrappers to the
   * parameters stored internally in `parametersMap_`. Each element in the
   * returned vector references a parameter in the map, allowing for efficient,
   * non-owning access.
   *
   * @return std::vector<std::reference_wrapper<mm::comm::Parameter>>
   *   Vector of references to parameters.
   */
  std::vector<std::reference_wrapper<mm::comm::base::Parameter>> parameters()
      override;

  mm::comm::base::Parameter& findParameter(uint16_t index,
                                           uint8_t subindex) override;

  mm::comm::base::Parameter& upload(
      const uint16_t index, const uint8_t subindex,
      const std::chrono::steady_clock::duration expiryTime =
          std::chrono::milliseconds(3000)) override;

  /**
   * @brief Uploads a parameter from the device and returns its value as the
   * specified type.
   *
   * This templated overload uploads a parameter using the given index and
   * subindex, updates the corresponding parameter in the local store, and
   * returns its value as type `T`.
   *
   * @tparam T The type to extract from the uploaded parameter value.
   * @param index The 16-bit index of the parameter in the object dictionary.
   * @param subindex The 8-bit subindex of the parameter.
   * @param expiryTime The maximum duration to wait for the SDO upload
   * operation. Defaults to 5000 milliseconds.
   * @return The uploaded parameter value cast to type `T`.
   *
   * @throws std::runtime_error If the SDO upload fails or the value cannot be
   * extracted as type `T`.
   * @throws std::bad_variant_access If the internal variant does not hold the
   * requested type `T`.
   */
  template <typename T>
  T upload(uint16_t index, uint8_t subindex,
           const std::chrono::steady_clock::duration expiryTime =
               std::chrono::milliseconds(5000)) {
    mm::comm::base::Parameter& parameter = upload(index, subindex, expiryTime);
    return parameter.getValue<T>();
  }

  void download(uint16_t index, uint8_t subindex,
                const std::chrono::steady_clock::duration expiryTime =
                    std::chrono::milliseconds(5000)) override;

  void download(uint16_t index, uint8_t subindex,
                const mm::comm::base::ParameterValue& value,
                const std::chrono::steady_clock::duration expiryTime =
                    std::chrono::milliseconds(5000)) override;

  /**
   * @brief Sets and downloads a parameter value to the device using the
   * specified type.
   *
   * This templated overload wraps the given value in a
   * `mm::comm::ParameterValue`, updates the local parameter store, and performs
   * an SDO download to transfer the data to the device.
   *
   * @tparam T The type of the value to set and download.
   * @param index The 16-bit index of the parameter in the object dictionary.
   * @param subindex The 8-bit subindex of the parameter.
   * @param value The new parameter value to set and download.
   * @param expiryTime The maximum duration to wait for the SDO download
   * operation. Defaults to 5000 milliseconds.
   *
   * @throws std::runtime_error If the parameter data is empty after setting the
   * value, or if the SDO download operation fails.
   */
  template <typename T>
  void download(uint16_t index, uint8_t subindex, const T& value,
                const std::chrono::steady_clock::duration expiryTime =
                    std::chrono::milliseconds(5000)) {
    download(index, subindex, mm::comm::base::ParameterValue(value));
  }

  /**
   * @brief Retrieves the server information from the connected SPoE device.
   *
   * Sends a SERVER_INFO request message, waits for the response within the
   * specified timeout, and extracts the protocol version from the response. The
   * monitoring mode is currently set to 0 by default.
   *
   * @param expiryTime The maximum duration to wait for a response. Defaults to
   * 3000 milliseconds.
   * @return ServerInfo Struct containing the protocol version and monitoring
   * mode.
   *
   * @throws DeviceResponseException If the response indicates an unexpected
   * status code.
   */
  ServerInfo getServerInfo(const std::chrono::steady_clock::duration
                               expiryTime = std::chrono::milliseconds(3000));

  bool setPdoMode(PdoMode mode,
                  const std::chrono::steady_clock::duration expiryTime =
                      std::chrono::milliseconds(3000));

  /**
   * @brief Exchanges process data (PDO) with the device.
   *
   * Constructs and sends a PDO_RXTX_FRAME message containing the specified
   * process data payload, then waits for the device to respond within the given
   * expiry time. If the device responds with an unexpected status code, an
   * DeviceResponseException is thrown.
   *
   * @param data The process data to transmit to the device.
   * @param expiryTime The maximum duration to wait for the response.
   * @return A vector of bytes containing the process data returned by the
   * device.
   *
   * @throws DeviceResponseException If the response indicates an unexpected
   * status code.
   */

  std::vector<uint8_t> sendAndReceiveProcessData(
      const std::vector<uint8_t>& data,
      const std::chrono::steady_clock::duration expiryTime =
          std::chrono::milliseconds(1000));

  /**
   * @brief Perform asynchronous process data exchange with the SPoE device.
   *
   * Prepares the receive (RX) process data buffer according to the RxPDO
   * mapping, sends it, and waits for the response. The received data is then
   * split into fixed-size TX PDO frames (in bytes) and pushed to the transmit
   * (TX) PDO queue. If the TX PDO queue exceeds a defined size, the oldest
   * entries are removed to keep the queue manageable.
   *
   * Any exceptions thrown during send or receive are caught and stored for
   * later handling. The sending flag is always cleared to indicate that the
   * operation has finished.
   *
   * @note The received TxPDO data may contain one or more fixed-size blocks
   * concatenated together (for example, multiple blocks combined in a single
   * TCP frame). Each block contains multiple PDO entries of varying lengths.
   * This function splits the data into individual blocks and pushes them to the
   * TX PDO queue.
   */
  void performProcessDataExchangeAsync();

  /**
   * @brief Exchanges process data and updates device parameters.
   *
   * This function checks whether a process data exchange is already running,
   * and if not, starts it asynchronously. It then checks for any exceptions
   * thrown during the previous exchange and rethrows them to notify the caller.
   * If new TX PDO data is available, it updates the corresponding parameters
   * according to the TxPDO mapping by copying the received data slices into
   * each mapped parameter.
   *
   * @param missedCycles Number of cycles missed since the last call.
   *
   * @exception Any exception thrown during the asynchronous process data
   * exchange will be rethrown here to the caller.
   */
  void exchangeProcessDataAndUpdateParameters(
      uint64_t missedCycles = 0) override;

  mm::comm::base::PdoMappingStatus getPdoMappingStatus(
      uint16_t index, uint8_t subindex) const override;

  /**
   * @brief Retrieves the mapped parameter values for the SPoE device.
   *
   * This function iterates over the PDO mappings configured for the device,
   * fetching the current parameter values associated with each mapping.
   * It separates the mapped parameters into RX and TX groups based on the
   * PDO direction and returns them in a structured format.
   *
   * @return mm::comm::MappedParameterValues
   *   A structure containing two vectors:
   *   - rx: mapped parameter values for received PDOs.
   *   - tx: mapped parameter values for transmitted PDOs.
   */
  mm::comm::base::MappedParameterValues getMappedParameterValues();

  /**
   * @brief Checks if the SPoE device is online.
   *
   * This method tries to retrieve basic server information with a short
   * timeout. If the request succeeds, the device is considered online. If it
   * fails (e.g., due to timeout, network error, or protocol error), the method
   * returns false.
   *
   * @return true if the device responds within the timeout; false otherwise.
   */
  bool isOnline() override;

  /**
   * @brief Get a reference to the device parameter refresher.
   * @return Reference to the DeviceParameterRefresher.
   */
  DeviceParameterRefresher& refresher() { return refresher_; }

 private:
  boost::asio::io_context ioContext_;  ///< The Boost.Asio I/O context for
                                       ///< managing asynchronous operations.
  boost::asio::ip::tcp::endpoint
      endpoint_;  ///< The endpoint (IP address and port) to which the client
                  ///< will connect.
  boost::asio::ip::tcp::socket
      socket_;  ///< The TCP socket used for communication with the server.

  /**
   * @brief Mutex used to synchronize access to socket read/write operations.
   * This is a recursive mutex, allowing the same thread to acquire the lock
   * multiple times. Useful in cases where nested operations occur, such as
   * reading parameters while simultaneously performing SDO reads.
   */
  std::recursive_mutex
      mutex_;  ///< Mutex for thread-safe access to socket operations.

  std::mutex disconnectMutex_;  ///< Mutex to synchronize disconnect operations.

  /**
   * @brief Atomic sequence identifier (16-bit unsigned integer).
   *
   * This variable is used to store and increment the sequence ID atomically,
   * ensuring thread safety in a multithreaded environment. The value is
   * initialized to 0.
   */
  std::atomic<uint16_t> seqId_{0};  ///< Sequence ID for message tracking.

  uint16_t position_;  ///< Device position in the network chain, used for
                       ///< identifying the device in a multi-device setup.

  uint8_t state_;  ///< Current state of the device, representing the EtherCAT
                   ///< state machine states (INIT, PREOP, SAFEOP, OP, BOOT).

  std::unordered_map<mm::comm::base::Parameter::Address,
                     mm::comm::base::Parameter>
      parametersMap_;  ///< Map of parameters indexed by (index, subindex)
                       ///< pairs.
  mm::comm::base::PdoMappings
      pdoMappings_;  ///< Mapped PDO entries for the slave.

  std::atomic_flag isSendingAndReceivingProcessData_ =
      ATOMIC_FLAG_INIT;  ///< Flag to indicate if process data exchange is in
                         ///< progress.

  mm::core::util::ThreadSafeQueue<std::vector<uint8_t>>
      txPdoQueue_;  ///< Queue for the received TxPDO data blocks.

  std::mutex
      sendAndReceiveProcessDataExceptionPtrMutex_;  ///< Mutex for
                                                    ///< synchronizing access to
                                                    ///< the exception pointer.

  std::exception_ptr sendAndReceiveProcessDataExceptionPtr_ =
      nullptr;  ///< Exception pointer for capturing errors during
                ///< asynchronous process data exchange.

  DeviceParameterRefresher refresher_;  ///< The refresher for
                                        ///< device parameters.
};

}  // namespace mm::comm::spoe
