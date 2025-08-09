#pragma once

#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include "base.h"
#include "ethercat.h"
#include "loguru.h"

namespace mm::comm::soem {

/**
 * @brief Maps MAC addresses to their corresponding network interface names.
 *
 * Scans available network adapters and constructs a map where each entry
 * associates a MAC address (as a string) with its corresponding interface name.
 *
 * This is useful for identifying interfaces by their hardware address, e.g.,
 * when selecting a specific NIC for EtherCAT communication.
 *
 * @return A map with MAC address strings as keys and interface names as values.
 */
std::map<std::string, std::string> mapMacAddressesToInterfaces();

/**
 * @brief Default UI configuration used for PDO mapping.
 *
 * This JSON string defines the default Process Data Object (PDO) mapping
 * configuration, including Receive (Rx) and Transmit (Tx) PDOs. It is used as a
 * fallback when reading the configuration file from an EtherCAT slave fails.
 *
 * The structure follows a JSON schema compatible with `mm::comm::UiConfigJson`,
 * and includes:
 * - Rx PDO assignments under `"rx"` (e.g., 0x1600, 0x1601, ...)
 * - Tx PDO assignments under `"tx"` (e.g., 0x1a00, 0x1a01, ...)
 *
 * @note This string should remain synchronized with the expected JSON schema in
 * the `mm::comm::UiConfigJson` parser implementation.
 */
inline std::string uiConfigWithDefaultPdoMapping = R"(
{
  "pdoMapping": {
    "rx": {
      "0x1600": [
        "0x60400010",
        "0x60600008",
        "0x60710010",
        "0x607a0020",
        "0x60ff0020",
        "0x60b20010",
        "0x27010020"
      ],
      "0x1601": [
        "0x60fe0120",
        "0x60fe0220"
      ],
      "0x1602": [
        "0x27030020",
        "0x60b10020"
      ]
    },
    "tx": {
      "0x1a00": [
        "0x60410010",
        "0x60610008",
        "0x60640020",
        "0x606c0020",
        "0x60770010",
        "0x60f40020",
        "0x21110220",
        "0x21130220"
      ],
      "0x1a01": [
        "0x24010010",
        "0x24020010",
        "0x24030010",
        "0x24040010",
        "0x27020020"
      ],
      "0x1a02": [
        "0x60fd0020"
      ],
      "0x1a03": [
        "0x27040020",
        "0x20f00020",
        "0x60fc0020",
        "0x606b0020",
        "0x60740010",
        "0x60790020",
        "0x20410120"
      ]
    }
  }
})";

/**
 * @brief Sets PDO mapping for a specified EtherCAT slave using UI
 * configuration.
 *
 * This function attempts to read a `ui.config.json` file from the given
 * EtherCAT slave using File-over-EtherCAT (FoE). If the read fails, it falls
 * back to using a built-in default configuration.
 *
 * The function then:
 * - Parses the configuration JSON to obtain PDO mapping details.
 * - Clears existing PDO and Sync Manager (SM) mappings on the slave.
 * - Writes new PDO mapping entries and SM assignments to the slave's object
 * dictionary.
 *
 * @param context Pointer to the EtherCAT master context (`ecx_contextt`).
 * @param slave Index of the slave device (starting from 1).
 * @return int Returns 1 on success, or 0 on failure (e.g., JSON parse error).
 *
 * @note This function requires the `mm::comm::UiConfigJson` struct to be
 * properly defined and compatible with the JSON structure in `ui.config.json`.
 *
 * @warning Uses hardcoded buffer size for reading the file. Make sure the file
 * fits.
 */
int setPdoMappingFromUiConfig(ecx_contextt *context, uint16 slave);

/**
 * @brief Diagnoses and prints the PDO mapping for a given slave.
 *
 * This function reads the number of RxPDO and TxPDO assignments from the slave,
 * then iterates through them to read and print the mappings and their entries.
 * It checks for RxPDOs in the range 0x1600-0x16FF and TxPDOs in the range
 * 0x1A00-0x1AFF. For each PDO, it reads the index and subindex values and
 * prints them along with the length (in bits) of the entry.
 *
 * @param context The EtherCAT context for communication.
 * @param slave The slave address to be diagnosed.
 *
 * @note This function uses `ecx_SDOread` to fetch the assignment counts and
 * entries for both RxPDO and TxPDO. It prints out the data to the console for
 * inspection. It assumes the slave supports PDO mappings in the specified
 * ranges.
 */
void checkPdoMapping(ecx_contextt *context, uint16 slave);

/**
 * @brief Retrieves the current state of a specific EtherCAT slave.
 *
 * This function returns the current state of a specified EtherCAT slave.
 * If @p forceRefresh is true, the function explicitly reads the state
 * from the slave device using `ecx_FPRD`, bypassing the cached value.
 * The retrieved state is then stored in the master context.
 *
 * The function accesses the AL status register and status code register
 * of the slave to update the internal state tracking. This method avoids
 * using `ecx_readstate`, which reads all slaves' states and may introduce
 * delays, making this function more suitable for single-slave queries.
 *
 * @param context       Pointer to the EtherCAT master context.
 * @param slave         The slave index in the master's list, starting at 1
 * (0 is reserved for the master).
 * @param forceRefresh  If true, the state is re-read from the slave
 * hardware.
 *
 * @return The current state of the specified slave (e.g., EC_STATE_INIT,
 *         EC_STATE_OPERATIONAL). If @p forceRefresh is false, the cached
 *         state is returned.
 */
uint16_t getSlaveState(ecx_contextt *context, uint16_t slave,
                       bool forceRefresh);

/**
 * @brief Converts a numeric EtherCAT state value into a human-readable string.
 *
 * This function maps a numeric EtherCAT state (e.g., EC_STATE_INIT,
 * EC_STATE_OPERATIONAL) to its corresponding human-readable string
 * representation. If the state includes the ERROR flag, the string will append
 * " + ERROR" to the state name.
 *
 * The function recognizes common EtherCAT states such as INIT, PRE-OPERATIONAL,
 * SAFE-OPERATIONAL, and OPERATIONAL, as well as combinations with the ERROR
 * flag. If the state is unrecognized, the function returns the string
 * "UNRECOGNIZED".
 *
 * @param state  The numeric EtherCAT state value to convert.
 *
 * @return A string representing the EtherCAT state, including its numeric value
 * in parentheses. If the state is unrecognized, the string "UNRECOGNIZED" is
 * returned.
 */
std::string convertSlaveStateToString(int state);

/**
 * @brief Transitions a slave to a higher EtherCAT state.
 *
 * This function attempts to transition an EtherCAT slave to a specified target
 * state. If the slave is in an error state, it acknowledges the error and
 * attempts to transition to the target state. The function performs state
 * checks during the process and ensures that the slave transitions correctly.
 *
 * @param context      A pointer to the EtherCAT master context.
 * @param slave        The slave index in the master's list, starting at 1
 * (0 is reserved for the master).
 * @param targetState  The desired target state for the slave.
 *
 * @return True if the slave successfully transitions to the target state, false
 * otherwise.
 */
bool transitionToHigherSlaveState(ecx_contextt *context, uint16_t slave,
                                  int targetState);

/**
 * @brief Updates the mailbox and sync manager (SM) configurations for an
 *        EtherCAT slave when transitioning to specific states (e.g., BOOT or
 * INIT).
 *
 * This function configures the sync managers (SM0 and SM1) for the target slave
 * based on the target state. It handles the following scenarios:
 * - For the BOOT state, it reads and configures the BOOT RX and TX mailbox
 * data, and programs the corresponding sync manager configurations (SM0 and
 * SM1).
 * - For the INIT state, it reads the RX and TX mailbox addresses and lengths,
 * and configures the respective sync managers (SM0 and SM1) for the slave.
 *
 * @param context A pointer to the EtherCAT master context.
 * @param slave The position of the target slave in the master's slave list.
 * @param target_state The target EtherCAT state the slave is transitioning
 * to (e.g., EC_STATE_BOOT, EC_STATE_INIT).
 *
 * @note This function programs the mailbox configurations for the slave based
 * on the specified state and also handles special cases for the BOOT and INIT
 * states.
 */
void updateMailboxSyncManagersOnNextState(ecx_contextt *context, uint16_t slave,
                                          int targetState);

/**
 * @brief Reads the detected SMM module ID from the slave device and configures
 *        the module identification object accordingly.
 *
 * This function reads the detected module identification (object 0xF050:02)
 * from the specified slave device using the given EtherCAT context. If a valid
 * module ID is found (non-zero), it writes this ID back to the configured
 * module identification object (0xF030:02) on the slave.
 *
 * @param[in,out] context Pointer to the EtherCAT master context.
 * @param[in]     slave   Slave device number to query.
 *
 * @note Logs informational messages about detected modules and errors if
 *       writing the configured module ID fails.
 */
void configureDetectedSmmModule(ecx_contextt *context, uint16_t slave);

/**
 * @brief Sets the state of a specified EtherCAT slave.
 *
 * This function attempts to transition the state of a slave to the target
 * state. If the slave is in an error state, it acknowledges and clears the
 * error before transitioning to the target state. The function handles special
 * cases for state transitions, including moving to the INIT or BOOT states.
 *
 * @param context      A pointer to the EtherCAT master context.
 * @param slave        The slave index in the master's list, starting at 1
 * (0 is reserved for the master).
 * @param targetState  The desired state to set for the slave.
 *
 * @return True if the slave successfully transitions to the target state, false
 * otherwise.
 */
bool setSlaveState(ecx_contextt *context, uint16_t slave, int targetState);

/**
 * @brief Read and parse all mapped PDO entries for a given slave.
 *
 * This function reads the PDO assignments (0x1C12 and 0x1C13) and gathers
 * all mapped objects into categorized RxPDO and TxPDO vectors.
 *
 * @param context Pointer to the active EtherCAT context.
 * @param slave Slave index (starting from 1).
 * @return A `PdoMappings` structure containing the parsed Rx and Tx PDO
 * mappings.
 */
mm::comm::base::PdoMappings readPdoMappings(ecx_contextt *context,
                                            uint16_t slave);

/**
 * @brief Logs PDO mapping entries grouped by PDO index.
 *
 * This function organizes PDO mapping entries by their PDO index and logs each
 * group, displaying entries in the order of their appearance. Within each PDO
 * group, the entries are displayed in the format:
 * - PDO index
 * - Index, Subindex, and Bitlength of each entry in the group
 *
 * @param entries A vector of PDO mapping entries to be logged, where each entry
 * contains the PDO index, object index, subindex, and bit length.
 *
 * @note This function uses `LOG_F(INFO, ...)` for logging the details.
 */
void logPdoMappingEntries(
    const std::vector<mm::comm::base::PdoMappingEntry> &entries);

/**
 * @brief Logs the I/O map as a formatted hexadecimal string.
 *
 * This function takes a pointer to an I/O map and its total size in bytes,
 * then logs the contents in hexadecimal format using LOG_F.
 * Each byte is formatted as `0xNN` with uppercase hex digits and padded to two
 * characters.
 *
 * @param ioMap Pointer to the I/O map buffer.
 * @param totalBytes Total number of bytes in the I/O map.
 */
void logIoMap(uint8_t *ioMap, size_t totalBytes);

/**
 * @brief Structure representing the Fieldbus configuration and context for
 * EtherCAT communication.
 *
 * This structure contains the necessary components to interact with the
 * EtherCAT network, including the context, network interface, slave and group
 * information, and other configurations related to EtherCAT communication.
 *
 * @note The context (`ecx_contextt`) holds information about the EtherCAT
 * master and slave states, and other communication parameters. The `iface` is
 * the name of the network interface used for EtherCAT communication. Other
 * members are used to store the network configuration, error states, and
 * EtherCAT protocol-specific data.
 */
struct Fieldbus {
  ecx_contextt context;
  std::string iface;
  uint8 group;
  int roundtripTime;

  /* Used by the context */
  uint8 map[4096];
  ecx_portt port;
  ec_slavet slavelist[EC_MAXSLAVE];
  int slavecount;
  ec_groupt grouplist[EC_MAXGROUP];
  uint8 esibuf[EC_MAXEEPBUF];
  uint32 esimap[EC_MAXEEPBITMAP];
  ec_eringt elist;
  ec_idxstackT idxstack;
  boolean ecaterror;
  int64 DCtime;
  ec_SMcommtypet SMcommtype[EC_MAX_MAPT];
  ec_PDOassignt PDOassign[EC_MAX_MAPT];
  ec_PDOdesct PDOdesc[EC_MAX_MAPT];
  ec_eepromSMt eepSM;
  ec_eepromFMMUt eepFMMU;
};

/**
 * @brief Initializes a Fieldbus instance for EtherCAT communication.
 *
 * This function allocates and initializes a new Fieldbus object using the
 * specified network interface name. It sets default values and links internal
 * pointers in the `ecx_contextt` structure to the corresponding members of the
 * Fieldbus object.
 *
 * @param[out] fieldbus A reference to a unique pointer that will hold the newly
 * created Fieldbus instance.
 * @param iface The name of the network interface (e.g., "eth0") to use for
 * EtherCAT communication.
 *
 * @note The contents of the allocated Fieldbus object are zero-initialized
 * before assignment. Internal SOEM context structures are set up to refer to
 * this object’s members.
 */
void initFieldbus(std::unique_ptr<Fieldbus> &fieldbus,
                  const std::string &iface);

/**
 * @class Slave
 * @brief Represents a slave in the EtherCAT network.
 */
class Slave {
 public:
  /**
   * @brief Constructs a Slave object with a given context and position.
   *
   * This constructor initializes the `Slave` object with a provided context
   * and position, which are used for further operations involving the slave.
   *
   * @param context A reference to the `ecx_contextt` structure that contains
   * the context for the EtherCAT communication.
   * @param position The position of the slave in the EtherCAT network.
   */
  explicit Slave(ecx_contextt &context, uint16_t position);

  /**
   * @brief Gets the position of the slave.
   *
   * This method returns the position of the slave, which is stored in the
   * `position_` member variable.
   *
   * @return uint16_t The position of the slave.
   */
  uint16_t getPosition() const;

  /**
   * @brief Gets the Vendor ID of the slave device.
   *
   * This function retrieves the Vendor ID stored in the EEPROM of the slave
   * device at the current position in the slave list.
   *
   * @return The Vendor ID as an integer.
   */
  uint32_t getVendorId() const;

  /**
   * @brief Gets the Product Code of the slave device.
   *
   * This function retrieves the Product Code stored in the EEPROM of the slave
   * device at the current position in the slave list.
   *
   * @return The Product Code as an integer.
   */
  uint32_t getProductCode() const;

  /**
   * @brief Gets the Revision Number of the slave device.
   *
   * This function retrieves the Revision Number stored in the EEPROM of the
   * slave device at the current position in the slave list.
   *
   * @return The Revision Number as an integer.
   */
  uint32_t getRevisionNumber() const;

  /**
   * @brief Retrieves the current state of the slave.
   *
   * This method calls `getSlaveState` to fetch the state of the slave based on
   * its context and position. The function is `const` because it doesn't modify
   * any internal state of the `Slave` object.
   *
   * @param forceRefresh If true, the state is re-read from the slave hardware.
   *
   * @return int The current state of the slave.
   */
  int getState(bool forceRefresh = true) const;

  /**
   * @brief Attempts to set the slave to the specified EtherCAT state and
   * optionally waits until it is reached.
   *
   * This function first sends a request to change the slave's state to @p
   * targetState. If @p timeoutMs is greater than 0, it repeatedly checks the
   * current state at intervals of @p intervalMs milliseconds, up to the
   * specified timeout.
   *
   * @param targetState The desired EtherCAT state to set (e.g.,
   * EC_STATE_OPERATIONAL).
   * @param timeoutMs Maximum time to wait (in milliseconds) for the slave to
   * reach the target state. Default is 0 (no waiting).
   * @param intervalMs Interval (in milliseconds) between state checks when
   * waiting. Default is 100.
   *
   * @return true if the slave state was successfully set and (if waiting)
   * confirmed, false otherwise.
   */
  bool setState(int targetState, int timeoutMs = 0, int intervalMs = 100);

  /**
   * @brief Loads the parameters (object dictionary entries) from the EtherCAT
   * slave device.
   *
   * This function reads the object dictionary (OD) of the slave device and
   * populates the internal `parametersMap_` with discovered entries. It first
   * verifies that no parameters have been loaded before and ensures that the
   * slave is in the PRE-OPERATIONAL state. It then reads the object dictionary
   * list and entry details using SOEM API calls (`ecx_readODlist`,
   * `ecx_readODdescription`, and `ecx_readOE`).
   *
   * @param readValues If true, the function reads SDO values from the slave.
   *
   * @throws std::runtime_error if:
   * - Parameters are already loaded.
   * - The device is not in the PRE-OPERATIONAL state.
   * - Reading the object dictionary list or entries fails.
   *
   * Object entries are added to `parametersMap_`, each uniquely identified by
   * a pair of index and subindex.
   */
  void loadParameters(bool readValues = false);

  /**
   * @brief Logs all loaded object dictionary parameters.
   *
   * This function retrieves all parameters stored in the `parametersMap_`,
   * sorts them by index and subindex, and then logs each parameter's details,
   * including its index, subindex, bit length, access type, and name. The
   * parameters are logged in a sorted order with a sequence number for easy
   * identification.
   *
   * @note The parameters are sorted before being logged, ensuring that the
   * output is consistent and easy to interpret.
   */
  void logParameters() const;

  /**
   * @brief Logs all mapped RxPDO parameters with their current values.
   *
   * This function iterates through the RxPDO mappings, retrieves the
   * corresponding parameter values, and logs them in the format
   * `parameterId=value`, all in one line.
   *
   * The log entry contains the device position, the number of RxPDO mappings,
   * and the list of parameter IDs and their values. The output is formatted as:
   *
   * Device [position]: RxPDO mapped parameters ([size]): [parameterId1=value1]
   * [parameterId2=value2] ...
   *
   * @note The log output is a single line, with parameters separated by spaces.
   */
  void logRxPdoMappedParameters();

  /**
   * @brief Logs all mapped TxPDO parameters with their current values.
   *
   * This function iterates through the TxPDO mappings, retrieves the
   * corresponding parameter values, and logs them in the format
   * `parameterId=value`, all in one line.
   *
   * The log entry contains the device position, the number of TxPDO mappings,
   * and the list of parameter IDs and their values. The output is formatted as:
   *
   * Device [position]: TxPDO mapped parameters ([size]): [parameterId1=value1]
   * [parameterId2=value2] ...
   *
   * @note The log output is a single line, with parameters separated by spaces.
   */
  void logTxPdoMappedParameters();

  /**
   * @brief Clears all loaded object dictionary parameters.
   *
   * This function removes all entries from the internal parameter map,
   * effectively resetting the parameter list of the slave device.
   */
  void clearParameters();

  /**
   * @brief Returns a reference to the internal parameters map.
   *
   * Provides direct access to the map of parameters, where each entry is
   * identified by a pair of index and subindex.
   *
   * @return Reference to the parameters map.
   */
  std::unordered_map<mm::comm::base::Parameter::Address,
                     mm::comm::base::Parameter> &
  getParametersMap();

  /**
   * @brief Finds and returns a reference to a Parameter in the parameters map.
   *
   * This function searches for a parameter in the parametersMap_ using the
   * provided index and subindex. If the parameter is found, a reference to it
   * is returned. If the parameter is not found, an exception is thrown.
   *
   * @param index The index of the parameter to find.
   * @param subindex The subindex of the parameter to find.
   * @return A reference to the found Parameter.
   * @throws std::runtime_error If the parameter is not found in the map.
   */
  mm::comm::base::Parameter &findParameter(uint16_t index, uint8_t subindex);

  /**
   * @brief Uploads a parameter value from the device's object dictionary.
   *
   * This function reads the value of the specified parameter identified by
   * its index and subindex from the EtherCAT slave device. If `refresh` is
   * set to true, an SDO read will be issued to obtain the most recent value
   * from the device. The parameter is returned either as its raw byte
   * representation (`std::vector<uint8_t>`) or converted to a requested type
   * `T` if supported by the parameter's variant.
   *
   * @tparam T The expected return type. Defaults to `std::vector<uint8_t>`.
   *           If a different type is specified, the internal variant value is
   *           extracted and cast to `T`. A `std::bad_variant_access` exception
   *           is thrown if the types are incompatible.
   *
   * @param index The index of the object in the object dictionary.
   * @param subindex The subindex of the object in the object dictionary.
   * @param refresh If true (default), refreshes the parameter by performing
   *                an SDO read from the device.
   *
   * @return The parameter value as type `T`.
   *
   * @throws std::runtime_error if the device is in an invalid state.
   * @throws std::runtime_error if the SDO read fails.
   * @throws std::bad_variant_access if the parameter cannot be cast to type
   * `T`.
   */
  template <typename T = std::vector<uint8_t>>
  T upload(uint16_t index, uint8_t subindex, bool refresh = true) {
    // Lock the mailbox mutex to ensure thread safety
    std::lock_guard<std::mutex> lock(mailboxMutex_);

    // Check if the device is in a valid state for uploading
    auto state = getState(false);
    if (state == EC_STATE_INIT || state == EC_STATE_BOOT) {
      throw std::runtime_error(
          "To upload object dictionary entries, the device must be in the (2) "
          "PRE-OPERATIONAL, (4) SAFE-OPERATIONAL, or (8) OPERATIONAL state. "
          "The current state is " +
          convertSlaveStateToString(state) + ".");
    }

    // Find the previously loaded parameter
    auto &parameter = findParameter(index, subindex);

    // Refresh data if needed (SDO read)
    if (refresh) {
      const std::unique_ptr<std::uint8_t[]> data =
          std::make_unique<std::uint8_t[]>(parameter.byteLength);
      auto wkc = ecx_SDOread(&context_, position_, parameter.index,
                             parameter.subindex, false, &parameter.byteLength,
                             data.get(), EC_TIMEOUTRXM * 3);
      if (wkc <= 0) {
        LOG_F(ERROR, "Device %d: SDO read failed for %#04x:%02x! WKC: %d",
              position_, parameter.index, parameter.subindex, wkc);
        auto id = mm::comm::base::makeParameterId(parameter.index,
                                                  parameter.subindex);
        throw std::runtime_error("Device " + std::to_string(position_) +
                                 ": SDO read failed for " + id +
                                 "! WKC: " + std::to_string(wkc));
      } else {
        std::vector<uint8_t> receivedData(data.get(),
                                          data.get() + parameter.byteLength);
        if (!parameter.trySetValue(receivedData)) {
          LOG_F(ERROR,
                "Device %d: Failed to set the value of parameter %#04x:%02x!",
                position_, parameter.index, parameter.subindex);
        }
      }
    }

    // Return the appropriate type
    if constexpr (std::is_same_v<T, std::vector<uint8_t>>) {
      return parameter.data;  // parameter.data should be std::vector<uint8_t>
    } else {
      // Otherwise, try to get the value as the requested type T
      auto value = parameter.getValue();
      if (auto ptr = std::get_if<T>(&value)) {
        return *ptr;
      } else {
        throw std::bad_variant_access();  // Type mismatch
      }
    }
  }

  /**
   * @brief Downloads a value to the specified object dictionary entry via SDO.
   *
   * This templated function allows writing either raw byte data or typed values
   * (e.g., int32_t, float) to a device's object dictionary. If a typed value is
   * provided, it is first converted to its raw byte representation using the
   * parameter model.
   *
   * The function verifies that the device is in a state that allows SDO access,
   * performs value-to-bytes conversion if necessary, and then sends the data
   * using an SDO write operation. The `trySetValue()` method is used for typed
   * values to serialize them into the appropriate byte format.
   *
   * @tparam T The type of value to write. Defaults to std::vector<uint8_t> if
   * not specified.
   * @param index The index of the object dictionary entry.
   * @param subindex The subindex of the object within the dictionary.
   * @param value The value to write, either as raw bytes or a typed variant.
   * @return true if the write was successful; false if value conversion failed
   * or SDO write failed.
   *
   * @throws std::runtime_error if the device is in an invalid state or if the
   * data size does not match the expected size of the parameter.
   */
  template <typename T = std::vector<uint8_t>>
  bool download(uint16_t index, uint8_t subindex, const T &value) {
    std::lock_guard<std::mutex> lock(mailboxMutex_);

    // Check if device is in valid state for download
    auto state = getState(false);
    if (state == EC_STATE_INIT || state == EC_STATE_BOOT) {
      throw std::runtime_error(
          "To download object dictionary entries, the device must be in the "
          "(2) PRE-OPERATIONAL, (4) SAFE-OPERATIONAL, or (8) OPERATIONAL "
          "state. The current state is " +
          convertSlaveStateToString(state) + ".");
    }

    auto &parameter = findParameter(index, subindex);

    // Convert value to raw byte data
    std::vector<uint8_t> data;
    if constexpr (std::is_same_v<T, std::vector<uint8_t>>) {
      data = value;
    } else {
      // Set the value in the parameter model
      if (!parameter.trySetValue(value)) {
        LOG_F(ERROR,
              "Device %d: Failed to set the value of parameter %#04x:%02x!",
              position_, parameter.index, parameter.subindex);
        return false;
      }
      data = parameter.data;  // Use the parameter's byte representation
    }

    // Check if data size matches the expected parameter size
    if (data.size() != parameter.byteLength) {
      LOG_F(ERROR, "Data size mismatch: expected %d, got %zu",
            parameter.byteLength, data.size());
      throw std::runtime_error("Data size does not match the parameter size!");
    }

    // Perform the SDO write
    auto wkc = ecx_SDOwrite(&context_, position_, parameter.index,
                            parameter.subindex, false, parameter.byteLength,
                            data.data(), EC_TIMEOUTRXM * 3);

    // Check write result
    if (wkc <= 0) {
      LOG_F(ERROR, "Device %d: SDO write failed for %#04x:%02x! WKC: %d",
            position_, parameter.index, parameter.subindex, wkc);
      return false;
    }

    return true;
  }

  /**
   * @brief Reads the PDO mappings from the slave device.
   *
   * This function logs the operation, reads the PDO mappings, and returns the
   * retrieved mappings.
   *
   * @return The PDO mappings of the slave device.
   */
  mm::comm::base::PdoMappings readPdoMappings() const;

  /**
   * @brief Updates the PDO mapping entries for the slave.
   *
   * This function reads the current PDO mappings using `readPdoMappings()`
   * and replaces the contents of the existing `rxPdos` and `txPdos` vectors
   * in the referenced `pdoMappings_` object. The reference itself remains
   * unchanged, ensuring consistency for other components that may rely on it.
   */
  void updatePdoMappings();

  /**
   * @brief Logs the PDO (Process Data Object) mappings for the device.
   *
   * This function logs the mapped RxPDO (Receive PDO) and TxPDO (Transmit
   * PDO) entries for the device. The mappings are retrieved from the
   * `pdoMappings_` object, and the entries are logged using the
   * `logPdoMappingEntries` function.
   *
   * @note The function first logs the RxPDO mappings followed by the TxPDO
   * mappings.
   */
  void logPdoMappings() const;

  /**
   * @brief Returns a reference to the cached PDO mappings.
   *
   * @return Reference to the internal PDO mappings of the slave device.
   */
  mm::comm::base::PdoMappings &getPdoMappings();

  /**
   * @brief Updates the outputs for the slave device.
   *
   * This function iterates through the received PDO (Process Data Object)
   * mappings and updates the output data based on the corresponding
   * parameters from the slave's context. The data is copied from the
   * parameters to the output buffer in the slave's context. The size of each
   * data entry is determined based on the bitlength of the PDO mapping, and
   * the data is copied into the output buffer using `std::memcpy`.
   *
   * The function works by calculating the size of each PDO mapping, finding
   * the corresponding parameter in the slave's context, and copying the
   * parameter's data into the appropriate location in the output buffer.
   *
   * @note The function assumes that the `pdoMappings_.rxPdos` is properly
   * initialized and that the slave's context contains valid parameters for
   * each entry in the mappings.
   *
   * @see findParameter
   *
   * @return void
   */
  void updateOutputs();

  /**
   * @brief Updates parameters from the received input data.
   *
   * This function iterates through the transmitted PDO (Process Data Object)
   * mappings and updates the parameters with the corresponding data from the
   * slave's input buffer. The data is copied from the input buffer into the
   * parameters using `std::memcpy`. The size of each entry is determined
   * based on the bitlength of the PDO mapping.
   *
   * The function works by calculating the size of each PDO mapping, finding
   * the corresponding parameter in the slave's context, and copying the data
   * from the input buffer into the parameter's data structure.
   *
   * @note The function assumes that the `pdoMappings_.txPdos` is properly
   * initialized and that the slave's context contains valid parameters for
   * each entry in the mappings.
   *
   * @see findParameter
   *
   * @return void
   */
  void updateParametersFromInputs();

 private:
  ecx_contextt &context_;    ///< Reference to the EtherCAT context.
  const uint16_t position_;  ///< Position of the slave in the EtherCAT
                             ///< network. Position starts at 1, where 0 is
                             ///< reserved for the master.
  std::mutex mailboxMutex_;  ///< Mutex for synchronizing mailbox access.
  std::unordered_map<mm::comm::base::Parameter::Address,
                     mm::comm::base::Parameter>
      parametersMap_;  ///< Map of parameters indexed by (index, subindex)
                       ///< pairs.
  mm::comm::base::PdoMappings
      pdoMappings_;  ///< Mapped PDO entries for the slave.
};

class Master {
 public:
  /**
   * @brief Default constructor for the Master class.
   *
   * Does not perform any initialization. You must call `initFieldbus()` before
   * use.
   */
  explicit Master();

  /**
   * @brief Constructs and initializes the Master with the given network
   * interface.
   *
   * Delegates to `initFieldbus(iface)`. See `initFieldbus()` for detailed
   * initialization steps.
   *
   * @param iface The name of the network interface (e.g., "eth0").
   * @throws std::runtime_error if the fieldbus initialization fails.
   */
  explicit Master(const std::string &iface);

  /**
   * @brief Initializes the fieldbus using the specified network interface.
   *
   * Sets up the EtherCAT context and underlying Ethernet interface required
   * for communication. This includes configuring the context to prevent
   * devices without firmware from entering the SAFE-OPERATIONAL state —
   * a condition that would otherwise require a power cycle to recover from.
   *
   * The following operations are performed:
   * - Initializes the SOEM fieldbus context using `ecx_init()`.
   * - Prepares the network interface for EtherCAT communication.
   * - Sets up the NIC, including:
   *   - Semaphore creation.
   *   - Frame buffer allocation.
   *   - Binding to the multiplex Ethernet driver (if applicable).
   * - Supports both primary and redundant (secondary) configurations.
   * - Initializes Ethernet headers for EtherCAT frame communication.
   *
   * Detailed steps performed by `ecx_init()`:
   * 1. Opens a raw socket on the specified interface (e.g., "eth0", "enp3s0").
   *    - Uses a Linux raw socket (AF_PACKET) or Windows WinPcap/Npcap to
   * directly send/receive Ethernet frames (not IP/TCP).
   * 2. Binds the socket and enables promiscuous mode to capture EtherCAT
   * frames.
   *    - EtherCAT uses a custom Ethernet type (0x88A4) that standard NICs
   * ignore.
   * 3. Configures internal SOEM structures (`ecx_context`, `port`, `redport`,
   * etc.):
   *    - Initializes socket pointers, frame buffers, timing variables, slave
   * tables, and state trackers.
   * 4. Prepares transmission (Tx) and reception (Rx) buffers.
   * 5. Resets EtherCAT frame sequence counters and working counters.
   * 6. Sets up retry and retransmission mechanisms.
   *
   * @note
   * - This method performs only the low-level Ethernet setup.
   * - Slave scanning and PDO configuration are not performed here.
   *   To do so, call `ecx_config_init()` separately — this is handled by
   * `initSlaves()`.
   *
   * @warning
   * Potential issues during initialization:
   * - Incorrect interface name (e.g., "eth1" vs. "eth0").
   * - Insufficient permissions (requires `CAP_NET_RAW` or root).
   * - NIC driver or hardware limitations (e.g., promiscuous mode restrictions).
   * - On Windows, Npcap must be correctly installed and configured.
   *
   * @param iface The name of the Ethernet interface to initialize (e.g.,
   * "eth0").
   * @throws std::runtime_error if `ecx_init()` fails or setup cannot complete.
   */
  void init(const std::string &iface);

  /**
   * @brief Deinitializes the fieldbus and releases associated resources.
   *
   * Calls `ecx_close()` to properly close the EtherCAT context and
   * free related system resources such as sockets and buffers.
   *
   * Logs a confirmation message upon successful closure.
   *
   * This method is also invoked by the destructor to ensure clean shutdown.
   */
  void deinit();

  /**
   * @brief Destructor for the Master class.
   *
   * Ensures proper cleanup by calling `deinit()` to close the EtherCAT context
   * and release all allocated resources.
   */
  ~Master();

  /**
   * @brief Returns a reference to the Fieldbus instance.
   *
   * @return Reference to the underlying Fieldbus object.
   * @throws std::runtime_error If the fieldbus is not initialized (i.e.,
   * null).
   *
   * @note Assumes that fieldbus_ has been properly initialized.
   */
  Fieldbus &getFieldbus() const;

  /**
   * @brief Retrieves the interface name used by the fieldbus.
   *
   * This function returns the network interface name associated with the
   * fieldbus, which was set during initialization.
   *
   * @return The interface name as a string.
   */
  std::string getInterfaceName() const;

  /**
   * @brief Returns the total size of the I/O map for the current group.
   *
   * The total size is calculated as the sum of output bytes (`Obytes`)
   * and input bytes (`Ibytes`) defined in the current group of the fieldbus.
   *
   * @return Total number of bytes in the I/O map.
   */
  size_t getIoMapSize() const;

  /**
   * @brief Initializes the slave devices on the EtherCAT network.
   *
   * This function configures the slaves on the EtherCAT fieldbus by calling
   * `ecx_config_init()` with the provided context. It scans the network for
   * all connected slaves, assigns logical addresses to them, and configures
   * their parameters (such as PDO mapping, communication settings, etc.).
   *
   * If `ecx_config_init()` fails (e.g., no slaves detected, communication
   * issues, or network misconfiguration), an error is logged, and a
   * `std::runtime_error` is thrown, providing the cause of the failure.
   *
   * Once the slaves are successfully initialized, the method loops through
   * each detected slave and adds it to the `slaves_` vector using
   * `emplace_back`. This enables subsequent operations to be performed on
   * each slave via the `slaves_` collection, allowing communication and
   * control after initialization.
   *
   * Detailed initialization process:
   * 1. Calls `ecx_config_init()` to scan and configure slaves based on the
   * provided context:
   *    - **Broadcasts a datagram** to identify all connected slaves on the
   * network.
   *    - **Assigns logical addresses** to each slave based on the provided
   * configuration table.
   *    - For each slave:
   *      - **Reads identity data** (e.g., Vendor ID, Product Code, Serial
   * Number, etc.).
   *      - **Configures communication parameters** such as frame sizes,
   * synchronization settings, etc.
   *      - **Sets up PDO mappings** if `usetable` is `TRUE`, using the
   * configuration table to map I/O data.
   *    - **Initializes internal structures** (e.g., `ec_slave[]`,
   * `ec_group[]`) to store slave and group information.
   * 2. If slave initialization is successful, iterates through the detected
   * slaves.
   * 3. Each slave is added to the `slaves_` vector using `emplace_back`,
   * allowing for further operations.
   *
   * This method should typically be called after `ecx_init()` to initialize
   * the EtherCAT interface. It prepares the slaves for communication,
   * allowing the system to begin exchanging data or transitioning to various
   * states such as OPERATIONAL or SAFE-OPERATIONAL.
   *
   * @returns The number of slaves found and successfully initialized on the
   * EtherCAT network.
   *
   * @throws std::runtime_error if `ecx_config_init()` fails (e.g., no slaves
   * found, network issues, or timeout).
   *
   * @warning Common issues during initialization:
   * - **Slaves not found**: Verify physical connections, check cables, or
   * ensure correct interface settings.
   * - **Incorrect slave address mapping**: Ensure the configuration table is
   * correct or manually configure slave addresses.
   *
   * @see ecx_init() for setting up the EtherCAT communication interface.
   * @see ecx_config_init() for details on slave enumeration and
   * configuration.
   * @see ecx_config_map_group() for further manual configuration of slave PDO
   * mappings and groups.
   */
  int initSlaves();

  /**
   * @brief Returns a reference to the list of slaves.
   *
   * This function provides access to the internal vector of slave devices.
   * The returned reference allows modification of the vector of
   * `std::unique_ptr<Slave>` elements. Ownership of the slave objects
   * remains with the `Master` object.
   *
   * @return A reference to a `std::vector<std::unique_ptr<Slave>>`
   * containing the slaves.
   */
  const std::vector<std::unique_ptr<Slave>> &getSlaves() const;

  /**
   * @brief Calculates the expected working counter (WKC) for the current
   * fieldbus group.
   *
   * Computes the expected WKC as twice the number of outputs plus the number
   * of inputs for the selected group in the fieldbus context.
   *
   * @return The expected WKC value.
   */
  inline int expectedWkc() const {
    return fieldbus_->context.grouplist[fieldbus_->group].outputsWKC * 2 +
           fieldbus_->context.grouplist[fieldbus_->group].inputsWKC;
  }

  /**
   * @brief Performs a complete send/receive process data cycle on the
   * fieldbus.
   *
   * Sends process data and then waits to receive the corresponding process
   * data response.
   *
   * @return The received working counter (WKC) from the fieldbus.
   */
  int roundtrip();

  /**
   * @brief Exchanges process data with slaves and updates parameters based on
   * the response.
   *
   * This function initiates the process of exchanging data with all connected
   * slaves. It first updates the output values for each slave using the
   * `updateOutputs` method. Then, it performs a round-trip communication and
   * checks the Working Counter (WKC).
   *
   * If the WKC is greater than or equal to the expected WKC, it updates the
   * parameters for each slave by calling `updateParametersFromInputs`.
   * Otherwise, it logs an error indicating that the WKC is not as expected,
   * and provides details about the slave responses and their states.
   *
   * The function assumes that all slaves are properly initialized and that
   * `slavecount` is a valid value.
   *
   * @note The function relies on the `roundtrip()` function to perform the
   * communication with slaves, and uses the `expectedWkc()` to compare the
   * result with the expected WKC.
   *
   * @see updateOutputs
   * @see updateParametersFromInputs
   * @see roundtrip
   * @see expectedWkc
   *
   * @return void
   */
  void exchangeProcessDataAndUpdateParameters();

  /**
   * @brief Initializes and starts the fieldbus master.
   *
   * Maps process data I/O, configures distributed clocks, and transitions all
   * slaves through SAFE-OPERATIONAL to OPERATIONAL state. Also sends an
   * initial process data roundtrip to ensure proper output synchronization on
   * the slaves.
   *
   * @return True if the start sequence completes successfully.
   */
  bool start();

 private:
  /**
   * @brief Unique pointer to the Fieldbus object.
   *
   * This private member variable holds a unique pointer to a `Fieldbus`
   * object, which is responsible for managing the communication and
   * interaction with the fieldbus system. The `fieldbus_` object is typically
   * initialized during the constructor and used throughout the class for
   * fieldbus-related operations.
   *
   * @note A `std::unique_ptr` is used to ensure proper memory management and
   * to prevent access violations (e.g., SEH exception 0xc0000005) that can
   * occur when calling `ecx_close()` in the destructor.
   */
  std::unique_ptr<Fieldbus> fieldbus_;

  /**
   * @brief A vector containing the Slave objects.
   *
   * This vector stores all initialized slave devices in the fieldbus network.
   * The `Slave` objects are added to this vector during the initialization
   * process.
   *
   * @note The vector stores `std::unique_ptr<Slave>` to manage the lifetime
   * of `Slave` objects because `Slave` contains a `std::mutex`, which is
   * non-copyable due to its deleted copy constructor.
   */
  std::vector<std::unique_ptr<Slave>> slaves_;
};
};  // namespace mm::comm::soem
