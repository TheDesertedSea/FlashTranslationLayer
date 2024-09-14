#pragma once

/*
 * @file 746FlashSim.h
 * @brief This file contains the heart of flash simulator. It is the parent side
 * of code, segregated from child code to safeguard against malicious attacks
 * on this code.
 *
 * This file contains the implementation of CMU's 18746/15746: Storage Systems
 * course project framework
 *
 * @author Ziqi Wang
 * @author (Tweaked) Saksham Jain (sakshamj)
 * @author (Tweaked) Ankush Jain (ankushj)
 */

#include <poll.h>

#include "common.h"
#include "config.h"
#include "memcheck.h"
#if (CONFIG_TWOPROC == 0)
#include "myFTL.h"
#endif

/* Maximum number of pages */
#define MAX_NUM_PAGES (1 << 20)

/* Random data to write - Used by data store*/
#define DS_RAND_DATA 0x12345678

/* Offset to write at  - Used by data store to check for sparse file support */
#define DS_LARGE_FILE_OFFSET 1024 * 1024 * 4

extern int mem_base;
extern bool is_inf;
extern int writes_possible;

typedef size_t (*GetPeakMemUsageHook)();

#define DEBUG_PRINT

#ifdef DEBUG_PRINT

#define dbg_printf(fmt, ...)                                     \
  do {                                                           \
    fprintf(stderr, "%-24s: " fmt, __FUNCTION__, ##__VA_ARGS__); \
    fflush(stdout);                                              \
  } while (0);

#else

#define dbg_printf(fmt, ...)

#endif

#define MIN(a, b) ((a) < (b) ? (a) : (b))

/* Forward declaration */
template <typename PageType>
class FlashSimExecCallBack;

class FlashSimTest;

IPC_Format send_req_to_ftl(class FlashSimTest *fs_test, IPC_Format *tx_msg);

template <typename PageType>
class FlashSimFTL;

#if ENABLE_TRANS_TRACING
extern FILE *trans_trace_fp;
#endif

/* Function declarations */
void init_flashsim();
void deinit_flashsim();
/************************ class FlashSimException starts **********************/

/*
 * Base class for all Exceptions
 */
class FlashSimException : public std::exception {
 private:
  std::string s;

 public:
  FlashSimException(std::string ss) : s(ss) {}

  ~FlashSimException() throw() {}

  const char *what() const throw() { return s.c_str(); }
};

/************************ class FlashSimException starts **********************/

/*************************** class Configuration starts ***********************/

/*
 * class FlashSimConf - Contains all flash configuration information taken from
 * configuration file
 *
 * This is the parent side of the Conf class
 *
 * The format of accepted configuration file is:
 *
 * Comment always starts with a hash tag '#'
 * Parameter and value are separated by one or more space characters
 * PARAMETER_1 1000
 * PARAMETER_2 1001
 *
 * Empty lines are ignored
 * All values are stored as strings. If you want to get a numeric literal
 * out of it, please call the corresponding method
 */
class FlashSimConf : public ConfBase {
 private:
  /* The name of configuration file that this object is associated with */
  std::string file_name;

  /*
   * Stores the actual key-value mapping from the configuration file
   *
   * The value type_ is a pair, and the first element is the value itself,
   * with the second value being the line number
   */
  std::map<std::string, std::pair<std::string, int>> configuration_map;

 public:
  FlashSimConf(const std::string &p_file_name)
      : file_name{p_file_name}, configuration_map{} {
    /* Open the file */
    std::ifstream fp{file_name};

    /* If file opening fails, throw an exceoption */
    if (fp.is_open() == false) {
      ThrowFileNotFoundError();
    }

    std::string line{};

    int line_num = 0;

    /*
     * While the file is valid for read (i.e. not EOF and no error)
     * read lines from the file and process
     */
    while (fp) {
      /* Skippable characters */
      static const std::string space_pattern{" \t\r"};

      line_num++;

      /*
       * Note that std::getline() extracts \n and discard it
       * (i.e. the new line character is not in the string)
       */
      std::getline(fp, line);

      size_t start_index = line.find_first_not_of(space_pattern);

      /*
       * If all characters are space then it's an empty line
       * a corner case would be there is no characters on the
       * line which should also cause this branch being taken
       */
      if (start_index == std::string::npos) {
        continue;
      }

      /*
       * After this line the index should point to a valid
       * character inside the line
       */

      /*
       * The line is comment line starting with hash tag
       * So skip
       */
      if (line[start_index] == '#') {
        continue;
      }

      /*
       * End index points to the first character that is a
       * space - this is the delimiter between key substring
       * and value substring
       */
      size_t end_index = line.find_first_of(space_pattern, start_index);

      /*
       * Since there must be a value, it could not be the case
       * that there is no space after the key
       */
      if (end_index == std::string::npos) {
        ThrowNoValueError(line_num, line);
      }

      /*
       * Use substr() to get the substring which is the key
       * the second argument is the length of the substring
       */
      std::string key = line.substr(start_index, end_index - start_index);

      /* Then find the value */

      start_index = line.find_first_not_of(space_pattern, end_index);

      /*
       * If all characters are space then it's an empty line
       * a corner case would be there is no characters on the
       * line which should also cause this branch being taken
       */
      if (start_index == std::string::npos) {
        ThrowNoValueError(line_num, line);
      }

      /* Find the end index for value */
      end_index = line.find_first_of(space_pattern, start_index);

      /*
       * It is possible that there is no space character after
       * the value in that case just set end index to be the
       * length of the string to make substring length
       * calulation consistent
       */
      if (end_index == std::string::npos) {
        end_index = line.size();
      }

      std::string value = line.substr(start_index, end_index - start_index);

      /*
       * It is required that keys are not
       * redefined in the configuration file
       */
      if (configuration_map.find(key) != configuration_map.end()) {
        ThrowKeyAlreadyExistsError(key);
      }

      configuration_map[key] = std::make_pair(value, line_num);
    } /* while fp is not EOF */

    return;
  }

  ~FlashSimConf() {}

  /*
   * Print() - Print out the configuration
   *
   * This function prints out configuration in a human readable form,
   * the first column being key and the second column being the value.
   * And at the beginning of the printing there are also other basic
   * information such as file name and configuration line numbers
   *
   * Note: dbg_printf() must be enabled to see prints
   * XXX: This is not present in the base class.
   * One way around to use this for debugging purpose is to place this
   * function in the constructor of this class
   */

  void Print() const {
    dbg_printf("========== Configuration ==========\n");
    dbg_printf("File Name: %s\n", file_name.c_str());
    dbg_printf("Configuration Count: %lu\n", configuration_map.size());
    dbg_printf("\n");

    /*
     * Then iterate through all elements in the map
     * and print out line number, key and value
     */
    for (const auto &key_value_pair : configuration_map) {
      dbg_printf("Line %d: %s = %s\n", key_value_pair.second.second,
                 key_value_pair.first.c_str(),
                 key_value_pair.second.first.c_str());
    }

    return;
  }

  /* Functions overriding base class virtual functions */

  /* Returns the number of packages in flash */
  size_t GetSSDSize(void) const { return (size_t)GetInteger(CONF_S_SSD_SIZE); }

  /* Returns the number of dies in flash */
  size_t GetPackageSize(void) const {
    return (size_t)GetInteger(CONF_S_PACKAGE_SIZE);
  }

  /* Returns the number of planes in flash */
  size_t GetDieSize(void) const { return (size_t)GetInteger(CONF_S_DIE_SIZE); }

  /* Returns the number of blocks in flash */
  size_t GetPlaneSize(void) const {
    return (size_t)GetInteger(CONF_S_PLANE_SIZE);
  }

  /* Returns the number of pages in flash */
  size_t GetBlockSize(void) const {
    return (size_t)GetInteger(CONF_S_BLOCK_SIZE);
  }

  /* Returns the block lifetime of flash */
  size_t GetBlockEraseCount(void) const {
    return (size_t)GetInteger(CONF_S_BLOCK_ERASES);
  }

  /* Returns the garbage collection policy of flash */
  virtual size_t GetGCPolicy(void) const {
    return (size_t)GetInteger(CONF_S_GCPOLICY);
  }

  /* Returns the overprovisioning (as percentage) of flash */
  size_t GetOverprovisioning(void) const {
    return (size_t)GetInteger(CONF_S_OVERPROVISIONING);
  }

  // Configs for checkpoint 3 grading

  /* Returns the amount of memory under which full credit is assigned */
  size_t GetMemoryBaseline(void) const {
    return (size_t)GetInteger(CONF_S_MEMORY_BASELINE);
  }

  /* Returns the number of writes which should be possible to achieve */
  size_t GetWritesBaseline(void) const {
    return (size_t)GetInteger(CONF_S_WRITES_BASELINE);
  }

  /* Returns the multiplier applied to total writes for an FTL design
   * (i.e., buffer) when comparing against total possible writes for
   * calculating the write amplification score.
   */
  double GetWriteAmplificationThreshold(void) const {
    return GetDouble(CONF_S_WRITE_AMPLIFICATION_THRESHOLD);
  }

  /* Returns the multiplier applied to total writes for an FTL design
   * (i.e., buffer) when comparing against total possible writes for
   * calculating the write endurance score.
   */
  double GetWritesThreshold(void) const {
    return GetDouble(CONF_S_WRITES_THRESHOLD);
  }

  /* Returns the weight (out of 100) attributed to write amplification for
   * infinite-running tests.
   */
  size_t GetWeightWriteAmplificationInfinite(void) const {
    return (size_t)GetInteger(CONF_S_WEIGHT_WRITE_AMPLIFICATION_INFINITE);
  }

  /* Returns the weight (out of 100) attributed to memory usage for
   * infinite-running tests.
   */
  size_t GetWeightMemoryInfinite(void) const {
    return (size_t)GetInteger(CONF_S_WEIGHT_MEMORY_INFINITE);
  }

  /* Returns the weight (out of 100) attributed to write endurance for
   * infinite-running tests.
   */
  size_t GetWeightEnduranceInfinite(void) const {
    return (size_t)GetInteger(CONF_S_WEIGHT_ENDURANCE_INFINITE);
  }

  /* Returns the weight (out of 100) attributed to write amplification for
   * finite-running tests.
   */
  size_t GetWeightWriteAmplificationFinite(void) const {
    return (size_t)GetInteger(CONF_S_WEIGHT_WRITE_AMPLIFICATION_FINITE);
  }

  /* Returns the weight (out of 100) attributed to memory usage for
   * finite-running tests.
   */
  size_t GetWeightMemoryFinite(void) const {
    return (size_t)GetInteger(CONF_S_WEIGHT_MEMORY_FINITE);
  }

  /*
   * GetString() - Returns a string which is the value of some key
   *
   * This function returns a value without any atoi operation. The
   * returned value is the original value recorded in the configuration
   * file
   *
   * The string is returned in a const reference form such that the caller
   * could not modify it
   *
   * If the key does not exist in the configuration file then an exception
   * will be thrown
   */
  const std::string &GetString(const std::string &key) const {
    auto it = configuration_map.find(key);

    /* The key must exist */
    if (it == configuration_map.end()) {
      ThrowKeyDoesNotExistError(key);
    }

    return it->second.first;
  }

  /*
   * GetInteger() - Fetch the value of a given key and convert it into an
   *                integer
   *
   * Note that the value must be convertable to an integer, otherwise the
   * behavior is undefined, and should not be relied on
   *
   * Accepted forms of integer are:
   *
   *   0ddd...d octoal
   *   dddd...d decimal
   *   0xdd...d hex
   *   0Xdd...d hex
   *
   * where d is 0 - 7 for octal, 0 - 9 for decimal, and 0 - 9, a-f, A-F
   * for hex integers.
   *
   * When only part of the string forms an integer starting from the
   * beginning, the integer returned is the integer formed by the prefix
   * of the value string. The remianing parts will be discarded.
   */
  int GetInteger(const std::string &key) const {
    const std::string value = GetString(key);

    return std::stoi(value);
  }

  /*
   * GetDouble() - Fetch the value of a given key and convert it into a
   *               double
   */
  double GetDouble(const std::string &key) const {
    const std::string value = GetString(key);

    return std::stod(value);
  }

 private:
  /* Functions only used internally by the class */

  /*
   * ThrowFileNotFoundError() - When file open failed call this
   */
  void ThrowFileNotFoundError() const {
    throw FlashSimException("The given configuration file " + file_name +
                            " could not be found!");
  }

  /*
   * ThrowKeyAlreadyExistsError() - When there are multiple defines of a
   *                                key in the file this is called
   */
  void ThrowKeyAlreadyExistsError(const std::string &key) const {
    throw FlashSimException("Key \"" + key + "\" already exists in the map");
  }

  /*
   * ThrowNoValueError() - Throw exception when in the configuration file
   *                       there is no value for a key
   */
  void ThrowNoValueError(int line_num, const std::string line) const {
    throw FlashSimException("Configuration line " + std::to_string(line_num) +
                            " : \"" + line +
                            "\" contains a key without any value!");
  }

  /*
   * ThrowKeyDoesNotExistError() - This error is thrown when we request a
   *                               key from the configuration file, but
   *                               the key does not exist
   */
  void ThrowKeyDoesNotExistError(const std::string &key) const {
    throw FlashSimException("Key " + key + " does not exist");
  }
};

/***************************** class Configuration ends ***********************/

/**************************** class DataStore starts **************************/

/*
 * class DataStore - The actual storage where bytes inside SSD is stored
 *
 * Currently we just use a disk temporary file to achieve the effect. In
 * the future this could be extended to use mmap-ed file or even customized
 * page slab allocator (i.e. a memory-disk hybrid one)
 *
 * The data store is abstracted in a way that it consists of an array of
 * slots of a certain size (both slot count and slot size could be configured)
 * by the constructor. Each read or write command just specifies the slot id
 * withoug having to specify the size of slot since they are all known
 * to the module
 *
 * For the function call that creates a temporary file, see
 *   http://linux.die.net/man/3/tmpfile
 *
 * Note that since the data store layer is regarded as a simulation of real
 * SSD, it also mimics the characteristics of a SSD plus some reinforcing
 * properties:
 *
 *   1. A slot could only be read after being written
 *   2. A slot could not be overwritten without being erased in between
 *   3. An active slot could be erased to make it inactive (i.e. ready
 *      for write)
 *   4. A slot becomes active after being written
 *
 * If any of these conditions are violated an exception will be thrown
 *
 * Also note that this class is templatized such that the templated type_
 * must be a plain old data type_, i.e. being trivially copiable by memcpy.
 * This is consistent with the data characteristic in a real SSD
 */

/* sizeof(T) is the size of the slot */
template <typename T>
class DataStore {
 private:
  /*
   * File pointer to a temporary file created by system call
   * the file name is even unknown to the module, and the file
   * will be deleted automatically
   */
  FILE *fp;

  /* The number of slots in the data store */
  size_t slot_count;

  /* This records slots that are currently active */
  std::unordered_set<size_t> active_slot_set;

 public:
  DataStore(size_t p_slot_count)
      : fp{tmpfile()},            /* Open temp file */
        slot_count{p_slot_count}, /* Count of slots */
        active_slot_set{} {
    /* Check whether we have created the temp file successfully */
    if (fp == nullptr) {
      ThrowCreateTmpFileError();
    }

    /*
     * Next determine whether sparse file is supported
     * If not supported then we throw an exception
     * We need sparse files as data to spread out
     */
    bool sparse_file_support = DetermineSparseFileSupport();

    if (sparse_file_support == false) {
      ThrowSparseFileNotSupportedError();
    }

    return;
  }

  /*
   * ~DataStore() - Closes the temporary file used as data store
   *
   * It is not a necessary step since the temp file will automatically be
   * closed after program exits
   */
  ~DataStore() {
    int ret = fclose(fp);
    assert(ret == 0);

    return;
  }

  /*
   * ReadSlot() - Reads a specified slot into the given buffer
   *
   * The buffer size must be larger than sizeof(T),
   * and there is no run time checking to ensure this.
   * Also if the slot ID is too large then an exception is thrown
   *
   * Note that if the slot being read is not active then an exception will
   * be thrown because all read content will be garbage
   */

  void ReadSlot(T *buffer, size_t slot_id) {
    /*
     * First move file pointer to the correct offset
     * If slot ID is not valid this will throw an exception
     * We need to do this before checking for activeness
     * because out of bound is a more severe error
     */
    MoveToSlot(slot_id);

    /*
     * Then check whether the slot is currently active or not
     * Note that the order of checking the set and checking bound
     * are slightly different in ReadSlot() and WriteSlot()
     */
    auto it = active_slot_set.find(slot_id);
    if (it == active_slot_set.end()) {
      return;
    }

    /*
     * Issue read command,
     * and verify return value which must be a success
     */
    int ret = fread(buffer, sizeof(T), 1, fp);
    assert(ret == 1);

    return;
  }

  /*
   * WriteSlot() - Write buffer content into a given slot
   *
   * This function is almost the same as its counterpart ReadSlot(), just
   * changing fread() to fwrite()
   *
   * Writing to a slot might result in a sparse file.
   * Since we only keep this file as a temp file it is OK because
   * no external application could read the potentially large file
   * after this program exits
   * If the slot is currently active then an exception is thrown since
   * a slot could not be overwritten without being erased first
   */

  void WriteSlot(const T &data, size_t slot_id) {
    /* First check whether the slot is currently active or not */
    auto it = active_slot_set.find(slot_id);
    if (it != active_slot_set.end()) {
      ThrowOverwriteSlotError(slot_id);
    }

    /* Insert the slot ID into active set since it is now active */
    active_slot_set.insert(slot_id);

    /* And then just finish actual write operation */
    MoveToSlot(slot_id);
    int ret = fwrite(&data, sizeof(T), 1, fp);
    assert(ret == 1);

    return;
  }

  /*
   * EraseSlot() - Mark the slot as not used
   *
   * Erasing a slot does not erase data in the file system because that
   * requires punching a hole in the file which is not a
   * standard operation.
   * Therefore, the way we erase a slot is to just remove the slot ID from
   * the slot id set inside the class to mark the slot as inactive
   * and next time if a slot is read, an exception will be thrown
   *
   * This function is just a wrapper to EraseRange() which is used
   * for testing but the semantics are the same
   */
  void EraseSlot(size_t slot_id) {
    EraseRange(slot_id, slot_id);
    return;
  }

  /*
   * EraseRange() - Erase all slots in a range
   *
   * Arguments start_slot_id and end_slot_id specifies the range of slots
   * being erased. This range must be valid (i.e. start <= end and
   * end < slot_count), otherwise an exception will be thrown.
   *
   * Also this function treats the range as a single unit for erasure,
   * i.e. it throws exception only when all slots in the range are empty
   * this was done before any slot ID is removed from the active slot set
   *
   * On an SSD this operation is similar to a block-level erasure, but
   * here we do not reuqire extra block alignment to make the
   * implementation slightly simpler
   */
  void EraseRange(size_t start_slot_id, size_t end_slot_id) {
    /* start < end && end < count */
    if ((start_slot_id > end_slot_id) || (end_slot_id >= slot_count)) {
      ThrowErasingInvalidRangeError(start_slot_id, end_slot_id);
    }

    for (size_t slot_id = start_slot_id; slot_id <= end_slot_id; slot_id++) {
      auto it = active_slot_set.find(slot_id);

      /*
       * If the iterator is not a valid one then the
       * slot id is not active and exception will be thrown
       */
      if (it != active_slot_set.end()) {
        active_slot_set.erase(it);
      }
    }

    return;
  }

  /*
   * Print() - Report logical file size and block usage, etc.
   *
   * Note: For this to work, debug must be enabled
   */
  void Print() {
    int ret;

    /* Must flush the file descriptor first */
    ret = fflush(fp);
    assert(ret == 0);

    struct stat info;
    int fno = fileno(fp);

    ret = fstat(fno, &info);
    assert(ret == 0);

    dbg_printf("========== Data Store Statictics ==========\n");
    dbg_printf("Block Usage: %lu\n", info.st_blocks);
    dbg_printf("Logical File Size: %lu\n", info.st_size);

    return;
  }

 private:
  /* Functions only used by the class internally */

  /*
   * DetermineSparseFileSupport() - Determines whether sparse is supported
   *                                on the current platform
   *
   * We do this by actually writing to an opened file, seek to large
   * offset, write and then call fstat on its file descriptor to
   * check whether block count is significantly smaller than the logical
   * file size (they are all included in struct stat)
   * The return value is an indication whether sparse file is supported on
   * the current platform. This function creates a temp file that will
   * be deleted after the function returns
   */

  /*
   * TODO: As an optimization, instead of a file, memory map this file
   * for faster access - But need to know the max size of the file
   */
  bool DetermineSparseFileSupport() {
    FILE *fp_temp = tmpfile();

    int t = DS_RAND_DATA;

    int ret;

    /*
     * If any of the return value of fseek() fwrite() and fflush()
     * is not as expected then we return false
     * to avoid any further damage a sparse file might do to the
     * system
     * e.g. a super large file that eats out all available space or
     * gets you banned from the shared system
     */

    ret = fseek(fp_temp, DS_LARGE_FILE_OFFSET, SEEK_SET);
    if (ret != 0) {
      return false;
    }

    ret = (int)fwrite(&t, sizeof(t), 1, fp_temp);
    if (ret != 1) {
      return false;
    }

    /*
     * Flush must be done since we will circumvent the stdio layer
     * and directly go to the file system to get the file descriptor
     * If we do not flush then the contents are not actually written
     */
    ret = fflush(fp_temp);
    if (ret != 0) {
      return false;
    }

    struct stat info;

    /*
     * Extracts the OS level file descriptor inside the libc
     * level fp
     * */
    int fno = fileno(fp_temp);

    /*
     * This must be zero - no exception since we opened the file and
     * we know the file is valid
     */
    ret = fstat(fno, &info);
    assert(ret == 0);

    /*
     * Must close the temp file for testing, so that it could
     * be deleted before this function returns
     */
    ret = fclose(fp_temp);
    assert(ret == 0);

    /*
     * If the size of block * number of blocks is smaller than
     * the logical file size then we know it is a sparse file
     * and it is safe to use a sparse file to use as the data store
     */
    if (info.st_blksize * info.st_blocks < info.st_size) {
      return true;
    }

    return false;
  }

  /*
   * MoveToSlot() - Moves the file pointer inside this class to the byte
   *                offset of a slot ID
   *
   * This function is the common routine for reading and writing the
   * temp file, so we put it as a separate prcedure
   */

  void MoveToSlot(size_t slot_id) {
    /* If the slot is an invalid one just throw exception */
    if (slot_id >= slot_count) {
      ThrowSlotOutOfBoundError(slot_id);
    }

    /*
     * Otherwise get to the offset and issue read to the file system
     * If the slot has not been written yet then this returns all 0
     * into the buffer. This is accepted behavior since anyway we
     * have to validate the value inside the testing routine
     *
     * Each slot is allocated sizeof(T) bytes only
     */
    size_t byte_offset = slot_id * sizeof(T);

    /*
     * Seek from the start of file to the position of the slot,
     * offset is calculated from file beginning
     */
    int ret = fseek(fp, byte_offset, SEEK_SET);
    assert(ret == 0);

    return;
  }

  /*
   * ThrowCreateTmpFileError() - Throw an error indicating we failed
   * 			       creating the temporary file
   */
  void ThrowCreateTmpFileError() const {
    /* Must save it first to avoid it being overwritten */
    int t = errno;

    /* Print string form of the error */
    dbg_printf("%s\n", strerror(t));

    throw FlashSimException("Could not create temporary file! Errno = " +
                            std::to_string(t));
  }

  /*
   * ThrowSparseFileNotSupportedError() - If sparse file is not supported
   *                                      then this is called
   */
  void ThrowSparseFileNotSupportedError() const {
    throw FlashSimException(
        "Sparse file is not supported"
        " by the current platform. Please consider switcing"
        " to a different file system");
  }

  /*
   * ThrowSlotOutOfBoundError() - This is called when the slot ID is
   *                              out of bound
   *
   * This should be a rare case and could only happen if there is bug
   */
  void ThrowSlotOutOfBoundError(size_t slot_id) const {
    throw FlashSimException(std::string{"Slot "} + std::to_string(slot_id) +
                            " is out of bound");
  }

  /*
   * ThrowOverwriteSlotError() - This is called when a slot is overwritten
   *                             without being erased first
   */
  void ThrowOverwriteSlotError(size_t slot_id) const {
    throw FlashSimException(std::string{"Slot "} + std::to_string(slot_id) +
                            " could not be overwritten (write error)");
  }

  /*
   * ThrowErasingInvalidRangeError() - This is called if the the range in
   *                                   EraseRange() is invalid
   */
  void ThrowErasingInvalidRangeError(size_t start_slot_id, size_t end_slot_id) {
    throw FlashSimException(
        std::string{"The range ["} + std::to_string(start_slot_id) + ',' +
        std::to_string(end_slot_id) + "] is either invalid or out of bound");
  }
};

/***************************** class DataStore ends ***************************/

/*************************** class Controller starts **************************/

/*
 * class Controller
 *
 * Controller is the central control unit of the SSD. All commands and
 * addresses are sent to and from this object, where decisions are made.
 *
 * Note that this class is templatized as carrying an extra argument as the
 * PageType. PageType is used to represent internal data inside the SSD,
 * and they are copied into the internal buffer of the controller when
 * a read command is issued, and are written into the data store when
 * a write command is issued.
 *
 * To further simplify your task we do not impose any hard limit on the size
 * of the internal buffer, which indicates that command pattern such as
 * READ-READ-READ-...-WRITE-WRITE-WRITE..WRITE is acceptable.
 * However, when erasing a block the buffer must be empty, since in practice
 * erasing a block is considered as a time consuming task, and if power
 * failure happens during a block erasure, the SSD is in an undefined state
 * after power recovers. So in general, caching data while erasure is going
 * on should be prohibited.
 *
 * The internal buffer is organized as a queue, which means that WRITE
 * command will write out pages read previously into the buffer in a FIFO
 * manner.
 */
template <typename PageType>
class Controller {
  friend class FlashSimExecCallBack<PageType>;

 private:
  /*
   * This is a pointer to the interface class
   * Only member functions defined in the base class are available
   * to Controller
   */
  FTLBase<PageType> *ftl_p;

  /*
   * This is the place where actual data is stored
   * Please note that, in a real SSD, the data componen is usually
   * not part of the Controller. But here for simplicity of the
   * code and for your easiness of understanding, we put
   * class DataStore here
   */
  DataStore<PageType> *ds_p;

  /* This is the configuration of SSD */
  FlashSimConf *config_p;

  /*
   * This is the internal buffer of the controller which is organized
   * as a FIFO queue
   * In most cases single element queue is sufficient - but you could
   * treat it as an infinite sized queue as you wish
   *
   * One important thing to mention is that when you issue an erase
   * command please make sure the page buffer is empty, otherwise
   * an exception will be thrown
   *
   * The second component of the element is the logical LBA
   * associated with this piece of data
   * This is used to verify that we actually read the correct page
   */
  std::queue<std::pair<PageType, size_t>> page_buffer;

  /*
   * We intentionally make it a ordered map such that we could
   * iterate block address in an ordered manner
   *
   * The key to this map is the linear block address, which is exactly
   * the linear address of the first page of the block
   * (so the linear address should be a multiple of page per Block).
   * And the value is number of erasures remaining
   */
  std::map<size_t, size_t> block_erasure_map;

  /*
   * This is the map that maps physical LBA to logical LBA
   * This is used to verify that each read/write operation actually get to
   * the location where the page is
   *
   * If there is no entry for a physical block, then the physical block
   * is a fresh block. Otherwise the block is either most up-to-date
   * or obsolete. We could not decide which one is the case, and this
   * will be detected by the data component since reading obsolete
   * pages will give back incorrect data
   */
  std::map<size_t, size_t> physical_logical_map;

  /* Number of packages inside an SSD */
  size_t ssd_size;

  /* Number of dies in a package */
  size_t package_size;

  /* Number of planes in a die */
  size_t die_size;

  /* Number of blocks in a plane */
  size_t plane_size;

  /* Number of pages in a block */
  size_t block_size;

  /* Max erases a block can handle */
  size_t block_erase_count;

  /* As name suggests */
  size_t page_per_block;
  size_t page_per_plane;
  size_t page_per_die;
  size_t page_per_package;

  /*
   * This one is special - it is not invokved in computing the
   * LBA, but instead it is used to verify it
   * This is the total number of pages in the SSD
   */
  size_t page_per_ssd;

  /* Counters for each operations */
  uint64_t num_writes;
  uint64_t num_reads;
  uint64_t num_erases;

 public:
  /*
   * Constructor - Initialize member object pointers
   */
  Controller(FTLBase<PageType> *p_ftl_p, DataStore<PageType> *p_ds_p,
             FlashSimConf *p_config_p)
      :

        ftl_p{p_ftl_p},
        ds_p{p_ds_p},
        config_p{p_config_p},
        page_buffer{},
        block_erasure_map{},
        physical_logical_map{},
        /* Get configuration and calcuate various parameters */
        ssd_size{config_p->GetSSDSize()},
        package_size{config_p->GetPackageSize()},
        die_size{config_p->GetDieSize()},
        plane_size{config_p->GetPlaneSize()},
        block_size{config_p->GetBlockSize()},
        block_erase_count{config_p->GetBlockEraseCount()},
        page_per_block{block_size},
        page_per_plane{page_per_block * plane_size},
        page_per_die{page_per_plane * die_size},
        page_per_package{page_per_die * package_size},
        page_per_ssd{page_per_package * ssd_size},
        num_writes(0),
        num_reads(0),
        num_erases(0) {}

  /*
   * Destructor - Free member objects
   *
   * Ownership of member objects have been transferred to this class
   */
  ~Controller() {}

  /*
   * AddressToLBA() - Convers a hierarchical Address object to LBA
   */
  size_t AddressToLBA(const Address &addr) {
    return addr.page + addr.block * page_per_block +
           addr.plane * page_per_plane + addr.die * page_per_die +
           addr.package * page_per_package;
  }

  /*
   * ExecuteCommand() - Given a list of commands, execute them one by one
   *
   * Commands could be READ, WRITE and ERASE, and as their names suggest
   * they represent the physical operation on the physical page
   * specified by the Address object.
   *
   * Several restrictions are present on how commands could be executed:
   *
   *   (1) ERASE must be executed when the buffer is empty
   *
   * These restrictions are used to prevent caching pages inside the
   * controller DRAM, in case of power failure causing data to be lost.
   */

  void ExecuteCommand(OpCode operation, Address addr) {
    switch (operation) {
      case OpCode::READ: {
        PageType page{};

        size_t logical_lba;

        /*
         * This is the physical LBA where data will be
         * read from
         */
        size_t physical_lba = AddressToLBA(addr);

        /*
         * We also need to find the logical LBA associated
         * with this physical LBA
         */
        auto it = physical_logical_map.find(physical_lba);
        if (it == physical_logical_map.end()) {
          /*
           * If the mapping for the physical does not
           * yet exist then the physical page is clean.
           * Reading from it would results in undefined
           * data
           */
          ThrowInvalidReadError(physical_lba);
        } else {
          /*
           * If the read operation is correct then get
           * its logical LBA
           */
          logical_lba = it->second;
        }

        /*
         * Read the actual content of the page into
         * local page object
         */
        ds_p->ReadSlot(&page, physical_lba);

        /*
         * And then push the page object back into the queue
         * We need both the page data and logical LBA
         * to let the following write operation know what is
         * the logical LBA associated with a page
         */
        page_buffer.push(std::make_pair(page, logical_lba));

        num_reads++;
        break;
      }

      case OpCode::WRITE: {
        size_t physical_lba = AddressToLBA(addr);

        /* Keep a reference to the front of the page buffer */
        const PageType &page = page_buffer.front().first;

        /*
         * This is the LBA that the physical LBA will
         * be associated to
         */
        size_t logical_lba = page_buffer.front().second;

        /*
         * If there is already an entry for the physical address
         * then we could not associate it with another logical
         * LBA, and this is an error
         */
        auto insert_ret = physical_logical_map.insert(
            std::make_pair(physical_lba, logical_lba));

        /* This indicates that the physical LBA already exist */
        if (insert_ret.second == false) {
          ThrowWriteDirtyPageError(physical_lba);
        }

        /* And then write front element into the data store*/
        ds_p->WriteSlot(page, physical_lba);

        /* Remove the front object from the page buffer */
        page_buffer.pop();

#if ENABLE_TRANS_TRACING
        fprintf(trans_trace_fp, "W 1 %zu <%d,%d,%d>\n", logical_lba, addr.plane,
                addr.block, addr.page);
#endif

        num_writes++;
        break;
      }

      case OpCode::ERASE: {
        /*
         * First check whether the page buffer is empty
         * If not this is an error
         */
        EnsureStateIsClean();

        /*
         * Nullify the page so that we get the LBA pointing to
         * the starting page of a certain block
         */
        addr.page = 0;

        /*
         * Compute the range of pages we need to erase
         * Note: If all pages inside the range are
         * empty pages then an exception will be thrown since
         * we erased empty block which should be prohibited
         */
        size_t start_lba = AddressToLBA(addr);
        size_t end_lba = start_lba + page_per_block - 1;

        /*
         * TODO: No exception is thrown currently if block
         * is clean - Not an issue, but might give student's
         * hint in issues within their design
         */
        ds_p->EraseRange(start_lba, end_lba);

        /*
         * The last step is to remove physical-logical
         * LBA mapping within range [start_lba, end_lba]
         */
        for (size_t physical_lba = start_lba; physical_lba <= end_lba;
             physical_lba++) {
          /*
           * STL happily accepts it even if physical
           * lba_ does not yet exist
           */
          physical_logical_map.erase(physical_lba);
        }

        num_erases++;
#if ENABLE_TRANS_TRACING
        fprintf(trans_trace_fp, "E <%d,%d>\n", addr.plane, addr.block);
#endif
        UpdateBlockErasure(start_lba);
        break;
      }

      default: {
        ThrowUnknownOpCodeError(operation);
      }

    } /*switch op code */

    return;
  }

  /*
   * ReadLBA() - Reads a linear page address
   *
   * The controller calls FTL through the virtual base class to
   * execute one or more operations, and perform them on physical pages
   * (i.e. the linear array on data store object)
   *
   * Read operation call on FTL must return a target address. Operations
   * reulting from read are executed by coltroller inside the FTL function
   * and then the target address is read into the argument provided to
   * this function.
   *
   * The return value indicates the result of execution, which could be
   * either SUCCESS or FAILURE.
   */
  ExecState ReadLBA(PageType *page_p, size_t lba) {
    /*
     * Call FTL to translate single LBA read into a series of
     * commands
     */
#if (CONFIG_TWOPROC == 1)
    auto ret = ftl_p->ReadTranslate(lba, ExecCallBack<PageType>());
#else
    auto ret = ftl_p->ReadTranslate(lba, FlashSimExecCallBack<PageType>(this));
#endif

    /* Make sure nothing is left in page buffer after translation */
    EnsureStateIsClean();

    /* If the return value is FAILURE then simply return */
    if (ret.first == ExecState::FAILURE) {
      return ExecState::FAILURE;
    }

    /* Perform read operation on the target address */
    ExecuteCommand(OpCode::READ, ret.second);

    /*
     * Copy the PageType object back to the argument
     * and remove the object from the page buffer
     */
    *page_p = page_buffer.front().first;
    page_buffer.pop();

    return ExecState::SUCCESS;
  }

  /*
   * WriteLBA() - Write an LBA with given data
   *
   * Write command might result in zero or more write amplification
   * operations, plus a target address for writing the given piece of data.
   * Operations are executed inside FTL, and then the target address is
   * written with the given data
   *
   */
  ExecState WriteLBA(const PageType &page, size_t lba) {
    /*
     * Call FTL to translate single LBA read into a
     * series of commands
     */
#if (CONFIG_TWOPROC == 1)
    auto ret = ftl_p->WriteTranslate(lba, ExecCallBack<PageType>());
#else
    auto ret =
        ftl_p->WriteTranslate(lba, FlashSimExecCallBack<PageType>(this));
#endif
    /* Make sure nothing is left in page buffer after translation */
    EnsureStateIsClean();

    /* If the return value is FAILURE then simply return */
    if (ret.first == ExecState::FAILURE) {
      return ExecState::FAILURE;
    }

    /*
     * Push the page into the page buffer for writing
     * Note that the logical LBA is also required in order to
     * associate a physical page with a logical LBA
     */
    page_buffer.push(std::make_pair(page, lba));

    /*
     * And then write the page data using the address returned from
     * the FTL as the result of translation
     */
    ExecuteCommand(OpCode::WRITE, ret.second);

    return ExecState::SUCCESS;
  }

  /*
   * Trim() - Trim a given LBA
   *
   * Trim command tells the FTL that a given LBA doesn't hold useful
   * information any more, and that the FTL is free to use this information
   * to guide itself in cleaning/wear-leveling.
   *
   */
  ExecState Trim(size_t lba) {
    /* Call FTL to trim LBA */
#if (CONFIG_TWOPROC == 1)
    auto ret = ftl_p->Trim(lba, ExecCallBack<PageType>());
#else
    auto ret = ftl_p->Trim(lba, FlashSimExecCallBack<PageType>(this));
#endif
    /* Make sure nothing is left in page buffer after translation */
    EnsureStateIsClean();
    return ret;
  }

  /* Returns the stack size used by FTL */
  size_t GetFTLStackSize(void) { return ftl_p->GetFTLStackSize(); }

  /*
   * Return the total number of operations performed.
   */
  uint64_t TotalOps(OpCode code) {
    switch (code) {
      case OpCode::READ:
        return num_reads;
      case OpCode::WRITE:
        return num_writes;
      case OpCode::ERASE:
        return num_erases;
      default:
        ThrowUnknownOpCodeError(code);
    }

    return 0;
  }

  /*
   * Returns true if at least one block has no erases remaining. This
   * checks that an FTL didn't finish a stress test before it should.
   */
  bool AtLeastOneBlockWornOut() {
    for (auto it = block_erasure_map.begin(); it != block_erasure_map.end();
         ++it) {
      if (it->second == 0) {
        return true;
      }
    }
    return false;
  }

 private:
  /* Functions used internally in class */

  /*
   * UpdateBlockErasure() - Decrease block erasure for a certain block
   * by 1
   *
   * The argument should be a valid block level page address,
   * i.e. pointing to the first page of a block.
   * If this address is malformed assertion would fail
   *
   * We maintain this map in a lazy manner
   * i.e. If an item is missing just create it and initialize it with
   * max erasure; Then we find the item and decrease the count by 1
   */
  void UpdateBlockErasure(size_t block_lba) {
    /* It should be a multiple of page_per_block */
    assert((block_lba % page_per_block) == 0);

    /*
     * We tentatively insert this into the map, and if it fails
     * then simply decrease the count by 1
     */
    const auto &value = std::make_pair(block_lba, block_erase_count);

    /*
     * If the key already exists this will not insert new value
     * but always returns the iterator to the key-value pair
     */
    const auto &ret = block_erasure_map.insert(value);

    /* If the block is already 0 then we are finished */
    if (ret.first->second == 0) {
      ThrowBlockDeadError(block_lba);
    } else {
      /*
       * Decrease the count anyway (we do not care whether
       * the insert succeeded or not since we always need to
       * decrease counter)
       */
      ret.first->second--;
    }

    return;
  }

  /*
   * EnsureStateIsClean() - Make sure the state of the coltroller
   *                        is clean
   *
   * A clean state must have the controller buffer size == 0. Otherwise
   * an exception will be thrown
   */
  void EnsureStateIsClean() {
    /*
     * If the size of page buffer > 0 then we are caching
     * pages in the buffer
     */
    if (page_buffer.size() != 0) {
      ThrowStateNotCleanedError(page_buffer.size());
    }

    return;
  }

  /*
   * ThrowUnknownOpCodeError() - Throws error because we have seen an
   * 			       unknown opcode
   */
  void ThrowUnknownOpCodeError(OpCode op_code) {
    throw FlashSimException("Unknown OpCode: " +
                            std::to_string(static_cast<int>(op_code)));
  }

  /*
   * ThrowStateNotCleanedError() - This is called when ERASE command
   *                               is executed with buffer not empty
   */
  void ThrowStateNotCleanedError(size_t size) {
    throw FlashSimException("State not clean; buffer size = " +
                            std::to_string(size));
  }

  /*
   * ThrowBlockDeadError() - This is called when a block has been worn out
   */
  void ThrowBlockDeadError(size_t block_lba) {
    throw FlashSimException("Block " + std::to_string(block_lba / block_size) +
                            " (linear block ID) has worn out");
  }

  /*
   * ThrowWriteDirtyPageError() - Write on a page that has already been
   *                              written and not erased
   */
  void ThrowWriteDirtyPageError(size_t physical_lba) {
    throw FlashSimException("Write operation on dirty physical page " +
                            std::to_string(physical_lba));
  }

  /*
   * ThrowInvalidReadError() - Write on a page that has already been
   *                              written and not erased
   */
  void ThrowInvalidReadError(size_t physical_lba) {
    throw FlashSimException("Read operation on invalid physical page " +
                            std::to_string(physical_lba));
  }
};

/**************************** class Controller ends ***************************/

/********************** class FlashSimExecCallBack starts *********************/

/*
 * class FlashSimExecCallBack() - Proxy class for controller to let FTL call
 *                        	  its function without exposing controller
 *                        	  internals to the FTL
 *
 * This is the parent side of ExecCallBack
 *
 * The sole purpose of this class is to act as a intermediate to enable
 * communication between parent and child ExecCallBack classes
 */

template <typename PageType>
class FlashSimExecCallBack : public ExecCallBack<PageType> {
 private:
  Controller<PageType> *controller_p;

 public:
  /*
   * Constructor
   */
  FlashSimExecCallBack(Controller<PageType> *p_controller_p)
      : controller_p{p_controller_p} {}

  /*
   * Destructor
   */
  ~FlashSimExecCallBack() {}

  /*
   * operator() - Mimics the behavior of a function call that calls
   *              ExecuteCommand() of class Controller
   */
  void operator()(OpCode operation, Address addr) const {
    controller_p->ExecuteCommand(operation, addr);
  };
};

/*********************** class FlashSimExecCallBack ends **********************/

/************************* class FlashSimTest starts **************************/

/*
 * class FlashSimTest - Test wrapper for conducting read/write tests on
 *                      746FlashSim
 */
class FlashSimTest {
  using PageType = TEST_PAGE_TYPE;

  /* Allow FlashSimFTL to directly access conf */
  friend class FlashSimFTL<PageType>;

 private:
  FlashSimConf conf;
  DataStore<PageType> store;
  FTLBase<PageType> *ftl;
  Controller<PageType> ctrl;

  uint64_t writes_requested;
  uint64_t writes_done;
  uint64_t trims_requested;
  uint64_t trims_done;

  /* Public to allow tests to call this */
 public:
  /*
   * Constructor - Call with the configuration file path
   */
  FlashSimTest(const std::string &fpath)
      : conf(fpath),
        store(MAX_NUM_PAGES),
#if (CONFIG_TWOPROC == 1)
        ftl(CreateFlashSimFTL(this)),
#else
        ftl(CreateMyFTL(&conf)),
#endif
        ctrl(ftl, &store, &conf),
        writes_requested{0},
        writes_done{0},
        trims_requested{0},
        trims_done{0} {
#if ENABLE_TRANS_TRACING
    trans_trace_fp = fopen(TRANS_TRACE_FILE, "w");
    if (trans_trace_fp == NULL) {
      std::cout << "!!! Error opening trace file " << TRANS_TRACE_FILE
                << std::endl;
      ;
      exit(-1);
    }
#endif
  }

  /*
   * Destructor - Freeing memory
   */
  ~FlashSimTest() {
#if ENABLE_TRANS_TRACING
    fclose(trans_trace_fp);
#endif

    /* ftl was created using new and its pointer passed to us */
    delete ftl;
  }

  FTLBase<TEST_PAGE_TYPE> *CreateFlashSimFTL(FlashSimTest *fs_test);

  /*
   * Write() - Testing writing page into the given LBA
   *
   * If return value is 1 then the write operation is successful
   * If return value is 0 then FTL terminates operation, which might
   * be a recoverable error such as block wears out, LBA out of range,
   * etc.
   * If return value is -1 then the controller terminates operation
   * and we have a fatal error (i.e. non recoverable error)
   */
  int Write(FILE *log, size_t addr, const TEST_PAGE_TYPE &buf) {
    if (log) fprintf(log, "----------------\nWriting LBA %zu\n", addr);

    ExecState status;

    try {
      writes_requested++;
      status = ctrl.WriteLBA(buf, addr);

    } catch (FlashSimException &err) {
      std::cout << "!!! Error writing LBA " << addr << " !!!" << std::endl
                << err.what() << std::endl;
      ;
      return -1;
    }

    if (status != ExecState::SUCCESS) {
      if (log) fprintf(log, "LBA %zu not writable\n", addr);
      return 0;
    } else {
      writes_done++;
      if (log) fprintf(log, "LBA %zu written\n", addr);

      return 1;
    }
  }

  /*
   * Read() - Testing reading page into the given LBA
   *
   * Regarding the meanging of return values please refer to Write()
   */
  int Read(FILE *log, size_t addr, TEST_PAGE_TYPE *buf) {
    if (log) fprintf(log, "----------------\nReading LBA %zu\n", addr);

    ExecState status;

    try {
      /*
       * TODO: Check that addr remains same
       * This works as a check that user is doing everything
       * correctly
       */
      status = ctrl.ReadLBA(buf, addr);

    } catch (FlashSimException &err) {
      std::cout << "!!! Error reading LBA " << addr << " !!!" << std::endl
                << err.what();
      return -1;
    }

    if (status != ExecState::SUCCESS) {
      if (log) fprintf(log, "LBA %zu not readable\n", addr);
      return 0;
    } else {
      if (log) fprintf(log, "LBA %zu read\n", addr);

      return 1;
    }
  }

  /*
   * Trim() - Suggesting the FTL to trim a page
   */
  int Trim(FILE *log, size_t addr) {
    if (log) fprintf(log, "----------------\nTrimming LBA %zu\n", addr);

    ExecState status;

    try {
      trims_requested++;
      status = ctrl.Trim(addr);

    } catch (FlashSimException &err) {
      std::cout << "!!! Error trimming LBA " << addr << " !!!" << std::endl
                << err.what();
      return -1;
    }

    if (status != ExecState::SUCCESS) {
      if (log) fprintf(log, "LBA %zu not trimmed\n", addr);

      return 0;

    } else {
      trims_done++;
      if (log) fprintf(log, "LBA %zu trimmed\n", addr);
      return 1;
    }
  }

  int Report(FILE *log) {
    double write_amp = double(TotalWritesPerformed()) / writes_done;
    fprintf(log, "-----------------------------------------------------\n");
    fprintf(log, "WRITES REQUESTED = %lu\n", writes_requested);
    fprintf(log, "WRITES DONE BY YOUR FTL = %lu\n", writes_done);
    fprintf(log, "INTERNAL WRITE_AMPLIFICATION = %f\n", write_amp);
    fprintf(log, "TRIMS REQUESTED = %lu\n", trims_requested);
    fprintf(log, "TRIMS DONE BY YOUR FTL = %lu\n", trims_done);
    fprintf(log, "-----------------------------------------------------\n");

#if MEMCHECK_ENABLED
    /*
     * FIXME: Since we are collecting stack size before the
     * termination of the child, we get a lower reporting
     * of the child's stack. Isn't an issue currently as the extra
     * overhead is probably use to the RPC infrastructure.
     */
    size_t mem_usage = get_child_total_mem(ctrl.GetFTLStackSize());
    printf("Memory usage: %zu bytes\n", mem_usage);

#else
    /* Simplify. If assign zero, then can't divide */
    size_t mem_usage = 1;
    printf(
        "WARNING: Memory check disabled."
        " Defaulting to max mem score.\n");
#endif

    int score;

    double endurance_score, amp_score, mem_score;
    if (is_inf) {
      endurance_score = (double)conf.GetWeightEnduranceInfinite() *
                        (MIN((writes_done * conf.GetWritesThreshold() /
                              conf.GetWritesBaseline()),
                             1.0));
      amp_score =
          (double)conf.GetWeightWriteAmplificationInfinite() *
          (MIN((double)conf.GetWriteAmplificationThreshold() / write_amp, 1.0));
      mem_score = (double)conf.GetWeightMemoryInfinite() *
                  (MIN((double)conf.GetMemoryBaseline() / mem_usage, 1.0));
      score = endurance_score + amp_score + mem_score;

      printf("Endurance Score: %f/%lu\n", endurance_score,
             conf.GetWeightEnduranceInfinite());
      printf("Amp Score: %f/%lu\n", amp_score,
             conf.GetWeightWriteAmplificationInfinite());
      printf("Mem Score: %f/%lu\n", mem_score, conf.GetWeightMemoryInfinite());
      printf("Total Score: %d/%lu\n", score,
             conf.GetWeightEnduranceInfinite() +
                 conf.GetWeightWriteAmplificationInfinite() +
                 conf.GetWeightMemoryInfinite());
    } else {
      /* For finite tests, no need of endurance score, other
       * weights are adjusted accordingly */
      endurance_score = 0;
      amp_score =
          (double)conf.GetWeightWriteAmplificationFinite() *
          (MIN((double)conf.GetWriteAmplificationThreshold() / write_amp, 1.0));
      mem_score = (double)conf.GetWeightMemoryFinite() *
                  (MIN((double)conf.GetMemoryBaseline() / mem_usage, 1.0));
      score = endurance_score + amp_score + mem_score;

      printf("Endurance Score: %f/%d\n", endurance_score, 0);
      printf("Amp Score: %f/%lu\n", amp_score,
             conf.GetWeightWriteAmplificationFinite());
      printf("Mem Score: %f/%lu\n", mem_score, conf.GetWeightMemoryFinite());
      printf("Total Score: %d/%lu\n", score,
             conf.GetWeightWriteAmplificationFinite() +
                 conf.GetWeightMemoryFinite());
    }

    return score;
  }

  /*
   * Return the total number of erase operation performed so far.
   */
  uint64_t TotalErasesPerformed() { return ctrl.TotalOps(OpCode::ERASE); }

  /*
   * Return the total number of write operation performed so far.
   */
  uint64_t TotalWritesPerformed() { return ctrl.TotalOps(OpCode::WRITE); }

  /*
   * Returns true if at least one block has no erases remaining. This
   * checks that an FTL didn't finish a stress test before it should.
   */
  bool AtLeastOneBlockWornOut() { return ctrl.AtLeastOneBlockWornOut(); }
};

/************************** class FlashSimTest ends ***************************/

/************************** class FlashSimFTL starts **************************/

/*
 * FlashSimFTL - This is the parent side FTL
 *
 * The sole purpose of this class is to act as a intermediate to enable
 * communication between parent and child FTLs
 *
 * This class doesn't implement any actual functionality of the
 * Flash Translation Layer
 *
 * To known more about this class, look at the definition of the base class
 */
template <typename PageType>
class FlashSimFTL : public FTLBase<PageType> {
 private:
  FlashSimTest *fs_test;

  /*
   * Make all interface classes public so that class Controller
   * has access to them
   */

 public:
  FlashSimFTL(FlashSimTest *fs_test) : fs_test(fs_test){};

  /*
   * The destructor must be made virtual to make deleting the object
   * through base class pointer possible without any memory leak
   */
  ~FlashSimFTL(){};

  void Print() {}

  std::pair<ExecState, Address> ReadTranslate(size_t lba,
                                              const ExecCallBack<PageType> &) {
    IPC_Format tx_msg, rx_msg;

    tx_msg.owner_ = OWNER_FLASHSIM;
    tx_msg.type_ = MSG_FTL_INSTR_READ;
    tx_msg.lba_ = lba;

    /* Send the IPC message to FTL and get response */
    SendReqToFtl(&tx_msg, &rx_msg);

    return std::make_pair(rx_msg.ftl_resp_execstate_, rx_msg.ftl_resp_addr_);
  }

  std::pair<ExecState, Address> WriteTranslate(size_t lba,
                                               const ExecCallBack<PageType> &) {
    IPC_Format tx_msg, rx_msg;

    tx_msg.owner_ = OWNER_FLASHSIM;
    tx_msg.type_ = MSG_FTL_INSTR_WRITE;
    tx_msg.lba_ = lba;

    /* Send the IPC message to FTL and get response */
    SendReqToFtl(&tx_msg, &rx_msg);

    return std::make_pair(rx_msg.ftl_resp_execstate_, rx_msg.ftl_resp_addr_);
  }

  ExecState Trim(size_t lba, const ExecCallBack<PageType> &) {
    IPC_Format tx_msg, rx_msg;

    tx_msg.owner_ = OWNER_FLASHSIM;
    tx_msg.type_ = MSG_FTL_INSTR_TRIM;
    tx_msg.lba_ = lba;

    /* Send the IPC message to FTL and get response */
    SendReqToFtl(&tx_msg, &rx_msg);

    return rx_msg.ftl_resp_execstate_;
  }

  /* Returns the FTLs stack size used till now */
  size_t GetFTLStackSize(void) {
    IPC_Format tx_msg, rx_msg;

    tx_msg.owner_ = OWNER_FLASHSIM;
    tx_msg.type_ = MSG_FTL_STACK_SIZE_REQ;

    /* Send the IPC message to FTL and get response */
    SendReqToFtl(&tx_msg, &rx_msg);

    return rx_msg.child_stack_size_;
  }

 private:
  /*
   * SendChildBytes - Sends the child process bytes over pipe (IPC)
   *
   * If failure here, probably can't continue, so terminate
   *
   * buf - Buffer which hold's data to send
   * size - Size of data in buffer in bytes
   */
  void SendChildBytes(void *buf, size_t size) {
    ssize_t ret = write(Common.pipefd[PIPE_TX_END], buf, size);
    if (ret < 0) {
      perror("FATAL: Couldn't send child data");
      assert(0 && "Failure in writing to child's rx pipe");
    }
  }

  /*
   * RecvChildBytes - Recevives from child process bytes over pipe (IPC)
   *
   * If failure here, probably can't continue, so terminate
   *
   * buf - Buffer which hold's data to send
   * size_t - Size of data that buffer can hold
   *
   * Returns the actual bytes reveived
   */
  size_t RecvChildBytes(void *buf, size_t size) {
    ssize_t ret = read(Common.pipefd[PIPE_RX_END], buf, size);
    if (ret < 0) {
      perror("FATAL: Couldn't recv child data");
      assert(0 && "Failure in reading from child's tx pipe");
    }

    if (ret == 0) assert(0 && "Did child died?");

    return ret;
  }

  /*
   * IsRecMsgPending - Indicates if any read messages are pending
   *
   * Returns 1 if pending else 0
   */
  int IsRecvMsgPending(void) {
    IPC_Format msg;
    struct pollfd pollfd;
    int ret;

    pollfd.fd = Common.pipefd[PIPE_RX_END];
    pollfd.events = POLLIN;

    do {
      ret = poll(&pollfd, 1, 0);

    } while (ret < 0 && errno == EINTR);

    if (ret < 0) {
      perror("FATAL: Poll failed");
      assert(0 && "Poll failed on pipe read");
    }

    if (ret == 0) /* empty */
      return 0;
    else
      return 1;
  }

  /*
   * RecvMsgFromFtl - Receives message from the ftl (child)
   *
   * should_block - Block or nonblock
   * rx_msg - Pointer to where to store the received message.
   * Returns empty message if no mesaage is in pipe
   */
  void RecvMsgFromFtl(IPC_Format *rx_msg, int should_block) {
    int ret;
    size_t size;
    int timeout;
    struct pollfd pollfd;

    pollfd.fd = Common.pipefd[PIPE_RX_END];
    pollfd.events = POLLIN;

    if (should_block) /* Infinite timeout */
      timeout = -1;
    else
      /* Non blocking */
      timeout = 0;

    do {
      ret = poll(&pollfd, 1, timeout);

    } while (ret < 0 && errno == EINTR);

    if (ret < 0) {
      perror("FATAL: Poll failed");
      assert(0 && "Poll failed on pipe read");
    }

    if (ret == 0) {
      /* No data to read - Return empty message */
      assert(should_block == 0 && "No data even on blocking poll");
      rx_msg->type_ = MSG_EMPTY;
      return;

    } else {
      /* Copy data */
      size = RecvChildBytes((void *)rx_msg, sizeof(*rx_msg));
      if (size != sizeof(*rx_msg)) assert(0 && "Unknown message size");

      /* Message should be from child */
      assert(rx_msg->owner_ == OWNER_FTL && "Unknown owner_");
      return;
    }
  }

  /*
   * SendMsgToFtl - Send messages to the ftl (child)
   *
   * tx_msg - Pointer to the message.
   */
  void SendMsgToFtl(IPC_Format *tx_msg) {
    /* The message to be transmitted must come from flashsim here */
    assert((tx_msg->owner_ == OWNER_FLASHSIM) && "Unknown owner_");

    SendChildBytes((void *)tx_msg, sizeof(*tx_msg));
  }

  /*
   * ProcessRequest - Process and replies to the request pending in pipe
   * 		     from FTL
   *
   * recv_msg - 	Return last received non-request msg
   * 		(empty msg or resonse)
   * should_block - Should read block if no messages in read pipe?
   *
   * This function ends when either pipe is empty, or pipe now contains a
   * response from FTL
   */
  void ProcessRequests(IPC_Format *recv_msg, int should_block) {
    IPC_Format send_msg;
    OpCode sim_req_opcode;
    Address sim_req_addr;

    send_msg.owner_ = OWNER_FLASHSIM;

    while (true) {
      RecvMsgFromFtl(recv_msg, should_block);

      switch (recv_msg->type_) {
        /* Not usefule. Just initial handshake */
        case MSG_FTL_WAKEUP:
          continue;

        case MSG_EMPTY:
          /* Pipe empty */
          return;

        /* FTL asks for various configurations */
        case MSG_CONF_REQ_SSD_SIZE:
          send_msg.type_ = MSG_CONF_RES_SSD_SIZE;
          send_msg.conf_resp_ = fs_test->conf.GetSSDSize();
          break;

        case MSG_CONF_REQ_PACKAGE_SIZE:
          send_msg.type_ = MSG_CONF_RES_PACKAGE_SIZE;
          send_msg.conf_resp_ = fs_test->conf.GetPackageSize();
          break;

        case MSG_CONF_REQ_DIE_SIZE:
          send_msg.type_ = MSG_CONF_RES_DIE_SIZE;
          send_msg.conf_resp_ = fs_test->conf.GetDieSize();
          break;

        case MSG_CONF_REQ_PLANE_SIZE:
          send_msg.type_ = MSG_CONF_RES_PLANE_SIZE;
          send_msg.conf_resp_ = fs_test->conf.GetPlaneSize();
          break;

        case MSG_CONF_REQ_BLOCK_SIZE:
          send_msg.type_ = MSG_CONF_RES_BLOCK_SIZE;
          send_msg.conf_resp_ = fs_test->conf.GetBlockSize();
          break;

        case MSG_CONF_REQ_BLOCK_ERASES:
          send_msg.type_ = MSG_CONF_RES_BLOCK_ERASES;
          send_msg.conf_resp_ = fs_test->conf.GetBlockEraseCount();
          break;

        case MSG_CONF_REQ_OVERPROVISIONING:
          send_msg.type_ = MSG_CONF_RES_OVERPROVISIONING;
          send_msg.conf_resp_ = fs_test->conf.GetOverprovisioning();
          break;

        case MSG_CONF_REQ_GCPOLICY:
          send_msg.type_ = MSG_CONF_RES_GCPOLICY;
          send_msg.conf_resp_ = fs_test->conf.GetGCPolicy();
          break;

        /* FTL asks for simulation services */
        case MSG_SIM_REQ_READ:  /* Fall through */
        case MSG_SIM_REQ_WRITE: /* Fall through */
        case MSG_SIM_REQ_ERASE: /* Fall through */

          sim_req_opcode = recv_msg->sim_req_opcode_;
          sim_req_addr = recv_msg->sim_req_addr_;

          fs_test->ctrl.ExecuteCommand(sim_req_opcode, sim_req_addr);

          send_msg.type_ = MSG_EMPTY;
          break;

        /* Various responses */
        case MSG_FTL_READ_RESP:
          return;

        case MSG_FTL_WRITE_RESP:
          return;

        case MSG_FTL_TRIM_RESP:
          return;

        case MSG_FTL_STACK_SIZE_RESP:
          return;

        default:
          assert(0 && "Unknown message from FTL");
      } /* Switch */

      /* Send the response now */
      SendMsgToFtl(&send_msg);
    } /* Switch */
  }

  /*
   * SendReqToFtl - Sends request to the ftl (child)
   *
   * tx_msg - The message (request) to send to ftl
   * rx_msg - The response of the message
   *
   * Returns another msg that is the response of the msg from ftl
   */
  void SendReqToFtl(IPC_Format *tx_msg, IPC_Format *rx_msg) {
    /* Expected message type_ of the response to msg transmitted */
    enum message_type_t exp_rx_typ;

    switch (tx_msg->type_) {
      case MSG_FTL_INSTR_READ:
        exp_rx_typ = MSG_FTL_READ_RESP;
        break;

      case MSG_FTL_INSTR_WRITE:
        exp_rx_typ = MSG_FTL_WRITE_RESP;
        break;

      case MSG_FTL_INSTR_TRIM:
        exp_rx_typ = MSG_FTL_TRIM_RESP;
        break;

      case MSG_FTL_STACK_SIZE_REQ:
        exp_rx_typ = MSG_FTL_STACK_SIZE_RESP;
        break;

      default:
        assert(0 && "Unknown msg typ");
    }

    /* Send the child request */
    SendMsgToFtl(tx_msg);

    while (1) {
      /* Now process request */
      ProcessRequests(rx_msg, 1);

      if (rx_msg->type_ == MSG_EMPTY)
        continue;
      else if (rx_msg->type_ == exp_rx_typ)
        break;
      else
        assert(0 && "Unknown response received");
    }

    /* Received message is the response */
  }
};
