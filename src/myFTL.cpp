/**
 * @file myFTL.cpp
 * @brief This file contains the implementation of MyFTL class.
 *
 * Implements following functions:
 * - ReadTranslate: Translates LBAs to physical addresses for read operations
 * - WriteTranslate: Translates LBAs to physical addresses for
 *                     write operations
 * - Trim: Trims a lba
 * 
 * Key Features:
 * Uses a page-to-page mapping scheme.
 * Uses a Cost-Benefit-Ratio based garbage collection policy.
 * Uses cold data migration to improve wear leveling.
 * Uses an implicit LRU list for quickly finding cold blocks.
 *
 * @author Cundao Yu <cundaoy@andrew.cmu.edu>
 */
#include "myFTL.h"

#include "common.h"

#define DEBUG false

#if DEBUG 
#define DEBUG_INST(inst) inst;
#else
#define DEBUG_INST(inst) ;
#endif

/** Invalid page address */
#define INVALID_PAGE 0xFFFF

/** 
 * Cold Data Migration threshold ratio
 *
 * Max erases * this ratio is the threshold for cold data migration
 */
#define MIGRATE_THRESHOLD_RATIO 0.2

/** minimum age of a cold block
 *
 * One block with age equal to or larger than this threshold will be
 * considered as a cold block
 */
#define MIN_AGE_COLD 100000

/** Minimum valid count proportion of cold block for migration
 *
 * if the valid page count of a cold block is less than this proportion,
 * it will not be considered for migration 
 */
#define MIN_VALID_PROPORTION_MIGRATION 0.9375

/**
 * class MyFTL - MyFTL implementation
 *
 * Implements ReadTranslate, WriteTranslate and Trim functions
 */
template <typename PageType>
class MyFTL : public FTLBase<PageType> {

  /**
   * struct MyAddress - Compact format of page address
   * 
   * Represents a physical page address in a compact format.
   */
  struct MyAddress {
    /** Raw address */
    uint16_t addr_;

    /**
     * Constructor
     */
    MyAddress() : addr_(INVALID_PAGE) {}

    /**
     * Check if the address is valid
     *
     * @return true if the address is valid
     */
    inline bool is_valid() const {
      return addr_ != INVALID_PAGE;
    }

    /**
     * Get block id
     *
     * Valid block id is from 0 to total block number - 1
     * @return block id
     */
    inline uint16_t get_block_id() const {
      return addr_ >> page_idx_bit_num_;
    }

    /**
     * Get page index
     *
     * Page index inside the block, from 0 to block size - 1
     * @return page index
     */
    inline uint16_t get_page_idx() const {
      return addr_ & page_idx_mask_;
    }

    /**
     * Set this address with block id and page index
     *
     * @param block_id block id
     * @param page_idx page index
     */
    inline void set(uint16_t block_id, uint16_t page_idx) {
      addr_ = (block_id << page_idx_bit_num_) | page_idx;
    }

    /**
     * Set this address with a LBA
     *
     * @param lba LBA
     */
    inline void set(uint16_t lba) {
      addr_ = lba;
    }

    /**
     * Set this address invalid.
     */
    inline void set_invalid() {
      addr_ = INVALID_PAGE;
    }
  };

  /**
   * struct PhysicalBlock - Physical block
   *
   * Stores internal information of a physical block.
   */
  struct PhysicalBlock {
    /**
     * Mapped logical pages
     * 
     * Map physical pages to LBAs.
     * Stores Address objects representing mapped LBAs for each physical pages
     * in this block. Size of this vector is equal to block size. Access this
     * vector using the indices of physical pages in this physical block
     * (from 0 to block size - 1).
     * If the MyAddress object at an index is invalid, it means the physical
     * page of this index is empty or stores outdated data.
     */
    std::vector<MyAddress> mapped_logical_pages_;

    /**
     * Next empty page index
     *
     * Start from 0, increase by 1 when a page is written.
     * Empty means the page is not written yet.
     */
    uint8_t next_empty_page_idx_;

    /**
     * Remaining erases
     */
    uint8_t erases_;

    /**
     * Valid page count
     */
    uint16_t valid_page_count_;

    /**
     * Prev pointer in a implicit LRU list
     *
     * Access physical_blocks_ in MyFTL class using prev block id
     * to get the PhysicalBlock object of the prev block.
     * -1: null, >=0: prev block id
     */
    int16_t lru_prev_block_id_;

    /**
     * Next pointer in a implicit LRU list
     *
     * Access physical_blocks_ in MyFTL class using next block id
     * to get the PhysicalBlock object of the next block.
     * -1: null, >=0: next block id
     */
    int16_t lru_next_block_id_;

    /**
     * Timestamp
     */
    size_t ts_;

    /**
     * Constructor
     */
    PhysicalBlock() : next_empty_page_idx_(0), erases_(block_erases_)
      , valid_page_count_(0), ts_(0) {
      mapped_logical_pages_.resize(block_capacity_);
    }

    /**
     * Check if the block has empty page
     *
     * @return true if the block has empty page
     */
    inline bool has_empty_page() {
      return next_empty_page_idx_ == block_capacity_;
    }

    /**
     * Update related internal states and return the page index for writing on
     *
     * @param lba lba
     * @return page index for writing on
     */
    inline uint16_t write_page(uint16_t lba) {
      mapped_logical_pages_[next_empty_page_idx_].set(lba);
      valid_page_count_++;
      ts_ = current_ts_++;
      return next_empty_page_idx_++;
    }

    /**
     * Update related internal states when a page is moved 
     * 
     * @param page_idx page index
     */
    inline void page_moved(uint16_t page_idx) {
      mapped_logical_pages_[page_idx].set_invalid();
      valid_page_count_--;
    }

    /**
     * Check if the block is worn out
     *
     * @return true if the block is worn out
     */
    inline bool is_worn_out() {
      return erases_ == 0;
    }

    /**
     * Check if the block is cold
     *
     * @return true if the block is cold
     */
    inline bool is_cold() {
      return current_ts_ - ts_ >= MIN_AGE_COLD;
    }

    /**
     * Reset related internal states when the block is erased
     */
    void erase() {
      next_empty_page_idx_ = 0;
      erases_--;
      valid_page_count_ = 0;
      ts_ = current_ts_;
      for(auto& addr : mapped_logical_pages_) {
        addr.set_invalid();
      }
    }

    /**
     * Update related internal states when a page is trimmed
     *
     * @param page_idx page index
     */
    inline void trim(uint16_t page_idx) {
      mapped_logical_pages_[page_idx].set_invalid();
      valid_page_count_--;
    }
  };

  /** Number of packages in a ssd */
  size_t ssd_size_;
  /** Number of dies in a package_ */
  size_t package_size_;
  /** Number of planes in a die_ */
  size_t die_size_;
  /** Number of blocks in a plane_ */
  size_t plane_size_;
  /** Number of pages in a block_ */
  size_t block_size_;
  /** Overprovioned blocks as a percentage of total number of blocks */
  size_t op_;
  /** Initial block erases */
  static size_t block_erases_;

  /** Number of pages in a block */
  static size_t block_capacity_;
  /** Number of pages in a plane */
  size_t plane_capacity_;
  /** Number of pages in a die */
  size_t die_capacity_;
  /** Number of pages in a package */
  size_t package_capacity_;
  /** Number of pages in a ssd */
  size_t ssd_capacity_;

  /** Number of bits used to represent page index */
  static size_t page_idx_bit_num_;
  /** Mask to get page index from a raw address(lba or pba) */
  static uint16_t page_idx_mask_;

  /** 
   * LBA to physical page address mapping
   *
   * Map LBAs to physical pages.
   * Size of this vector is equal to total number of LBAs.
   * Access this vector using LBA as index.
   * If the address object of a LBA is invalid, means this LBA
   * is not written yet
   */
  std::vector<MyAddress> lba_to_physical_page_;
  /** 
   * Physical blocks 
   *
   * Stores PhysicalBlock objects. Each store internal states of a
   * physical block.
   * Size of this vector is equal to total number of physical blocks.
   * Access this vector using block id as index.
   */
  std::vector<PhysicalBlock> physical_blocks_;

  /** Id of current block used for writing, all writes go here */
  uint16_t writing_block_id_;
  /** 
   * Id of reserved block for cleaning. 
   *
   * When doing Garbage Collection, data in cleaned block will
   * be moved to this block, and then the cleaning block will
   * be the new writing block.
   */
  uint16_t cleaning_block_id_;
  /** 
   * Next initially empty block id. 
   * 
   * After all initially empty blocks are used,
   * we need to do garbage collection to get a new empty block 
   */
  uint16_t next_init_empty_block_id_;

  /** Implicit LRU list head, which is the most recently used block */
  int16_t lrulist_head_;
  /** Implicit LRU list tail, which is the least recently used block */
  int16_t lrulist_tail_;

  /** Current timestamp */
  static size_t current_ts_;

  /** Threshold for cold data migration */
  size_t migrate_threshold_;

 public:
  /*
   * Constructor
   */
  MyFTL(const ConfBase *conf) {
    ssd_size_ = conf->GetSSDSize();
    package_size_ = conf->GetPackageSize();
    die_size_ = conf->GetDieSize();
    plane_size_ = conf->GetPlaneSize();
    block_size_ = conf->GetBlockSize();
    op_ = conf->GetOverprovisioning();
    block_erases_ = conf->GetBlockEraseCount();

    block_capacity_ = block_size_;
    plane_capacity_ = block_capacity_ * plane_size_;
    die_capacity_ = plane_capacity_ * die_size_;
    package_capacity_ = die_capacity_ * package_size_;
    ssd_capacity_ = package_capacity_ * ssd_size_;

    /* Calculate page index bit number and mask */
    page_idx_bit_num_ = 0;
    while(((size_t)1 << page_idx_bit_num_) < block_size_) {
      page_idx_bit_num_++;
    }
    page_idx_mask_ = (1 << page_idx_bit_num_) - 1;

    /* Initialize lba to physical page mapping, lba space is calculated based
     on ssd capacity and overprovisioning */
    lba_to_physical_page_.resize(ssd_capacity_ * ((double)(100 - op_) / 100));

    /* Initialize physical blocks, size is the number of physical blocks */
    physical_blocks_.resize(ssd_capacity_ / block_capacity_);

    /* Initialize implicit LRU list */
    physical_blocks_[0].lru_prev_block_id_ = -1;
    physical_blocks_[0].lru_next_block_id_ = 1;
    for(size_t i = 1; i < physical_blocks_.size() - 1; i++) {
      physical_blocks_[i].lru_prev_block_id_ = i - 1;
      physical_blocks_[i].lru_next_block_id_ = i + 1;
    }
    physical_blocks_[physical_blocks_.size() - 1].lru_prev_block_id_
      = physical_blocks_.size() - 2;
    physical_blocks_[physical_blocks_.size() - 1].lru_next_block_id_ = -1;
    lrulist_head_ = 0;
    lrulist_tail_ = physical_blocks_.size() - 1;

    writing_block_id_ = 1;
    cleaning_block_id_ = 0;
    next_init_empty_block_id_ = 2;

    current_ts_ = 0;

    /* Calculate threshold for cold data migration */
    migrate_threshold_ = (size_t)(block_erases_ * MIGRATE_THRESHOLD_RATIO);

    printf("SSD Configuration: %zu, %zu, %zu, %zu, %zu\n", ssd_size_,
           package_size_, die_size_, plane_size_, block_size_);
    printf("Capacity: %zu, %zu, %zu, %zu, %zu\n", ssd_capacity_,
           package_capacity_, die_capacity_, plane_capacity_
           , block_capacity_);
    printf("Max Erase Count: %zu, Overprovisioning: %zu%%\n", block_erases_
      , op_);
  }

  /*
   * Destructor - Plase keep it as virtual to allow destroying the
   *              object with base type_ pointer
   */
  virtual ~MyFTL() {}

  /*
   * ReadTranslate() - Translates read address
   *
   * This function translates a physical LBA into an Address object that will
   * be used as the target address of the read operation.
   *
   * If you need to issue extra operations, please use argument func to
   * interact with class Controller
   */
  std::pair<ExecState, Address> ReadTranslate(
      size_t lba, const ExecCallBack<PageType> &func) {
    (void)func;

    DEBUG_INST(std::cout << "[ReadTranslate] [Entry] read lba: " << lba
      << std::endl)
    if(!is_valid_lba(lba)) {
      // out of user lba space
      DEBUG_INST(std::cout << "[ReadTranslate] [Error] out of user lba space"
        << std::endl)
      return std::make_pair(ExecState::FAILURE, Address());
    }

    auto& physical_addr = lba_to_physical_page_[lba]; // get physical address
    if(!physical_addr.is_valid()) {
      // try to read from invalid page
      DEBUG_INST(std::cout << "[ReadTranslate] [Error] read from invalid page"
        << std::endl)
      return std::make_pair(ExecState::FAILURE, Address());
    }
    
    // successful read
    DEBUG_INST(std::cout << "[ReadTranslate] [Success] read from block: "
      << physical_addr.get_block_id() << ", page: "
      << physical_addr.get_page_idx() << std::endl)
    return std::make_pair(ExecState::SUCCESS, get_address(
      physical_addr.addr_));
  }

  /*
   * WriteTranslate() - Translates write address
   *
   * Please refer to ReadTranslate()
   */
  std::pair<ExecState, Address> WriteTranslate(
      size_t lba, const ExecCallBack<PageType> &func) {
    (void)func;

    DEBUG_INST(std::cout << "[WriteTranslate] [Entry] write lba: " << lba
      << std::endl)
    if(!is_valid_lba(lba)) {
      // out of user lba space
      DEBUG_INST(std::cout
        << "[WriteTranslate] [Error] out of user lba space" << std::endl)
      return std::make_pair(ExecState::FAILURE, Address());
    }

    if(physical_blocks_[writing_block_id_].has_empty_page()) {
      if(has_init_empty_block()) {
        // choose next initially empty block
        DEBUG_INST(std::cout
          << "[WriteTranslate] [Info] choose next initially empty block: "
          << next_init_empty_block_id_ << std::endl)
        writing_block_id_ = get_next_init_empty_block();
      } else {
        // do garbage collection
        DEBUG_INST(std::cout
          << "[WriteTranslate] [Info] do garbage collection" << std::endl)
        if(!garbage_collect(func)) {
          // garbage collection failed
          DEBUG_INST(std::cout
            << "[WriteTranslate] [Failed] garbage collection failed"
            << std::endl)
          return std::make_pair(ExecState::FAILURE, Address());
        }
      }
    }

    auto& physical_addr = lba_to_physical_page_[lba];
    auto& writing_block = physical_blocks_[writing_block_id_];
    if(physical_addr.is_valid()) {
      // modify old mapping
      physical_blocks_[physical_addr.get_block_id()].page_moved(
        physical_addr.get_page_idx());
      DEBUG_INST(std::cout << "[WriteTranslate] [Info] old mapping: block: "
        << physical_addr.get_block_id() << ", page: "
        << physical_addr.get_page_idx() << std::endl)
    }

    // map to new physical page
    physical_addr.set(writing_block_id_, writing_block.write_page(lba));
    update_lru(writing_block_id_); // update LRU list
    DEBUG_INST(std::cout << "[WriteTranslate] [Success] write to block: "
      << physical_addr.get_block_id() << ", page: "
      << physical_addr.get_page_idx() << std::endl)
    return std::make_pair(ExecState::SUCCESS, get_address(
      physical_addr.addr_));
  }

  /*
   * Optionally mark a LBA as a garbage.
   */
  ExecState Trim(size_t lba, const ExecCallBack<PageType> &func) {
    (void)func;

    DEBUG_INST(std::cout << "[Trim] [Entry] trim lba: " << lba << std::endl)
    if(!is_valid_lba(lba)) {
      // out of user lba space
      DEBUG_INST(std::cout << "[Trim] [Error] out of user lba space"
        << std::endl)
      return ExecState::FAILURE;
    }

    auto& physical_addr = lba_to_physical_page_[lba];
    if(!physical_addr.is_valid()) {
      // try to trim invalid page
      DEBUG_INST(std::cout << "[Trim] [Error] trim invalid page" << std::endl)
      return ExecState::FAILURE;
    }

    // trim the page
    physical_blocks_[physical_addr.get_block_id()].trim(
      physical_addr.get_page_idx());
    physical_addr.set_invalid();
    DEBUG_INST(std::cout << "[Trim] [Success] trim block: "
      << physical_addr.get_block_id() << ", page: "
      << physical_addr.get_page_idx() << std::endl)
    return ExecState::SUCCESS;
  }

 private:
  /**
   * Check if the lba is valid
   *
   * @param lba lba
   * @return true if the lba is valid
   */
  inline bool is_valid_lba(size_t lba) {
    return lba < lba_to_physical_page_.size();
  }

  /**
   * Check if there is initially empty block
   *
   * @return true if there is initially empty block
   */
  inline bool has_init_empty_block() {
    return next_init_empty_block_id_ < physical_blocks_.size();
  }

  /**
   * Get next initially empty block id
   *
   * @return next initially empty block id
   */
  inline uint16_t get_next_init_empty_block() {
    return next_init_empty_block_id_++;
  }

  /**
   * Update LRU list
   *
   * Move the block to the head of the LRU list
   * @param block_id block id
   */
  void update_lru(uint16_t block_id) {
    if(lrulist_head_ == block_id) {
      // already at head
      return;
    }

    auto& block = physical_blocks_[block_id];
    if(lrulist_tail_ == block_id) {
      auto& prev_block = physical_blocks_[block.lru_prev_block_id_];
      prev_block.lru_next_block_id_ = -1;
      lrulist_tail_ = block.lru_prev_block_id_;
    } else {
      auto& prev_block = physical_blocks_[block.lru_prev_block_id_];
      auto& next_block = physical_blocks_[block.lru_next_block_id_];
      prev_block.lru_next_block_id_ = block.lru_next_block_id_;
      next_block.lru_prev_block_id_ = block.lru_prev_block_id_;
    }

    block.lru_prev_block_id_ = -1;
    block.lru_next_block_id_ = lrulist_head_;
    physical_blocks_[lrulist_head_].lru_prev_block_id_ = block_id;
    lrulist_head_ = block_id;
  }

  /**
   * Get an Address object representing a lba
   *
   * @param lba lba
   * @return Address object
   */
  Address get_address(size_t lba) {
    auto package = lba / package_capacity_;
    lba %= package_capacity_;
    auto die = lba / die_capacity_;
    lba %= die_capacity_;
    auto plane = lba / plane_capacity_;
    lba %= plane_capacity_;
    auto block = lba / block_capacity_;
    auto page = lba % block_capacity_;
    return Address(package, die, plane, block, page);
  }

  /**
   * Find the least recently used cold block
   *
   * Find a suitable cold block for a block.
   * The data in the cold block will be migrated to the block.
   * @param block_id block id
   * @return cold block id
   */
  inline int16_t find_cold_block(uint16_t block_id) {
    // search from the tail of the LRU list
    auto cold_block_id = lrulist_tail_;
    
    /* A suitable cold block must satisfy the following conditions:
     * 1. not the block itself
     * 2. not worn out
     * 3. valid page count is equal to or larger than a proportion
     *    of the block capacity
     * 4. erase count is larger than target block's erase count
     */
    while(cold_block_id == block_id 
        || physical_blocks_[cold_block_id].is_worn_out() 
        || physical_blocks_[cold_block_id].valid_page_count_ 
          < block_capacity_ * MIN_VALID_PROPORTION_MIGRATION
        || physical_blocks_[cold_block_id].erases_ 
          <= physical_blocks_[block_id].erases_) {
      if(!physical_blocks_[cold_block_id].is_cold()) {
        // the block is not cold, later blocks are also not cold
        return -1;
      }

      // move to the previous block
      cold_block_id = physical_blocks_[cold_block_id].lru_prev_block_id_;
      if(cold_block_id == -1) {
        // reach the top end of the LRU list
        break;
      }
    }
    return cold_block_id;
  }

  /**
   * Pick a victim block for garbage collection
   *
   * Pick a victim block for garbage collection
   * based on the Cost-Benefit-Ratio policy.
   * @return victim block id
   */
  uint16_t pick_victim_block() {
    double max_cost_benefit_ratio = -1; // max cost-benefit ratio
    uint16_t victim_block_id = physical_blocks_.size(); // victim block id

    // traverse all blocks to find the block with max cost-benefit ratio
    for(size_t i = 0; i < physical_blocks_.size(); i++) {
      if(i == cleaning_block_id_) {
        // skip the cleaning block
        continue;
      }
      auto& block = physical_blocks_[i];
      if(block.is_worn_out()) {
        // skip worn out blocks
        continue;
      }

      // calculate the cost-benefit ratio
      double utilization = block.valid_page_count_ / (double)block_capacity_;
      double cost_benefit_ratio = (1 - utilization) / (1 + utilization)
        * (current_ts_ - block.ts_);

      // adjust the ratio based on remaining erases
      double ration_adjustment = block.erases_ / (double)block_erases_;
      cost_benefit_ratio *= ration_adjustment; 

      // update the max cost-benefit ratio and victim block id
      if(cost_benefit_ratio > max_cost_benefit_ratio) {
        max_cost_benefit_ratio = cost_benefit_ratio;
        victim_block_id = i;
      } else if(cost_benefit_ratio == max_cost_benefit_ratio
          && block.valid_page_count_ 
            < physical_blocks_[victim_block_id].valid_page_count_) {
        // we prefer the block with less valid pages if the cost-benefit
        // ratio is the same
        victim_block_id = i;
      }
    }
    if(victim_block_id != physical_blocks_.size()
        && physical_blocks_[victim_block_id].valid_page_count_
          == block_capacity_) {
      // if the found victim block is full, no benefit from cleaning it,
      // return a invalid block id
      return physical_blocks_.size();
    }
    return victim_block_id;
  }

  /**
   * Migrate cold data from a cold block to a block
   *
   * Migrate cold data from a cold block to a block.
   * @param block_id block id
   * @param func function to interact with class Controller
   * @return cold block id
   */
  uint16_t migrate_cold_block(uint16_t block_id
      , const ExecCallBack<PageType> &func) {
    auto cold_block_id = find_cold_block(block_id); // find a cold block
    if(cold_block_id == -1) {
      // no suitable cold block
      return block_id;
    }

    // migrate data from the cold block to the block
    auto& cold_block = physical_blocks_[cold_block_id];
    auto& block = physical_blocks_[block_id];
    for(size_t i = 0; i < cold_block.mapped_logical_pages_.size(); i++) {
      auto& logical_addr = cold_block.mapped_logical_pages_[i];
      if(logical_addr.is_valid()) {
        // migrate valid page
        func(OpCode::READ, get_address(cold_block_id * block_capacity_ + i));
        auto new_physical_page_idx = block.write_page(logical_addr.addr_);
        func(OpCode::WRITE, get_address(block_id * block_capacity_
          + new_physical_page_idx));
        lba_to_physical_page_[logical_addr.addr_].set(
          block_id, new_physical_page_idx);
      }
    }

    // erase the cold block
    func(OpCode::ERASE, get_address(cold_block_id * block_capacity_));
    cold_block.erase();

    update_lru(block_id); // update LRU list
    return cold_block_id;
  }

  /**
   * Clean a victim block
   *
   * Clean a victim block, move data from the block to the cleaning block.
   * @param block_id block id
   * @param func function to interact with class Controller
   * @return block id
   */
  uint16_t clean_block(uint16_t block_id
      , const ExecCallBack<PageType> &func) {
    auto& cleaning_block = physical_blocks_[cleaning_block_id_];
    auto& block = physical_blocks_[block_id];
    for(size_t i = 0; i < block.mapped_logical_pages_.size(); i++) {
      auto& logical_addr = block.mapped_logical_pages_[i];
      if(logical_addr.is_valid()) {
        // move valid page
        func(OpCode::READ, get_address(block_id * block_capacity_ + i));
        auto new_physical_page_idx = cleaning_block.write_page(
          logical_addr.addr_);
        func(OpCode::WRITE, get_address(cleaning_block_id_ * block_capacity_
          + new_physical_page_idx));
        lba_to_physical_page_[logical_addr.addr_].set(
          cleaning_block_id_, new_physical_page_idx);
      }
    }

    // erase the block
    func(OpCode::ERASE, get_address(block_id * block_capacity_));
    block.erase();

    update_lru(cleaning_block_id_); // update LRU list

    if(block.erases_ == migrate_threshold_) {
      // trigger cold data migration, try to find a cold block to migrate data
      // into the victim block
      block_id = migrate_cold_block(block_id, func);
    }
    
    return block_id;
  }

  /**
   * Garbage collection
   *
   * Garbage collection, clean a victim block and move data to the cleaning
   * block. the cleaning block will be the new writing block. And the victim
   * block will be the new cleaning block.
   * @param func function to interact with class Controller
   * @return true if garbage collection is successful
   */
  bool garbage_collect(const ExecCallBack<PageType> &func) {
    DEBUG_INST(std::cout << "[GarbageCollection] [Entry] cleaning block: "
      << cleaning_block_id_ << ", writing block: " << writing_block_id_
      << std::endl)
    auto victim_block_id = pick_victim_block(); // pick a victim block
    if(victim_block_id == physical_blocks_.size()) {
      // get a invalid block id, no block can be erased
      DEBUG_INST(std::cout
        << "[GarbageCollection] [Failed] no block can be erased" << std::endl)
      return false;
    }

    DEBUG_INST(std::cout << "[GarbageCollection] [Info] victim block: "
      << victim_block_id << std::endl)
    // clean the victim block
    victim_block_id = clean_block(victim_block_id, func); 

    // update writing block id and cleaning block id
    writing_block_id_ = cleaning_block_id_;
    cleaning_block_id_ = victim_block_id;
    DEBUG_INST(std::cout << "[GarbageCollection] [Success] cleaning block: "
      << cleaning_block_id_ << ", writing block: " << writing_block_id_
      << std::endl)
    return true;
  }
};

template <typename PageType>
size_t MyFTL<PageType>::block_capacity_ = 0;

template <typename PageType>
size_t MyFTL<PageType>::page_idx_bit_num_ = 0;

template <typename PageType>
uint16_t MyFTL<PageType>::page_idx_mask_ = 0;

template <typename PageType>
size_t MyFTL<PageType>::block_erases_ = 0;

template <typename PageType>
size_t MyFTL<PageType>::current_ts_ = 0;


/*
 * CreateMyFTL() - Creates class MyFTL object
 *
 * You do not need to modify this
 */
FTLBase<TEST_PAGE_TYPE> *CreateMyFTL(const ConfBase *conf) {
  MyFTL<TEST_PAGE_TYPE> *ftl = new MyFTL<TEST_PAGE_TYPE>(conf);
  return static_cast<FTLBase<TEST_PAGE_TYPE> *>(ftl);
}
