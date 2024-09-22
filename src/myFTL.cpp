#include "myFTL.h"

#include <memory>

#include <bitset>

#include "common.h"

#define DEBUG false

#if DEBUG 
#define DEBUG_INST(inst) inst;
#else
#define DEBUG_INST(inst) ;
#endif

/* Page index for a invalid page in a block*/
#define INVALID_PAGE 0xFFFF
/* Throttle threshold, trigger reduction of benefit-cost ratio */
#define THROTTLE_THRESHOLD_RATIO 0.75
/* factor used to reduce the benefit-cost ratio */
#define THROTTLE_REDUCTION_RATIO 0.5

template <typename PageType>
class MyFTL : public FTLBase<PageType> {

  /* Compact format of page address */
  struct MyAddress {
    uint16_t addr;

    MyAddress() : addr(INVALID_PAGE) {}

    inline bool IsValid() const {
      return addr != INVALID_PAGE;
    }

    inline uint16_t BlockId() const {
      return addr >> page_idx_bit_num_;
    }

    inline uint16_t PageIdx() const {
      return addr & page_idx_mask_;
    }

    inline void Set(uint16_t block_id, uint16_t page_idx) {
      addr = (block_id << page_idx_bit_num_) | page_idx;
    }

    inline void Set(uint16_t lba) {
      this->addr = lba;
    }

    inline void SetInvalid() {
      addr = INVALID_PAGE;
    }
  };

  /* Physical block state */
  struct PhysicalBlock {
    std::vector<MyAddress> mapped_logical_pages;
    uint8_t next_free_page_idx;
    uint8_t erase_count;
    uint16_t valid_page_count;
    int16_t lru_prev_block_id; // -1: null, >=0: valid
    int16_t lru_next_block_id; // -1: null, >=0: valid
    size_t ts;

    PhysicalBlock() : next_free_page_idx(0), erase_count(max_erases_), valid_page_count(0), ts(0) {
      mapped_logical_pages.resize(block_capacity_);
    }

    inline bool HasEmptyPage() {
      return next_free_page_idx == block_capacity_;
    }

    inline uint16_t WritePage(uint16_t lba) {
      mapped_logical_pages[next_free_page_idx].Set(lba);
      valid_page_count++;
      ts = current_ts_++;
      return next_free_page_idx++;
    }

    inline void PageMoved(uint16_t page_idx) {
      mapped_logical_pages[page_idx].SetInvalid();
      valid_page_count--;
    }

    inline bool IsWornOut() {
      return erase_count == 0;
    }

    void Erase() {
      next_free_page_idx = 0;
      erase_count--;
      valid_page_count = 0;
      ts = current_ts_;
      for(auto& addr : mapped_logical_pages) {
        addr.SetInvalid();
      }
    }

    inline void Trim(uint16_t page_idx) {
      mapped_logical_pages[page_idx].SetInvalid();
      valid_page_count--;
    }
  };

  /* Number of packages in a ssd */
  size_t ssd_size_;
  /* Number of dies in a package_ */
  size_t package_size_;
  /* Number of planes in a die_ */
  size_t die_size_;
  /* Number of blocks in a plane_ */
  size_t plane_size_;
  /* Number of pages in a block_ */
  size_t block_size_;
  /* Overprovioned blocks as a percentage of total number of blocks */
  size_t op_;
  /* max erases */
  static size_t max_erases_;

  /* Number of pages in a block */
  static size_t block_capacity_;
  /* Number of pages in a plane */
  size_t plane_capacity_;
  /* Number of pages in a die */
  size_t die_capacity_;
  /* Number of pages in a package */
  size_t package_capacity_;
  /* Number of pages in a ssd */
  size_t ssd_capacity_;

  /* number of bits used to represent page index */
  static size_t page_idx_bit_num_;
  /* mask to get page index */
  static uint16_t page_idx_mask_;

  /* lba to physical page mapping */
  std::vector<MyAddress> lba_to_physical_page_;
  /* physical blocks */
  std::vector<PhysicalBlock> physical_blocks_;

  /* current block used for writing */
  uint16_t writing_block_id_;
  /* reserved block for cleaning */
  uint16_t cleaning_block_id_;
  /* next free block */
  uint16_t next_init_empty_block_id_;

  int16_t lrulist_head_;
  int16_t lrulist_tail_;

  /* current ts */
  static size_t current_ts_;

  /* Throttle threshold, trigger reduction of benefit-cost ratio */
  // std::vector<size_t> migrate_thresholds_;
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
    max_erases_ = conf->GetBlockEraseCount();

    block_capacity_ = block_size_;
    plane_capacity_ = block_capacity_ * plane_size_;
    die_capacity_ = plane_capacity_ * die_size_;
    package_capacity_ = die_capacity_ * package_size_;
    ssd_capacity_ = package_capacity_ * ssd_size_;

    page_idx_bit_num_ = 0;
    while(((size_t)1 << page_idx_bit_num_) < block_size_) {
      page_idx_bit_num_++;
    }
    page_idx_mask_ = (1 << page_idx_bit_num_) - 1;

    lba_to_physical_page_.resize(ssd_capacity_ * ((double)(100 - op_) / 100));
    physical_blocks_.resize(ssd_capacity_ / block_capacity_);

    physical_blocks_[0].lru_prev_block_id = -1;
    physical_blocks_[0].lru_next_block_id = 1;
    for(size_t i = 1; i < physical_blocks_.size() - 1; i++) {
      physical_blocks_[i].lru_prev_block_id = i - 1;
      physical_blocks_[i].lru_next_block_id = i + 1;
    }
    physical_blocks_[physical_blocks_.size() - 1].lru_prev_block_id = physical_blocks_.size() - 2;
    physical_blocks_[physical_blocks_.size() - 1].lru_next_block_id = -1;

    writing_block_id_ = 1;
    cleaning_block_id_ = 0;
    next_init_empty_block_id_ = 2;

    lrulist_head_ = 0;
    lrulist_tail_ = physical_blocks_.size() - 1;

    current_ts_ = 0;

    migrate_threshold_ = (size_t)(0.2 * max_erases_);

    printf("SSD Configuration: %zu, %zu, %zu, %zu, %zu\n", ssd_size_,
           package_size_, die_size_, plane_size_, block_size_);
    printf("Capacity: %zu, %zu, %zu, %zu, %zu\n", ssd_capacity_,
           package_capacity_, die_capacity_, plane_capacity_, block_capacity_);
    printf("Max Erase Count: %zu, Overprovisioning: %zu%%\n", max_erases_, op_);
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

    // std::cout << "read: " << lba << std::endl;
    DEBUG_INST(std::cout << "[ReadTranslate] [Entry] read lba: " << lba << std::endl)
    if(!IsValidLBA(lba)) {
      // out of range
      DEBUG_INST(std::cout << "[ReadTranslate] [Error] out of range" << std::endl)
      return std::make_pair(ExecState::FAILURE, Address());
    }

    auto& physical_addr = lba_to_physical_page_[lba];
    if(!physical_addr.IsValid()) {
      // read from invalid page
      DEBUG_INST(std::cout << "[ReadTranslate] [Error] read from invalid page" << std::endl)
      return std::make_pair(ExecState::FAILURE, Address());
    }
    
    // successful read
    DEBUG_INST(std::cout << "[ReadTranslate] [Success] read from block: " << physical_addr.BlockId() << ", page: " << physical_addr.PageIdx() << std::endl)
    return std::make_pair(ExecState::SUCCESS, GetAddress(physical_addr.addr));
  }

  /*
   * WriteTranslate() - Translates write address
   *
   * Please refer to ReadTranslate()
   */
  std::pair<ExecState, Address> WriteTranslate(
      size_t lba, const ExecCallBack<PageType> &func) {
    (void)func;
    // std::cout << "write: " << lba << std::endl;
    DEBUG_INST(std::cout << "[WriteTranslate] [Entry] write lba: " << lba << std::endl)
    if(!IsValidLBA(lba)) {
      // out of range
      DEBUG_INST(std::cout << "[WriteTranslate] [Error] out of range" << std::endl)
      return std::make_pair(ExecState::FAILURE, Address());
    }

    if(physical_blocks_[writing_block_id_].HasEmptyPage()) {
      if(HasInitEmptyBlock()) {
        // choose next initially empty block
        DEBUG_INST(std::cout << "[WriteTranslate] [Info] choose next initially empty block: " << next_init_empty_block_id_ << std::endl)
        writing_block_id_ = GetNextInitEmptyBlock();
      } else {
        // do garbage collection
        DEBUG_INST(std::cout << "[WriteTranslate] [Info] do garbage collection" << std::endl)
        if(!GarbageCollection(func)) {
          // garbage collection failed
          DEBUG_INST(std::cout << "[WriteTranslate] [Failed] garbage collection failed" << std::endl)
          return std::make_pair(ExecState::FAILURE, Address());
        }
      }
    }

    auto& physical_addr = lba_to_physical_page_[lba];
    auto& writing_block = physical_blocks_[writing_block_id_];
    // modify old mapping
    if(physical_addr.IsValid()) {
      physical_blocks_[physical_addr.BlockId()].PageMoved(physical_addr.PageIdx());
      DEBUG_INST(std::cout << "[WriteTranslate] [Info] old mapping: block: " << physical_addr.BlockId() << ", page: " << physical_addr.PageIdx() << std::endl)
    }

    // map to new physical page
    physical_addr.Set(writing_block_id_, writing_block.WritePage(lba));
    UpdateLRU(writing_block_id_);
    DEBUG_INST(std::cout << "[WriteTranslate] [Success] write to block: " << physical_addr.BlockId() << ", page: " << physical_addr.PageIdx() << std::endl)
    return std::make_pair(ExecState::SUCCESS, GetAddress(physical_addr.addr));
  }

  /*
   * Optionally mark a LBA as a garbage.
   */
  ExecState Trim(size_t lba, const ExecCallBack<PageType> &func) {
    (void)func;
    // std::cout << "trim: " << lba << std::endl;
    DEBUG_INST(std::cout << "[Trim] [Entry] trim lba: " << lba << std::endl)
    if(!IsValidLBA(lba)) {
      // out of range
      DEBUG_INST(std::cout << "[Trim] [Error] out of range" << std::endl)
      return ExecState::FAILURE;
    }

    auto& physical_addr = lba_to_physical_page_[lba];
    if(!physical_addr.IsValid()) {
      // trim invalid page
      DEBUG_INST(std::cout << "[Trim] [Error] trim invalid page" << std::endl)
      return ExecState::FAILURE;
    }

    physical_blocks_[physical_addr.BlockId()].Trim(physical_addr.PageIdx());
    physical_addr.SetInvalid();
    DEBUG_INST(std::cout << "[Trim] [Success] trim block: " << physical_addr.BlockId() << ", page: " << physical_addr.PageIdx() << std::endl)
    return ExecState::SUCCESS;
  }

 private:
  inline bool IsValidLBA(size_t lba) {
    return lba < lba_to_physical_page_.size();
  }

  inline bool HasInitEmptyBlock() {
    return next_init_empty_block_id_ < physical_blocks_.size();
  }

  inline uint16_t GetNextInitEmptyBlock() {
    return next_init_empty_block_id_++;
  }

  void UpdateLRU(uint16_t block_id) {
    if(lrulist_head_ == block_id) {
      // already at head
      return;
    }

    auto& block = physical_blocks_[block_id];
    if(lrulist_tail_ == block_id) {
      auto& prev_block = physical_blocks_[block.lru_prev_block_id];
      prev_block.lru_next_block_id = -1;
      lrulist_tail_ = block.lru_prev_block_id;
    } else {
      auto& prev_block = physical_blocks_[block.lru_prev_block_id];
      auto& next_block = physical_blocks_[block.lru_next_block_id];
      prev_block.lru_next_block_id = block.lru_next_block_id;
      next_block.lru_prev_block_id = block.lru_prev_block_id;
    }

    block.lru_prev_block_id = -1;
    block.lru_next_block_id = lrulist_head_;
    physical_blocks_[lrulist_head_].lru_prev_block_id = block_id;
    lrulist_head_ = block_id;
  }

  Address GetAddress(size_t lba) {
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

  inline int16_t PickColdBlock(uint16_t block_id) {
    // std::cout << current_ts_ - physical_blocks_[lrulist_tail_].ts << std::endl;
    if(current_ts_ - physical_blocks_[lrulist_tail_].ts < 100000) {
      return -1;
    }
    auto cold_block_id = lrulist_tail_;
    while(cold_block_id == block_id 
      || physical_blocks_[cold_block_id].IsWornOut() 
      || physical_blocks_[cold_block_id].valid_page_count < 15 * block_capacity_ / 16
      || physical_blocks_[cold_block_id].erase_count <= physical_blocks_[block_id].erase_count) {
      cold_block_id = physical_blocks_[cold_block_id].lru_prev_block_id;
      if(cold_block_id == -1) {
        break;
      }
    }
    return cold_block_id;
  }

  uint16_t PickVictimBlock() {
    double max_benefit_cost_ratio = -1;
    uint16_t victim_block_id = physical_blocks_.size();
    for(size_t i = 0; i < physical_blocks_.size(); i++) {
      if(i == cleaning_block_id_) {
        continue;
      }
      auto& block = physical_blocks_[i];
      if(block.IsWornOut()) {
        continue;
      }

      double utilization = block.valid_page_count / (double)block_capacity_;
      double benefit_cost_ratio = (1 - utilization) / (1 + utilization) * (current_ts_ - block.ts);
      double ration_adjustment = block.erase_count / (double)max_erases_;
      benefit_cost_ratio *= ration_adjustment;
      if(benefit_cost_ratio > max_benefit_cost_ratio) {
        max_benefit_cost_ratio = benefit_cost_ratio;
        victim_block_id = i;
      } else if(benefit_cost_ratio == max_benefit_cost_ratio && block.valid_page_count < physical_blocks_[victim_block_id].valid_page_count) {
        victim_block_id = i;
      }
    }
    if(victim_block_id != physical_blocks_.size() && physical_blocks_[victim_block_id].valid_page_count == block_capacity_) {
      return physical_blocks_.size();
    }
    return victim_block_id;
  }

  uint16_t Migrate(uint16_t block_id, const ExecCallBack<PageType> &func) {
    auto cold_block_id = PickColdBlock(block_id);
    if(cold_block_id == -1) {
      return block_id;
    }
    auto& cold_block = physical_blocks_[cold_block_id];
    auto& block = physical_blocks_[block_id];
    for(size_t i = 0; i < cold_block.mapped_logical_pages.size(); i++) {
      auto& logical_addr = cold_block.mapped_logical_pages[i];
      if(logical_addr.IsValid()) {
        func(OpCode::READ, GetAddress(cold_block_id * block_capacity_ + i));
        auto new_physical_page_idx = block.WritePage(logical_addr.addr);
        func(OpCode::WRITE, GetAddress(block_id * block_capacity_ + new_physical_page_idx));
        lba_to_physical_page_[logical_addr.addr].Set(block_id, new_physical_page_idx);
      }
    }
    func(OpCode::ERASE, GetAddress(cold_block_id * block_capacity_));
    cold_block.Erase();
    UpdateLRU(block_id);
    return cold_block_id;
  }

  uint16_t CleanBlock(uint16_t block_id, const ExecCallBack<PageType> &func) {
    auto& cleaning_block = physical_blocks_[cleaning_block_id_];
    auto& block = physical_blocks_[block_id];
    for(size_t i = 0; i < block.mapped_logical_pages.size(); i++) {
      auto& logical_addr = block.mapped_logical_pages[i];
      if(logical_addr.IsValid()) {
        func(OpCode::READ, GetAddress(block_id * block_capacity_ + i));
        auto new_physical_page_idx = cleaning_block.WritePage(logical_addr.addr);
        func(OpCode::WRITE, GetAddress(cleaning_block_id_ * block_capacity_ + new_physical_page_idx));
        lba_to_physical_page_[logical_addr.addr].Set(cleaning_block_id_, new_physical_page_idx);
      }
    }
    func(OpCode::ERASE, GetAddress(block_id * block_capacity_));
    block.Erase();
    UpdateLRU(cleaning_block_id_);
    if(cleaning_block.erase_count == migrate_threshold_) {
      block_id = Migrate(block_id, func);
    }
    
    return block_id;
  }

  bool GarbageCollection(const ExecCallBack<PageType> &func) {
    DEBUG_INST(std::cout << "[GarbageCollection] [Entry] cleaning block: " << cleaning_block_id_ << ", writing block: " << writing_block_id_ << std::endl)
    auto victim_block_id = PickVictimBlock();
    if(victim_block_id == physical_blocks_.size()) {
      // no block can be erased
      DEBUG_INST(std::cout << "[GarbageCollection] [Failed] no block can be erased" << std::endl)
      // for(int i = 0; i != physical_blocks_.size(); i++) {
      //   std::cout << "block: " << i << ", erase count: " << (int)physical_blocks_[i].erase_count << ", valid page count: " << physical_blocks_[i].valid_page_count << std::endl;
      // }
      return false;
    }

    DEBUG_INST(std::cout << "[GarbageCollection] [Info] victim block: " << victim_block_id << std::endl)
    victim_block_id = CleanBlock(victim_block_id, func);
    writing_block_id_ = cleaning_block_id_;
    cleaning_block_id_ = victim_block_id;
    DEBUG_INST(std::cout << "[GarbageCollection] [Success] cleaning block: " << cleaning_block_id_ << ", writing block: " << writing_block_id_ << std::endl)
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
size_t MyFTL<PageType>::max_erases_ = 0;

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
