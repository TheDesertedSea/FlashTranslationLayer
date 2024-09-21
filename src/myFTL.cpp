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
static const uint16_t INVALID_PAGE = 0xFFFF;

template <typename PageType>
class MyFTL : public FTLBase<PageType> {

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

  /* Compact format of page address */
  struct MyAddress {
    uint16_t addr;

    MyAddress() : addr(INVALID_PAGE) {}

    bool IsValid() const {
      return addr != INVALID_PAGE;
    }

    uint16_t BlockId() const {
      return addr >> page_idx_bit_num_;
    }

    uint16_t PageIdx() const {
      return addr & page_idx_mask_;
    }

    void Set(uint16_t block_id, uint16_t page_idx) {
      addr = (block_id << page_idx_bit_num_) | page_idx;
    }

    void Set(uint16_t lba) {
      this->addr = lba;
    }

    void SetInvalid() {
      addr = INVALID_PAGE;
    }
  };

  /* Physical block state */
  struct PhysicalBlock {
    std::vector<MyAddress> mapped_logical_pages;
    uint16_t next_free_page_idx;
    uint16_t erase_count;
    unsigned int valid_page_count;
    size_t ts;

    PhysicalBlock() : next_free_page_idx(0), erase_count(max_erases_), valid_page_count(0), ts(0) {
      mapped_logical_pages.resize(block_capacity_);
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
  };

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

  /* current ts */
  static size_t current_ts_;

  /* Throttle threshold, trigger reduction of benefit-cost ratio */
  size_t throttle_threshold_;
  /* factor used to reduce the benefit-cost ratio */
  double ratio_reduction_factor_;
  /* The threshold triggerring the migration of cold data in the block */
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

    writing_block_id_ = 1;
    cleaning_block_id_ = 0;
    next_init_empty_block_id_ = 2;

    current_ts_ = 0;

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

    DEBUG_INST(std::cout << "[ReadTranslate] [Entry] read lba: " << lba << std::endl)
    if(lba > lba_to_physical_page_.size()) {
      // out of range
      DEBUG_INST(std::cout << "[ReadTranslate] [Error] out of range" << std::endl)
      return std::make_pair(ExecState::FAILURE, Address());
    }

    auto& addr = lba_to_physical_page_[lba];
    if(!addr.IsValid()) {
      // read from invalid page
      DEBUG_INST(std::cout << "[ReadTranslate] [Error] read from invalid page" << std::endl)
      return std::make_pair(ExecState::FAILURE, Address());
    }
    
    // successful read
    DEBUG_INST(std::cout << "[ReadTranslate] [Success] read from block: " << addr.BlockId() << ", page: " << addr.PageIdx() << std::endl)
    return std::make_pair(ExecState::SUCCESS, GetAddress(addr.addr));
  }

  /*
   * WriteTranslate() - Translates write address
   *
   * Please refer to ReadTranslate()
   */
  std::pair<ExecState, Address> WriteTranslate(
      size_t lba, const ExecCallBack<PageType> &func) {
    (void)func;
    DEBUG_INST(std::cout << "[WriteTranslate] [Entry] write lba: " << lba << std::endl)
    if(lba > lba_to_physical_page_.size()) {
      // out of range
      DEBUG_INST(std::cout << "[WriteTranslate] [Error] out of range" << std::endl)
      return std::make_pair(ExecState::FAILURE, Address());
    }

    auto& addr = lba_to_physical_page_[lba];
    if(physical_blocks_[writing_block_id_].next_free_page_idx == block_capacity_) {
      if(next_init_empty_block_id_ == physical_blocks_.size()) {
        // do garbage collection
        DEBUG_INST(std::cout << "[WriteTranslate] [Info] do garbage collection" << std::endl)
        if(!GarbageCollection(func)) {
          // garbage collection failed
          DEBUG_INST(std::cout << "[WriteTranslate] [Failed] garbage collection failed" << std::endl)
          return std::make_pair(ExecState::FAILURE, Address());
        }
      } else {
        // choose next initially empty block
        DEBUG_INST(std::cout << "[WriteTranslate] [Info] choose next initially empty block: " << next_init_empty_block_id_ << std::endl)
        writing_block_id_ = next_init_empty_block_id_++;
        physical_blocks_[writing_block_id_] = physical_blocks_[writing_block_id_];
      }
    }

    auto& writing_block = physical_blocks_[writing_block_id_];
    // modify old mapping
    if(addr.IsValid()) {
      auto& prev_block = physical_blocks_[addr.BlockId()];
      prev_block.mapped_logical_pages[addr.PageIdx()].SetInvalid();
      prev_block.valid_page_count--;
      DEBUG_INST(std::cout << "[WriteTranslate] [Info] old mapping: block: " << addr.BlockId() << ", page: " << addr.PageIdx() << std::endl)
    }

    // map to new physical page
    addr.Set(writing_block_id_, writing_block.next_free_page_idx);
    writing_block.mapped_logical_pages[writing_block.next_free_page_idx].Set(lba);
    writing_block.next_free_page_idx++;
    writing_block.valid_page_count++;
    writing_block.ts = current_ts_++;

    DEBUG_INST(std::cout << "[WriteTranslate] [Success] write to block: " << addr.BlockId() << ", page: " << addr.PageIdx() << std::endl)
    return std::make_pair(ExecState::SUCCESS, GetAddress(addr.addr));
  }

  /*
   * Optionally mark a LBA as a garbage.
   */
  ExecState Trim(size_t lba, const ExecCallBack<PageType> &func) {
    (void)func;
    DEBUG_INST(std::cout << "[Trim] [Entry] trim lba: " << lba << std::endl)
    if(lba > lba_to_physical_page_.size()) {
      // out of range
      DEBUG_INST(std::cout << "[Trim] [Error] out of range" << std::endl)
      return ExecState::FAILURE;
    }

    auto& addr = lba_to_physical_page_[lba];
    if(!addr.IsValid()) {
      // trim invalid page
      DEBUG_INST(std::cout << "[Trim] [Error] trim invalid page" << std::endl)
      return ExecState::FAILURE;
    }

    auto& block = physical_blocks_[addr.BlockId()];
    block.mapped_logical_pages[addr.PageIdx()].SetInvalid();
    block.valid_page_count--;
    DEBUG_INST(std::cout << "[Trim] [Success] trim block: " << addr.BlockId() << ", page: " << addr.PageIdx() << std::endl)
    return ExecState::SUCCESS;
  }

 private:
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

  bool GarbageCollection(const ExecCallBack<PageType> &func) {
    DEBUG_INST(std::cout << "[GarbageCollection] [Entry] cleaning block: " << cleaning_block_id_ << ", writing block: " << writing_block_id_ << std::endl)
    auto& cleaning_block = physical_blocks_[cleaning_block_id_];
    double max_benefit_cost_ratio = -1;
    size_t gc_block_id = physical_blocks_.size();
    for(uint16_t i = 0; i < physical_blocks_.size(); i++) {
      if(i == cleaning_block_id_) {
        continue;
      }
      auto& block = physical_blocks_[i];
      if(block.erase_count == 0) {
        // block can not be erased
        continue;
      }
      double utilization = block.valid_page_count / block_capacity_;
      double benefit_cost_ratio = (1 - utilization) / (1 + utilization) * (current_ts_ - block.ts);
      if(block.erase_count < throttle_threshold_) {
        benefit_cost_ratio *= ratio_reduction_factor_;
      }
      if(benefit_cost_ratio > max_benefit_cost_ratio) {
        max_benefit_cost_ratio = benefit_cost_ratio;
        gc_block_id = i;
      } else if(benefit_cost_ratio == max_benefit_cost_ratio) {
        if(block.valid_page_count < physical_blocks_[gc_block_id].valid_page_count) {
          gc_block_id = i;
        }
      }
    }
    if(gc_block_id == physical_blocks_.size()) {
      // no block can be erased
      DEBUG_INST(std::cout << "[GarbageCollection] [Failed] no block can be erased(all worn out)" << std::endl)
      return false;
    }
    if(max_benefit_cost_ratio == 0 && physical_blocks_[gc_block_id].valid_page_count == block_capacity_) {
      // no block can be erased
      DEBUG_INST(std::cout << "[GarbageCollection] [Failed] no block can be erased" << std::endl)
      return false;
    }

    DEBUG_INST(std::cout << "[GarbageCollection] [Info] gc block: " << gc_block_id << std::endl)
    // move data from gc block to cleaning block
    auto& gc_block = physical_blocks_[gc_block_id];
    for(size_t i = 0; i < gc_block.mapped_logical_pages.size(); i++) {
      auto& addr = gc_block.mapped_logical_pages[i];
      if(addr.IsValid()) {
        func(OpCode::READ, GetAddress(gc_block_id * block_capacity_ + i));
        func(OpCode::WRITE, GetAddress(cleaning_block_id_ * block_capacity_ + cleaning_block.next_free_page_idx));
        lba_to_physical_page_[addr.addr].Set(cleaning_block_id_, cleaning_block.next_free_page_idx);
        cleaning_block.mapped_logical_pages[cleaning_block.next_free_page_idx] = addr;
        cleaning_block.next_free_page_idx++;
        cleaning_block.valid_page_count++;
        cleaning_block.ts = current_ts_++;
        // DEBUG_INST(std::cout << "[GarbageCollection] [Info] move data: lba: " << addr.addr << " from block: " << gc_block_id << ", page: " << i <<
        // " -> block: " << cleaning_block_id_ << ", page: " << cleaning_block.next_free_page_idx - 1 << std::endl)
      }
    }
    func(OpCode::ERASE, GetAddress(gc_block_id * block_capacity_));
    gc_block.Erase();
    writing_block_id_ = cleaning_block_id_;
    cleaning_block_id_ = gc_block_id;
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
