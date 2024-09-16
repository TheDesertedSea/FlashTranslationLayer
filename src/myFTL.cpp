#include "myFTL.h"

#include <memory>

#include <list>

#include "common.h"

#define DEBUG false

#if DEBUG 
#define DEBUG_INST(inst) inst;
#else
#define DEBUG_INST(inst) ;
#endif

class GCPolicyBase {
 public:
  GCPolicyBase() = default;
  virtual ~GCPolicyBase() = default;
  
  virtual void DataBlockWritten(size_t log_block_id) = 0;
  virtual void LogBlockWritten(size_t log_block_id, bool new_page) = 0;
  virtual size_t GetGCBlock() = 0;
};

class GCPolicyRoundRobin : public GCPolicyBase {

  size_t highest_log_block_id_;
  size_t highest_user_block_id_;
  size_t next_gc_block_id_;

 public:
  GCPolicyRoundRobin(size_t highest_log_block_id, size_t highest_user_block_id) 
    : highest_log_block_id_(highest_log_block_id), highest_user_block_id_(highest_user_block_id)
      , next_gc_block_id_(highest_log_block_id) {}
  GCPolicyRoundRobin() = default;
  virtual ~GCPolicyRoundRobin() = default;

  void DataBlockWritten(size_t log_block_id) override {
    (void)log_block_id;
  }

  void LogBlockWritten(size_t log_block_id, bool new_page) override {
    (void)log_block_id;
    (void)new_page;
  }

  size_t GetGCBlock() override {
    if(next_gc_block_id_ == highest_user_block_id_) {
      next_gc_block_id_ = highest_log_block_id_;
    }
    return next_gc_block_id_--;
  }
};

class GCPolicyLRU: public GCPolicyBase {
  struct Block{
    size_t log_block_id;
    Block* prev;
    Block* next;
    Block(size_t log_block_id) : log_block_id(log_block_id), prev(nullptr), next(nullptr) {}
    Block() : Block(0) {}
  };

  size_t lowest_log_block_id_;
  Block* head_;
  Block* tail_;
  std::vector<Block*> block_map_;

 public:
  GCPolicyLRU(size_t highest_log_block_id, size_t highest_user_block_id) {
    lowest_log_block_id_ = highest_user_block_id + 1;
    block_map_.resize(highest_log_block_id - highest_user_block_id, nullptr);
    head_ = new Block(highest_log_block_id);
    block_map_[highest_log_block_id - lowest_log_block_id_] = head_;
    auto cur = head_;
    for(size_t i = highest_log_block_id - 1; i > highest_user_block_id; i--) {
      Block* new_block = new Block(i);
      block_map_[i - lowest_log_block_id_] = new_block;
      cur->next = new_block;
      new_block->prev = cur;
      cur = new_block;
    }
    tail_ = cur;
  }

  virtual ~GCPolicyLRU() {
    auto cur = head_;
    while(cur) {
      auto next = cur->next;
      delete cur;
      cur = next;
    }
  }

  void DataBlockWritten(size_t log_block_id) override {
    (void)log_block_id;
  }

  void LogBlockWritten(size_t log_block_id, bool new_page) override {
    (void)new_page;
    UpdateLRU(log_block_id);
  }

  size_t GetGCBlock() override {
    DEBUG_INST(std::cout << "[GC Block]: " << tail_->log_block_id << std::endl);
    return tail_->log_block_id;
  }

 private:
  void UpdateLRU(size_t log_block_id) {
    auto block = block_map_[log_block_id - lowest_log_block_id_];
    if(block == head_) {
      return;
    }
    if(block == tail_) {
      tail_ = block->prev;
      tail_->next = nullptr;
    } else {
      block->prev->next = block->next;
      block->next->prev = block->prev;
    }
    block->next = head_;
    head_->prev = block;
    head_ = block;
  }
};

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
  /* Maximum number a block_ can be erased */
  size_t block_erase_count_;
  /* Overprovioned blocks as a percentage of total number of blocks */
  size_t op_;
  /* max erases */
  size_t max_erases_;

  /* Capacity of a block */
  size_t block_capacity_;
  /* Capacity of a plane */
  size_t plane_capacity_;
  /* Capacity of a die */
  size_t die_capacity_;
  /* Capacity of a package */
  size_t package_capacity_;
  /* Capacity of a ssd */
  size_t ssd_capacity_;


  /* cleaning block */
  size_t cleaning_block_id_;
  size_t cleaning_block_erase_count_;
  /* highest log-reservation block */
  size_t highest_log_block_id_;
  /* highest user block */
  size_t highest_user_block_id_;

  /* next log-reservation block */
  size_t next_log_block_id_;

  /* highest user lba */
  size_t highest_user_lba_;

  struct DataBlock {
    std::vector<int> page_states; // -1: empty, -2: written, >=0: log-reservation page id
    int log_block_id; // -1: no log-reservation mapping, >=0: log-reservation block id
    int next_log_page_id; // next free page id in the log-reservation block
    size_t erase_count; // erase count of the block
    DataBlock(size_t block_capacity) : page_states(block_capacity, -1), log_block_id(-1), next_log_page_id(0) {}
    DataBlock(): DataBlock(0) {}
  };

  struct LogBlock {
    size_t mapped_data_block_id;
    size_t erase_count;
    LogBlock(size_t mapped_data_block_id) : mapped_data_block_id(mapped_data_block_id), erase_count(0) {}
    LogBlock() : LogBlock(0) {}
  };

  /* blocks */
  std::vector<DataBlock> data_blocks_;
  std::vector<LogBlock> log_blocks_;
  std::vector<size_t> log_data_mapping_;

  std::shared_ptr<GCPolicyBase> gc_policy_;

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
    block_erase_count_ = conf->GetBlockEraseCount();
    op_ = conf->GetOverprovisioning();
    max_erases_ = conf->GetBlockEraseCount();

    block_capacity_ = block_size_;
    plane_capacity_ = block_capacity_ * plane_size_;
    die_capacity_ = plane_capacity_ * die_size_;
    package_capacity_ = die_capacity_ * package_size_;
    ssd_capacity_ = package_capacity_ * ssd_size_;

    cleaning_block_id_ = ssd_capacity_ / block_capacity_ - 1;
    cleaning_block_erase_count_ = 0;
    highest_log_block_id_ = cleaning_block_id_ - 1;
    highest_user_block_id_ = cleaning_block_id_ - (highest_log_block_id_ + 1) * ((double)op_ / 100);
    next_log_block_id_ = highest_log_block_id_;

    highest_user_lba_ = (highest_user_block_id_ + 1) * block_capacity_ - 1;

    data_blocks_.resize(highest_user_block_id_ + 1, DataBlock(block_capacity_));
    log_blocks_.resize(highest_log_block_id_ - highest_user_block_id_, LogBlock(-1));
    log_data_mapping_.resize(highest_log_block_id_ - highest_user_block_id_, cleaning_block_id_);

    switch(conf->GetGCPolicy()) {
      case 0:
        gc_policy_ = std::make_shared<GCPolicyRoundRobin>(highest_log_block_id_, highest_user_block_id_);
        break;
      case 1:
        gc_policy_ = std::make_shared<GCPolicyLRU>(highest_log_block_id_, highest_user_block_id_);
        break;
      default:
        throw std::runtime_error("Unknown GC policy");
    }

    printf("SSD Configuration: %zu, %zu, %zu, %zu, %zu\n", ssd_size_,
           package_size_, die_size_, plane_size_, block_size_);
    printf("Capacity: %zu, %zu, %zu, %zu, %zu\n", ssd_capacity_,
           package_capacity_, die_capacity_, plane_capacity_, block_capacity_);
    printf("Cleaning Block: %zu, Highest Log Block: %zu, Highest User Block: %zu\n", cleaning_block_id_, highest_log_block_id_,
           highest_user_block_id_);
    printf("Max Erase Count: %zu, Overprovisioning: %zu%%\n", block_erase_count_,
           op_);
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

    DEBUG_INST(std::cout << "read lba: " << lba << std::endl)
    if(lba > highest_user_lba_) {
      // out of range
      DEBUG_INST(std::cout << "out of range" << std::endl)
      return std::make_pair(ExecState::FAILURE, Address());
    }

    auto block_id = lba / block_capacity_;
    auto page_id = lba % block_capacity_;
    if(data_blocks_[block_id].page_states[page_id] == -1) {
      // empty page
      DEBUG_INST(std::cout << "read empty page" << std::endl)
      return std::make_pair(ExecState::FAILURE, Address());
    }
    
    DEBUG_INST(std::cout << "block_id: " << block_id << std::endl)
    if(data_blocks_[block_id].page_states[page_id] == -2) {
      // no log-reservation mapping, read original block
      DEBUG_INST(std::cout << "read original block" << std::endl)
      return std::make_pair(ExecState::SUCCESS, GetAddress(lba));
    }

    // read log-reservation page in the log-reservation block  
    auto mapped_page_id = (size_t)data_blocks_[block_id].page_states[page_id];
    auto mapped_lba = data_blocks_[block_id].log_block_id * block_capacity_ + mapped_page_id;
    auto mapped_addr = GetAddress(mapped_lba);
    DEBUG_INST(std::cout << "read log-reservation block" << std::endl)
    return std::make_pair(ExecState::SUCCESS, mapped_addr);
  }

  /*
   * WriteTranslate() - Translates write address
   *
   * Please refer to ReadTranslate()
   */
  std::pair<ExecState, Address> WriteTranslate(
      size_t lba, const ExecCallBack<PageType> &func) {
    (void)func;
    DEBUG_INST(std::cout << "write lba: " << lba << std::endl)
    if(lba > highest_user_lba_) {
      // out of range
      DEBUG_INST(std::cout << "out of range" << std::endl)
      return std::make_pair(ExecState::FAILURE, Address());
    }

    auto block_id = lba / block_capacity_;
    auto page_id = lba % block_capacity_;
    if (data_blocks_[block_id].page_states[page_id] == -1) {
      // write to empty page
      data_blocks_[block_id].page_states[page_id] = -2;
      if(data_blocks_[block_id].log_block_id != -1) {
        gc_policy_->DataBlockWritten(data_blocks_[block_id].log_block_id);
      }
      DEBUG_INST(std::cout << "write to empty page" << std::endl)
      return std::make_pair(ExecState::SUCCESS, GetAddress(lba));
    }
    // original page has been written
    
    DEBUG_INST(std::cout << "block_id: " << block_id << std::endl)
    if(data_blocks_[block_id].log_block_id == -1) {
      // need to add log-reservation block
      if(next_log_block_id_ == highest_user_block_id_) {
        // no more log-reservation block, do garbage collection
        auto gc_block_id = gc_policy_->GetGCBlock();
        CleanLogBlock(gc_block_id, func);
        data_blocks_[block_id].log_block_id = gc_block_id;
        data_blocks_[block_id].next_log_page_id = 0;
        log_data_mapping_[gc_block_id - highest_user_block_id_ - 1] = block_id;
        DEBUG_INST(std::cout << "mapping to gc block" << std::endl)
      } else {
        data_blocks_[block_id].log_block_id = next_log_block_id_--;
        data_blocks_[block_id].next_log_page_id = 0;
        log_data_mapping_[data_blocks_[block_id].log_block_id - highest_user_block_id_ - 1] = block_id;
        DEBUG_INST(std::cout << "mapping to free log-reservation block: " << data_blocks_[block_id].log_block_id << std::endl)
      }
    }
    
    if((size_t)data_blocks_[block_id].next_log_page_id == block_capacity_) {
      // no more log-reservation page, do garbage collection
      auto gc_block_id = data_blocks_[block_id].log_block_id;
      CleanLogBlock(gc_block_id, func);
      data_blocks_[block_id].log_block_id = gc_block_id;
      data_blocks_[block_id].next_log_page_id = 0;
      log_data_mapping_[gc_block_id - highest_user_block_id_ - 1] = block_id;
      DEBUG_INST(std::cout << "clean log-reservation block" << std::endl);
    }

    gc_policy_->LogBlockWritten(data_blocks_[block_id].log_block_id, data_blocks_[block_id].page_states[page_id] == -2);
    auto mapped_page_id = data_blocks_[block_id].next_log_page_id++;
    data_blocks_[block_id].page_states[page_id] = mapped_page_id;
    auto mapped_lba = data_blocks_[block_id].log_block_id * block_capacity_ + mapped_page_id;
    DEBUG_INST(std::cout << "write to log-reservation block, mapped_page_id: " << (int)mapped_page_id << std::endl)
    return std::make_pair(ExecState::SUCCESS, GetAddress(mapped_lba));
  }

  /*
   * Optionally mark a LBA as a garbage.
   */
  ExecState Trim(size_t lba, const ExecCallBack<PageType> &func) {
    (void)lba;
    (void)func;
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

  void CleanLogBlock(size_t log_block_id, const ExecCallBack<PageType> &func) {
    size_t data_block_id = log_data_mapping_[log_block_id - highest_user_block_id_ - 1];
    DEBUG_INST(std::cout << "[CleanLogBlock] " << log_block_id << " data block: " << data_block_id << std::endl)
    auto& data_block = data_blocks_[data_block_id];
    size_t next_cleaning_page_id = 0;
    // copy data from data block and log block to cleaning block
    for(size_t i = 0; i < block_capacity_; i++) {
      if(data_block.page_states[i] == -2) {
        auto lba = data_block_id * block_capacity_ + i;
        DEBUG_INST(std::cout << "copy from data block: " << data_block_id << " page: " << i << " to block: " << cleaning_block_id_ << std::endl)
        auto addr = GetAddress(lba);
        func(OpCode::READ, addr);
        auto dest_lba = cleaning_block_id_ * block_capacity_ + next_cleaning_page_id;
        auto dest_addr = GetAddress(dest_lba);
        func(OpCode::WRITE, dest_addr);
        data_block.page_states[i] = next_cleaning_page_id++;
      } else if(data_block.page_states[i] >= 0) {
        auto lba = data_block.log_block_id * block_capacity_ + data_block.page_states[i];
        DEBUG_INST(std::cout << "copy from log block: " << data_block.log_block_id << " page: " << i << " to cleaning block: " << cleaning_block_id_ << std::endl)
        auto addr = GetAddress(lba);
        func(OpCode::READ, addr);
        auto dest_lba = cleaning_block_id_ * block_capacity_ + next_cleaning_page_id;
        auto dest_addr = GetAddress(dest_lba);
        func(OpCode::WRITE, dest_addr);
        data_block.page_states[i] = next_cleaning_page_id++;
      }
    }
    // erase data block and log block
    func(OpCode::ERASE, GetAddress(data_block_id * block_capacity_));
    func(OpCode::ERASE, GetAddress(log_block_id * block_capacity_));

    // copy data from cleaning block to data block
    for(size_t i = 0; i < block_capacity_; i++) {
      if(data_block.page_states[i] >= 0) {
        auto lba = cleaning_block_id_ * block_capacity_ + data_block.page_states[i];
        DEBUG_INST(std::cout << "copy from cleaning block: " << cleaning_block_id_ << " page: " << i << " to block: " << data_block_id << std::endl)
        auto addr = GetAddress(lba);
        func(OpCode::READ, addr);
        auto dest_lba = data_block_id * block_capacity_ + i;
        auto dest_addr = GetAddress(dest_lba);
        func(OpCode::WRITE, dest_addr);
        data_block.page_states[i] = -2;
      }
    }

    // erase cleaning block
    func(OpCode::ERASE, GetAddress(cleaning_block_id_ * block_capacity_));
    // unmap log block
    data_block.log_block_id = -1;
    log_data_mapping_[log_block_id - highest_user_block_id_ - 1] = cleaning_block_id_;
  }

};

/*
 * CreateMyFTL() - Creates class MyFTL object
 *
 * You do not need to modify this
 */
FTLBase<TEST_PAGE_TYPE> *CreateMyFTL(const ConfBase *conf) {
  MyFTL<TEST_PAGE_TYPE> *ftl = new MyFTL<TEST_PAGE_TYPE>(conf);
  return static_cast<FTLBase<TEST_PAGE_TYPE> *>(ftl);
}
