#include "myFTL.h"

#include "common.h"

#define DEBUG false

#if DEBUG 
#define DEBUG_INST(inst) inst;
#else
#define DEBUG_INST(inst) ;
#endif

template <typename PageType>
class LogReservationBlockAllocatorBase {
 protected:
  size_t cleaning_block_id_;
  size_t highest_log_block_id_;
  size_t highest_user_block_id_;
 public:
  LogReservationBlockAllocatorBase(size_t cleaning_block_id, size_t highest_log_block_id, size_t highest_user_block_id) 
    : cleaning_block_id_(cleaning_block_id), highest_log_block_id_(highest_log_block_id)
      , highest_user_block_id_(highest_user_block_id) {}
  LogReservationBlockAllocatorBase(): LogReservationBlockAllocatorBase(0, 0, 0) {}
  virtual ~LogReservationBlockAllocatorBase(){};
  
  virtual void DataBlockWritten(size_t log_block_id) = 0;
  virtual void LogBlockWritten(size_t log_block_id, bool new_page) = 0;
  virtual size_t GetLogBlock(const ExecCallBack<PageType> &func) = 0;
};

template <typename PageType>
class RoundRobinAllocator : public LogReservationBlockAllocatorBase<PageType> {
  size_t next_log_block_id_;
 public:
  RoundRobinAllocator(size_t cleaning_block_id, size_t highest_log_block_id, size_t highest_user_block_id) 
    : LogReservationBlockAllocatorBase<PageType>(cleaning_block_id, highest_log_block_id, highest_user_block_id)
      , next_log_block_id_(highest_log_block_id) {}
  RoundRobinAllocator(): RoundRobinAllocator(0, 0, 0) {}
  virtual ~RoundRobinAllocator() {}

  void DataBlockWritten(size_t log_block_id) override {
    (void)block_id;
  }

  void LogBlockWritten(size_t log_block_id, bool new_page) override {
    (void)block_id;
    (void)new_page;
  }

  size_t GetLogBlock(const ExecCallBack<PageType> &func) override {
    if(next_log_block_id_ == cleaning_block_id_) {
      return highest_user_block_id_;
    }
    return next_log_block_id_--;
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
  /* highest log-reservation block */
  size_t highest_log_block_id_;
  /* highest user block */
  size_t highest_user_block_id_;

  /* next log-reservation block */
  size_t next_log_block_id_;

  /* highest user lba */
  size_t highest_user_lba_;

  // struct LogBlock{
  //   size_t block_id;
  //   unsigned char next_page;
  //   std::vector<char> page_mapping;
  //   LogBlock(size_t block_id, size_t block_capacity) : block_id(block_id), next_page(0), page_mapping(block_capacity, -1) {}
  //   LogBlock() = default;
  // };

  struct Block{
    std::vector<char> page_states; // -1: empty, -2: written, >=0: log-reservation page id
    int log_block_id;
    char next_log_page_id;
    Block(size_t block_capacity) : page_states(block_capacity, -1), log_block_id(-1), next_log_page_id(0) {}
    Block(): Block(0) {}
  };

  /* blocks */
  std::vector<Block> blocks;

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

    block_capacity_ = block_size_;
    plane_capacity_ = block_capacity_ * plane_size_;
    die_capacity_ = plane_capacity_ * die_size_;
    package_capacity_ = die_capacity_ * package_size_;
    ssd_capacity_ = package_capacity_ * ssd_size_;

    cleaning_block_id_ = ssd_capacity_ / block_capacity_ - 1;
    highest_log_block_id_ = cleaning_block_id_ - 1;
    highest_user_block_id_ = cleaning_block_id_ - (highest_log_block_id_ + 1) * ((double)op_ / 100);
    next_log_block_id_ = highest_log_block_id_;

    highest_user_lba_ = (highest_user_block_id_ + 1) * block_capacity_ - 1;

    blocks.resize(highest_user_block_id_ + 1, Block(block_capacity_));

    printf("SSD Configuration: %zu, %zu, %zu, %zu, %zu\n", ssd_size_,
           package_size_, die_size_, plane_size_, block_size_);
    printf("Capacity: %zu, %zu, %zu, %zu, %zu\n", ssd_capacity_,
           package_capacity_, die_capacity_, plane_capacity_, block_capacity_);
    printf("Highest Log Block: %zu, Highest User Block: %zu\n", highest_log_block_id_,
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
    if(blocks[block_id].page_states[page_id] == -1) {
      // empty page
      DEBUG_INST(std::cout << "read empty page" << std::endl)
      return std::make_pair(ExecState::FAILURE, Address());
    }
    
    DEBUG_INST(std::cout << "block_id: " << block_id << std::endl)
    if(blocks[block_id].page_states[page_id] == -2) {
      // no log-reservation mapping, read original block
      DEBUG_INST(std::cout << "read original block" << std::endl)
      return std::make_pair(ExecState::SUCCESS, GetAddress(lba));
    }

    // read log-reservation page in the log-reservation block  
    auto mapped_page_id = (size_t)blocks[block_id].page_states[page_id];
    auto mapped_lba = blocks[block_id].log_block_id * block_capacity_ + mapped_page_id;
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
    if (blocks[block_id].page_states[page_id] == -1) {
      // write to empty page
      blocks[block_id].page_states[page_id] = -2;
      DEBUG_INST(std::cout << "write to empty page" << std::endl)
      return std::make_pair(ExecState::SUCCESS, GetAddress(lba));
    }
    // original page has been written
    
    DEBUG_INST(std::cout << "block_id: " << block_id << std::endl)
    if(blocks[block_id].log_block_id == -1) {
      // need to add log-reservation block
      if(next_log_block_id_ == highest_user_block_id_) {
        // no more log-reservation block
        DEBUG_INST(std::cout << "no more log-reservation block" << std::endl)
        return std::make_pair(ExecState::FAILURE, Address());
      }
      blocks[block_id].log_block_id = next_log_block_id_--;
    }
    
    if((size_t)blocks[block_id].next_log_page_id == block_capacity_) {
      // no more log-reservation page
      DEBUG_INST(std::cout << "no more log-reservation page" << std::endl)
      return std::make_pair(ExecState::FAILURE, Address());
    }

    auto mapped_page_id = blocks[block_id].next_log_page_id++;
    blocks[block_id].page_states[page_id] = mapped_page_id;
    auto mapped_lba = blocks[block_id].log_block_id * block_capacity_ + mapped_page_id;
    DEBUG_INST(std::cout << "write to existing log-reservation block" << std::endl)
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
