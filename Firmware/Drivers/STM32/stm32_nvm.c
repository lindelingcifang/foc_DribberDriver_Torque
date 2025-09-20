/*
* Flash-based Non-Volatile Memory (NVM)
* 
* This file supports storing and loading persistent configuration based on
* the STM32 builtin flash memory.
*
* The STM32F405xx has 12 flash pages of heterogeneous size. We use the last
* two pages for configuration data. These pages have a size of 128kB each.
* Setting any bit in these pages to 0 is always possible, but setting them
* to 1 requires erasing the whole page.
*
* We consider each page as an array of 64-bit fields except the first N bytes, which we
* instead use as an allocation block. The allocation block is a compact bit-field (2 bit per entry)
* that keeps track of the state of each field (erased, invalid, valid).
*
* One page is always considered the valid (read) page and the other one is the
* target for the next write access: they can be considered to be ping-pong or double buffred.
*
* When writing a block of data, instead of always erasing the whole writable page the
* new data is appended in the erased area. This presumably increases flash life span.
* The writable page is only erased if there is not enough space for the new data.
*
* On startup, if there is exactly one page
* whose last non-erased value has the state "valid" that page is considered
* the valid page. In any other case the selection is undefined.
*
*
* To write a new block of data atomically we first mark all associated fields
* as "invalid" (in the allocation table) then write the data and then mark the
* fields as "valid" (in the direction of increasing address).
*/

#include "stm32_nvm.h"

#include <string.h>

#if defined(STM32G474xx)

#include <stm32g474xx.h>
#include <stm32g4xx_hal.h>

// refer to page 96 of reference manual
#define FLASH_PAGE_A 126U
#define FLASH_PAGE_A_BASE (const volatile uint8_t*)0x807E000UL
#define FLASH_PAGE_A_SIZE 0x1000UL
#define FLASH_PAGE_B 127U
#define FLASH_PAGE_B_BASE (const volatile uint8_t*)0x807F000UL
#define FLASH_PAGE_B_SIZE 0x1000UL

#else
#error "unknown flash page size"
#endif

typedef enum {
    VALID = 0,
    INVALID = 1,
    ERASED = 3
} field_state_t;

typedef struct {
    size_t index;               //!< next field to be written to (can be equal to n_data)
    const uint32_t page_id;   //!< HAL ID of this page
    const size_t n_data;        //!< number of 64-bit fields in this page
    const size_t n_reserved;    //!< number of 64-bit fields in this page that are reserved for the allocation table
    const volatile uint8_t* const alloc_table;
    const volatile uint64_t* const data;
} page_t;

page_t pages[] = { {
    .page_id = FLASH_PAGE_A,
    .n_data = FLASH_PAGE_A_SIZE >> 3,
    .n_reserved = (FLASH_PAGE_A_SIZE >> 3) >> 5,
    .alloc_table = FLASH_PAGE_A_BASE,
    .data = (uint64_t *)FLASH_PAGE_A_BASE
}, {
    .page_id = FLASH_PAGE_B,
    .n_data = FLASH_PAGE_B_SIZE >> 3,
    .n_reserved = (FLASH_PAGE_B_SIZE >> 3) >> 5,
    .alloc_table = FLASH_PAGE_B_BASE,
    .data = (uint64_t *)FLASH_PAGE_B_BASE
}};

uint8_t read_page_; // 0 or 1 to indicate which page to read from and which to write to
size_t n_staging_area_; // number of 64-bit values that were reserved using NVM_start_write
size_t n_valid_; // number of 64-bit fields that can be read

static const uint32_t FLASH_ERR_FLAGS =
#if defined(FLASH_FLAG_EOP)
        FLASH_FLAG_EOP |
#endif
#if defined(FLASH_FLAG_OPERR)
        FLASH_FLAG_OPERR |
#endif
#if defined(FLASH_FLAG_WRPERR)
        FLASH_FLAG_WRPERR |
#endif
#if defined(FLASH_FLAG_PGAERR)
        FLASH_FLAG_PGAERR |
#endif
#if defined(FLASH_FLAG_PGSERR)
        FLASH_FLAG_PGSERR |
#endif
#if defined(FLASH_FLAG_PGPERR)
        FLASH_FLAG_PGPERR |
#endif
        0;

static void HAL_FLASH_ClearError() {
    __HAL_FLASH_CLEAR_FLAG(FLASH_ERR_FLAGS);
}


// @brief Erases a flash page. This sets all bits in the page to 1.
// The page's current index is reset to the minimum value (n_reserved).
// @returns 0 on success or a non-zero error code otherwise
int erase(page_t *page) {
    FLASH_EraseInitTypeDef erase_struct = {
        .TypeErase = FLASH_TYPEERASE_PAGES,
#if defined(FLASH_OPTR_DBANK)
        .Banks = FLASH_BANK_1, // only used for mass erase
#endif
        .Page = page->page_id,
        .NbPages = 1,
    };
    HAL_FLASH_Unlock();
    HAL_FLASH_ClearError();
    uint32_t page_error;
    if (HAL_FLASHEx_Erase(&erase_struct, &page_error) != HAL_OK)
        goto fail;
    page->index = page->n_reserved;

    HAL_FLASH_Lock();
    return 0;
fail:
    HAL_FLASH_Lock();
    //printf("erase failed: %u \r\n", HAL_FLASH_GetError());
    return HAL_FLASH_GetError(); // non-zero
}


// @brief Writes states into the allocation table using 64-bit writes
// The write operation goes in the direction of increasing indices.
// @param state: 11: erased, 10: writing, 00: valid data
// @returns 0 on success or a non-zero error code otherwise
int set_allocation_state(page_t *page, size_t index, size_t count, field_state_t state) {
    if (index < page->n_reserved)
        return -1;
    if (index + count >= page->n_data)
        return -1;

    // Expand state to 64-bit value containing 32 states
    uint64_t states = 0;
    for (int i = 0; i < 32; i++) {
        states |= ((uint64_t)state << (i * 2));
    }
    
    // Handle unaligned start (at state level, not byte level)
    size_t start_offset = index % 32;
    if (start_offset > 0) {
        // Process partial group at start
        size_t partial_count = 32 - start_offset;
        if (partial_count > count) partial_count = count;
        
        // Create mask for partial write
        uint64_t mask = ~((1ULL << (start_offset * 2)) - 1);
        if (partial_count < 32) {
            mask &= (1ULL << ((start_offset + partial_count) * 2)) - 1;
        }
        
        // Calculate address (aligned to 8 bytes)
        size_t byte_index = (index / 4); // 4 states per byte
        uintptr_t addr = (uintptr_t)&page->alloc_table[byte_index];
        addr = (addr + 7) & ~7; // Align to 8-byte boundary
        
        HAL_FLASH_Unlock();
        HAL_FLASH_ClearError();
        
        // Write partial group
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, states & mask) != HAL_OK) {
            HAL_FLASH_Lock();
            return HAL_FLASH_GetError();
        }
        
        HAL_FLASH_Lock();
        
        count -= partial_count;
        index += partial_count;
    }

    // Write full 32-state groups
    while (count >= 32) {
        size_t byte_index = (index / 4); // 4 states per byte
        uintptr_t addr = (uintptr_t)&page->alloc_table[byte_index];
        
        HAL_FLASH_Unlock();
        HAL_FLASH_ClearError();
        
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, states) != HAL_OK) {
            HAL_FLASH_Lock();
            return HAL_FLASH_GetError();
        }
        
        HAL_FLASH_Lock();
        
        count -= 32;
        index += 32;
    }

    // Handle unaligned end
    if (count > 0) {
        // Create mask for partial write
        uint64_t mask = (1ULL << (count * 2)) - 1;
        
        size_t byte_index = (index / 4); // 4 states per byte
        uintptr_t addr = (uintptr_t)&page->alloc_table[byte_index];
        addr = (addr + 7) & ~7; // Align to 8-byte boundary
        
        HAL_FLASH_Unlock();
        HAL_FLASH_ClearError();
        
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, states & mask) != HAL_OK) {
            HAL_FLASH_Lock();
            return HAL_FLASH_GetError();
        }
        
        HAL_FLASH_Lock();
    }
    
    return 0;
}

// @brief Reads the allocation table from behind to determine how many fields match the
// reference state.
// @param page: The page on which to perform the search
// @param max_index: The maximum index that should be considered
// @param ref_state: The reference state
// @param state: Set to the first encountered state that is unequal to ref_state.
//               Set to ref_state if all encountered states are equal to ref_state.
// @returns The smallest index that points to a field with ref_state.
//          This value is at least page->n_reserved and at most max_index.
size_t scan_allocation_table(page_t *page, size_t max_index, field_state_t ref_state, field_state_t *state) {
    const uint8_t ref_states = (ref_state << 0) | (ref_state << 2) | (ref_state << 4) | (ref_state << 6);
    size_t index = (((max_index + 3) >> 2) << 2); // start at the max index but round up to a multiple of 4
    size_t ignore = index - max_index;
    uint8_t states = ref_states;

    //printf("scan from %08x to %08x for %02x\r\n", index, page->n_reserved, ref_states); osDelay(5);

    // read 4 states at a time
    for (; index >= (page->n_reserved + 4); index -= 4) {
        states = page->alloc_table[(index - 1) >> 2];
        if (ignore) { // ignore the upper 1, 2 or 3 states if max_index was unaligned
            uint8_t ignore_mask = ~(0xff >> (ignore << 1));
            states = (states & ~ignore_mask) | (ref_states & ignore_mask);
            ignore = 0;
        }
        if (states != ref_states)
            break;
    }

    // once we encounterd a byte with any state mismatch determine which of the 4 states it is
    for (; ((states >> 6) == (ref_states & 0x3)) && (index > page->n_reserved); index--) {
        states <<= 2;
    }
    
    *state = states >> 6;
    //printf("(it's %02x)\r\n", index); osDelay(5);
    return index;
}

// Loads the head of the NVM data.
// If this function fails subsequent calls to NVM functions (other than NVM_init or NVM_erase)
// cause undefined behavior.
// @returns 0 on success or a non-zero error code otherwise
int NVM_init(void) {
    field_state_t page0_state, page1_state;
    pages[0].index = scan_allocation_table(&pages[0], pages[0].n_data,
                ERASED, &page0_state);
    pages[1].index = scan_allocation_table(&pages[1], pages[1].n_data,
                ERASED, &page1_state);
    //printf("page states: %02x, %02x\r\n", page0_state, page1_state); osDelay(5);

    // Select valid page on a best effort basis
    // (in unfortunate cases valid_page might actually point
    // to an invalid or erased page)
    read_page_ = 0;
    if (page1_state == VALID)
        read_page_ = 1;
    
    // count the number of valid fields
    page_t *read_page = &pages[read_page_];
    uint8_t first_nonvalid_state;
    size_t min_valid_index = scan_allocation_table(read_page, read_page->index,
        VALID, &first_nonvalid_state);
    n_valid_ = read_page->index - min_valid_index;
    
    n_staging_area_ = 0;

    int status = 0;
    /*// bring non-valid pages into a known state
    this is not absolutely required
    if (page0_state != VALID)
        status |= erase(&pages[0]);
    if (page1_state != VALID)
        status |= erase(&pages[1]);
    */
    return status;
}

// @brief Erases all data in the NVM.
//
// If this function fails subsequent calls to NVM functions (other than NVM_init or NVM_erase)
// cause undefined behavior.
// Caution: this function may take a long time (like 1 second)
//
// @returns 0 on success or a non-zero error code otherwise
int NVM_erase(void) {
    read_page_ = 0;
    pages[0].index = pages[0].n_reserved;
    pages[1].index = pages[1].n_reserved;

    int state = 0;
    state |= erase(&pages[0]);
    state |= erase(&pages[1]);
    return state;
}

// @brief Returns the maximum number of bytes that can be read using NVM_read.
// This holds until NVM_commit is called.
size_t NVM_get_max_read_length(void) {
    return n_valid_ << 3;
}

// @brief Returns the maximum length (in bytes) that can passed to NVM_start_write.
// This holds until NVM_commit is called.
size_t NVM_get_max_write_length(void) {
    page_t *target = &pages[1 - read_page_];
    return (target->n_data - target->n_reserved) << 3;
}

// @brief Reads from the latest committed block in the non-volatile memory.
// The function either succeeds or leaves the provided buffer unmodified.
// @param offset: offset in bytes (0 meaning the beginning of the valid area)
// @param data: buffer to write to
// @param length: length in bytes (if (offset + length) is out of range, the function fails)
// @returns 0 on success or a non-zero error code otherwise
int NVM_read(size_t offset, uint8_t *data, size_t length) {
    if (offset + length > (n_valid_ << 3))
        return -1;
    page_t *read_page = &pages[read_page_];
    const uint8_t *src_ptr = ((const uint8_t *)&read_page->data[read_page->index - n_valid_]) + offset;
    memcpy(data, src_ptr, length);
    return 0;
}

// @brief Starts an atomic write operation.
//
// The most recent valid NVM data is not modified or invalidated until NVM_commit is called.
// The length must be at most equal to the size indicated by NVM_get_max_write_length().
//
// @param length: Length of the staging block that should be created
int NVM_start_write(size_t length) {
    int status = 0;
    page_t *target = &pages[1 - read_page_];

    length = (length + 7) >> 3; // round to multiple of 64 bit
    if (length > target->n_data - target->n_reserved)
        return -1;

    // make room for the new data
    if (length > target->n_data - target->index)
        if ((status = erase(target)))
            return status;

    // invalidate the fields we're about to write
    status = set_allocation_state(target, target->index, length, INVALID);
    if (status)
        return status;

    n_staging_area_ = length;
    return 0;
}

// @brief Writes to the current data block that was opened with NVM_start_write.
//
// The operation fails if (offset + length) is larger than the length passed to NVM_start_write.
// The most recent valid NVM data is not modified or invalidated until NVM_commit is called.
// Warning: Writing different data to the same area multiple times during a single transaction
// will cause data corruption.
//
// @param offset: The offset in bytes, 0 being the beginning of the staging block.
// @param data: Pointer to the data that should be written
// @param length: Data length in bytes
int NVM_write(size_t offset, uint8_t *data, size_t length) {
    if (offset + length > (n_staging_area_ << 3))
        return -1;
    page_t *target = &pages[1 - read_page_];
    uintptr_t base_addr = (uintptr_t)&target->data[target->index];
    uintptr_t write_addr = base_addr + offset;
    
    HAL_FLASH_Unlock();
    HAL_FLASH_ClearError();
    
    // 处理起始非对齐部分（小于8字节）
    size_t start_unaligned = write_addr & 0x7;
    if (start_unaligned && length > 0) {
        size_t chunk_size = 8 - start_unaligned;
        if (chunk_size > length) chunk_size = length;
        
        // 构造64位数据
        uint64_t aligned_data = 0xFFFFFFFFFFFFFFFF; // 初始化为全1（擦除状态）
        uint8_t *aligned_ptr = (uint8_t*)&aligned_data;
        
        // 复制数据到64位缓冲区
        for (size_t i = 0; i < chunk_size; i++) {
            aligned_ptr[start_unaligned + i] = data[i];
        }
        
        // 对齐地址
        uintptr_t aligned_addr = write_addr & ~0x7ULL;
        
        // 写入64位数据
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 
                              aligned_addr, aligned_data) != HAL_OK) {
            goto fail;
        }
        
        // 更新指针和长度
        data += chunk_size;
        length -= chunk_size;
        write_addr += chunk_size;
    }
    
    // 写入完整的64位块
    while (length >= 8) {
        // 直接使用64位数据
        uint64_t *data64 = (uint64_t*)data;
        
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 
                              write_addr, *data64) != HAL_OK) {
            goto fail;
        }
        
        data += 8;
        length -= 8;
        write_addr += 8;
    }
    
    // 处理尾部非对齐部分（小于8字节）
    if (length > 0) {
        // 构造64位数据
        uint64_t aligned_data = 0xFFFFFFFFFFFFFFFF; // 初始化为全1（擦除状态）
        uint8_t *aligned_ptr = (uint8_t*)&aligned_data;
        
        // 复制数据到64位缓冲区
        for (size_t i = 0; i < length; i++) {
            aligned_ptr[i] = data[i];
        }
        
        // 对齐地址
        uintptr_t aligned_addr = write_addr & ~0x7ULL;
        
        // 写入64位数据
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 
                              aligned_addr, aligned_data) != HAL_OK) {
            goto fail;
        }
    }
    
    HAL_FLASH_Lock();
    return 0;
    
fail:
    HAL_FLASH_Lock();
    return HAL_FLASH_GetError(); // non-zero
}

// @brief Commits the new data to NVM atomically.
int NVM_commit(void) {
    page_t *read_page = &pages[read_page_];
    page_t *write_page = &pages[1 - read_page_];

    // mark the newly-written fields as valid
    int status = set_allocation_state(write_page, write_page->index, n_staging_area_, VALID);
    if (status)
        return status;

    write_page->index += n_staging_area_;
    n_valid_ = n_staging_area_;
    n_staging_area_ = 0;
    read_page_ = 1 - read_page_;

    // invalidate the other page
    if (read_page->index < read_page->n_data) {
        status = set_allocation_state(read_page, read_page->index, 1, INVALID);
        read_page->index += 1;
    } else {
        status = erase(read_page);
    }

    return status;
}


#include <cmsis_os.h>
#include <stdio.h>
/** @brief Call this at startup to test/demo the NVM driver

 Expected output when starting with a fully erased NVM

    [1st boot]
    === NVM TEST ===
    NVM is empty
    write 0x00, ..., 0x25 to NVM
    new data committed to NVM
    
    [2nd boot]
    === NVM TEST ===
    NVM contains 40 valid bytes:
    00 01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f
    10 11 12 13 14 15 16 17 18 19 1a 1b 1c 1d 1e 1f
    20 21 22 23 24 25 ff ff
    write 0xbd, ..., 0xe2 to NVM
    new data committed to NVM

    [3rd boot]
    === NVM TEST ===
    NVM contains 40 valid bytes:
    bd be bf c0 c1 c2 c3 c4 c5 c6 c7 c8 c9 ca cb cc
    cd ce cf d0 d1 d2 d3 d4 d5 d6 d7 d8 d9 da db dc
    dd de df e0 e1 e2 ff ff
    write 0xcb, ..., 0xf0 to NVM
    new data committed to NVM
*/
void NVM_demo(void) {
    const size_t len = 38;
    uint8_t data[len];
    int progress = 0;
    uint8_t seed = 0;

    osDelay(100);
    printf("=== NVM TEST ===\r\n"); osDelay(5);
    //NVM_erase();
    if (progress++, NVM_init() != 0)
        goto fail;
    
    // load bytes from NVM and print them
    size_t available = NVM_get_max_read_length();
    if (available) {
        printf("NVM contains %d valid bytes:\r\n", available); osDelay(5);
        uint8_t buf[available];
        if (progress++, NVM_read(0, buf, available) != 0)
            goto fail;
        for (size_t pos = 0; pos < available; ++pos) {
            seed += buf[pos];
            printf(" %02x", buf[pos]);
            if ((((pos + 1) % 16) == 0) || ((pos + 1) == available))
                printf("\r\n");
            osDelay(2);
        }
    } else {
        printf("NVM is empty\r\n"); osDelay(5);
    }

    // store new bytes in NVM (data based on seed)
    printf("write 0x%02x, ..., 0x%02x to NVM\r\n", seed, seed + len - 1); osDelay(5);
    for (size_t i = 0; i < len; i++)
        data[i] = seed++;
    if (progress++, NVM_start_write(len) != 0)
        goto fail;
    if (progress++, NVM_write(0, data, len / 2))
        goto fail;
    if (progress++, NVM_write(len / 2, &data[len / 2], len - (len / 2)))
        goto fail;
    if (progress++, NVM_commit())
        goto fail;
    printf("new data committed to NVM\r\n"); osDelay(5);

    return;

fail:
    printf("NVM test failed at %d!\r\n", progress);
}
