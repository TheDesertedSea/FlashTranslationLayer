#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#define BLOCK_SIZE 128
#define BLOCK_ERASES 500
#define MAX_LBA_ADDRESS 8000

#include "746FlashSim.h"

static FILE *log_file_stream;
static char log_file_path[255];

int main(int argc, char *argv[]) {
    if (argc != 3) {
        printf("usage: test_2_3 <config_file_name> <log_file_path>\n");
        exit(EXIT_FAILURE);
    }
    int ret = 1;
    strcpy(log_file_path, argv[2]);
    log_file_stream = fopen(log_file_path, "w+");
    assert(log_file_stream != NULL);

    fprintf(log_file_stream, "------------------------------------------------------------\n");

    init_flashsim();

    int r;
    srand(15746);
    TEST_PAGE_TYPE data[10000];
    FlashSimTest test(argv[1]);

    size_t base_addr_increment = 200;
    size_t max_base_addr = 8000;
    size_t default_num_writes = 18; // Writes directly 18 times per sequence

    for (size_t base_addr = 0; base_addr <= max_base_addr; base_addr += base_addr_increment) {

        // Write to LBA 0
        {
            size_t addr = 0;
            TEST_PAGE_TYPE page_value = rand() % 18746;
            data[addr] = page_value;
            fprintf(log_file_stream, "Writing LBA %zu\n", addr);
            r = test.Write(log_file_stream, addr, page_value);
            if (r != 1) goto failed;
            fprintf(log_file_stream, "LBA %zu written\n----------------\n", addr);
        }

        size_t num_writes = default_num_writes;

        // Write to LBAs from base_addr to base_addr + num_writes - 1
        for (size_t addr = base_addr; addr < base_addr + num_writes; addr++) {
            if (addr > MAX_LBA_ADDRESS) break; // Ensure we don't go beyond MAX_LBA_ADDRESS
            TEST_PAGE_TYPE page_value = rand() % 18746;
            data[addr] = page_value;
            fprintf(log_file_stream, "Writing LBA %zu\n", addr);
            r = test.Write(log_file_stream, addr, page_value);
            if (r != 1) goto failed;
            fprintf(log_file_stream, "LBA %zu written\n----------------\n", addr);
        }

        // Write to base_addr again
        {
            size_t addr = base_addr;
            TEST_PAGE_TYPE page_value = rand() % 18746;
            data[addr] = page_value;
            fprintf(log_file_stream, "Writing LBA %zu\n", addr);
            r = test.Write(log_file_stream, addr, page_value);
            if (r != 1) goto failed;
            fprintf(log_file_stream, "LBA %zu written\n----------------\n", addr);
        }

        // Read from base_addr to verify the data
        {
            size_t addr = base_addr;
            TEST_PAGE_TYPE read_page_value;
            fprintf(log_file_stream, "Reading LBA %zu\n", addr);
            r = test.Read(log_file_stream, addr, &read_page_value);
            if (r != 1) goto failed;
            fprintf(log_file_stream, "LBA %zu read\n----------------\n", addr);

            if (read_page_value != data[addr]) {
                fprintf(log_file_stream, "Reading LBA %zu does not get the right value\n", addr);
                goto failed;
            }
        }

        // Write to LBA 0 again
        {
            size_t addr = 0;
            TEST_PAGE_TYPE page_value = rand() % 18746;
            data[addr] = page_value;
            fprintf(log_file_stream, "Writing LBA %zu\n", addr);
            r = test.Write(log_file_stream, addr, page_value);
            if (r != 1) goto failed;
            fprintf(log_file_stream, "LBA %zu written\n----------------\n", addr);
        }
    }


    ret = 0;
    printf("SUCCESS ...Check %s for more details.\n", log_file_path);
    goto done;
failed:
    printf("FAILED ...Check %s for more details.\n", log_file_path);
done:
    fflush(log_file_stream);
    fclose(log_file_stream);

    deinit_flashsim();

    return ret;
}
