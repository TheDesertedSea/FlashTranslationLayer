#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#define SSD_SIZE 4
#define PACKAGE_SIZE 8
#define DIE_SIZE 2
#define PLANE_SIZE 10
#define BLOCK_SIZE 16
#define OVERPROVISIONING 0.05
#include "746FlashSim.h"

static FILE *log_file_stream;
static char log_file_path[255];

int main(int argc, char *argv[]) {
    if (argc != 3) {
        printf("usage: test_2_2 <config_file_name> <log_file_path>\n");
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
    const size_t num_raw_blocks = SSD_SIZE * PACKAGE_SIZE * DIE_SIZE * PLANE_SIZE;
    const size_t num_nondata_blocks = OVERPROVISIONING * num_raw_blocks;

    // While no GC is performed, for each data block:
    // Write 1st page;
    // Rewrite 1st page;
    // Check write; 
    size_t num_block_touched = 0;
    FlashSimTest test(argv[1]);
    while (test.TotalErasesPerformed() == 0) {
        if (num_block_touched <= num_nondata_blocks) {
            const size_t addr = num_block_touched * BLOCK_SIZE;
            TEST_PAGE_TYPE page_value1 = rand() % 18746;
            // std::cout << "addr: " << addr << " page_value1: " << page_value1 << std::endl;
            TEST_PAGE_TYPE page_value2;
            r = test.Write(log_file_stream, addr, ~page_value1);
            if (r != 1) goto failed;
            r = test.Write(log_file_stream, addr, page_value1);
            if (r != 1) goto failed;
            r = test.Read(log_file_stream, addr, &page_value2);
            if (r != 1) goto failed;

            if (page_value1 != page_value2) {
                fprintf(log_file_stream, "Reading LBA %zu does not get the right value\n", addr);
                goto failed;
            }
        } else {
            fprintf(log_file_stream, "No GC activity detected\n");
            goto failed;
        }
        num_block_touched++;
    }

    fprintf(log_file_stream, ">>> GC detected <<<\n");

    // try to read the 1st page in each data block
    // Note that we re-seed the random generator, so the same sequence of
    // numbers should be replayed.
    srand(15746);
    for (size_t i = 0; i < num_block_touched; i++) {
        const size_t addr = i * BLOCK_SIZE;
        const TEST_PAGE_TYPE page_value1 = rand() % 18746;
        TEST_PAGE_TYPE page_value2;
        r = test.Read(log_file_stream, addr, &page_value2);
        if (r != 1) goto failed;

        // std::cout << "addr: " << addr << " page_value1: " << page_value1 << " page_value2: " << page_value2 << std::endl;
        if (page_value1 != page_value2) {
            fprintf(log_file_stream, "Reading LBA %zu does not get the right value\n", addr);
            goto failed;
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
