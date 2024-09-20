#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <algorithm>
#include <random>
#include <vector>

#define SSD_SIZE 4
#define PACKAGE_SIZE 8
#define DIE_SIZE 2
#define PLANE_SIZE 10
#define BLOCK_SIZE 64
#define OVERPROVISIONING 0.05
#include "746FlashSim.h"

static FILE *log_file_stream;
static char log_file_path[255];

/**
 * Test 3_3 - Constant write to 0-address
 */
int main(int argc, char *argv[]) {
    if (argc != 3) {
        printf("usage: test_3_3 <config_file_name> <log_file_path>\n");
        exit(EXIT_FAILURE);
    }
    int score = 0;
    int ret = 1;
    strcpy(log_file_path, argv[2]);
    log_file_stream = fopen(log_file_path, "w+");
    assert(log_file_stream != NULL);

    init_flashsim();

    int r;
    const size_t num_raw_blocks = SSD_SIZE * PACKAGE_SIZE * DIE_SIZE * PLANE_SIZE;
    const size_t num_nondata_blocks = OVERPROVISIONING * num_raw_blocks;
    const size_t num_blocks = num_raw_blocks - num_nondata_blocks;
    const size_t num_pages = num_blocks * BLOCK_SIZE;
    FlashSimTest test(argv[1]);

    TEST_PAGE_TYPE latest_value = 0;
    std::default_random_engine rng(15746);
    std::uniform_int_distribution<size_t> uni_dist(1, 18746);
    while (true) {
        if (latest_value != 0) {
            TEST_PAGE_TYPE buffer;
            r = test.Read(nullptr, 0, &buffer);
            if (r == -1) {
                fprintf(log_file_stream, "Error reading LBA 0\n");
                goto failed;
            } else if (r != 1 || buffer != latest_value) {
                fprintf(log_file_stream, "Data corrupted or lost in LBA 0\n");
                goto failed;
            }
        }
        const TEST_PAGE_TYPE value = uni_dist(rng);
        r = test.Write(nullptr, 0, value);
        if (r == -1) {
            fprintf(log_file_stream, "Error writing LBA 0\n");
            goto failed;
        } else if (r == 1) {
            latest_value = value;
        } else {
            goto check;
        }
    }

check:
    // Verify that at least one block is out of erases. The FTL should not fail
    // any writes while cleaning is still possible.
    if (!test.AtLeastOneBlockWornOut()) {
        fprintf(log_file_stream, "FTL should not fail to write while all "
                "blocks still have erases remaining (i.e., cleaning is easily "
                "possible).\n");
        goto failed;
    }

    fprintf(log_file_stream, ">>> Stress completed <<<\n");

    for (size_t addr = 0; addr < num_pages; addr++) {
        TEST_PAGE_TYPE buffer;
        r = test.Read(nullptr, addr, &buffer);
        if (r == 1) {
            if (addr != 0 || latest_value == 0 || buffer != latest_value) {
                fprintf(log_file_stream, "Reading LBA %zu get garbage or corrupted value\n", addr);
                goto failed;
            }
        } else if (r == 0) {
            if (addr == 0 && latest_value != 0) {
                fprintf(log_file_stream, "Lost data in LBA %zu\n", addr);
                goto failed;
            }
        } else {
            fprintf(log_file_stream, "Error reading LBA %zu\n", addr);
            goto failed;
        }
    }

    ret = 0;
    score = test.Report(log_file_stream);
    printf("SUCCESS ...Check %s for more details.\n", log_file_path);
    goto done;
failed:
    printf("FAILED ...Check %s for more details.\n", log_file_path);
done:
    fprintf(log_file_stream, "Score:\n%d\n", score);
    fflush(log_file_stream);
    fclose(log_file_stream);
    printf("%d\n", score);

    deinit_flashsim();

    return ret;
}
