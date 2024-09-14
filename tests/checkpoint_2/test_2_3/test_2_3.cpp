#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#define BLOCK_SIZE 128
#define BLOCK_ERASES 500
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
    TEST_PAGE_TYPE data[BLOCK_SIZE];
    FlashSimTest test(argv[1]);
    for (size_t addr = 0; addr < BLOCK_SIZE; addr++) {
        const TEST_PAGE_TYPE page_value = rand() % 18746;
        data[addr] = page_value;
        r = test.Write(log_file_stream, addr, page_value);
        if (r != 1) goto failed;
    }

    // Verify that, with GC, one block can handle BLOCK_ERASES * BLOCK_SIZE writes
    for (int i = 0; i < BLOCK_ERASES; i++) {
        for (int j = 0; j < BLOCK_SIZE; j++) {
            // always writing to the 1st block
            const size_t addr = rand() % BLOCK_SIZE;
            const TEST_PAGE_TYPE page_value1 = rand() % 18746;
            TEST_PAGE_TYPE page_value2;
            r = test.Write(log_file_stream, addr, page_value1);
            if (r != 1) goto failed;
            r = test.Read(log_file_stream, addr, &page_value2);
            if (r != 1) goto failed;

            if (page_value1 != page_value2) {
                fprintf(log_file_stream, "Reading LBA %zu does not get the right value\n", addr);
                goto failed;
            }

            data[addr] = page_value1;
        }
    }

    for (size_t addr = 0; addr < BLOCK_SIZE; addr++) {
        TEST_PAGE_TYPE page_value;
        r = test.Read(log_file_stream, addr, &page_value);
        if (r != 1) goto failed;
        if (page_value != data[addr]) {
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
