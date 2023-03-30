#include <stdio.h>
#include <stdint.h>
#include <assert.h>
#include <stdbool.h>

// We assume the CPU is running in little endian mode.

// Useful structures
struct key {
    union {
        uint8_t bytes[8];
        uint64_t raw;
    };
};

struct key_preamble {
    union {
        struct {
            uint8_t low;
            uint8_t high;
        };
        uint16_t raw;
    };
};


// Magic value tables

// Validation function
static uint8_t validate_xor_table[8] = {0x20, 0x1a, 0x47, 0x11, 0x50, 0x93, 0x36, 0x8f};
static uint8_t validate_and_table[8] = {0xec, 0x3f, 0xd7, 0xdb, 0x79, 0xf7, 0xbe, 0xef};

// Preamble gen function
static uint8_t preamble_xor_key_table[8] = {0xfe, 0xff, 0x1c, 0x7c, 0xfc, 0x0c, 0x04, 0x3c};
static uint8_t preamble_and_table[8] = {0x41, 0x10, 0x28, 0x86, 0x08, 0xc0, 0x13, 0x24};

// Validate the key
static bool is_valid_key(struct key k) {
    // Xor index variables
    const uint8_t xor_start_offset = 2;
    uint8_t nb_xor = 1;

    // Key as bytes
    uint8_t* k_b = k.bytes;

    uint8_t k_deriv = 0;
    uint8_t tmp;

    for (uint8_t i = 0; i < 8; ++i) {
        tmp = 0;
        // Xor key bytes
        for (uint8_t j = 0; j < nb_xor; ++j) {
            if (j < 6) {
                tmp ^= k_b[xor_start_offset + j];
            } else if (j == 6) {
                tmp ^= k_b[1];
            } else {
                tmp ^= k_b[0];
            }
        }

        // And op
        tmp &= validate_and_table[i];

        // Xor op
        tmp ^= validate_xor_table[i];

        k_deriv |= tmp;
        ++nb_xor;
    }

    return k_deriv == 0;
}

// Generate the key's preamble
// Precondition: the key should be valid
static uint16_t gen_key_preamble(struct key k) {
    struct key_preamble k_p = {0};

    // Key as bytes
    uint8_t* k_b = k.bytes;

    uint8_t tmp = 0;
    uint8_t tmp2;

    for (uint8_t i = 0; i < 8; ++i) {
        tmp2 = 0;
        if (i == 4) {
            k_p.low = tmp;
            tmp = 0;
        }

        for (uint8_t j = 0; j < 8; ++j) {
            if (preamble_xor_key_table[i] & (1 << j)) {
                tmp2 ^= k_b[j];
            }
        }

        tmp2 &= preamble_and_table[i];
        tmp |= tmp2;
    }

    k_p.high = tmp;

    return k_p.raw;
}

int main() {
    struct key k;

    printf("Key: ");
    assert(scanf("%lx", &k.raw) == 1);
    printf("\n");

    if (is_valid_key(k)) {
        uint16_t k_preamble = gen_key_preamble(k);

        printf("The provided key is correct, and the corresponding preamble is: 0x%x\n", k_preamble);
    } else {
        printf("Error: invalid key\n");
        return 1;
    }

    return 0;
}
