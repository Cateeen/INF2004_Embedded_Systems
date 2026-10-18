
#include <assert.h>
#include <stdio.h>

#include "encoder.h"

int main(void)
{
    encoder_t encoder;

    encoder_init(&encoder, 0U, 0U);

    assert(encoder_get_count(&encoder) == 0);
    printf("PASS: Encoder initialisation\n");

    /* One complete positive quadrature cycle. */
    encoder_update(&encoder, 0U, 1U);
    encoder_update(&encoder, 1U, 1U);
    encoder_update(&encoder, 1U, 0U);
    encoder_update(&encoder, 0U, 0U);

    assert(encoder_get_count(&encoder) == 4);
    printf("PASS: Forward quadrature sequence\n");

    /* Reverse cycle should return to zero. */
    encoder_update(&encoder, 1U, 0U);
    encoder_update(&encoder, 1U, 1U);
    encoder_update(&encoder, 0U, 1U);
    encoder_update(&encoder, 0U, 0U);

    assert(encoder_get_count(&encoder) == 0);
    printf("PASS: Reverse quadrature sequence\n");

    /* A simultaneous two-bit transition is invalid. */
    encoder_update(&encoder, 1U, 1U);

    assert(encoder_get_count(&encoder) == 0);
    printf("PASS: Invalid transition ignored\n");

    encoder_reset_count(&encoder);

    assert(encoder_get_count(&encoder) == 0);
    printf("PASS: Encoder count reset\n");

    printf("\nAll encoder tests passed!\n");

    return 0;
}
