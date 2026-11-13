#include <stdio.h>
#include <stdint.h>

#include "hcsr04.h"
#include "hcsr04_capture.h"

#define CHECK(condition) do { \
	if (!(condition)) { \
		printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition); \
		return 1; \
	} \
} while (0)

int main(void)
{
	uint16_t distance = 123U;
	uint32_t width = 77U;
	hcsr04_capture_t capture = {0};

	CHECK(hcsr04_distance_mm(1000U, 343000U, &distance));
	CHECK(distance == 172U);
	CHECK(hcsr04_distance_mm(5000U, 343000U, &distance));
	CHECK(distance == 858U);
	distance = 123U;
	CHECK(!hcsr04_distance_mm(0U, 343000U, &distance));
	CHECK(distance == 123U);
	CHECK(!hcsr04_distance_mm(1000U, 0U, &distance));
	CHECK(!hcsr04_distance_mm(UINT32_MAX, 400000U, &distance));
	CHECK(distance == 123U);
	puts("[TEST] OBS-U07 PASS");

	CHECK(!hcsr04_capture_init(&capture, 0U, 1000U));
	CHECK(hcsr04_capture_init(&capture, 30000U, 25000U));
	CHECK(!hcsr04_capture_init(&capture, 30000U, 25000U));
	CHECK(!hcsr04_capture_init(&(hcsr04_capture_t){0}, 30000U, 30001U));
	CHECK(hcsr04_capture_poll(&capture, false, 1U, &width) ==
	      HCSR04_CAPTURE_INVALID);
	CHECK(width == 77U);
	CHECK(hcsr04_capture_start(&capture, UINT32_MAX - 100U));
	CHECK(hcsr04_capture_poll(&capture, false, UINT32_MAX - 50U, &width) ==
	      HCSR04_CAPTURE_PENDING);
	CHECK(hcsr04_capture_poll(&capture, true, UINT32_MAX - 20U, &width) ==
	      HCSR04_CAPTURE_PENDING);
	CHECK(hcsr04_capture_poll(&capture, true, 29U, &width) ==
	      HCSR04_CAPTURE_PENDING);
	CHECK(hcsr04_capture_poll(&capture, false, 879U, &width) ==
	      HCSR04_CAPTURE_READY);
	CHECK(width == 900U);
	CHECK(hcsr04_capture_poll(&capture, false, 900U, &width) ==
	      HCSR04_CAPTURE_READY);
	CHECK(width == 900U);
	CHECK(hcsr04_capture_start(&capture, 1000U));
	CHECK(hcsr04_capture_poll(&capture, false, 31001U, &width) ==
	      HCSR04_CAPTURE_TIMEOUT);
	CHECK(capture.state == HCSR04_CAPTURE_FAULT);
	CHECK(width == 900U);
	CHECK(!hcsr04_capture_start(&capture, 32000U));
	CHECK(hcsr04_capture_reset(&capture));
	CHECK(hcsr04_capture_start(&capture, 32000U));
	CHECK(hcsr04_capture_reset(&capture));
	CHECK(capture.state == HCSR04_CAPTURE_IDLE);
	CHECK(hcsr04_capture_start(&capture, 33000U));
	CHECK(hcsr04_capture_reset(&capture));
	puts("[TEST] OBS-U08 PASS");
	return 0;
}
