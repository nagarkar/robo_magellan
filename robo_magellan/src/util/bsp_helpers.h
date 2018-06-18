/*
 * bsp_helpers.cpp
 *
 *  Created on: Jun 17, 2018
 *      Author: chinm_000
 */

#pragma once
#include <bbahrs/madgwick_mahony_common.h>
#include <iostream>
#include <stdlib.h>
#include <string.h>      // for memcpy() and memset()
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>
#include <math.h>


static void normalizeQ(Q_cxyz &q) {
	float norm = q.c*q.c + q.x*q.x + q.y*q.y + q.z*q.z;
	float norm_root = sqrtf(norm);
	q.c = q.c / norm_root;
	q.x = q.x / norm_root;
	q.y = q.y / norm_root;
	q.z = q.z / norm_root;
}

uint64_t BSP_nanos() {
	struct timespec spec;
	clock_gettime(CLOCK_REALTIME, &spec);
	return spec.tv_sec * 1.0e9 + spec.tv_nsec;
}

uint64_t BSP_millis() {
	return BSP_nanos() / 1.0e6;
}
