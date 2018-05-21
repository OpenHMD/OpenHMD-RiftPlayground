/*
 * LED pattern detection and identification
 * Copyright 2015 Philipp Zabel
 * SPDX-License-Identifier:	LGPL-2.0+ or BSL-1.0
 */
#ifndef __FLICKER_H__
#define __FLICKER_H__

struct blob;
struct leds;

void flicker_process(struct blob *blobs, int num_blobs, int skipped,
		     int num_patterns, const uint16_t *patterns);

#endif /* __BLOBWATCH_H__*/
