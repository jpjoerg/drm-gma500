/*
 * Copyright 2013 Patrik Jakobsson 
 * All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * VA LINUX SYSTEMS AND/OR ITS SUPPLIERS BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef __UAPI_GMA_DRM_H__
#define __UAPI_GMA_DRM_H__

#include <drm/drm.h>

#define DRM_GMA_PARAM_CHIP_ID		0x00

struct drm_gma_param {
	__u64 param;
	__u64 value;
};

/* Buffer object types */
#define GMA_BO_STOLEN	0x01
#define GMA_BO_DISPLAY	0x02
#define GMA_BO_CURSOR	0x03
#define GMA_BO_OVERLAY	0x04
#define GMA_BO_BLIT	0x05

struct drm_gma_gem_create {
	__u32 size;	/* In/out */
	__u32 type;
	__u32 flags;
	__u32 handle;	/* Out */
};

struct drm_gma_gem_mmap {
	__u64 offset;
	__u32 handle;
	__u32 __pad;
};

/* Execute command stream in GEM buffer on the desired engine */
struct drm_gma_gem_blt_submit {
	__u32 size;
	__u32 handle;	/* GEM handle for cmd buffer */
	__u32 flags;
	__u32 __pad;
};

#define DRM_GMA_GET_PARAM	0x00
#define DRM_GMA_SET_PARAM	0x01
#define DRM_GMA_GEM_CREATE	0x02
#define DRM_GMA_GEM_MMAP	0x03
#define DRM_GMA_GEM_BLT_SUBMIT	0x04

#define DRM_IOCTL_GMA_GET_PARAM		DRM_IOWR(DRM_COMMAND_BASE + DRM_GMA_GET_PARAM, struct drm_gma_param)
#define DRM_IOCTL_GMA_SET_PARAM		DRM_IOW (DRM_COMMAND_BASE + DRM_GMA_SET_PARAM, struct drm_gma_param)
#define DRM_IOCTL_GMA_GEM_CREATE	DRM_IOWR(DRM_COMMAND_BASE + DRM_GMA_GEM_CREATE, struct drm_gma_gem_create)
#define DRM_IOCTL_GMA_GEM_MMAP		DRM_IOWR(DRM_COMMAND_BASE + DRM_GMA_GEM_MMAP, struct drm_gma_gem_mmap)
#define DRM_IOCTL_GMA_GEM_BLT_SUBMIT	DRM_IOWR(DRM_COMMAND_BASE + DRM_GMA_GEM_BLT_SUBMIT, struct drm_gma_gem_blt_submit)

#endif
