/*
 * Copyright (c) 2007-2011, Intel Corporation.
 * Copyright (c) 2014, Patrik Jakobsson
 * All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * Authors: Patrik Jakobsson <patrik.r.jakobsson@gmail.com>
 */

#include "psb_drv.h"

#include "blitter.h"
#include "psb_reg.h"

/* Wait for the blitter to be completely idle */
int gma_blt_wait_idle(struct drm_psb_private *dev_priv)
{
	unsigned long stop = jiffies + HZ;
	int busy = 1;

	/* First do a quick check */
	if ((PSB_RSGX32(PSB_CR_2D_SOCIF) == _PSB_C2_SOCIF_EMPTY) &&
	    ((PSB_RSGX32(PSB_CR_2D_BLIT_STATUS) & _PSB_C2B_STATUS_BUSY) == 0))
		return 0;

	do {
		busy = (PSB_RSGX32(PSB_CR_2D_SOCIF) != _PSB_C2_SOCIF_EMPTY);
	} while (busy && !time_after_eq(jiffies, stop));

	if (busy)
		return -EBUSY;

	do {
		busy = ((PSB_RSGX32(PSB_CR_2D_BLIT_STATUS) &
			_PSB_C2B_STATUS_BUSY) != 0);
	} while (busy && !time_after_eq(jiffies, stop));

	/* If still busy, we probably have a hang */
	return (busy) ? -EBUSY : 0;
}

static int gma_blt_wait_available(struct drm_psb_private *dev_priv,
				  unsigned size)
{
	uint32_t avail = PSB_RSGX32(PSB_CR_2D_SOCIF);
	unsigned long t = jiffies + HZ;

	while (avail < size) {
		avail = PSB_RSGX32(PSB_CR_2D_SOCIF);
		if (time_after(jiffies, t))
			return -EIO;
	}
	return 0;
}

/**
 *	gma_blt_submit - Send the blitter commands to hardware
 *	@dev_priv: our DRM device
 *	@cmdbuf: command to issue
 *	@size: length (in dwords)
 */
static int gma_blt_send(struct drm_psb_private *dev_priv, uint32_t *cmdbuf,
			unsigned size)
{
	int ret = 0;
	int i;
	unsigned submit_size;
	unsigned long flags;

	spin_lock_irqsave(&dev_priv->lock_2d, flags);

	while (size > 0) {
		submit_size = (size < 0x60) ? size : 0x60;
		size -= submit_size;
		ret = gma_blt_wait_available(dev_priv, submit_size);
		submit_size <<= 2;

		for (i = 0; i < submit_size; i += 4) {
			PSB_WSGX32(*cmdbuf++, PSB_SGX_2D_SLAVE_PORT + i);
		}

		(void)PSB_RSGX32(PSB_SGX_2D_SLAVE_PORT + i - 4);
	}

	/* We currently sync our blits here */
	ret = gma_blt_wait_idle(dev_priv);
	if (ret) {
		DRM_ERROR("Blitter hang!");
		goto out;
	}

out:
	spin_unlock_irqrestore(&dev_priv->lock_2d, flags);

	return ret;
}

static inline int gma_blt_reloc_surf(struct drm_file *file,
				     struct drm_device *dev, u32 *cmdbuf,
				     u32 pos, u32 size,
				     struct drm_gem_object **obj,
				     u32 *gpu_addr)
{
	struct gtt_range *gt;

	/* FIXME: Check cmdbuf bounds */
	*obj = drm_gem_object_lookup(dev, file, cmdbuf[pos]);
	if (!*obj)
		return -EINVAL;

	gt = container_of(*obj, struct gtt_range, gem);
	psb_gtt_pin(gt); /* Unpinned when blit is completed */
	*gpu_addr = gt->offset + cmdbuf[pos + 1]; /* gt->offset + offset */

	return 0;
}

/* Replace gem handles with gpu offsets and take gem obj refs */
static int gma_blt_reloc(struct drm_file *file, struct drm_device *dev,
			 u32 *cmdbuf, u32 *size,
			 struct drm_gem_object **obj_src,
			 struct drm_gem_object **obj_dst,
			 struct drm_gem_object **obj_mask)
{
	u32 in, out, gpu_addr;
	int ret = 0;

	*obj_src = *obj_dst = *obj_mask = NULL;

	/* Check the CS for relocation entries and restuff it */
	in = out = 0;
	while (in < *size) {
		cmdbuf[out] = cmdbuf[in];

		switch (cmdbuf[in] & PSB_2D_BH_MASK) {
		case PSB_2D_DST_SURF_BH:
			ret = gma_blt_reloc_surf(file, dev, cmdbuf, in + 1,
						 *size, obj_dst, &gpu_addr);
			if (ret) {
				DRM_DEBUG_DRIVER("Reloc of destination failed");
				goto out_err;
			}
			in += 2;
			out += 1;
			cmdbuf[out] = gpu_addr;
			break;
		case PSB_2D_SRC_SURF_BH:
			ret = gma_blt_reloc_surf(file, dev, cmdbuf, in + 1,
						 *size, obj_src, &gpu_addr);
			if (ret) {
				DRM_DEBUG_DRIVER("Reloc of source failed");
				goto out_err;
			}
			in += 2;
			out += 1;
			cmdbuf[out] = gpu_addr;
			break;
		case PSB_2D_MASK_SURF_BH:
			ret = gma_blt_reloc_surf(file, dev, cmdbuf, in + 1,
						 *size, obj_mask, &gpu_addr);
			if (ret) {
				DRM_DEBUG_DRIVER("Reloc of mask failed");
				goto out_err;
			}
			in += 2;
			out += 1;
			cmdbuf[out] = gpu_addr;
			break;
		}
		in++;
		out++;
	}

	*size = out;

out_err:
	return ret;
}

int gma_blt_submit(struct drm_file *file, struct drm_device *dev,
		   u32 size, u32 cs_handle)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct drm_gem_object *obj_cs, *obj_src, *obj_dst, *obj_mask;
	struct gtt_range *gt;
	uint32_t *cmdbuf;
	int ret = 0;
	static int blit_num = 0;

	obj_src = obj_dst = obj_mask = NULL;

	/* At the moment we don't take a CS bigger than the slave port width */
	if (size > 96)
		return -EINVAL;

	obj_cs = drm_gem_object_lookup(dev, file, cs_handle);
	if (!obj_cs) {
		DRM_ERROR("Failed to lookup CS handle");
		return -ENOENT;
	}

	gt = container_of(obj_cs, struct gtt_range, gem);

	/* The CS always fits in a single page */
	cmdbuf = kmap(gt->pages[0]);
	if (!cmdbuf) {
		DRM_ERROR("Failed to map CS");
		ret = -ENOMEM;
		goto unref;
	}

	ret = gma_blt_reloc(file, dev, cmdbuf, &size,
			   &obj_src, &obj_dst, &obj_mask);
	if (ret) {
		DRM_ERROR("Failed to relocate CS buffers");
		goto unmap;
	}

	ret = gma_blt_send(dev_priv, cmdbuf, size);
	if (ret)
		goto unmap;

	blit_num++;

	/* Blit is done so release everything */
unmap:
	kunmap(gt->pages[0]);

unref:
	if (obj_cs)
		drm_gem_object_unreference(obj_cs);
	if (obj_src) {
		gt = container_of(obj_src, struct gtt_range, gem);
		psb_gtt_unpin(gt);
		drm_gem_object_unreference(obj_src);
	}
	if (obj_dst) {
		gt = container_of(obj_dst, struct gtt_range, gem);
		psb_gtt_unpin(gt);
		drm_gem_object_unreference(obj_dst);
	}
	if (obj_mask) {
		gt = container_of(obj_mask, struct gtt_range, gem);
		psb_gtt_unpin(gt);
		drm_gem_object_unreference(obj_mask);
	}

	return ret;
}
