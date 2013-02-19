/*
 * USB HOST XHCI Controller stack
 *
 * Copyright (C) 2012 Samsung Electronics Co.Ltd
 *	Vivek Gautam <gautam.vivek@samsung.com>
 *	Vikas Sajjan <vikas.sajjan@samsung.com>
 *
 * Based on xHCI host controller driver in linux-kernel
 * by Sarah Sharp.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301 USA
 */

/**
 * This file gives the xhci stack for usb3.0 looking into
 * xhci specification Rev1.0 (5/21/10).
 * The quirk devices support hasn't been given yet.
 */

#include <common.h>
#include <asm/byteorder.h>
#include <usb.h>
#include <asm/io.h>
#include <malloc.h>
#include <watchdog.h>
#include <asm/cache.h>
#include <asm-generic/errno.h>
#include "xhci.h"

#ifndef CONFIG_USB_MAX_CONTROLLER_COUNT
#define CONFIG_USB_MAX_CONTROLLER_COUNT 1
#endif

#define XHCI_TRB 0xC0DEABCD
#define XHCI_CTX 0xCAFEABCD

static struct descriptor {
	struct usb_hub_descriptor hub;
	struct usb_device_descriptor device;
	struct usb_linux_config_descriptor config;
	struct usb_linux_interface_descriptor interface;
	struct usb_endpoint_descriptor endpoint;
	struct usb_ss_ep_comp_descriptor ep_companion;
} __attribute__ ((packed)) descriptor = {
	{
		0xc,		/* bDescLength */
		0x2a,		/* bDescriptorType: hub descriptor */
		2,		/* bNrPorts -- runtime modified */
		0,		/* wHubCharacteristics */
		10,		/* bPwrOn2PwrGood */
		0,		/* bHubCntrCurrent */
		{},		/* Device removable */
		{}		/* at most 7 ports! XXX */
	},
	{
		0x12,		/* bLength */
		1,		/* bDescriptorType: UDESC_DEVICE */
		cpu_to_le16(0x0300), /* bcdUSB: v3.0 */
		9,		/* bDeviceClass: UDCLASS_HUB */
		0,		/* bDeviceSubClass: UDSUBCLASS_HUB */
		3,		/* bDeviceProtocol: UDPROTO_SSHUBSTT */
		9,		/* bMaxPacketSize: 512 bytes  2^9 */
		0x0000,		/* idVendor */
		0x0000,		/* idProduct */
		cpu_to_le16(0x0100), /* bcdDevice */
		1,		/* iManufacturer */
		2,		/* iProduct */
		0,		/* iSerialNumber */
		1		/* bNumConfigurations: 1 */
	},
	{
		0x9,
		2,		/* bDescriptorType: UDESC_CONFIG */
		cpu_to_le16(0x19),
		1,		/* bNumInterface */
		1,		/* bConfigurationValue */
		0,		/* iConfiguration */
		0x40,		/* bmAttributes: UC_SELF_POWER */
		0		/* bMaxPower */
	},
	{
		0x9,		/* bLength */
		4,		/* bDescriptorType: UDESC_INTERFACE */
		0,		/* bInterfaceNumber */
		0,		/* bAlternateSetting */
		1,		/* bNumEndpoints */
		9,		/* bInterfaceClass: UICLASS_HUB */
		0,		/* bInterfaceSubClass: UISUBCLASS_HUB */
		0,		/* bInterfaceProtocol: UIPROTO_HSHUBSTT */
		0		/* iInterface */
	},
	{
		0x7,		/* bLength */
		5,		/* bDescriptorType: UDESC_ENDPOINT */
		0x81,		/* bEndpointAddress: IN endpoint 1 */
		3,		/* bmAttributes: UE_INTERRUPT */
		8,		/* wMaxPacketSize */
		255		/* bInterval */
	},
	{
		0x06,		/* ss_bLength */
		0x30,		/* ss_bDescriptorType: SS EP Companion */
		0x00,		/* ss_bMaxBurst: allows 1 TX between ACKs */
		/* ss_bmAttributes: 1 packet per service interval */
		0x00,
		/* ss_wBytesPerInterval: 15 bits for max 15 ports */
		0x02,
	},
};

struct xhci_ctrl xhcic[CONFIG_USB_MAX_CONTROLLER_COUNT];

/**
 * Aligns the given length to 64 bytes
 *
 * @param length	length to be aligned
 * @return aligned length
 */
unsigned int xhci_getalignedlength(unsigned int length)
{

	unsigned int aligned_length = 0;
	unsigned int remain  = 0;
	unsigned int toAdd  = 0;

	remain = (length % 0x40);
	toAdd  = remain ? (0x40 - remain) : 0;
	aligned_length  = length + toAdd;

	return aligned_length;
}

/**
 * flushs/invalidates the address passed till the length
 *
 * @param addr	pointer to "segement" to be freed
 * @param type_len	holds either the Data structure type
 * OR the length of the cache line to be flushed/invalidated
 * @param flush	boolean to hold the flag for flushing/invalidating
 * @return none
 */
void xhci_flush_inval_cache(uint32_t addr, u32 type_len, bool flush)
{

	if ((void *)addr == NULL || flush > 1 || type_len == 0) {
		debug("addr %u  flush %d type_len %u\n", addr, flush, type_len);
		return;
	}

	/*
	 * "type_len" holds either the Data structure type
	 * OR
	 * the length of the cache line to be flushed/invalidated
	 */
	switch(type_len){

	case XHCI_TRB:
		if (flush) {
			flush_dcache_range(addr, addr +
			xhci_getalignedlength(sizeof(union xhci_trb)));
		} else {
			invalidate_dcache_range(addr, addr +
			xhci_getalignedlength(sizeof(union xhci_trb)));
		}
		break;
	case XHCI_CTX:
		if (flush) {
			flush_dcache_range(addr, addr +
			xhci_getalignedlength(sizeof(struct xhci_container_ctx)));
		} else {
			invalidate_dcache_range(addr, addr +
			xhci_getalignedlength(sizeof(struct xhci_container_ctx)));
		}
		break;
	default:
		if (flush) {
			flush_dcache_range(addr, addr +
			xhci_getalignedlength(type_len));
		} else {
			invalidate_dcache_range(addr, addr +
			xhci_getalignedlength(type_len));
		}
		break;
	}

	return;
}

/**
 * frees the "segment" pointer passed
 *
 * @param ptr	pointer to "segement" to be freed
 * @return none
 */
static void xhci_segment_free(struct xhci_segment *seg)
{
	if (seg->trbs) {
		free(seg->trbs);
		seg->trbs = NULL;
	}

	free(seg);
}

/**
 * frees the "ring" pointer passed
 *
 * @param ptr	pointer to "ring" to be freed
 * @return none
 */
void xhci_ring_free(struct xhci_ring *ring)
{
	struct xhci_segment *seg;
	struct xhci_segment *first_seg;

	if (!ring)
		return;
	if (ring->first_seg) {
		first_seg = ring->first_seg;
		seg = first_seg->next;
		while (seg != first_seg) {
			struct xhci_segment *next = seg->next;
			xhci_segment_free(seg);
			seg = next;
		}
		xhci_segment_free(first_seg);
		ring->first_seg = NULL;
	}

	free(ring);
}

/**
 * frees the "xhci_container_ctx" pointer passed
 *
 * @param ptr	pointer to "xhci_container_ctx" to be freed
 * @return none
 */
static void xhci_free_container_ctx(struct xhci_container_ctx *ctx)
{
	if (!ctx)
		return;
	free(ctx->bytes);
	free(ctx);
}

/**
 * frees the virtual devices for "xhci_ctrl" pointer passed
 *
 * @param ptr	pointer to "xhci_ctrl" to be freed
 * @return 0 for Success freeing else 1 if the pointer or slot id is invalid.
 */
int xhci_free_virt_device(struct xhci_ctrl *ctrl)
{
	int i;
	int slot_id = 0;
	struct xhci_virt_device *virt_dev;

	if (!ctrl)
		return -1;

	slot_id = ctrl->slot_id;

	/* Slot ID 0 is reserved */
	if (slot_id == 0 || !ctrl->devs[slot_id]) {
		debug("Bad Slot ID %d\n", slot_id);
		return -1;
	}

	virt_dev = ctrl->devs[slot_id];

	ctrl->dcbaa->dev_context_ptrs[slot_id] = 0;

	if (!virt_dev)
		return -1;

	for (i = 0; i < 31; ++i) {
		if (virt_dev->eps[i].ring)
			xhci_ring_free(virt_dev->eps[i].ring);
	}

	if (virt_dev->in_ctx)
		xhci_free_container_ctx(virt_dev->in_ctx);
	if (virt_dev->out_ctx)
		xhci_free_container_ctx(virt_dev->out_ctx);

	free(ctrl->devs[slot_id]);
	ctrl->devs[slot_id] = NULL;

	return 0;
}

/**
 * frees all the memory allocated
 *
 * @param ptr	pointer to "xhci_ctrl" to be cleaned up
 * @return none
 */
void xhci_cleanup(struct xhci_ctrl *ctrl)
{
	xhci_ring_free(ctrl->event_ring);
	xhci_ring_free(ctrl->cmd_ring);
	xhci_free_virt_device(ctrl);
	free(ctrl->erst.entries);
	free(ctrl->dcbaa);
	memset(ctrl, '\0', sizeof(struct xhci_ctrl));
}

/**
 * Malloc the aligned memory
 *
 * @param ptr	pointer to "xhci_ctrl" to be cleaned up
 * @return allocates the memory and returns the aligned pointer
 */
void *xhci_malloc(unsigned int size)
{
	void *ptr = memalign(XHCI_ALIGNMENT, size);
	memset(ptr, '\0', size);
	return ptr;
}

/**
 * Waits for as per specified amount of time
 * for the "result" to match with "done"
 *
 * @param ptr	pointer to the register to be read
 * @param mask	mask for the value read
 * @param done	value to be campared with result
 * @param usec	time to wait till
 * @return 0 if handshake is success else -1 on failure
 */
static int handshake(uint32_t volatile *ptr, uint32_t mask,
					uint32_t done, int usec)
{
	uint32_t result;

	do {
		result = xhci_readl(ptr);
		if (result == ~(uint32_t)0)
			return -ENODEV;
		result &= mask;
		if (result == done)
			return 0;
		usec--;
		udelay(1);
	} while (usec > 0);

	return -ETIMEDOUT;
}

/**
 * Disable interrupts and begin the xHCI halting process.
 *
 * @param hcor	pointer to host controller operation registers
 * @return none
 */
void xhci_quiesce(struct xhci_hcor *hcor)
{
	uint32_t halted;
	uint32_t cmd;
	uint32_t mask;

	mask = ~(XHCI_IRQS);
	halted = xhci_readl(&hcor->or_usbsts) & STS_HALT;
	if (!halted)
		mask &= ~CMD_RUN;

	cmd = xhci_readl(&hcor->or_usbcmd);
	cmd &= mask;
	xhci_writel(&hcor->or_usbcmd, cmd);
}

/**
 * Force HC into halt state.
 * Disable any IRQs and clear the run/stop bit.
 * HC will complete any current and actively pipelined transactions, and
 * should halt within 16 ms of the run/stop bit being cleared.
 * Read HC Halted bit in the status register to see when the HC is finished.
 *
 * @param hcor	pointer to host controller operation registers
 * @return status of the handshake
 */
int xhci_halt(struct xhci_hcor *hcor)
{
	int ret;
	debug("Halt the HC\n");
	xhci_quiesce(hcor);

	ret = handshake(&hcor->or_usbsts,
			STS_HALT, STS_HALT, XHCI_MAX_HALT_USEC);
	if (ret)
		debug("Host not halted after %u microseconds.\n",
				XHCI_MAX_HALT_USEC);
	return ret;
}

/**
 * Set the run bit and wait for the host to be running.
 *
 * @param hcor	pointer to host controller operation registers
 * @return status of the Handshake
 */
static int xhci_start(struct xhci_hcor *hcor)
{
	u32 temp;
	int ret;

	printf("Starting the controller\n");
	temp = xhci_readl(&hcor->or_usbcmd);
	temp |= (CMD_RUN);
	xhci_writel(&hcor->or_usbcmd, temp);
	temp = xhci_readl(&hcor->or_usbcmd);

	/*
	 * Wait for the HCHalted Status bit to be 0 to indicate the host is
	 * running.
	 */
	ret = handshake(&hcor->or_usbsts, STS_HALT, 0, XHCI_MAX_HALT_USEC);
	if (ret)
		debug("Host took too long to start, "
				"waited %u microseconds.\n",
				XHCI_MAX_HALT_USEC);
	return ret;
}

/**
 * Resets the XHCI Controller
 *
 * @param hcor	pointer to host controller operation registers
 * @return -1 if XHCI Controller is halted else status of handshake
 */
int xhci_reset(struct xhci_hcor *hcor)
{
	u32 command;
	u32 state;
	int ret;

	state = xhci_readl(&hcor->or_usbsts);
	if ((state & STS_HALT) == 0) {
		debug("Host controller not halted, aborting reset.\n");
		return -1;
	}

	debug("// Reset the HC\n");
	command = xhci_readl(&hcor->or_usbcmd);
	command |= CMD_RESET;
	xhci_writel(&hcor->or_usbcmd, command);

	ret = handshake(&hcor->or_usbcmd, CMD_RESET, 0, XHCI_MAX_RESET_USEC);
	if (ret)
		return ret;

	/*
	 * xHCI cannot write to any doorbells or operational registers other
	 * than status until the "Controller Not Ready" flag is cleared.
	 */
	return handshake(&hcor->or_usbsts, STS_CNR, 0, XHCI_MAX_RESET_USEC);
}

/**
 * Allocates a generic ring segment from the ring pool, sets the dma address,
 * initializes the segment to zero, and sets the private next pointer to NULL.
 * Section 4.11.1.1:
 * "All components of all Command and Transfer TRBs shall be initialized to '0'"
 *
 * @param	none
 * @return pointer to the newly allocated SEGMENT
 */
static struct xhci_segment *xhci_segment_alloc(void)
{
	struct xhci_segment *seg;

	seg = (struct xhci_segment *)xhci_malloc(sizeof(struct xhci_segment));
	if (!seg)
		return NULL;

	seg->trbs = (union xhci_trb *)xhci_malloc(SEGMENT_SIZE);

	if (!seg->trbs)
		return NULL;

	seg->next = NULL;

	return seg;
}

/**
 * Make the prev segment point to the next segment.
 * Change the last TRB in the prev segment to be a Link TRB which points to the
 * address of the next segment.  The caller needs to set any Link TRB
 * related flags, such as End TRB, Toggle Cycle, and no snoop.
 *
 * @param prev	pointer to the previous segment
 * @param next	pointer to the next segment
 * @param link_trbs	flag to indicate whether to link the trbs or NOT
 * @return none
 */
static void xhci_link_segments(struct xhci_segment *prev,
				struct xhci_segment *next, bool link_trbs)
{
	u32 val;
	u64 val_64 = 0;

	if (!prev || !next)
		return;
	prev->next = next;
	if (link_trbs) {
		val_64 = (uintptr_t)next->trbs;
		prev->trbs[TRBS_PER_SEGMENT-1].link.segment_ptr = val_64;

		/*
		 * Set the last TRB in the segment to
		 * have a TRB type ID of Link TRB
		 */
		val = le32_to_cpu(prev->trbs[TRBS_PER_SEGMENT-1].link.control);
		val &= ~TRB_TYPE_BITMASK;
		val |= (TRB_LINK << TRB_TYPE_SHIFT);

		/*
		 * Always set the chain bit with 0.95 hardware
		 * Set chain bit for isoc rings on AMD 0.96 host
		 * NOT SUPPORTING ANY QUIRK DEVICE
		 */
		prev->trbs[TRBS_PER_SEGMENT-1].link.control = cpu_to_le32(val);
	}
}

/**
 * Initialises the Ring's enqueue,dequeue,enq_seg pointers
 *
 * @param ring	pointer to the RING to be intialised
 * @return none
 */
static void xhci_initialize_ring_info(struct xhci_ring *ring)
{
	/*
	 * The ring is empty, so the enqueue pointer == dequeue pointer
	 */
	ring->enqueue = ring->first_seg->trbs;
	ring->enq_seg = ring->first_seg;
	ring->dequeue = ring->enqueue;
	ring->deq_seg = ring->first_seg;

	/*
	 * The ring is initialized to 0. The producer must write 1 to the
	 * cycle bit to handover ownership of the TRB, so PCS = 1.
	 * The consumer must compare CCS to the cycle bit to
	 * check ownership, so CCS = 1.
	 */
	ring->cycle_state = 1;
	/*
	 * Not necessary for new rings, but needed for re-initialized rings
	 */
	ring->enq_updates = 0;
	ring->deq_updates = 0;
}

/**
 * Create a new ring with zero or more segments.
 *
 * Link each segment together into a ring.
 * Set the end flag and the cycle toggle bit on the last segment.
 * See section 4.9.1 and figures 15 and 16.
 *
 * @param num_segs	number of segments in the ring
 * @param link_trbs	flag to indicate whether to link the trbs or NOT
 * @return pointer to the newly created RING
 */
static struct xhci_ring *xhci_ring_alloc(unsigned int num_segs, bool link_trbs)
{
	struct xhci_ring *ring;
	struct xhci_segment *prev;

	ring = (struct xhci_ring *)malloc(sizeof(struct xhci_ring));
	if (!ring)
		return NULL;

	if (num_segs == 0)
		return ring;

	ring->first_seg = xhci_segment_alloc();
	if (!ring->first_seg)
		return NULL;
	num_segs--;

	prev = ring->first_seg;
	while (num_segs > 0) {
		struct xhci_segment *next;

		next = xhci_segment_alloc();
		if (!next)
			return NULL;
		xhci_link_segments(prev, next, link_trbs);

		prev = next;
		num_segs--;
	}
	xhci_link_segments(prev, ring->first_seg, link_trbs);
	if (link_trbs) {
		/* See section 4.9.2.1 and 6.4.4.1 */
		prev->trbs[TRBS_PER_SEGMENT-1].link.control |=
					cpu_to_le32(LINK_TOGGLE);
	}
	xhci_initialize_ring_info(ring);

	return ring;
}

/**
 * Checks whether the enqueue trb is a link trb or NOT
 *
 * @param ring	pointer to the RING
 * @return 1 if the enqueue TRB is the link TRB else 0
 */
static int enqueue_is_link_trb(struct xhci_ring *ring)
{
	struct xhci_link_trb *link = &ring->enqueue->link;

	return ((link->control & cpu_to_le32(TRB_TYPE_BITMASK)) ==
			cpu_to_le32(TRB_LINK << TRB_TYPE_SHIFT));
}

/**
 * Is this TRB a link TRB or was the last TRB the last TRB in this event ring
 * segment?  I.e. would the updated event TRB pointer step off the end of the
 * event seg ?
 *
 * @param ctrl	Host controller data structure
 * @param ring	pointer to the ring
 * @param seg	poniter to the segment to which TRB belongs
 * @return 1 if this TRB a link TRB else 0
 */
static int last_trb(struct xhci_ctrl *ctrl, struct xhci_ring *ring,
			struct xhci_segment *seg, union xhci_trb *trb)
{
	if (ring == ctrl->event_ring)
		return trb == &seg->trbs[TRBS_PER_SEGMENT];
	else
		return ((trb->link.control & cpu_to_le32(TRB_TYPE_BITMASK)) ==
				cpu_to_le32(TRB_LINK << TRB_TYPE_SHIFT));
}

/**
 * Does this link TRB point to the first segment in a ring,
 * or was the previous TRB the last TRB on the last segment in the ERST?
 *
 * @param ctrl	Host controller data structure
 * @param ring	pointer to the ring
 * @param seg	poniter to the segment to which TRB belongs
 * @return 1 if this TRB is the last TRB on the last segment else 0
 */
static bool last_trb_on_last_seg(struct xhci_ctrl *ctrl, struct xhci_ring *ring,
				struct xhci_segment *seg, union xhci_trb *trb)
{
	if (ring == ctrl->event_ring)
		return ((trb == &seg->trbs[TRBS_PER_SEGMENT]) &&
			(seg->next == ring->first_seg));
	else
		return le32_to_cpu(trb->link.control) & LINK_TOGGLE;
}

/**
 * See Cycle bit rules. SW is the consumer for the event ring only.
 * Don't make a ring full of link TRBs.  That would be dumb and this would loop.
 *
 * If we've just enqueued a TRB that is in the middle of a TD (meaning the
 * chain bit is set), then set the chain bit in all the following link TRBs.
 * If we've enqueued the last TRB in a TD, make sure the following link TRBs
 * have their chain bit cleared (so that each Link TRB is a separate TD).
 *
 * Section 6.4.4.1 of the 0.95 spec says link TRBs cannot have the chain bit
 * set, but other sections talk about dealing with the chain bit set.  This was
 * fixed in the 0.96 specification errata, but we have to assume that all 0.95
 * xHCI hardware can't handle the chain bit being cleared on a link TRB.
 *
 * @param ctrl	Host controller data structure
 * @param ring	pointer to the ring
 * @param consumer	flag to indicate whether caller is consumer or producer
 * @param more_trbs_coming	flag to indicate whether more trbs
 *				are expected or NOT.
 *				Will you enqueue more TRBs before calling
 *				prepare_ring()?
 * @return none
 */
static void inc_enq(struct xhci_ctrl *ctrl, struct xhci_ring *ring,
				bool consumer, bool more_trbs_coming)
{
	u32 chain;
	union xhci_trb *next;

	chain = le32_to_cpu(ring->enqueue->generic.field[3]) & TRB_CHAIN;
	next = ++(ring->enqueue);

	ring->enq_updates++;
	/*
	 * Update the dequeue pointer further if that was a link TRB or we're at
	 * the end of an event ring segment (which doesn't have link TRBS)
	 */
	while (last_trb(ctrl, ring, ring->enq_seg, next)) {
		if (!consumer) {
			if (ring != ctrl->event_ring) {
				/*
				 * If the caller doesn't plan on enqueueing more
				 * TDs before ringing the doorbell, then we
				 * don't want to give the link TRB to the
				 * hardware just yet.  We'll give the link TRB
				 * back in prepare_ring() just before we enqueue
				 * the TD at the top of the ring.
				 */
			if (!chain && !more_trbs_coming)
				break;

				/*
				 * If we're not dealing with 0.95 hardware or
				 * isoc rings on AMD 0.96 host,
				 * carry over the chain bit of the previous TRB
				 * (which may mean the chain bit is cleared).
				 */
				next->link.control &= cpu_to_le32(~TRB_CHAIN);
				next->link.control |= cpu_to_le32(chain);

				next->link.control ^= cpu_to_le32(TRB_CYCLE);
			}
			/* Toggle the cycle bit after the last ring segment. */
			if (last_trb_on_last_seg(ctrl, ring,
						ring->enq_seg, next))
				ring->cycle_state = (ring->cycle_state ? 0 : 1);
		}
		ring->enq_seg = ring->enq_seg->next;
		ring->enqueue = ring->enq_seg->trbs;
		next = ring->enqueue;

		xhci_flush_inval_cache((uint32_t)ring->enqueue, XHCI_TRB, 1);
	}
}

/**
 * Generic function for queueing a TRB on a ring.
 * The caller must have checked to make sure there's room on the ring.
 *
 * @param	more_trbs_coming:   Will you enqueue more TRBs before calling
 *				prepare_ring()?
 * @param ctrl	Host controller data structure
 * @param ring	pointer to the ring
 * @param consumer	flag to indicate whether caller is consumer or producer
 * @param more_trbs_coming	flag to indicate whether more trbs
 * @param field4	field 4 of the Generic TRB
 * @return pointer to the enqueued trb
 */
static struct xhci_generic_trb *queue_trb(struct xhci_ctrl *ctrl,
					struct xhci_ring *ring,
					bool consumer,
					bool more_trbs_coming,
					u32 *trb_fields)
{
	struct xhci_generic_trb *trb;
	int i;

	trb = &ring->enqueue->generic;

	for (i = 0; i < 4; i++)
		trb->field[i] = cpu_to_le32(trb_fields[i]);

	xhci_flush_inval_cache((uint32_t)trb, XHCI_TRB, 1);

	inc_enq(ctrl, ring, consumer, more_trbs_coming);

	return trb;
}

/**
 * Does various checks on the endpoint ring, and makes it ready
 * to queue num_trbs.
 *
 * @param ctrl		Host controller data structure
 * @param ep_ring	pointer to the EP Transfer Ring
 * @param ep_state	State of the End Point
 * @return none
 */
static int prepare_ring(struct xhci_ctrl *ctrl, struct xhci_ring *ep_ring,
								u32 ep_state)
{
	/* Make sure the endpoint has been added to xHC schedule */
	switch (ep_state) {
	case EP_STATE_DISABLED:
		/*
		 * USB core changed config/interfaces without notifying us,
		 * or hardware is reporting the wrong state.
		 */
		debug("WARN urb submitted to disabled ep\n");
		return -ENOENT;
	case EP_STATE_ERROR:
		debug("WARN waiting for error on ep to be cleared\n");
		return -EINVAL;
	case EP_STATE_HALTED:
		debug("WARN halted endpoint, queueing URB anyway.\n");
	case EP_STATE_STOPPED:
	case EP_STATE_RUNNING:
		debug("EP STATE RUNNING.\n");
		break;
	default:
		debug("ERROR unknown endpoint state for ep\n");
		return -EINVAL;
	}

	if (enqueue_is_link_trb(ep_ring)) {
		struct xhci_ring *ring = ep_ring;
		union xhci_trb *next;

		next = ring->enqueue;

		while (last_trb(ctrl, ring, ring->enq_seg, next)) {
			/*
			 * If we're not dealing with 0.95 hardware or isoc rings
			 * on AMD 0.96 host, clear the chain bit.
			 */
			next->link.control &= cpu_to_le32(~TRB_CHAIN);

			next->link.control ^= cpu_to_le32(TRB_CYCLE);

			/* Toggle the cycle bit after the last ring segment. */
			if (last_trb_on_last_seg(ctrl, ring,
						ring->enq_seg, next))
				ring->cycle_state = (ring->cycle_state ? 0 : 1);
			ring->enq_seg = ring->enq_seg->next;
			ring->enqueue = ring->enq_seg->trbs;
			next = ring->enqueue;
		}
	}

	return 0;
}

/**
 * Generic function for queueing a command TRB on the command ring.
 * Check to make sure there's room on the command ring for one command TRB.
 *
 * @param ctrl	Host controller data structure
 * @param trb_fields	holds the field of the Generic TRB
 * @return
 */
static int queue_command(struct xhci_ctrl *ctrl, u32 *trb_fields)
{
	int ret;
	u32 fields[4];

	ret = prepare_ring(ctrl, ctrl->cmd_ring, EP_STATE_RUNNING);
	if (ret < 0) {
		debug("ERR: No room for command on command ring\n");
		return ret;
	}

	fields[0] = trb_fields[0];
	fields[1] = trb_fields[1];
	fields[2] = trb_fields[2];
	fields[3] = trb_fields[3] | ctrl->cmd_ring->cycle_state;

	queue_trb(ctrl, ctrl->cmd_ring, false, false, fields);

	xhci_flush_inval_cache((uint32_t)ctrl->cmd_ring->enqueue, XHCI_TRB, 1);

	return 0;
}

/**
 * Ring the host controller doorbell after placing a command on the ring
 *
 * @param ctrl	Host controller data structure
 * return none
 */
void xhci_ring_cmd_db(struct xhci_ctrl *ctrl)
{
	xhci_writel(&ctrl->dba->doorbell[0], DB_VALUE_HOST);

	/* Flush PCI posted writes */
	xhci_readl(&ctrl->dba->doorbell[0]);
}

/**
 * Ring the host controller doorbell after placing a command on the ring
 *
 * @param ctrl	Host controller data structure
 * @param slot_id	slot id of the Device
 * @param ep_index	End point to be ringed
 * @param stream_idx	Stream ID
 * return none
 */
void xhci_ring_ep_doorbell(struct xhci_ctrl *ctrl,
				unsigned int slot_id,
				unsigned int ep_index,
				unsigned int stream_id)
{
	xhci_writel(&ctrl->dba->doorbell[slot_id], DB_VALUE(ep_index, 0));
}

/**
 * Give the address of "trb" in the segment "seg"
 *
 * @param seg	pointer to the Segment
 * @param trb	Pointer to the TRB whose address is required
 * @return 0 if the TRB isn't in this segment, otherwise it returns the
 *	     address of the TRB.
 */
unsigned long trb_addr(struct xhci_segment *seg,
				union xhci_trb *trb)
{
	unsigned long segment_offset;

	if (!seg || !trb || trb < seg->trbs)
		return 0;

	/* offset in TRBs */
	segment_offset = trb - seg->trbs;
	if (segment_offset > TRBS_PER_SEGMENT)
		return 0;

	return (unsigned long)(seg->trbs + (segment_offset * sizeof(*trb)));
}

/**
 * The TD size is the number of bytes remaining in the TD (including this TRB),
 * right shifted by 10.
 * It must fit in bits 21:17, so it can't be bigger than 31.
 *
 * @param remainder	remaining packets to be sent
 * @return remainder if remainder is less than  max else max
 */
static u32 xhci_td_remainder(unsigned int remainder)
{
	u32 max = (1 << (21 - 17 + 1)) - 1;

	if ((remainder) >= max)
		return max << 17;
	else
		return (remainder) << 17;
}

/**
 * Used for passing endpoint bitmasks between the core and HCDs.
 * Find the index for an endpoint given its descriptor.
 * Use the return value to right shift 1 for the bitmask.
 *
 * Index  = (epnum * 2) + direction - 1,
 * where direction = 0 for OUT, 1 for IN.
 * For control endpoints, the IN index is used (OUT index is unused), so
 * index = (epnum * 2) + direction - 1 = (epnum * 2) + 1 - 1 = (epnum * 2)
 *
 * @param desc	USB enpdoint Descriptor
 * @param req	Request Type
 * @return index of the Endpoint
 */
unsigned int xhci_get_ep_index(struct usb_endpoint_descriptor *desc,
							struct devrequest *req)
{
	unsigned int index;
	if (req != NULL) {
		index = (unsigned int)((desc->bEndpointAddress &
					USB_ENDPOINT_NUMBER_MASK)*2);
	} else {
		index = (unsigned int)((desc->bEndpointAddress &
					USB_ENDPOINT_NUMBER_MASK)*2) +
					(((desc->bEndpointAddress &
					USB_ENDPOINT_DIR_MASK) == USB_DIR_IN) ?
					1 : 0) - 1;
	}

	return index;
}

/**
 * Ring the doorbell of the End Point
 *
 * @param slot_id	Slod id allocated by XHCI controller
 * @param ep_index	index of the endpoint
 * @param ctrl		Host controller data structure
 * @param start_cycle	cycle flag of the first TRB
 * @param start_trb	pionter to the first TRB
 * @return none
 */
static void giveback_first_trb(struct xhci_ctrl *ctrl, int slot_id,
					unsigned int ep_index,
					unsigned int stream_id,
					int start_cycle,
					struct xhci_generic_trb *start_trb)
{
	int delay;
	/*
	 * Pass all the TRBs to the hardware at once and make sure this write
	 * isn't reordered.
	 */
	if (start_cycle)
		start_trb->field[3] |= cpu_to_le32(start_cycle);
	else
		start_trb->field[3] &= cpu_to_le32(~TRB_CYCLE);

	xhci_flush_inval_cache((uint32_t)start_trb, XHCI_TRB, 1);

	xhci_ring_ep_doorbell(ctrl, slot_id, ep_index, stream_id);

	/*
	 * Not a Super Speed Device ,
	 * then give some time to Devices(FS/HS/LS) to respond
	 */
	delay = (ctrl->speed != USB_SPEED_SUPER) ? 110 : 1;
	mdelay(delay);

	return;
}

/**
 * Gets the EP context from based on the ep_index
 *
 * @param ctrl	Host controller data structure
 * @param ctx	context container
 * @param ep_index	index of the endpoint
 * @return pointer to the End point context
 */
struct xhci_ep_ctx *xhci_get_ep_ctx(struct xhci_ctrl *ctrl,
				    struct xhci_container_ctx *ctx,
				    unsigned int ep_index)
{
	/* increment ep index by offset of start of ep ctx array */
	ep_index++;
	if (ctx->type == XHCI_CTX_TYPE_INPUT)
		ep_index++;

	return (struct xhci_ep_ctx *)
		(ctx->bytes +
		(ep_index * CTX_SIZE(readl(&ctrl->hccr->cr_hccparams))));
}

/**
 * Handles the result of the configure endpoint Request
 *
 * @param ctrl	Host controller data structure
 * @param cmd_status	status of the Event
 * @return negative number if failure 0 on success
 */
static int xhci_configure_endpoint_result(struct xhci_ctrl *ctrl,
						u32 *cmd_status)
{
	int ret = 0;

	switch (*cmd_status) {
	case COMP_ENOMEM:
		debug("Not enough host controller resources"
			"for new device state.\n");
		ret = -ENOMEM;
		break;
	case COMP_BW_ERR:
	case COMP_2ND_BW_ERR:
		debug("Not enough bandwidth for new device state.\n");
		ret = -ENOSPC;
		break;
	case COMP_TRB_ERR:
		/* the HCD set up something wrong */
		debug("ERROR: Endpoint drop flag = 0, add flag = 1,"
			"and endpoint is not disabled.\n");
		ret = -EINVAL;
		break;
	case COMP_DEV_ERR:
		debug("ERROR: Incompatible device"
			"for endpoint configure command.\n");
		ret = -ENODEV;
		break;
	case COMP_SUCCESS:
		debug("Successful Endpoint Configure command\n");
		break;
	default:
		debug("ERROR: unexpected command completion code 0x%x.\n",
			*cmd_status);
		ret = -EINVAL;
		break;
	}

	return ret;
}

/**
 * Handles the result of the Evaluate endpoint Request
 *
 * @param cmd_status	command status to be evaluated
 * @return negative number if failure else 0 on success
 */
static int xhci_evaluate_context_result(u32 *cmd_status)
{
	int ret;

	switch (*cmd_status) {
	case COMP_EINVAL:
		debug("WARN: xHCI driver setup invalid evaluate"
			"context command.\n");
		ret = -EINVAL;
		break;
	case COMP_EBADSLT:
		debug("WARN: slot not enabled for evaluate"
			"context command.\n");
	case COMP_CTX_STATE:
		debug("WARN: invalid context state for evaluate"
			"context command.\n");

		ret = -EINVAL;
		break;
	case COMP_DEV_ERR:
		debug("ERROR: Incompatible device for evaluate"
			"context command.\n");
		ret = -ENODEV;
		break;
	case COMP_MEL_ERR:
		/* Max Exit Latency too large error */
		debug("WARN: Max Exit Latency too large\n");
		ret = -EINVAL;
		break;
	case COMP_SUCCESS:
		debug("Successful evaluate context command\n");
		ret = 0;
		break;
	default:
		debug("ERROR: unexpected command completion code 0x%x.\n",
			*cmd_status);
		ret = -EINVAL;
		break;
	}

	return ret;
}

/**
 * Queue a configure endpoint command TRB
 *
 * @param ctrl	Host controller data structure
 * @param in_ctx_bytes	pointer to input context
 * @param slot_id	slot it allocated by XHCI controller
 * @return status of the queue_command
 */
int xhci_queue_configure_endpoint(struct xhci_ctrl *ctrl, u8 *in_ctx_bytes,
								u32 slot_id)
{
	u64 val_64;
	u32 trb_fields[4];

	val_64 = (uintptr_t)in_ctx_bytes;

	trb_fields[0] = lower_32_bits(val_64);
	trb_fields[1] = upper_32_bits(val_64);
	trb_fields[2] = 0;
	trb_fields[3] = ((TRB_CONFIG_EP << TRB_TYPE_SHIFT) |
			((slot_id & SLOT_ID_FOR_TRB_MASK) <<
			SLOT_ID_FOR_TRB_SHIFT));

	return queue_command(ctrl, trb_fields);
}

/**
 * Queue an evaluate context command TRB
 *
 * @param ctrl	Host controller data structure
 * @param in_ctx_bytes	pointer to input context
 * @param slot_id	slot it allocated by XHCI controller
 * @return status of the queue_command
 */
int xhci_queue_evaluate_context(struct xhci_ctrl *ctrl, u8 *in_ctx_bytes,
								u32 slot_id)
{
	u64 val_64;
	u32 trb_fields[4];

	val_64 = (uintptr_t)in_ctx_bytes;

	trb_fields[0] = lower_32_bits(val_64);
	trb_fields[1] = upper_32_bits(val_64);
	trb_fields[2] = 0;
	trb_fields[3] = ((TRB_EVAL_CONTEXT << TRB_TYPE_SHIFT) |
			((slot_id & SLOT_ID_FOR_TRB_MASK) <<
			SLOT_ID_FOR_TRB_SHIFT));

	return queue_command(ctrl, trb_fields);
}

/**
 * Issue a configure endpoint command or evaluate context command
 * and wait for it to finish.
 *
 * @param usbdev	pointer to the Device Data Structure
 * @param ctx_change	flag to indicate the Context has changed or NOT
 * @return -ENOMEM if xhci_queue_configure_endpoint or
 * 		   xhci_queue_evaluate_context fails else status of
 * 		   xhci_queue_configure_endpoint or xhci_queue_evaluate_context.
 */
static int xhci_configure_endpoint(struct usb_device *usbdev, bool ctx_change)
{
	int ret;
	struct xhci_container_ctx *in_ctx;
	u32 *cmd_status;
	struct xhci_virt_device *virt_dev;
	struct xhci_ctrl *ctrl = usbdev->controller;

	virt_dev = ctrl->devs[ctrl->slot_id];
	in_ctx = virt_dev->in_ctx;

	cmd_status = &virt_dev->cmd_status;

	if (!ctx_change)
		ret = xhci_queue_configure_endpoint(ctrl, in_ctx->bytes,
							ctrl->slot_id);
	else
		ret = xhci_queue_evaluate_context(ctrl, in_ctx->bytes,
							ctrl->slot_id);

	if (ret < 0)
		return -ENOMEM;

	xhci_ring_cmd_db(ctrl);

	/* Give some time to the DEVICE to Respond */
	mdelay(5);

	xhci_poll_and_HandleEvent(usbdev);

	if (!ctx_change)
		ret = xhci_configure_endpoint_result(ctrl, cmd_status);
	else
		ret = xhci_evaluate_context_result(cmd_status);

	return ret;
}

/**
 * Copy output xhci_ep_ctx to the input xhci_ep_ctx copy.
 * Useful when you want to change one particular aspect of the endpoint and then
 * issue a configure endpoint command.

 * Full speed devices may have a max packet size greater than 8 bytes, but the
 * USB core doesn't know that until it reads the first 8 bytes of the
 * descriptor.  If the usb_device's max packet size changes after that point,
 * we need to issue an evaluate context command and wait on it.
 *
 * @param usbdev	pointer to the USB device structure
 * @return returns the status of the xhci_configure_endpoint
 */
static int xhci_configure_ep(struct usb_device *usbdev)
{
	struct xhci_container_ctx *in_ctx;
	struct xhci_container_ctx *out_ctx;
	struct xhci_input_control_ctx *ctrl_ctx;
	struct xhci_slot_ctx *slot_ctx;
	struct xhci_ep_ctx *ep_ctx[MAX_EP_CTX_NUM];
	int ret = 0;
	int cur_ep;
	int max_ep_flag = 0;
	int ep_index;
	unsigned int dir;
	unsigned int ep_type;
	struct xhci_ctrl *ctrl = usbdev->controller;
	int num_of_ep;
	int ep_flag = 0;
	u64 trb_64 = 0;
	int slot_id = ctrl->slot_id;
	struct xhci_virt_device *virt_dev = ctrl->devs[slot_id];
	struct usb_interface *ifdesc;

	out_ctx = virt_dev->out_ctx;
	in_ctx = virt_dev->in_ctx;

	xhci_flush_inval_cache((uint32_t)in_ctx, XHCI_CTX, 0);
	xhci_flush_inval_cache((uint32_t)out_ctx, XHCI_CTX, 0);
	xhci_flush_inval_cache((uint32_t)in_ctx->bytes, in_ctx->size, 0);
	xhci_flush_inval_cache((uint32_t)out_ctx->bytes, out_ctx->size, 0);

	num_of_ep = usbdev->config.if_desc[0].no_of_ep;
	ifdesc = &usbdev->config.if_desc[0];

	/*
	 * STEP 1: Set up the input context flags for the command
	 * FIXME: This won't work if a non-default control endpoint
	 * changes max packet sizes.
	 */
	ctrl_ctx = xhci_get_input_control_ctx(ctrl->devs[slot_id]->in_ctx);

	/* EP_FLAG gives values 1 & 4 for EP1OUT and EP2IN */
	for (cur_ep = 0; cur_ep < num_of_ep; cur_ep++) {
		ep_flag = xhci_get_ep_index(&ifdesc->ep_desc[cur_ep].ep_desc,
									NULL);
		ctrl_ctx->add_flags |= cpu_to_le32(1 << (ep_flag + 1));
		if (max_ep_flag < ep_flag)
			max_ep_flag = ep_flag;
	}
	ctrl_ctx->drop_flags = 0;

	/* STEP2: slot context */
	xhci_slot_copy(ctrl, in_ctx, out_ctx);
	slot_ctx = xhci_get_slot_ctx(ctrl, in_ctx);
	slot_ctx->dev_info &= ~(0x1f <<  27);
	slot_ctx->dev_info |= cpu_to_le32(LAST_CTX(max_ep_flag + 1) | 0);

	xhci_endpoint_copy(ctrl, in_ctx, out_ctx, 0);

	ep_index = 0;
	/* STEP4 & 5: filling up ep contexts */
	for (cur_ep = 0; cur_ep < num_of_ep; cur_ep++) {
		struct usb_endpoint_descriptor *endpt_desc = NULL;

		endpt_desc = &ifdesc->ep_desc[cur_ep].ep_desc;
		trb_64 = 0;

		ep_index = xhci_get_ep_index(endpt_desc, NULL);
		ep_ctx[ep_index] = xhci_get_ep_ctx(ctrl, in_ctx, ep_index);

		/* Allocate the ep rings */
		virt_dev->eps[ep_index].ring = xhci_ring_alloc(1, true);
		if (!virt_dev->eps[ep_index].ring)
			return -1;

		/*NOTE: ep_desc[0] actually represents EP1 and so on */
		dir = (((endpt_desc->bEndpointAddress) & (0x80)) >> 7);
		ep_type = (((endpt_desc->bmAttributes) & (0x3)) | (dir << 2));
		ep_ctx[ep_index]->ep_info2 =
			cpu_to_le32(ep_type << EP_TYPE_SHIFT);
		ep_ctx[ep_index]->ep_info2 |=
			cpu_to_le32(MAX_PACKET(endpt_desc->wMaxPacketSize));

		/*
		 * EP 0 can handle "burst" sizes of 1,
		 * so Max Burst Size field is 0
		 */
		ep_ctx[ep_index]->ep_info2 |=
			cpu_to_le32(((0 & MAX_BURST_MASK) << MAX_BURST_SHIFT) |
			((3 & ERROR_COUNT_MASK) << ERROR_COUNT_SHIFT));

		trb_64 = (uintptr_t)
				virt_dev->eps[ep_index].ring->first_seg->trbs;
		ep_ctx[ep_index]->deq = cpu_to_le64(trb_64 |
				virt_dev->eps[ep_index].ring->cycle_state);
	}

	xhci_flush_inval_cache((uint32_t)virt_dev->in_ctx, XHCI_CTX, 1);
	xhci_flush_inval_cache((uint32_t)virt_dev->in_ctx->bytes,
					virt_dev->in_ctx->size, 1);

	ret = xhci_configure_endpoint(usbdev, false);

	return ret;
}

/**
 * Finds out the remanining packets to be sent
 *
 * @param usbdev	pointer to the USB device structure
 * @param running_total	total size sent so far
 * @param trb_buff_len	length of the TRB Buffer
 * @param total_packet_count	total packet count
 * @param ep_desc	end point Descriptor
 * @return 0 if running_total or trb_buff_len is 0, else remainder
 */
static u32 xhci_v1_0_td_remainder(struct usb_device *usbdev, int running_total,
					int trb_buff_len,
					unsigned int total_packet_count,
					struct usb_endpoint_descriptor ep_desc)
{
	int packets_transferred;

	/* One TRB with a zero-length data packet. */
	if (running_total == 0 && trb_buff_len == 0)
		return 0;

	/*
	 * All the TRB queueing functions don't count the current TRB in
	 * running_total.
	 */

	if ((running_total + trb_buff_len) >=
			le16_to_cpu(ep_desc.wMaxPacketSize))
		packets_transferred = (running_total + trb_buff_len) /
					(le16_to_cpu(ep_desc.wMaxPacketSize));
	else
		packets_transferred = 1;

	return xhci_td_remainder(total_packet_count - packets_transferred);
}

/**
 * Queues up the BULK Request
 *
 * @param usbdev	pointer to the USB device structure
 * @param pipe		contains the DIR_IN or OUT , devnum
 * @param buffer	buffer to be read/written based on the request
 * @param length	length of the buffer
 * @param req		request type
 * @param ring		Ring on which request be queued
 * @param ep_index	index of the End point
 * @return returns 0 if successful else 0 on failure
 */
int xhci_queue_bulk_tx(struct usb_device *usbdev, unsigned long pipe,
			struct xhci_ring *ring,
			struct usb_endpoint_descriptor ep_desc,
			struct devrequest *req, int length, void *buffer)
{
	int num_trbs = 0;
	struct xhci_generic_trb *start_trb;
	bool first_trb = 0;
	bool more_trbs_coming = 0;
	int start_cycle;
	u32 field = 0;
	u32 length_field = 0;
	struct xhci_ctrl *ctrl = usbdev->controller;
	int slot_id = ctrl->slot_id;
	int ep_index = 0;

	int running_total, trb_buff_len;
	unsigned int total_packet_count;
	u64 addr;
	u64 val_64 = 0;
	u32 trb_fields[4];
	val_64 = (uintptr_t)buffer;

	num_trbs = 0;

	/*
	 * How much data is (potentially) left before the 64KB boundary?
	 * XHCI Spec puts restriction( TABLE 49 and 6.4.1 section of XHCI Spec)
	 * that the buffer should not span 64KB boundary. if so
	 * we send request in more than 1 TRB by chaining them.
	 */
	running_total = TRB_MAX_BUFF_SIZE -
			(lower_32_bits(val_64) & (TRB_MAX_BUFF_SIZE - 1));
	running_total &= TRB_MAX_BUFF_SIZE - 1;

	/*
	 * If there's some data on this 64KB chunk, or we have to send a
	 * zero-length transfer, we need at least one TRB
	 */
	if (running_total != 0 || length == 0)
		num_trbs++;

	/* How many more 64KB chunks to transfer, how many more TRBs? */
	while (running_total < length) {
		num_trbs++;
		running_total += TRB_MAX_BUFF_SIZE;
	}

	if (enqueue_is_link_trb(ring)) {
		union xhci_trb *next;
		next = ring->enqueue;

		while (last_trb(ctrl, ring, ring->enq_seg, next)) {
			/*
			 * we're not dealing with 0.95 hardware or isoc rings
			 * on AMD 0.96 host, clear the chain bit.
			 */
			next->link.control &= cpu_to_le32(~TRB_CHAIN);

			next->link.control ^= cpu_to_le32(TRB_CYCLE);

			xhci_flush_inval_cache((uint32_t)next, XHCI_TRB, 1);

			/* Toggle the cycle bit after the last ring segment. */
			if (last_trb_on_last_seg(ctrl, ring,
							ring->enq_seg, next))
				ring->cycle_state = (ring->cycle_state ? 0 : 1);

			ring->enq_seg = ring->enq_seg->next;
			ring->enqueue = ring->enq_seg->trbs;
			next = ring->enqueue;

			xhci_flush_inval_cache((uint32_t)ring->enqueue,
								XHCI_TRB, 1);
		}
	}

	/*
	 * Don't give the first TRB to the hardware (by toggling the cycle bit)
	 * until we've finished creating all the other TRBs.  The ring's cycle
	 * state may change as we enqueue the other TRBs, so save it too.
	 */
	start_trb = &ring->enqueue->generic;
	start_cycle = ring->cycle_state;

	running_total = 0;
	total_packet_count = DIV_ROUND_UP(length, ep_desc.wMaxPacketSize);

	ep_index = xhci_get_ep_index(&ep_desc, NULL);
	/* How much data is in the first TRB? */
	/*
	 * How much data is (potentially) left before the 64KB boundary?
	 * XHCI Spec puts restriction( TABLE 49 and 6.4.1 section of XHCI Spec)
	 * that the buffer should not span 64KB boundary. if so
	 * we send request in more than 1 TRB by chaining them.
	 */
	addr = val_64;
	trb_buff_len = TRB_MAX_BUFF_SIZE -
			(lower_32_bits(val_64) & (TRB_MAX_BUFF_SIZE - 1));

	if (trb_buff_len > length)
		trb_buff_len = length;

	first_trb = true;

	/* Queue the first TRB, even if it's zero-length */
	do {
		u32 remainder = 0;
		field = 0;
		/* Don't change the cycle bit of the first TRB until later */
		if (first_trb) {
			first_trb = false;
			if (start_cycle == 0)
				field |= 0x1;
		} else {
			field |= ring->cycle_state;
		}

		/*
		 * Chain all the TRBs together; clear the chain bit in the last
		 * TRB to indicate it's the last TRB in the chain.
		 */
		if (num_trbs > 1)
			field |= TRB_CHAIN;

		/* Only set interrupt on short packet for IN endpoints */
		if (usb_pipein(pipe))
			field |= TRB_ISP;

		/* Set the TRB length, TD size, and interrupter fields. */
		if (HC_VERSION(xhci_readl(&ctrl->hccr->cr_capbase)) < 0x100)
			remainder = xhci_td_remainder(length - running_total);
		else
			remainder = xhci_v1_0_td_remainder(usbdev,
							running_total,
							trb_buff_len,
							total_packet_count,
							ep_desc);

		length_field = ((trb_buff_len & TRB_LEN_MASK) |
				remainder |
				((0 & TRB_INTR_TARGET_MASK) <<
				TRB_INTR_TARGET_SHIFT));

		if (num_trbs > 1)
			more_trbs_coming = true;
		else
			more_trbs_coming = false;

		trb_fields[0] = lower_32_bits(addr);
		trb_fields[1] = upper_32_bits(addr);
		trb_fields[2] = length_field;
		trb_fields[3] = field | (TRB_NORMAL << TRB_TYPE_SHIFT);
		queue_trb(ctrl, ring, false, more_trbs_coming, trb_fields);

		--num_trbs;

		running_total += trb_buff_len;

		/* Give some time before Queueing further more TRBs */
		mdelay(5);

		xhci_flush_inval_cache((uint32_t)ring->enqueue, XHCI_TRB, 1);
		xhci_flush_inval_cache((uint32_t)lower_32_bits(addr),
								length, 1);
		xhci_flush_inval_cache((uint32_t)lower_32_bits(addr),
								length, 0);

		/* Calculate length for next transfer */
		addr += trb_buff_len;
		trb_buff_len = length - running_total;
		if (trb_buff_len > TRB_MAX_BUFF_SIZE)
			trb_buff_len = TRB_MAX_BUFF_SIZE;
	} while (running_total < length);

	giveback_first_trb(ctrl, slot_id, ep_index, 0, start_cycle, start_trb);

	xhci_flush_inval_cache((uint32_t)ring->enqueue, XHCI_TRB, 1);
	xhci_flush_inval_cache((uint32_t)lower_32_bits(addr), length, 1);
	xhci_flush_inval_cache((uint32_t)lower_32_bits(addr), length, 0);

	usbdev->act_len = length;
	usbdev->status = 0;

	/* Not a Super Speed Device , give time to device to respond  */
	if (ctrl->speed != USB_SPEED_SUPER)
		mdelay(5);

	return 0;
}

/**
 * Queues up the Control Transfer Request
 *
 * @param usbdev	pointer to the USB device structure
 * @param pipe		contains the DIR_IN or OUT , devnum
 * @param buffer	buffer to be read/written based on the request
 * @param length	length of the buffer
 * @param req		request type
 * @param ring		Ring on which request be queued
 * @param ep_index	index of the End point
 * @return returns 0 if successful else 0 on failure
 */
int xhci_queue_ctrl_tx(struct usb_device *usbdev, struct xhci_ring *ring,
			unsigned long pipe, struct devrequest *req,
			unsigned int length, unsigned int ep_index,
			void *buffer)
{
	int ret;
	int start_cycle;
	int num_trbs;
	u32 field;
	u32 length_field;
	u64 buf_64 = 0;
	struct xhci_ring *ep_ring = ring;
	struct xhci_generic_trb *start_trb;
	struct xhci_ctrl *ctrl = usbdev->controller;
	int slot_id = ctrl->slot_id;
	u32 trb_fields[4];
	struct xhci_virt_device *virt_dev = ctrl->devs[slot_id];

	xhci_flush_inval_cache((uint32_t)virt_dev->out_ctx, XHCI_CTX, 0);
	xhci_flush_inval_cache((uint32_t)virt_dev->out_ctx->bytes,
						virt_dev->out_ctx->size, 0);
	xhci_flush_inval_cache((uint32_t)ring->enqueue, XHCI_TRB, 0);

	struct xhci_ep_ctx *ep_ctx = NULL;
	ep_ctx = xhci_get_ep_ctx(ctrl, ctrl->devs[slot_id]->out_ctx, ep_index);

	/* 1 TRB for setup, 1 for status */
	num_trbs = 2;
	/*
	 * Don't need to check if we need additional event data and normal TRBs,
	 * since data in control transfers will never get bigger than 16MB
	 * XXX: can we get a buffer that crosses 64KB boundaries?
	 */

	if (length > 0)
		num_trbs++;

	ret = prepare_ring(ctrl, ring,
				le32_to_cpu(ep_ctx->ep_info) & EP_STATE_MASK);

	if (ret < 0)
		return ret;

	/*
	 * Don't give the first TRB to the hardware (by toggling the cycle bit)
	 * until we've finished creating all the other TRBs.  The ring's cycle
	 * state may change as we enqueue the other TRBs, so save it too.
	 */
	start_trb = &ring->enqueue->generic;
	start_cycle = ring->cycle_state;

	debug("start_trb %p, start_cycle %d\n", start_trb, start_cycle);

	/* Queue setup TRB - see section 6.4.1.2.1 */
	/* FIXME better way to translate setup_packet into two u32 fields? */
	field = 0;
	field |= TRB_IDT | (TRB_SETUP << TRB_TYPE_SHIFT);
	if (start_cycle == 0)
		field |= 0x1;

	/* xHCI 1.0 6.4.1.2.1: Transfer Type field */
	if (HC_VERSION(xhci_readl(&ctrl->hccr->cr_capbase)) == 0x100) {
		if (length > 0) {
			if (req->requesttype & USB_DIR_IN)
				field |= (TRB_DATA_IN << TRB_TX_TYPE_SHIFT);
			else
				field |= (TRB_DATA_OUT << TRB_TX_TYPE_SHIFT);
		}
	}

	debug("req->requesttype = %d, req->request = %d,"
		"le16_to_cpu(req->value) = %d,"
		"le16_to_cpu(req->index) = %d,"
		"le16_to_cpu(req->length) = %d\n",
		req->requesttype, req->request, le16_to_cpu(req->value),
		le16_to_cpu(req->index), le16_to_cpu(req->length));

		trb_fields[0] = req->requesttype | req->request << 8 |
					le16_to_cpu(req->value) << 16;
		trb_fields[1] = le16_to_cpu(req->index) |
				le16_to_cpu(req->length) << 16;
		/* TRB_LEN | (TRB_INTR_TARGET) */
		trb_fields[2] = (8 | ((0 & TRB_INTR_TARGET_MASK) <<
				TRB_INTR_TARGET_SHIFT));
		/* Immediate data in pointer */
		trb_fields[3] = field;
		queue_trb(ctrl, ring, false, true, trb_fields);

		xhci_flush_inval_cache((uint32_t)ring->enqueue, XHCI_TRB, 1);

	mdelay(1);

	/* Re-initializing field to zero */
	field = 0;
	/* If there's data, queue data TRBs */
	/* Only set interrupt on short packet for IN endpoints */
	if (usb_pipein(pipe))
		field = TRB_ISP | (TRB_DATA << TRB_TYPE_SHIFT);
	else
		field = (TRB_DATA << TRB_TYPE_SHIFT);

	length_field = (length & TRB_LEN_MASK) | xhci_td_remainder(length) |
			((0 & TRB_INTR_TARGET_MASK) << TRB_INTR_TARGET_SHIFT);
	debug("length_field = %d, length = %d,"
		"xhci_td_remainder(length) = %d , TRB_INTR_TARGET(0) = %d\n",
		length_field, (length & TRB_LEN_MASK),
		xhci_td_remainder(length), 0);

	if (length > 0) {
		if (req->requesttype & USB_DIR_IN)
			field |= TRB_DIR_IN;
		buf_64 = (uintptr_t)buffer;

		trb_fields[0] = lower_32_bits(buf_64);
		trb_fields[1] = upper_32_bits(buf_64);
		trb_fields[2] = length_field;
		trb_fields[3] = field | ep_ring->cycle_state;
		queue_trb(ctrl, ring, false, true, trb_fields);

		mdelay(1);

		xhci_flush_inval_cache((uint32_t)ring->enqueue, XHCI_TRB, 1);
		xhci_flush_inval_cache((uint32_t)lower_32_bits(buf_64),
								length, 0);
	}

	/*
	 * Queue status TRB -
	 * see Table 7 and sections 4.11.2.2 and 6.4.1.2.3
	 */

	/* If the device sent data, the status stage is an OUT transfer */
	field = 0;
	if (length > 0 && req->requesttype & USB_DIR_IN)
		field = 0;
	else
		field = TRB_DIR_IN;

	trb_fields[0] = 0;
	trb_fields[1] = 0;
	trb_fields[2] = ((0 & TRB_INTR_TARGET_MASK) << TRB_INTR_TARGET_SHIFT);
		/* Event on completion */
	trb_fields[3] = field | TRB_IOC |
			(TRB_STATUS << TRB_TYPE_SHIFT) |
			ep_ring->cycle_state;

	queue_trb(ctrl, ring, false, false, trb_fields);

	xhci_flush_inval_cache((uint32_t)ring->enqueue, XHCI_TRB, 1);
	xhci_flush_inval_cache((uint32_t)virt_dev->out_ctx, XHCI_CTX, 1);
	xhci_flush_inval_cache((uint32_t)virt_dev->out_ctx->bytes,
						virt_dev->out_ctx->size, 1);

	mdelay(1);

	giveback_first_trb(ctrl, slot_id, ep_index, 0, start_cycle, start_trb);

	xhci_flush_inval_cache((uint32_t)ring->enqueue, XHCI_TRB, 1);
	xhci_flush_inval_cache((uint32_t)lower_32_bits(buf_64), length, 0);

	usbdev->act_len = length;
	usbdev->status = 0;

	/* Not a Super Speed Device , give time to device to respond  */
	if (ctrl->speed != USB_SPEED_SUPER)
		mdelay(5);

	return 0;
}

/**
 * Submits the asynchronous Control Requests to XHCI Host controller
 *
 * @param usbdev	pointer to the USB device structure
 * @param pipe		contains the DIR_IN or OUT , devnum
 * @param buffer	buffer to be read/written based on the request
 * @param length	length of the buffer
 * @param req		request type
 * @return returns 0 if successful else 0 on failure
 *
 */
int xhci_submit_async(struct usb_device *usbdev,
			unsigned long pipe, void *buffer,
			int length, struct devrequest *req)
{
	struct xhci_ctrl *ctrl = usbdev->controller;
	unsigned int slot_id;
	unsigned int ep_index;
	int ret;
	struct usb_endpoint_descriptor ep_descIN;
	struct usb_endpoint_descriptor ep_descOUT;
	struct usb_endpoint_descriptor ep_desc;
	struct xhci_ring *ep_txring;
	int cur_ep;
	struct usb_interface *ifdesc;
	struct xhci_virt_device *virt_dev;
	struct xhci_container_ctx *out_ctx;
	struct xhci_container_ctx *in_ctx;

	ifdesc = &usbdev->config.if_desc[0];

	debug("dev=%p, pipe=%lx, buffer=%p, length=%d, req=%p\n", usbdev, pipe,
		buffer, length, req);
	if (req != NULL)
		debug("req=%u (%#x), type=%u (%#x), value=%u (%#x), index=%u\n",
			req->request, req->request,
			req->requesttype, req->requesttype,
			le16_to_cpu(req->value), le16_to_cpu(req->value),
			le16_to_cpu(req->index));

	slot_id = ctrl->slot_id;

	if (ctrl->slot_id == 0) {
		debug("SLOT-ID IS NOT ALLOCATED YET\n");
		usbdev->act_len = 21;
		usbdev->status = 0;
		return 0;
	}

	virt_dev = ctrl->devs[slot_id];
	in_ctx = virt_dev->in_ctx;
	out_ctx = virt_dev->out_ctx;

	if (usb_pipecontrol(pipe) && req != NULL) {
		/*
		 * Check to see if the max packet size for the default control
		 * endpoint changed during FS device enumeration
		 */
		ep_index = 0;
		ret = xhci_queue_ctrl_tx(usbdev,
					ctrl->devs[slot_id]->eps[ep_index].ring,
					pipe, req, length, ep_index, buffer);
		if (ret)
			return -1;
	} else if (usb_pipebulk(pipe) && req == NULL) {

		xhci_flush_inval_cache((uint32_t)in_ctx, XHCI_CTX, 0);
		xhci_flush_inval_cache((uint32_t)out_ctx, XHCI_CTX, 0);
		xhci_flush_inval_cache((uint32_t)in_ctx->bytes,
							in_ctx->size, 0);
		xhci_flush_inval_cache((uint32_t)out_ctx->bytes,
							out_ctx->size, 0);

		for (cur_ep = 0; cur_ep < ifdesc->no_of_ep; cur_ep++) {
			struct usb_endpoint_descriptor epdesc;

			epdesc = ifdesc->ep_desc[cur_ep].ep_desc;
			if (usb_endpoint_is_bulk_in(&epdesc))
				ep_descIN = epdesc;
			else if (usb_endpoint_is_bulk_out(&epdesc))
				ep_descOUT = epdesc;
			else
				BUG();
		}

		if (usb_pipein(pipe))
			ep_desc = ep_descIN;
		else if (usb_pipeout(pipe))
			ep_desc = ep_descOUT;
		else
			BUG();

		ep_index = xhci_get_ep_index(&ep_desc, req);
		ep_txring = ctrl->devs[slot_id]->eps[ep_index].ring;
		ret = xhci_queue_bulk_tx(usbdev, pipe, ep_txring,
					ep_desc, req, length, buffer);
		if (ret)
			return -1;
	}

	return 0;
}

/**
 * Clears the Change bits of the Port Status Register
 *
 * @param wValue	request value
 * @param wIndex	request index
 * @param addr		address of posrt status register
 * @param port_status	state of port status register
 * @return none
 */
static void xhci_clear_port_change_bit(u16 wValue,
		u16 wIndex, uint32_t *addr, u32 port_status)
{
	char *port_change_bit;
	u32 status;

	switch (wValue) {
	case USB_PORT_FEAT_C_RESET:
		status = PORT_RC;
		port_change_bit = "reset";
		break;
	case USB_PORT_FEAT_C_CONNECTION:
		status = PORT_CSC;
		port_change_bit = "connect";
		break;
	case USB_PORT_FEAT_C_OVER_CURRENT:
		status = PORT_OCC;
		port_change_bit = "over-current";
		break;
	case USB_PORT_FEAT_C_ENABLE:
		status = PORT_PEC;
		port_change_bit = "enable/disable";
		break;
	case USB_PORT_FEAT_C_SUSPEND:
		status = PORT_PLC;
		port_change_bit = "suspend/resume";
		break;
	default:
		/* Should never happen */
		return;
	}

	/* Change bits are all write 1 to clear */
	xhci_writel(addr, port_status | status);

	port_status = xhci_readl(addr);
	debug("clear port %s change, actual port %d status  = 0x%x\n",
			port_change_bit, wIndex, port_status);
}

/**
 * Save Read Only (RO) bits and save read/write bits where
 * writing a 0 clears the bit and writing a 1 sets the bit (RWS).
 * For all other types (RW1S, RW1CS, RW, and RZ), writing a '0' has no effect.
 *
 * @param state	state of the Port Status and Control Regsiter
 * @return a value that would result in the port being in the
 * 	   same state, if the value was written to the port
 * 	   status control register.
 */
u32 xhci_port_state_to_neutral(u32 state)
{
	/* Save read-only status and port state */
	return (state & XHCI_PORT_RO) | (state & XHCI_PORT_RWS);
}

/**
 * Gets the Speed as per the Port Status
 * @param usbdev	pointer to USB device structure
 * @param port_status	state of port status register
 * @return bit-set for usb speed for corresponding state of
 * 	   port status register
 */
static unsigned int xhci_port_speed(struct usb_device *usbdev,
					unsigned int port_status)
{
	struct xhci_ctrl *ctrl = usbdev->controller;

	if (DEV_FULLSPEED(port_status)) {
		usbdev->speed = 0;
		debug("SPEED = FULLSPEED\n");
		ctrl->speed = usbdev->speed;
		return 0x1;
	}
	if (DEV_LOWSPEED(port_status)) {
		debug("SPEED = LOWSPEED\n");
		usbdev->speed = 1;
		ctrl->speed = usbdev->speed;
		return 0x2;
	}
	if (DEV_HIGHSPEED(port_status)) {
		debug("SPEED = HIGHSPEED\n");
		usbdev->speed = 2;
		ctrl->speed = usbdev->speed;
		return 0x3;
	}
	if (DEV_SUPERSPEED(port_status)) {
		debug("SPEED = SUPERSPEED\n");
		usbdev->speed = 3;
		ctrl->speed = usbdev->speed;
		return 0x4;
	}

	return 0;
}

/**
 * Submits the Requests to the XHCI Host Controller
 *
 * @param usbdev pointer to the USB device structure
 * @param pipe contains the DIR_IN or OUT , devnum
 * @param buffer buffer to be read/written based on the request
 * @param length length of the buffer
 * @return returns 0 if successful else -1 on failure
 */
int xhci_submit_root(struct usb_device *usbdev, unsigned long pipe,
			void *buffer, int length, struct devrequest *req)
{
	uint8_t tmpbuf[4];
	u16 typeReq;
	void *srcptr = NULL;
	int len, srclen;
	uint32_t reg;
	uint32_t *status_reg;
	struct xhci_ctrl *ctrl = usbdev->controller;
	struct xhci_hcor *hcor = ctrl->hcor;

	if (le16_to_cpu(req->index) > CONFIG_SYS_USB_XHCI_MAX_ROOT_PORTS) {
		printf("The request port(%d) is not configured\n",
			le16_to_cpu(req->index) - 1);
		return -1;
	}

	status_reg = (uint32_t *)
		     (&hcor->PortRegs[le16_to_cpu(req->index) - 1].or_portsc);
	srclen = 0;

	typeReq = req->request | req->requesttype << 8;

	switch (typeReq) {
	case DeviceRequest | USB_REQ_GET_DESCRIPTOR:
		switch (le16_to_cpu(req->value) >> 8) {
		case USB_DT_DEVICE:
			debug("USB_DT_DEVICE request\n");
			srcptr = &descriptor.device;
			srclen = 0x12;
			break;
		case USB_DT_CONFIG:
			debug("USB_DT_CONFIG config\n");
			srcptr = &descriptor.config;
			srclen = 0x19;
			break;
		case USB_DT_STRING:
			debug("USB_DT_STRING config\n");
			switch (le16_to_cpu(req->value) & 0xff) {
			case 0:	/* Language */
				srcptr = "\4\3\1\0";
				srclen = 4;
				break;
			case 1:	/* Vendor String  */
				srcptr = "\16\3u\0-\0b\0o\0o\0t\0";
				srclen = 14;
				break;
			case 2:	/* Product Name */
				srcptr = "\52\3X\0H\0C\0I\0 "
					 "\0H\0o\0s\0t\0 "
					 "\0C\0o\0n\0t\0r\0o\0l\0l\0e\0r\0";
				srclen = 42;
				break;
			default:
				debug("unknown value DT_STRING %x\n",
					le16_to_cpu(req->value));
				goto unknown;
			}
			break;
		default:
			debug("unknown value %x\n", le16_to_cpu(req->value));
			goto unknown;
		}
		break;
	case USB_REQ_GET_DESCRIPTOR | ((USB_DIR_IN | USB_RT_HUB) << 8):
		switch (le16_to_cpu(req->value) >> 8) {
		case USB_DT_HUB:
			debug("USB_DT_HUB config\n");
			srcptr = &descriptor.hub;
			srclen = 0x8;
			break;
		default:
			debug("unknown value %x\n", le16_to_cpu(req->value));
			goto unknown;
		}
		break;
	case USB_REQ_ALLOC_DEV | (USB_RECIP_DEVICE << 8):
		xhci_alloc_dev(usbdev);
		break;
	case USB_REQ_SET_ADDRESS | (USB_RECIP_DEVICE << 8):
		if (usbdev->devnum == 1)
			ctrl->rootdev = le16_to_cpu(req->value);
		else
			xhci_address_device(usbdev);
		break;
	case DeviceOutRequest | USB_REQ_SET_CONFIGURATION:
		/* FIXME: need to put code for configuring endpoint here */
			xhci_configure_ep(usbdev);
		break;
	case USB_REQ_GET_STATUS | ((USB_DIR_IN | USB_RT_HUB) << 8):
		tmpbuf[0] = 1;	/* USB_STATUS_SELFPOWERED */
		tmpbuf[1] = 0;
		srcptr = tmpbuf;
		srclen = 2;
		break;
	case USB_REQ_GET_STATUS | ((USB_RT_PORT | USB_DIR_IN) << 8):
		memset(tmpbuf, 0, 4);
		reg = xhci_readl(status_reg);
		if (reg & PORT_CONNECT) {
			tmpbuf[0] |= USB_PORT_STAT_CONNECTION;
			tmpbuf[1] |= (xhci_port_speed(usbdev, reg) << 2);
		}
		if (reg & PORT_PE)
			tmpbuf[0] |= USB_PORT_STAT_ENABLE;
		if (reg & XDEV_U3)
			tmpbuf[0] |= USB_PORT_STAT_SUSPEND;
		if (reg & PORT_OC)
			tmpbuf[0] |= USB_PORT_STAT_OVERCURRENT;
		if (reg & PORT_RESET)
			tmpbuf[0] |= USB_PORT_STAT_RESET;
		if (reg & PORT_POWER)
			tmpbuf[1] |= USB_SS_PORT_STAT_POWER >> 8;
		if ((reg & PORT_PLC))
			tmpbuf[2] |= USB_PORT_STAT_C_LINK_STATE;
		if ((reg & PORT_WRC))
			tmpbuf[2] |= USB_PORT_STAT_C_BH_RESET;
		if (reg & PORT_CSC)
			tmpbuf[2] |= USB_PORT_STAT_C_CONNECTION;
		if (reg & PORT_PEC)
			tmpbuf[2] |= USB_PORT_STAT_C_ENABLE;
		if (reg & PORT_OCC)
			tmpbuf[2] |= USB_PORT_STAT_C_OVERCURRENT;
		if (reg & PORT_RC)
			tmpbuf[2] |= USB_PORT_STAT_C_RESET;

		srcptr = tmpbuf;
		srclen = 4;
		break;
	case USB_REQ_SET_FEATURE | ((USB_DIR_OUT | USB_RT_PORT) << 8):
		reg = xhci_readl(status_reg);
		switch (le16_to_cpu(req->value)) {
		case USB_PORT_FEAT_ENABLE:
			reg |= PORT_PE;
			xhci_writel(status_reg, reg);
			break;
		case USB_PORT_FEAT_POWER:
			reg |= PORT_POWER;
			xhci_writel(status_reg, reg);
			break;
		case USB_PORT_FEAT_RESET:
			reg = xhci_readl(status_reg);
			reg |= PORT_RESET;
			xhci_writel(status_reg, reg);
			xhci_poll_and_HandleEvent(usbdev);
			break;
		default:
			debug("unknown feature %x\n", le16_to_cpu(req->value));
			goto unknown;
		}
		/* unblock posted writes */
		xhci_readl(&hcor->or_usbcmd);
		break;
	case USB_REQ_CLEAR_FEATURE | ((USB_DIR_OUT | USB_RT_PORT) << 8):
		reg = xhci_readl(status_reg);
		reg = xhci_port_state_to_neutral(reg);
		switch (le16_to_cpu(req->value)) {
		case USB_PORT_FEAT_ENABLE:
			reg &= ~PORT_PE;
			break;
		case USB_PORT_FEAT_C_RESET:
		case USB_PORT_FEAT_C_CONNECTION:
		case USB_PORT_FEAT_C_OVER_CURRENT:
		case USB_PORT_FEAT_C_ENABLE:
			xhci_clear_port_change_bit((le16_to_cpu(req->value)),
							le16_to_cpu(req->index),
							status_reg, reg);
			break;
		default:
			goto unknown;
		}
		xhci_writel(status_reg, reg);
		/* unblock posted write */
		xhci_readl(&hcor->or_usbcmd);
		break;
	default:
		debug("Unknown request\n");
		goto unknown;
	}

	mdelay(5);

	debug("scrlen = %d\n req->length = %d\n, length = %d\n",
		srclen, le16_to_cpu(req->length), length);

	len = min3(srclen, le16_to_cpu(req->length), length);

	if (srcptr != NULL && len > 0)
		memcpy(buffer, srcptr, len);
	else
		debug("Len is 0\n");

	usbdev->act_len = len;
	usbdev->status = 0;

	return 0;

unknown:
	usbdev->act_len = 0;
	usbdev->status = USB_ST_STALLED;

	return -1;
}

/**
 * Allocates the Container context
 *
 * @param ctrl	Host controller data structure
 * @param type type of Context
 * @return NULL if fialed else pointer to the context on success
 */
static struct
xhci_container_ctx *xhci_alloc_container_ctx(struct xhci_ctrl *ctrl,
						int type)
{
	struct xhci_container_ctx *ctx;

	ctx = (struct xhci_container_ctx *)
		xhci_malloc(sizeof(struct xhci_container_ctx));

	if (!ctx)
		return NULL;

	BUG_ON((type != XHCI_CTX_TYPE_DEVICE) && (type != XHCI_CTX_TYPE_INPUT));
	ctx->type = type;
	ctx->size = HCC_64BYTE_CONTEXT(readl(&ctrl->hccr->cr_hccparams)) ?
								2048 : 1024;
	if (type == XHCI_CTX_TYPE_INPUT)
		ctx->size += CTX_SIZE(readl(&ctrl->hccr->cr_hccparams));

	ctx->bytes = (u8 *)xhci_malloc(ctx->size);

	return ctx;
}

/**
 * Give the input control context for the passed container context
 *
 * @param ctx	pointer to the context
 * @return pointer to the Input control context data
 */
struct xhci_input_control_ctx *xhci_get_input_control_ctx(
					      struct xhci_container_ctx *ctx)
{
	BUG_ON(ctx->type != XHCI_CTX_TYPE_INPUT);
	return (struct xhci_input_control_ctx *)ctx->bytes;
}

/**
 * Give the slot context for the passed container context
 *
 * @param ctx	pointer to the context
 * @param ctrl	Host controller data structure
 * @return pointer to the slot control context data
 */
struct xhci_slot_ctx *xhci_get_slot_ctx(struct xhci_ctrl *ctrl,
				struct xhci_container_ctx *ctx)
{
	if (ctx->type == XHCI_CTX_TYPE_DEVICE)
		return (struct xhci_slot_ctx *)ctx->bytes;

	return (struct xhci_slot_ctx *)
		(ctx->bytes + CTX_SIZE(readl(&ctrl->hccr->cr_hccparams)));
}

/**
 * See Cycle bit rules. SW is the consumer for the event ring only.
 * Don't make a ring full of link TRBs.  That would be dumb and this would loop.
 *
 * @param ctrl	Host controller data structure
 * @param ring	Ring whose Dequeue TRB pointer needs to be incremented.
 * @param consumer	flag to indicate whether S/W is consumer of this RING or NOT
 * return none
 */
static void inc_deq(struct xhci_ctrl *ctrl, struct xhci_ring *ring,
							bool consumer)
{
	union xhci_trb *next = ++(ring->dequeue);

	ring->deq_updates++;
	/*
	 * Update the dequeue pointer further if that was a link TRB or we're at
	 * the end of an event ring segment (which doesn't have link TRBS)
	 */
	while (last_trb(ctrl, ring, ring->deq_seg, next)) {
		if (consumer &&
			last_trb_on_last_seg(ctrl, ring, ring->deq_seg, next))
			ring->cycle_state = (ring->cycle_state ? 0 : 1);

		ring->deq_seg = ring->deq_seg->next;
		ring->dequeue = ring->deq_seg->trbs;
		next = ring->dequeue;
	}

	xhci_flush_inval_cache((uint32_t)ring->dequeue, XHCI_TRB, 1);
}

/**
 * Handles the port status event
 *
 * @param ctrl	Host controller data structure
 * @param event event genrated by the XHCI controller
 * @return none
 */
static void handle_port_status(struct xhci_ctrl *ctrl, union xhci_trb *event)
{
	int max_ports;
	int port_id;

	/* Port status change events always have a successful completion code */
	if (((le32_to_cpu(event->generic.field[2]) &
			COMP_CODE_MASK) >> COMP_CODE_SHIFT) != COMP_SUCCESS)
		return;

	port_id = (((le32_to_cpu(event->generic.field[0]) &
			PORT_ID_MASK) >> PORT_ID_SHIFT));
	if (ctrl->handle_portStatus_Flag & (1 << port_id))
		return;

	ctrl->handle_portStatus_Flag |= (1 << port_id);
	ctrl->port_id = port_id;
	max_ports = ((readl(&ctrl->hccr->cr_hcsparams1) &
			HCS_MAX_PORTS_MASK) >> HCS_MAX_PORTS_SHIFT);

	if ((ctrl->port_id <= 0) || (ctrl->port_id > max_ports))
		debug("Invalid port id %d\n", ctrl->port_id);

	return;
}

/**
 * Handles the command completion Event
 *
 * @param ctrl	Host controller data structure
 * @param event Event generated by the XHCI controller
 * @return none
 */
static void handle_cmd_completion(struct xhci_ctrl *ctrl,
			struct xhci_event_cmd *event)
{
	int slot_id = ((le32_to_cpu(event->flags) &
				TRB_TO_SLOT_ID_MASK) >> TRB_TO_SLOT_ID_SHIFT);
	u64 cmd_addr;
	unsigned long cmd_dequeue_addr;

	xhci_flush_inval_cache((uint32_t)ctrl->cmd_ring->dequeue, XHCI_TRB, 0);

	cmd_addr = le64_to_cpu(event->cmd_trb);
	cmd_dequeue_addr = trb_addr(ctrl->cmd_ring->deq_seg,
			ctrl->cmd_ring->dequeue);

	/* Is the command ring deq ptr out of sync with the deq seg ptr? */
	if (cmd_dequeue_addr == 0)
		return;

	/*
	 * Does the cmd trb's address match our internal
	 * dequeue pointer address?
	 */
	if (cmd_addr != (u64) cmd_dequeue_addr)
		return;

	switch ((le32_to_cpu(ctrl->cmd_ring->dequeue->generic.field[3]) &
				TRB_TYPE_BITMASK) >> TRB_TYPE_SHIFT) {
	case TRB_ENABLE_SLOT:
		if (((le32_to_cpu(event->status) &
			COMP_CODE_MASK) >> COMP_CODE_SHIFT) == COMP_SUCCESS)
			ctrl->slot_id = slot_id;
		else
			ctrl->slot_id = 0;
		break;

	case TRB_ADDR_DEV:
		ctrl->devs[slot_id]->cmd_status = ((le32_to_cpu(event->status) &
							COMP_CODE_MASK) >>
							COMP_CODE_SHIFT);
		break;

	case TRB_RESET_DEV:
		debug("Completed reset device command.\n");
		break;

	case TRB_EVAL_CONTEXT:
		ctrl->devs[slot_id]->cmd_status =
			GET_COMP_CODE(le32_to_cpu(event->status));
		break;

	case TRB_CONFIG_EP:
		debug("Configure Endpoint command completed .\n");
		break;
	default:
		/* Skip over unknown commands on the event ring */
		break;
	}

	inc_deq(ctrl, ctrl->cmd_ring, false);
}

/**
 * This function handles all CPU-owned events on the event ring.
 *
 * @param ctrl	Host controller data structure
 * @return 1 incase of success or -1 on failure
 */
static int xhci_handle_event(struct xhci_ctrl *ctrl)
{
	union xhci_trb *event;
	int update_ptrs = 1;
	unsigned long deq;
	u64 temp_64;

	if (!ctrl->event_ring || !ctrl->event_ring->dequeue)
		return -1;

	xhci_flush_inval_cache((uint32_t)ctrl->event_ring->dequeue,
							XHCI_TRB, 0);

	event = ctrl->event_ring->dequeue;

	/* FIXME: Handle more event types. */
	switch ((le32_to_cpu(event->event_cmd.flags) &
				TRB_TYPE_BITMASK) >> TRB_TYPE_SHIFT) {
	case TRB_COMPLETION:
		handle_cmd_completion(ctrl, &event->event_cmd);
		debug("Handling commanding completion\n");
		break;

	case TRB_PORT_STATUS:
		handle_port_status(ctrl, event);
		break;
	default:
		if ((le32_to_cpu(event->event_cmd.flags) &
						TRB_TYPE_BITMASK) == 0)
			update_ptrs = 0;
		break;
	}

	if (update_ptrs) {
		debug("Upadting dequeue pointer\n");
		/* Update SW event ring dequeue pointer */
		inc_deq(ctrl, ctrl->event_ring, true);
	}

	xhci_flush_inval_cache((uint32_t)ctrl->event_ring->dequeue,
							XHCI_TRB, 0);

	temp_64 = xhci_readl_64(&ctrl->ir_set->erst_dequeue);
	/* If necessary, update the HW's version of the event ring deq ptr. */
	deq = (u32)(ctrl->event_ring->dequeue);

	xhci_flush_inval_cache((uint32_t)deq, XHCI_TRB, 1);

	if (deq == 0)
		debug("WARN something wrong with SW event "
				"ring dequeue ptr.\n");

	/* Update HC event ring dequeue pointer */
	temp_64 &= ERST_PTR_MASK;
	temp_64 |= ((u64) deq & (u64) ~ERST_PTR_MASK);

	/* Clear the event handler busy flag (RW1C); event ring is empty. */
	temp_64 |= ERST_EHB;
	xhci_writel_64(&ctrl->ir_set->erst_dequeue, temp_64);

	return 1;
}

/**
 * Checks the ownership of the Event Command
 *
 * @param ctrl	Host controller data structure
 * @return 0 if failure else 0 on success
 */
static int xhci_check_ownership(struct xhci_ctrl *ctrl)
{
	union xhci_trb *event;

	if (!ctrl->event_ring || !ctrl->event_ring->dequeue)
		return 0;

	xhci_flush_inval_cache((uint32_t)ctrl->event_ring->dequeue,
							XHCI_TRB, 0);

	event = ctrl->event_ring->dequeue;
	/* Does the HC or OS own the TRB? */
	if ((le32_to_cpu(event->event_cmd.flags) & TRB_CYCLE) !=
		ctrl->event_ring->cycle_state)
		return 0;

	return 1;
}

/**
 * Checks the ownership of the Event Command
 *
 * @param ctrl	Host controller data structure
 * @return ownership flag
 */
int xhci_poll(struct xhci_ctrl *ctrl)
{
	return xhci_check_ownership(ctrl);
}

/**
 * Queue a slot enable or disable request on the command ring
 *
 * @param ctrl	Host controller data structure
 * @param trb_type	type of TRB to be queued in
 * @param slot_id	slot id allocated by the XHCI controller
 * @return status of the "queue_command"
 */
int xhci_queue_slot_control(struct xhci_ctrl *ctrl, u32 trb_type, u32 slot_id)
{

	u32 trb_fields[4];

	trb_fields[0] = 0;
	trb_fields[1] = 0;
	trb_fields[2] = 0;
	trb_fields[3] = ((trb_type << TRB_TYPE_SHIFT) |
				((slot_id & SLOT_ID_FOR_TRB_MASK) <<
				SLOT_ID_FOR_TRB_SHIFT));

	return queue_command(ctrl, trb_fields);
}

/**
 * Allocating virtual device
 *
 * @param usbdev	pointer to USB deivce structure
 * @param slot_id	slot id allocated by the XHCI controller
 * @return 0 on success else -1 on failure
 */
int xhci_alloc_virt_device(struct usb_device *usbdev, int slot_id)
{
	u64 byte_64 = 0;
	struct xhci_virt_device *virt_dev;
	struct xhci_ctrl *ctrl = usbdev->controller;

	/* Slot ID 0 is reserved */
	if (slot_id == 0 || ctrl->devs[slot_id]) {
		debug("Bad Slot ID %d\n", slot_id);
		return -1;
	}

	ctrl->devs[slot_id] = (struct xhci_virt_device *)
					malloc(sizeof(struct xhci_virt_device));

	if (!ctrl->devs[slot_id]) {
		debug("Failed to allocate virtual device\n");
		return -1;
	}

	memset(ctrl->devs[slot_id], 0, sizeof(struct xhci_virt_device));
	virt_dev = ctrl->devs[slot_id];

	/* Allocate the (output) device context that will be used in the HC. */
	virt_dev->out_ctx = xhci_alloc_container_ctx(ctrl,
					XHCI_CTX_TYPE_DEVICE);
	if (!virt_dev->out_ctx) {
		debug("Failed to allocate out context for virt dev\n");
		return -1;
	}

	/* Allocate the (input) device context for address device command */
	virt_dev->in_ctx = xhci_alloc_container_ctx(ctrl,
					XHCI_CTX_TYPE_INPUT);
	if (!virt_dev->in_ctx) {
		debug("Failed to allocate in context for virt dev\n");
		return -1;
	}

	/* Allocate endpoint 0 ring */
	virt_dev->eps[0].ring = xhci_ring_alloc(1, true);
	if (!virt_dev->eps[0].ring) {
		debug("Failed to allocate EP(0) ring\n");
		return -1;
	}

	byte_64 = (uintptr_t)(virt_dev->out_ctx->bytes);

	/* Point to output device context in dcbaa. */
	ctrl->dcbaa->dev_context_ptrs[slot_id] = byte_64;

	xhci_flush_inval_cache((uint32_t)
				&ctrl->dcbaa->dev_context_ptrs[slot_id],
				sizeof(__le64), 1);
	return 0;
}

/**
 * Copy output xhci_ep_ctx to the input xhci_ep_ctx copy.
 * Useful when you want to change one particular aspect of the endpoint
 * and then issue a configure endpoint command.
 *
 * @param ctrl	Host controller data structure
 * @param in_ctx contains the inpout context
 * @param out_ctx contains the inpout context
 * @param ep_index index of the end point
 * @return none
 */
void xhci_endpoint_copy(struct xhci_ctrl *ctrl,
			struct xhci_container_ctx *in_ctx,
			struct xhci_container_ctx *out_ctx,
			unsigned int ep_index)
{
	struct xhci_ep_ctx *out_ep_ctx;
	struct xhci_ep_ctx *in_ep_ctx;

	out_ep_ctx = xhci_get_ep_ctx(ctrl, out_ctx, ep_index);
	in_ep_ctx = xhci_get_ep_ctx(ctrl, in_ctx, ep_index);

	in_ep_ctx->ep_info = out_ep_ctx->ep_info;
	in_ep_ctx->ep_info2 = out_ep_ctx->ep_info2;
	in_ep_ctx->deq = out_ep_ctx->deq;
	in_ep_ctx->tx_info = out_ep_ctx->tx_info;
}

/**
 * Copy output xhci_slot_ctx to the input xhci_slot_ctx.
 * Useful when you want to change one particular aspect of the endpoint
 * and then issue a configure endpoint command.
 * Only the context entries field matters, but
 * we'll copy the whole thing anyway.
 *
 * @param ctrl	Host controller data structure
 * @param in_ctx contains the inpout context
 * @param out_ctx contains the inpout context
 * @return none
 */
void xhci_slot_copy(struct xhci_ctrl *ctrl, struct xhci_container_ctx *in_ctx,
					struct xhci_container_ctx *out_ctx){

	struct xhci_slot_ctx *in_slot_ctx;
	struct xhci_slot_ctx *out_slot_ctx;

	in_slot_ctx = xhci_get_slot_ctx(ctrl, in_ctx);
	out_slot_ctx = xhci_get_slot_ctx(ctrl, out_ctx);

	in_slot_ctx->dev_info = out_slot_ctx->dev_info;
	in_slot_ctx->dev_info2 = out_slot_ctx->dev_info2;
	in_slot_ctx->tt_info = out_slot_ctx->tt_info;
	in_slot_ctx->dev_state = out_slot_ctx->dev_state;
}

/**
 * Queue an address device command TRB
 *
 * @param ctrl	Host controller data structure
 * @param in_ctx_ptr
 * @param slot_id
 * @return status of the "queue_command"
 */
int xhci_queue_address_device(struct xhci_ctrl *ctrl, u8 *in_ctx_ptr,
					u32 slot_id)
{
	u64 val_64 = 0;
	u32 trb_fields[4];

	val_64 = (uintptr_t)in_ctx_ptr;
	trb_fields[0] = lower_32_bits(val_64);
	trb_fields[1] = upper_32_bits(val_64);
	trb_fields[2] = 0;
	trb_fields[3] = ((TRB_ADDR_DEV << TRB_TYPE_SHIFT) |
				((slot_id & SLOT_ID_FOR_TRB_MASK) <<
				SLOT_ID_FOR_TRB_SHIFT));

	return queue_command(ctrl, trb_fields);
}

/**
 * Copies the EP O Transfer ring to Input Context
 *
 * @param ctrl	Host controller data structure
 * @return none
 */
void xhci_copy_ep0_dequeue_into_input_ctx(struct xhci_ctrl *ctrl)
{
	struct xhci_virt_device *virt_dev;
	struct xhci_ep_ctx	*ep0_ctx;
	struct xhci_ring	*ep_ring;

	virt_dev = ctrl->devs[ctrl->slot_id];
	ep0_ctx = xhci_get_ep_ctx(ctrl, virt_dev->in_ctx, 0);
	ep_ring = virt_dev->eps[0].ring;
	/*
	 * FIXME we don't keep track of the dequeue pointer very well after a
	 * Set TR dequeue pointer, so we're setting the dequeue pointer of the
	 * host to our enqueue pointer.  This should only be called after a
	 * configured device has reset, so all control transfers should have
	 * been completed or cancelled before the reset.
	 */
	ep0_ctx->deq = cpu_to_le64(trb_addr(ep_ring->enq_seg,
				ep_ring->enqueue) | ep_ring->cycle_state);
}

/**
 * Setup an xHCI virtual device for a Set Address command
 *
 * @param usbdev pointer to the Device Data Structure
 * @return returns negative value on failure else 0 on success
 */
int xhci_setup_addressable_virt_dev(struct usb_device *usbdev)
{
	struct xhci_virt_device *virt_dev;
	struct xhci_ep_ctx *ep0_ctx;
	struct xhci_slot_ctx *slot_ctx;
	u32 port_num = 0;
	u64 trb_64 = 0;

	struct xhci_ctrl *ctrl = usbdev->controller;

	virt_dev = ctrl->devs[ctrl->slot_id];

	/* Slot ID 0 is reserved */
	if (ctrl->slot_id == 0 || !virt_dev) {
		debug("Slot ID %d is not assigned to this device\n",
							ctrl->slot_id);
		return -EINVAL;
	}

	/* Extract the EP0 and Slot Ctrl */
	ep0_ctx = xhci_get_ep_ctx(ctrl, virt_dev->in_ctx, 0);
	slot_ctx = xhci_get_slot_ctx(ctrl, virt_dev->in_ctx);

	/* 3) Only the control endpoint is valid - one endpoint context */
	slot_ctx->dev_info |= cpu_to_le32(LAST_CTX(1) | 0);

	switch (ctrl->speed) {

	case USB_SPEED_SUPER:
		slot_ctx->dev_info |= cpu_to_le32(SLOT_SPEED_SS);
		break;
	case USB_SPEED_HIGH:
		slot_ctx->dev_info |= cpu_to_le32(SLOT_SPEED_HS);
		break;
	case USB_SPEED_FULL:
		slot_ctx->dev_info |= cpu_to_le32(SLOT_SPEED_FS);
		break;
	case USB_SPEED_LOW:
		slot_ctx->dev_info |= cpu_to_le32(SLOT_SPEED_LS);
		break;
	default:
		/* Speed was set earlier, this shouldn't happen. */
		BUG();
	}

	port_num = ctrl->port_id;
	debug("port_num = %d\n", port_num);

	slot_ctx->dev_info2 |=
			cpu_to_le32(((port_num & ROOT_HUB_PORT_MASK) <<
				ROOT_HUB_PORT_SHIFT));

	/* Step 4 - ring already allocated */
	/* Step 5 */
	ep0_ctx->ep_info2 = cpu_to_le32(CTRL_EP << EP_TYPE_SHIFT);
	/*
	 * XXX: Not sure about wireless USB devices.
	 */
	debug("SPEED = %d\n", ctrl->speed);

	switch (ctrl->speed) {
	case USB_SPEED_SUPER:
		ep0_ctx->ep_info2 |= cpu_to_le32(((512 & MAX_PACKET_MASK) <<
					MAX_PACKET_SHIFT));
		debug("Setting Packet size = 512bytes\n");
		break;
	case USB_SPEED_HIGH:
	/* USB core guesses at a 64-byte max packet first for FS devices */
	case USB_SPEED_FULL:
		ep0_ctx->ep_info2 |= cpu_to_le32(((64 & MAX_PACKET_MASK) <<
					MAX_PACKET_SHIFT));
		debug("Setting Packet size = 64bytes\n");
		break;
	case USB_SPEED_LOW:
		ep0_ctx->ep_info2 |= cpu_to_le32(((8 & MAX_PACKET_MASK) <<
					MAX_PACKET_SHIFT));
		debug("Setting Packet size = 8bytes\n");
		break;
	default:
		/* New speed? */
		BUG();
	}

	/* EP 0 can handle "burst" sizes of 1, so Max Burst Size field is 0 */
	ep0_ctx->ep_info2 |=
			cpu_to_le32(((0 & MAX_BURST_MASK) << MAX_BURST_SHIFT) |
			((3 & ERROR_COUNT_MASK) << ERROR_COUNT_SHIFT));

	trb_64 = (uintptr_t)virt_dev->eps[0].ring->first_seg->trbs;
	ep0_ctx->deq = cpu_to_le64(trb_64 | virt_dev->eps[0].ring->cycle_state);

	/* Steps 7 and 8 were done in xhci_alloc_virt_device() */

	return 0;
}

/**
 * Polls the XHCI controller
 * and Handles the Event generated by the XHCI Controller
 *
 * @param usbdev pointer to the Device Data Structure
 * @return none
 */
void xhci_poll_and_HandleEvent(struct usb_device *usbdev)
{
	unsigned long ts;
	struct xhci_ctrl *ctrl = usbdev->controller;
	int event_occured = 0;
	union xhci_trb *event;
	union xhci_trb	*first_trb;

	xhci_flush_inval_cache((uint32_t)ctrl->event_ring->dequeue,
							XHCI_TRB, 0);

	event = ctrl->event_ring->dequeue;

	ts = get_timer(0);

	do {
		if (xhci_poll(ctrl)) {
			debug("Poll success\n");
			event_occured = 1;
			break;
		}
	} while (get_timer(ts) < USB_CTRL_SET_TIMEOUT);

	if (event_occured) {
		while (((le32_to_cpu(event->event_cmd.flags) &
					TRB_TYPE_BITMASK) >>
					TRB_TYPE_SHIFT) == TRB_PORT_STATUS) {
			if (xhci_handle_event(ctrl) < 0) {
				debug("Error in handling the event\n");
				return;
			}

			first_trb = ctrl->event_ring->first_seg->trbs;

			xhci_flush_inval_cache((uint32_t)first_trb,
					      (sizeof(union xhci_trb) * 64), 0);

			event = ctrl->event_ring->dequeue;
		}
		/*
		 * If this is the 1st time POLL is called
		 * then handle event one more time, since
		 * 1st event would be "PORT STATUS CHANGE EVENT"
		 */
		if (xhci_handle_event(ctrl) < 0) {
			debug("Error in handling the event\n");
			return;
		}
		usbdev->act_len = 0;
		usbdev->status = 0;
	} else {
		debug("NO EVENT OCCURED\n");
	}

	return;
}

/**
 * Issue an Address Device command (which will issue a SetAddress request to
 * the device).
 * We add one to the device address issued by the hardware because the USB core
 * uses address 1 for the root hubs (even though they're not really devices).
 *
 * @param usbdev pointer to the Device Data Structure
 * @return 0 if successful else error code on failure
 */
int xhci_address_device(struct usb_device *usbdev)
{
	int ret = 0;
	struct xhci_ctrl *ctrl = usbdev->controller;
	struct xhci_slot_ctx *slot_ctx;
	struct xhci_input_control_ctx *ctrl_ctx;
	struct xhci_virt_device *virt_dev;
	int slot_id = ctrl->slot_id;

	virt_dev = ctrl->devs[slot_id];

	slot_ctx = xhci_get_slot_ctx(ctrl, virt_dev->in_ctx);

	/*
	 * If this is the first Set Address since device plug-in or
	 * virt_device realloaction after a resume with an xHCI power loss,
	 * then set up the slot context.
	 */

	xhci_flush_inval_cache((uint32_t)virt_dev->eps[0].ring->first_seg->trbs,
						XHCI_TRB, 0);
	xhci_flush_inval_cache((uint32_t)virt_dev->in_ctx,
						XHCI_CTX, 0);
	xhci_flush_inval_cache((uint32_t)virt_dev->out_ctx,
						XHCI_CTX, 0);
	xhci_flush_inval_cache((uint32_t)virt_dev->in_ctx->bytes,
						virt_dev->in_ctx->size, 0);
	xhci_flush_inval_cache((uint32_t)virt_dev->out_ctx->bytes,
						virt_dev->out_ctx->size, 0);

	if (!slot_ctx->dev_info) {
		debug("Setting up addressable devices\n");
		xhci_setup_addressable_virt_dev(usbdev);
	} else {
		/* Otherwise, update the control EP ring enqueue pointer. */
		debug("update the control endpoint ring enqueue pointer\n");
		xhci_copy_ep0_dequeue_into_input_ctx(ctrl);
	}

	ctrl_ctx = xhci_get_input_control_ctx(virt_dev->in_ctx);
	ctrl_ctx->add_flags = cpu_to_le32(SLOT_FLAG | EP0_FLAG);
	ctrl_ctx->drop_flags = 0;

	xhci_flush_inval_cache((uint32_t)virt_dev->in_ctx,
						XHCI_CTX, 1);
	xhci_flush_inval_cache((uint32_t)virt_dev->out_ctx,
						XHCI_CTX, 1);
	xhci_flush_inval_cache((uint32_t)virt_dev->in_ctx->bytes,
						virt_dev->in_ctx->size, 1);
	xhci_flush_inval_cache((uint32_t)virt_dev->out_ctx->bytes,
						virt_dev->out_ctx->size, 1);

	ret = xhci_queue_address_device(ctrl, virt_dev->in_ctx->bytes,
							slot_id);

	xhci_ring_cmd_db(ctrl);
	mdelay(5);
	xhci_poll_and_HandleEvent(usbdev);

	switch (virt_dev->cmd_status) {
	case COMP_CTX_STATE:
	case COMP_EBADSLT:
		debug("Setup ERROR: address device command for slot %d.\n",
								slot_id);
		ret = -EINVAL;
		break;
	case COMP_TX_ERR:
		debug("Device not responding to set address.\n");
		ret = -EPROTO;
		break;
	case COMP_DEV_ERR:
		debug("ERROR: Incompatible device"
					"for address device command.\n");
		ret = -ENODEV;
		break;
	case COMP_SUCCESS:
		debug("Successful Address Device command\n");
		break;
	default:
		debug("ERROR: unexpected command completion code 0x%x.\n",
							virt_dev->cmd_status);
		/*FIXME*/
		ret = 0;
		break;
	}
	if (ret < 0)
		return ret;
	/*
	 * USB core uses address 1 for the roothubs, so we add one to the
	 * address given back to us by the HC.
	 */

	xhci_flush_inval_cache((uint32_t)virt_dev->out_ctx,
						XHCI_CTX, 0);
	xhci_flush_inval_cache((uint32_t)virt_dev->out_ctx->bytes,
						virt_dev->out_ctx->size, 1);

	slot_ctx = xhci_get_slot_ctx(ctrl, virt_dev->out_ctx);

	/* store xHC assigned  Address locally */
	virt_dev->address = (le32_to_cpu(slot_ctx->dev_state) &
						DEV_ADDR_MASK) + 1;

	 if (virt_dev->address == 1)
		debug("ADDRESS assigned is ZERO\n");

	/* Zero the input context control for later use */
	ctrl_ctx->add_flags = 0;
	ctrl_ctx->drop_flags = 0;

	return 0;
}

/**
 * Issue Enable slot command to the controller to allocate
 * device slot and assign the slot id. It fails if the xHC
 * ran out of device slots, the Enable Slot command timed out,
 * or allocating memory failed.
 *
 * @param usbdev	pointer to the Device Data Structure
 * @return Returns 0 on succes else return -1 on failure
 */
int xhci_alloc_dev(struct usb_device *usbdev)
{
	struct xhci_ctrl *ctrl = usbdev->controller;
	int ret;
	ret = xhci_queue_slot_control(ctrl, TRB_ENABLE_SLOT, 0);
	if (ret) {
		debug("FIXME: allocate a command ring segment\n");
		return -1;
	}
	xhci_ring_cmd_db(ctrl);
	mdelay(5);
	xhci_poll_and_HandleEvent(usbdev);
	if (!ctrl->slot_id) {
		debug("Error while assigning device slot ID\n");
		return -1;
	}

	/*
	 * Use GFP_NOIO, since this function can be called from
	 * xhci_discover_or_reset_device(), which may be called as part of
	 * mass storage driver error handling.
	 */
	if (xhci_alloc_virt_device(usbdev, ctrl->slot_id) < 0)
		debug("Could not allocate xHCI USB device data structures\n");

	return 0;
}

/**
 * Submits the INT request to XHCI Host cotroller
 *
 * @param usbdev	pointer to the USB device
 * @param pipe		contains the DIR_IN or OUT , devnum
 * @param buffer	buffer to be read/written based on the request
 * @param interval	interval of the interrupt
 * @return 0
 */
int
submit_int_msg(struct usb_device *usbdev, unsigned long pipe, void *buffer,
						int length, int interval)
{
	/* Not addressing any interrupt type transfer requests */
	return 0;
}

/**
 * submit the BULK type of request to the USB Device
 *
 * @param usbdev	pointer to the USB device
 * @param pipe		contains the DIR_IN or OUT , devnum
 * @param buffer	buffer to be read/written based on the request
 * @param length	length of the buffer
 * @return returns 0 if successful else -1 on failure
 */
int
submit_bulk_msg(struct usb_device *usbdev, unsigned long pipe, void *buffer,
								int length)
{
	if (usb_pipetype(pipe) != PIPE_BULK) {
		debug("non-bulk pipe (type=%lu)", usb_pipetype(pipe));
		return -1;
	}
	return xhci_submit_async(usbdev, pipe, buffer, length, NULL);
}

/**
 * submit the control type of request to the Root hub/Device based on the devnum
 *
 * @param usbdev	pointer to the USB device
 * @param pipe		contains the DIR_IN or OUT , devnum
 * @param buffer	buffer to be read/written based on the request
 * @param length	length of the buffer
 * @param setup		Request type
 * @return returns 0 if successful else -1 on failure
 */
int
submit_control_msg(struct usb_device *usbdev, unsigned long pipe, void *buffer,
					int length, struct devrequest *setup)
{
	struct xhci_ctrl *ctrl = usbdev->controller;
	int ret = -1;

	if (usb_pipetype(pipe) != PIPE_CONTROL) {
		debug("non-control pipe (type=%lu)", usb_pipetype(pipe));
		return -1;
	}
	if (setup->request == USB_REQ_SET_CONFIGURATION) {
		if (usbdev->devnum > 1) {
			xhci_submit_root(usbdev, pipe, buffer, length, setup);
			return xhci_submit_async(usbdev, pipe, buffer,
								length, setup);
		} else {
			usbdev->status = 0;
			usbdev->act_len = length;
			return 0;
		}
	}

	if (usb_pipedevice(pipe) == ctrl->rootdev) {
		if (ctrl->rootdev == 0)
			usbdev->speed = USB_SPEED_SUPER;

		return xhci_submit_root(usbdev, pipe, buffer, length, setup);
	}

	if ((setup->request == USB_REQ_SET_ADDRESS))
		return xhci_submit_root(usbdev, pipe, buffer, length, setup);

	ret = xhci_submit_async(usbdev, pipe, buffer, length, setup);

	xhci_flush_inval_cache((uint32_t)buffer, length, 0);

	return ret;
}

/**
 * Allocates the necessary data structures
 * for XHCI host controller
 *
 * @param index	index to the host controller data structure
 * @param hccr	pointer HOST Controller Control Registers
 * @param hcor	pointer HOST Controller Operational Registers
 * @return 0 if successful else -1 on failure
 */
int xhci_mem_init(int index, struct xhci_hccr *hccr, struct xhci_hcor *hcor)
{
	uint64_t val_64;
	uint64_t trb_64;
	uint32_t val;
	uint32_t temp;
	unsigned long deq;
	int i;
	struct xhci_segment *seg;
	struct xhci_ctrl *ctrl = &xhcic[index];

	/* DCBAA initialization */
	ctrl->dcbaa = (struct xhci_device_context_array *)
			xhci_malloc(sizeof(struct xhci_device_context_array));
	if (ctrl->dcbaa == NULL) {
		debug("unable to allocate DCBA\n");
		return -1;
	}

	val_64 = (uintptr_t)ctrl->dcbaa;
	/* Set the pointer in DCBAA register */
	xhci_writel_64(&hcor->or_dcbaap, val_64);

	/* Command ring control pointer register initialization */
	ctrl->cmd_ring = xhci_ring_alloc(1, true);
	if (ctrl->cmd_ring == NULL) {
		debug("unable to allocate TRB RING\n");
		free(ctrl->dcbaa);
		return -1;
	}

	/* Set the address in the Command Ring Control register */
	trb_64 = (uintptr_t)ctrl->cmd_ring->first_seg->trbs;
	val_64 = xhci_readl_64(&hcor->or_crcr);
	val_64 = (val_64 & (u64) CMD_RING_RSVD_BITS) |
		(trb_64 & (u64) ~CMD_RING_RSVD_BITS) |
		ctrl->cmd_ring->cycle_state;
	xhci_writel_64(&hcor->or_crcr, val_64);

	/* write the addres of db register */
	val = xhci_readl(&hccr->cr_dboff);
	val &= DBOFF_MASK;
	ctrl->dba = (struct xhci_doorbell_array *)((char *)hccr + val);

	/* write the addres of runtime register */
	val = xhci_readl(&hccr->cr_rtsoff);
	val &= RTSOFF_MASK;
	ctrl->run_regs = (struct xhci_run_regs *)((char *)hccr + val);

	/* writting the addrwess of ir_set structure */
	ctrl->ir_set = &ctrl->run_regs->ir_set[0];

	/* Event ring does not maintain link TRB */
	ctrl->event_ring = xhci_ring_alloc(ERST_NUM_SEGS, false);
	ctrl->erst.entries = (struct xhci_erst_entry *)
		xhci_malloc(sizeof(struct xhci_erst_entry) * ERST_NUM_SEGS);

	ctrl->erst.num_entries = ERST_NUM_SEGS;

	for (val = 0, seg = ctrl->event_ring->first_seg;
			val < ERST_NUM_SEGS;
			val++) {
		trb_64 = 0;
		trb_64 = (uintptr_t)seg->trbs;
		struct xhci_erst_entry *entry = &ctrl->erst.entries[val];
		xhci_writel_64(&entry->seg_addr, trb_64);
		entry->seg_size = cpu_to_le32(TRBS_PER_SEGMENT);
		entry->rsvd = 0;
		seg = seg->next;
		xhci_flush_inval_cache((uint32_t)entry,
					sizeof(struct xhci_erst_entry), 1);

	}

	deq = (unsigned long)ctrl->event_ring->dequeue;

	xhci_flush_inval_cache((uint32_t)deq, XHCI_TRB, 1);

	/* Update HC event ring dequeue pointer */
	temp = xhci_readl_64(&ctrl->ir_set->erst_dequeue);
	temp &= ERST_PTR_MASK;

	/*
	 * Don't clear the EHB bit (which is RW1C) because
	 * there might be more events to service.
	 */
	/* clearing the EHB bit here prior to writting the ERST dequeue */
	temp &= ~ERST_EHB;
	xhci_writel_64(&ctrl->ir_set->erst_dequeue,
			(((u64) deq & (u64) ~ERST_PTR_MASK) | temp));

	/* set ERST count with the number of entries in the segment table */
	val = xhci_readl(&ctrl->ir_set->erst_size);
	val &= ERST_SIZE_MASK;
	val |= ERST_NUM_SEGS;
	xhci_writel(&ctrl->ir_set->erst_size, val);

	/* this is the evenet ring segment table pointer */
	val_64 = xhci_readl_64(&ctrl->ir_set->erst_base);
	val_64 &= ERST_PTR_MASK;
	val_64 |= ((u32)(ctrl->erst.entries) & ~ERST_PTR_MASK);

	xhci_writel_64(&ctrl->ir_set->erst_base, val_64);

	/* initializing the viretualdevices to NULL */
	for (i = 0; i < MAX_HC_SLOTS; ++i)
		ctrl->devs[i] = NULL;

	return 0;
}

/**
 * Intialises the XHCI host controller
 * and allocates the necessary data structures
 *
 * @param index	index to the host controller data structure
 * @return pointer to the intialised controller
 */
int usb_lowlevel_init(int index, void **controller)
{
	uint32_t val;
	uint32_t val2;
	uint32_t temp;
	uint32_t reg;
	struct xhci_hccr *hccr;
	struct xhci_hcor *hcor;
	struct xhci_ctrl *ctrl;

	if (xhci_hcd_init(index, &hccr, (struct xhci_hcor **)&hcor) != 0)
		return -ENODEV;

	if (xhci_halt(hcor) != 0)
		return -ENODEV;

	if (xhci_reset(hcor) != 0)
		return -ENODEV;

	ctrl = &xhcic[index];

	ctrl->hccr = hccr;
	ctrl->hcor = hcor;

	/*
	 * Program the Number of Device Slots Enabled field in the CONFIG
	 * register with the max value of slots the HC can handle.
	 */
	val = (xhci_readl(&hccr->cr_hcsparams1) & HCS_SLOTS_MASK);
	val2 = xhci_readl(&hcor->or_config);
	val |= (val2 & ~HCS_SLOTS_MASK);
	xhci_writel(&hcor->or_config, val);

	/* initializing xhci data structures */
	if (xhci_mem_init(index, hccr, hcor) < 0)
		return -ENOMEM;

	temp = xhci_readl(&hcor->or_dnctrl);
	temp &= ~DEV_NOTE_MASK;
	temp |= DEV_NOTE_FWAKE;
	xhci_writel(&hcor->or_dnctrl, temp);

	reg = xhci_readl(&hccr->cr_hcsparams1);
	descriptor.hub.bNbrPorts = ((reg & HCS_MAX_PORTS_MASK) >>
						HCS_MAX_PORTS_SHIFT);
	printf("Register %x NbrPorts %d\n", reg, descriptor.hub.bNbrPorts);
	/* Port Indicators */
	if (HCS_INDICATOR(reg))
		descriptor.hub.wHubCharacteristics |= 0x80;
	/* Port Power Control */
	if (HCC_PPC(reg))
		descriptor.hub.wHubCharacteristics |= 0x01;
	else
		descriptor.hub.wHubCharacteristics |= 0x02;

	/* per port overcurrent reporting */
	descriptor.hub.wHubCharacteristics |= 0x08;

	if (xhci_start(hcor)) {
		xhci_halt(hcor);
		return -ENODEV;
	}

	temp = xhci_readl(&ctrl->ir_set->irq_control);
	temp &= ~ER_IRQ_INTERVAL_MASK;
	temp |= (u32) 160;
	xhci_writel(&ctrl->ir_set->irq_control, temp);

	/* Set the HCD state before we enable the irqs */
	temp = xhci_readl(&hcor->or_usbcmd);
	temp |= (CMD_EIE);
	xhci_writel(&hcor->or_usbcmd, temp);

	temp = xhci_readl(&ctrl->ir_set->irq_pending);
	xhci_writel(&ctrl->ir_set->irq_pending, ER_IRQ_ENABLE(temp));

	reg = HC_VERSION(xhci_readl(&hccr->cr_capbase));
	printf("USB XHCI %x.%02x\n", reg >> 8, reg & 0xff);

	ctrl->rootdev = 0;

	*controller = &xhcic[index];
	return 0;
}

/**
 * Stops the XHCI host controller
 * and cleans up all the related data structures
 *
 * @param index	index to the host controller data structure
 * @return none
 */
int usb_lowlevel_stop(int index)
{
	struct xhci_ctrl *ctrl = (xhcic + index);

	xhci_halt(ctrl->hcor);
	xhci_reset(ctrl->hcor);

	xhci_hcd_stop(index);

	xhci_cleanup(ctrl);

	return 0;
}
