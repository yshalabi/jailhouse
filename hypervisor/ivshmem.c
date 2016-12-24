/*
 * Jailhouse, a Linux-based partitioning hypervisor
 *
 * Copyright (c) Siemens AG, 2014-2016
 *
 * Authors:
 *  Henning Schild <henning.schild@siemens.com>
 *  Jan Kiszka <jan.kiszka@siemens.com>
 *
 * This work is licensed under the terms of the GNU GPL, version 2.  See
 * the COPYING file in the top-level directory.
 */

/** @addtogroup IVSHMEM
 * Inter Cell communication using a virtual PCI device. The device provides
 * shared memory and interrupts based on MSI-X.
 *
 * The implementation in Jailhouse provides a shared memory device between
 * exactly 2 cells. The link between the two PCI devices is established by
 * choosing the same BDF, memory location, and memory size.
 */

#include <jailhouse/ivshmem.h>
#include <jailhouse/mmio.h>
#include <jailhouse/pci.h>
#include <jailhouse/printk.h>
#include <jailhouse/string.h>
#include <jailhouse/utils.h>
#include <jailhouse/processor.h>
#include <asm/percpu.h>

#define VIRTIO_VENDOR_ID		0x1af4
#define IVSHMEM_DEVICE_ID		0x1110

#define IVSHMEM_CFG_VENDOR_CAP		0x40
#define IVSHMEM_CFG_MSIX_CAP		(IVSHMEM_CFG_VENDOR_CAP + \
					 IVSHMEM_CFG_VENDOR_LEN)

/*
 * We cannot allow dynamic remapping of the shared memory locations under
 * Jailhouse. Therefore, address and size are reported via a vendor capability
 * instead of BARs.
 */
#define IVSHMEM_CFG_SHMEM_ADDR0		(IVSHMEM_CFG_VENDOR_CAP + 4)
#define IVSHMEM_CFG_SHMEM_SIZE0		(IVSHMEM_CFG_VENDOR_CAP + 12)
#define IVSHMEM_CFG_VENDOR_LEN		52

/* Flags in IVSHMEM_CFG_VENDOR_CAP + 3 */
#define IVHSMEM_CFGFLAG_INTX		(1 << (0 + 24))

#define IVSHMEM_MSIX_VECTORS		1

/*
 * Make the region two times as large as the MSI-X table to guarantee a
 * power-of-2 size (encoding constraint of a BAR).
 */
#define IVSHMEM_BAR4_SIZE		(0x10 * IVSHMEM_MSIX_VECTORS * 2)

#define IVSHMEM_REG_ID			0x00
#define IVSHMEM_REG_DOORBELL		0x04
#define IVSHMEM_REG_LSTATE		0x08
#define IVSHMEM_REG_RSTATE		0x0c
#define IVSHMEM_REG_RSTATE_WRITE	0x10

#define IVSHMEM_RSTATE_WRITE_ENABLE	(1ULL << 0)
#define IVSHMEM_RSTATE_WRITE_REGION1	(1ULL << 1)
#define IVSHMEM_RSTATE_OFFSET_MASK	BIT_MASK(63, 2)

struct ivshmem_link {
	struct ivshmem_endpoint eps[2];
	spinlock_t lock;
	u16 bdf;
	struct ivshmem_link *next;
};

static struct ivshmem_link *ivshmem_links;

static const u32 default_cspace[IVSHMEM_CFG_SIZE / sizeof(u32)] = {
	[0x00/4] = (IVSHMEM_DEVICE_ID << 16) | VIRTIO_VENDOR_ID,
	[0x04/4] = (PCI_STS_CAPS << 16),
	[0x08/4] = PCI_DEV_CLASS_OTHER << 24,
	[0x2c/4] = (IVSHMEM_DEVICE_ID << 16) | VIRTIO_VENDOR_ID,
	[PCI_CFG_CAPS/4] = IVSHMEM_CFG_VENDOR_CAP,
	[IVSHMEM_CFG_VENDOR_CAP/4] = (IVSHMEM_CFG_VENDOR_LEN << 16) |
				(IVSHMEM_CFG_MSIX_CAP << 8) | PCI_CAP_VENDOR,
	[IVSHMEM_CFG_MSIX_CAP/4] = (IVSHMEM_MSIX_VECTORS - 1) << 16 |
				   (0x00 << 8) | PCI_CAP_MSIX,
	[(IVSHMEM_CFG_MSIX_CAP + 0x4)/4] = 4,
	[(IVSHMEM_CFG_MSIX_CAP + 0x8)/4] = 0x10 * IVSHMEM_MSIX_VECTORS | 4,
};

static void ivshmem_write_rstate(struct ivshmem_endpoint *ive, u32 new_state)
{
	unsigned long page_virt = TEMPORARY_MAPPING_BASE +
		this_cpu_id() * PAGE_SIZE * NUM_TEMPORARY_PAGES;
	unsigned int region =
		(ive->rstate_write & IVSHMEM_RSTATE_WRITE_REGION1) ? 1 : 0;
	const struct jailhouse_memory *shmem = &ive->shmem[region];
	unsigned long rstate_offs;
	u32 *rstate;

	rstate_offs = ive->rstate_write & IVSHMEM_RSTATE_OFFSET_MASK;

	if (!(ive->rstate_write & IVSHMEM_RSTATE_WRITE_ENABLE) ||
	    rstate_offs > shmem->size - sizeof(*rstate))
		return;

	rstate = (u32 *)(page_virt + (rstate_offs & PAGE_OFFS_MASK));

	/*
	 * Cannot fail: upper levels of page table were already created by
	 * paging_init, and we always map single pages, thus only update the
	 * leaf entry and do not have to deal with huge pages.
	 */
	paging_create(&hv_paging_structs, shmem->phys_start + rstate_offs,
		      PAGE_SIZE, page_virt, PAGE_DEFAULT_FLAGS,
		      PAGING_NON_COHERENT);

	*rstate = new_state;
	memory_barrier();
}

int ivshmem_update_msix(struct pci_device *device)
{
	struct ivshmem_endpoint *ive = device->ivshmem_endpoint;
	union pci_msix_registers cap;
	bool enabled;
	int err;

	if (device->info->num_msix_vectors == 0)
		return 0;

	cap.raw = ive->cspace[IVSHMEM_CFG_MSIX_CAP/4];
	enabled = cap.enable && !cap.fmask &&
		!ive->device->msix_vectors[0].masked &&
		ive->cspace[PCI_CFG_COMMAND/4] & PCI_CMD_MASTER;

	spin_lock(&ive->link->lock);
	err = arch_ivshmem_update_msix(device, enabled);
	spin_unlock(&ive->link->lock);

	return err;
}

static void ivshmem_update_intx(struct ivshmem_endpoint *ive)
{
	bool enabled = ive->cspace[IVSHMEM_CFG_VENDOR_CAP/4] &
		IVHSMEM_CFGFLAG_INTX;
	bool masked = ive->cspace[PCI_CFG_COMMAND/4] & PCI_CMD_INTX_OFF;

	spin_lock(&ive->link->lock);
	if (ive->device->info->num_msix_vectors == 0)
		arch_ivshmem_update_intx(ive, enabled && !masked);
	spin_unlock(&ive->link->lock);
}

static enum mmio_result ivshmem_register_mmio(void *arg,
					      struct mmio_access *mmio)
{
	struct ivshmem_endpoint *ive = arg;

	switch (mmio->address) {
	case IVSHMEM_REG_ID:
		/* read-only ID */
		mmio->value = ive->id;
		break;
	case IVSHMEM_REG_DOORBELL:
		if (mmio->is_write) {
			/*
			 * Hold the link lock while sending the interrupt so
			 * that ivshmem_exit can synchronize on the completion
			 * of the delivery.
			 */
			spin_lock(&ive->link->lock);
			if (ive->remote)
				arch_ivshmem_trigger_interrupt(ive->remote);
			spin_unlock(&ive->link->lock);
		} else {
			mmio->value = 0;
		}
		break;
	case IVSHMEM_REG_LSTATE:
		if (mmio->is_write) {
			spin_lock(&ive->link->lock);

			ive->state = mmio->value;
			if (ive->remote) {
				ivshmem_write_rstate(ive->remote, ive->state);
				arch_ivshmem_trigger_interrupt(ive->remote);
			}

			spin_unlock(&ive->link->lock);
		} else {
			mmio->value = ive->state;
		}
		break;
	case IVSHMEM_REG_RSTATE:
		/* read-only remote state */
		spin_lock(&ive->link->lock);
		mmio->value = ive->remote ? ive->remote->state : 0;
		spin_unlock(&ive->link->lock);
		break;
	case IVSHMEM_REG_RSTATE_WRITE:
		if (mmio->is_write) {
			spin_lock(&ive->link->lock);

			ive->rstate_write = mmio->value;
			ivshmem_write_rstate(ive,
					ive->remote ? ive->remote->state : 0);

			spin_unlock(&ive->link->lock);
		} else {
			mmio->value = ive->rstate_write;
		}
		break;
	default:
		/* ignore any other access */
		mmio->value = 0;
		break;
	}
	return MMIO_HANDLED;
}

static enum mmio_result ivshmem_msix_mmio(void *arg, struct mmio_access *mmio)
{
	struct ivshmem_endpoint *ive = arg;
	u32 *msix_table = (u32 *)ive->device->msix_vectors;

	if (mmio->address % 4)
		goto fail;

	/* MSI-X PBA */
	if (mmio->address >= 0x10 * IVSHMEM_MSIX_VECTORS) {
		if (mmio->is_write) {
			goto fail;
		} else {
			mmio->value = 0;
			return MMIO_HANDLED;
		}
	/* MSI-X Table */
	} else {
		if (mmio->is_write) {
			msix_table[mmio->address / 4] = mmio->value;
			if (ivshmem_update_msix(ive->device))
				return MMIO_ERROR;
		} else {
			mmio->value = msix_table[mmio->address / 4];
		}
		return MMIO_HANDLED;
	}

fail:
	panic_printk("FATAL: Invalid PCI MSI-X table/PBA access, device "
		     "%02x:%02x.%x\n", PCI_BDF_PARAMS(ive->device->info->bdf));
	return MMIO_ERROR;
}

/**
 * update the command register
 * note that we only accept writes to two flags
 */
static int ivshmem_write_command(struct ivshmem_endpoint *ive, u16 val)
{
	u16 *cmd = (u16 *)&ive->cspace[PCI_CFG_COMMAND/4];
	struct pci_device *device = ive->device;
	int err;

	if ((val & PCI_CMD_MASTER) != (*cmd & PCI_CMD_MASTER)) {
		*cmd = (*cmd & ~PCI_CMD_MASTER) | (val & PCI_CMD_MASTER);
		err = ivshmem_update_msix(device);
		if (err)
			return err;
	}

	if ((val & PCI_CMD_MEM) != (*cmd & PCI_CMD_MEM)) {
		if (*cmd & PCI_CMD_MEM) {
			mmio_region_unregister(device->cell, ive->bar0_address);
			mmio_region_unregister(device->cell, ive->bar4_address);
		}
		if (val & PCI_CMD_MEM) {
			ive->bar0_address = (*(u64 *)&device->bar[0]) & ~0xfL;
			/*
			 * Derive the size of region 0 from its BAR mask.
			 * This reasonably assumes that all unmodifiable bits
			 * of the BAR, i.e. all zeros, are in the lower dword.
			 */
			mmio_region_register(device->cell, ive->bar0_address,
					     ~device->info->bar_mask[0] + 1,
					     ivshmem_register_mmio, ive);

			ive->bar4_address = (*(u64 *)&device->bar[4]) & ~0xfL;
			mmio_region_register(device->cell, ive->bar4_address,
					     IVSHMEM_BAR4_SIZE,
					     ivshmem_msix_mmio, ive);
		}
		*cmd = (*cmd & ~PCI_CMD_MEM) | (val & PCI_CMD_MEM);
	}

	if ((val & PCI_CMD_INTX_OFF) != (*cmd & PCI_CMD_INTX_OFF)) {
		*cmd = (*cmd & ~PCI_CMD_INTX_OFF) | (val & PCI_CMD_INTX_OFF);
		ivshmem_update_intx(ive);
	}

	return 0;
}

static int ivshmem_write_msix_control(struct ivshmem_endpoint *ive, u32 val)
{
	union pci_msix_registers *p = (union pci_msix_registers *)&val;
	union pci_msix_registers newval = {
		.raw = ive->cspace[IVSHMEM_CFG_MSIX_CAP/4]
	};

	newval.enable = p->enable;
	newval.fmask = p->fmask;
	if (ive->cspace[IVSHMEM_CFG_MSIX_CAP/4] != newval.raw) {
		ive->cspace[IVSHMEM_CFG_MSIX_CAP/4] = newval.raw;
		return ivshmem_update_msix(ive->device);
	}
	return 0;
}

/**
 * Handler for MMIO-write-accesses to PCI config space of this virtual device.
 * @param device	The device that access should be performed on.
 * @param row		Config space DWORD row of the access.
 * @param mask		Mask selected the DWORD bytes to write.
 * @param value		DWORD to write to the config space.
 *
 * @return PCI_ACCESS_REJECT or PCI_ACCESS_DONE.
 *
 * @see pci_cfg_write_moderate
 */
enum pci_access ivshmem_pci_cfg_write(struct pci_device *device,
				      unsigned int row, u32 mask, u32 value)
{
	struct ivshmem_endpoint *ive = device->ivshmem_endpoint;

	if (row >= ARRAY_SIZE(default_cspace))
		return PCI_ACCESS_REJECT;

	value |= ive->cspace[row] & ~mask;

	switch (row) {
	case PCI_CFG_COMMAND / 4:
		if (ivshmem_write_command(ive, value))
			return PCI_ACCESS_REJECT;
		break;
	case IVSHMEM_CFG_MSIX_CAP / 4:
		if (ivshmem_write_msix_control(ive, value))
			return PCI_ACCESS_REJECT;
		break;
	case IVSHMEM_CFG_VENDOR_CAP / 4:
		ive->cspace[IVSHMEM_CFG_VENDOR_CAP/4] &= ~IVHSMEM_CFGFLAG_INTX;
		ive->cspace[IVSHMEM_CFG_VENDOR_CAP/4] |=
			value & IVHSMEM_CFGFLAG_INTX;
		ivshmem_update_intx(ive);
		break;
	}

	return PCI_ACCESS_DONE;
}

/**
 * Handler for MMIO-read-accesses to PCI config space of this virtual device.
 * @param device	The device that access should be performed on.
 * @param address	Config space address accessed.
 * @param value		Pointer to the return value.
 *
 * @return PCI_ACCESS_DONE.
 *
 * @see pci_cfg_read_moderate
 */
enum pci_access ivshmem_pci_cfg_read(struct pci_device *device, u16 address,
				     u32 *value)
{
	struct ivshmem_endpoint *ive = device->ivshmem_endpoint;

	if (address < sizeof(default_cspace))
		*value = ive->cspace[address / 4] >> ((address % 4) * 8);
	else
		*value = -1;
	return PCI_ACCESS_DONE;
}

/**
 * Register a new ivshmem device.
 * @param cell		The cell the device should be attached to.
 * @param device	The device to be registered.
 *
 * @return 0 on success, negative error code otherwise.
 */
int ivshmem_init(struct cell *cell, struct pci_device *device)
{
	const struct jailhouse_pci_device *dev_info = device->info;
	const struct jailhouse_memory *mem, *peer_mem;
	struct ivshmem_endpoint *ive, *remote;
	struct pci_device *peer_dev;
	struct ivshmem_link *link;
	unsigned int id = 0;

	printk("Adding virtual PCI device %02x:%02x.%x to cell \"%s\"\n",
	       PCI_BDF_PARAMS(dev_info->bdf), cell->config->name);

	if (dev_info->shmem_regions_start + 2 >=
	    cell->config->num_memory_regions)
		return trace_error(-EINVAL);

	mem = jailhouse_cell_mem_regions(cell->config) +
		dev_info->shmem_regions_start;

	for (link = ivshmem_links; link; link = link->next)
		if (link->bdf == dev_info->bdf)
			break;

	if (link) {
		id = link->eps[0].device ? 1 : 0;
		if (link->eps[id].device)
			return trace_error(-EBUSY);

		peer_dev = link->eps[id ^ 1].device;
		peer_mem = jailhouse_cell_mem_regions(peer_dev->cell->config) +
			peer_dev->info->shmem_regions_start;

		/*
		 * Check that the regions and protocols of both peers match.
		 * The 3rd region of both devices must be read-only.
		 */
		if (mem[0].phys_start != peer_mem[0].phys_start ||
		    mem[0].size != peer_mem[0].size ||
		    mem[1].phys_start != peer_mem[2].phys_start ||
		    mem[1].size != peer_mem[2].size ||
		    mem[2].phys_start != peer_mem[1].phys_start ||
		    mem[2].size != peer_mem[1].size ||
		    (mem[2].flags | peer_mem[2].flags) & JAILHOUSE_MEM_WRITE ||
		    dev_info->shmem_protocol != peer_dev->info->shmem_protocol)
			return trace_error(-EINVAL);

		printk("Shared memory connection established: "
		       "\"%s\" <--> \"%s\"\n",
		       cell->config->name, peer_dev->cell->config->name);
	} else {
		link = page_alloc(&mem_pool, 1);
		if (!link)
			return -ENOMEM;

		link->bdf = dev_info->bdf;
		link->next = ivshmem_links;
		ivshmem_links = link;
	}

	ive = &link->eps[id];
	remote = &link->eps[id ^ 1];

	ive->device = device;
	ive->link = link;
	ive->shmem = mem;
	ive->id = id;
	device->ivshmem_endpoint = ive;
	if (remote->device) {
		ive->remote = remote;
		remote->remote = ive;
	}

	device->cell = cell;
	pci_reset_device(device);

	return 0;
}

void ivshmem_reset(struct pci_device *device)
{
	struct ivshmem_endpoint *ive = device->ivshmem_endpoint;
	unsigned int n;

	if (ive->cspace[PCI_CFG_COMMAND/4] & PCI_CMD_MEM) {
		mmio_region_unregister(device->cell, ive->bar0_address);
		mmio_region_unregister(device->cell, ive->bar4_address);
	}

	memset(device->bar, 0, sizeof(device->bar));
	device->msix_registers.raw = 0;

	device->bar[0] = PCI_BAR_64BIT;

	memcpy(ive->cspace, &default_cspace, sizeof(default_cspace));

	ive->cspace[0x08/4] |= device->info->shmem_protocol << 8;

	if (device->info->num_msix_vectors == 0) {
		/* let the PIN rotate based on the device number */
		ive->cspace[PCI_CFG_INT/4] =
			(((device->info->bdf >> 3) & 0x3) + 1) << 8;
		/* disable MSI-X capability */
		ive->cspace[IVSHMEM_CFG_VENDOR_CAP/4] &= 0xffff00ff;
	} else {
		device->bar[4] = PCI_BAR_64BIT;
	}

	for (n = 0; n < 3; n++) {
		ive->cspace[IVSHMEM_CFG_SHMEM_ADDR0/4 + n*4] =
			(u32)ive->shmem[n].virt_start;
		ive->cspace[IVSHMEM_CFG_SHMEM_ADDR0/4 + n*4 + 1] =
			(u32)(ive->shmem[n].virt_start >> 32);
		ive->cspace[IVSHMEM_CFG_SHMEM_SIZE0/4 + n*4] =
			(u32)ive->shmem[n].size;
		ive->cspace[IVSHMEM_CFG_SHMEM_SIZE0/4 + n*4 + 1] =
			(u32)(ive->shmem[n].size >> 32);
	}

	ive->state = 0;
}

/**
 * Unregister a ivshmem device, typically when the corresponding cell exits.
 * @param device	The device to be stopped.
 *
 */
void ivshmem_exit(struct pci_device *device)
{
	struct ivshmem_endpoint *ive = device->ivshmem_endpoint;
	struct ivshmem_endpoint *remote = ive->remote;
	struct ivshmem_link **linkp;

	if (remote) {
		/*
		 * The spinlock synchronizes the disconnection of the remote
		 * device with any in-flight interrupts targeting the device
		 * to be destroyed and any changes to the remote's rstate_write
		 * register.
		 */
		spin_lock(&ive->link->lock);

		remote->remote = NULL;
		ivshmem_write_rstate(remote, 0);
		arch_ivshmem_trigger_interrupt(remote);

		spin_unlock(&ive->link->lock);

		ive->device = NULL;
	} else {
		for (linkp = &ivshmem_links; *linkp; linkp = &(*linkp)->next)
			if (*linkp == ive->link) {
				*linkp = ive->link->next;
				page_free(&mem_pool, ive->link, 1);
				break;
			}
	}
}
