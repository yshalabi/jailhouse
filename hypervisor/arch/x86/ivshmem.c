/*
 * Jailhouse, a Linux-based partitioning hypervisor
 *
 * Copyright (c) Siemens AG, 2014-2016
 *
 * Author:
 *  Henning Schild <henning.schild@siemens.com>
 *  Jan Kiszka <jan.kiszka@siemens.com>
 *
 * This work is licensed under the terms of the GNU GPL, version 2.  See
 * the COPYING file in the top-level directory.
 */

#include <jailhouse/cell.h>
#include <jailhouse/ivshmem.h>
#include <jailhouse/pci.h>
#include <jailhouse/printk.h>
#include <asm/pci.h>

void arch_ivshmem_trigger_interrupt(struct ivshmem_endpoint *ive,
				    unsigned int vector)
{
	if (ive->arch.irq_msg[vector].valid)
		apic_send_irq(ive->arch.irq_msg[vector]);
}

int arch_ivshmem_update_msix(struct pci_device *device, unsigned int vector,
			     bool enabled)
{
	struct ivshmem_endpoint *ive = device->ivshmem_endpoint;
	struct apic_irq_message irq_msg = { .valid = 0 };
	union x86_msi_vector msi;

	if (enabled) {
		msi.raw.address = device->msix_vectors[vector].address;
		msi.raw.data = device->msix_vectors[vector].data;

		irq_msg = x86_pci_translate_msi(device, vector, 0, msi);

		if (irq_msg.valid &&
		    !apic_filter_irq_dest(device->cell, &irq_msg)) {
			panic_printk("FATAL: ivshmem MSI-X target outside of "
				     "cell \"%s\" device %02x:%02x.%x\n",
				     device->cell->config->name,
				     PCI_BDF_PARAMS(device->info->bdf));
			return -EPERM;
		}
	}

	ive->arch.irq_msg[vector] = irq_msg;

	return 0;
}

void arch_ivshmem_update_intx(struct ivshmem_endpoint *ive, bool enabled)
{
}
