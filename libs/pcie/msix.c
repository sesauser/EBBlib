#include "msix.h"
#include "../acpi/platform.h"
#include <l4/kip.h>

#define msi_control_reg(base)		(base + PCI_MSI_FLAGS)
#define msi_lower_address_reg(base)	(base + PCI_MSI_ADDRESS_LO)
#define msi_upper_address_reg(base)	(base + PCI_MSI_ADDRESS_HI)
#define msi_data_reg(base, is64bit)	\
	(base + ((is64bit == 1) ? PCI_MSI_DATA_64 : PCI_MSI_DATA_32))
#define msi_mask_reg(base, is64bit)	\
	(base + ((is64bit == 1) ? PCI_MSI_MASK_64 : PCI_MSI_MASK_32))
#define is_64bit_address(control)	(!!(control & PCI_MSI_FLAGS_64BIT))
#define is_mask_bit_support(control)	(!!(control & PCI_MSI_FLAGS_MASKBIT))

#define msix_table_offset_reg(base)	(base + PCI_MSIX_TABLE)
#define msix_pba_offset_reg(base)	(base + PCI_MSIX_PBA)
#define msix_table_size(control) 	((control & PCI_MSIX_FLAGS_QSIZE)+1)
#define multi_msix_capable(control)	msix_table_size((control))

int enable_msix(struct pcie_device* self, struct msix_entry *entries, int nr)
{
	static int initialized = 0;
	static int available = 0;
	if (!initialized)
	{
		initialized = 1;
		L4_KernelInterfacePage_t* kip = (L4_KernelInterfacePage_t *)L4_KernelInterface(0,0,0);
		available = L4_ThreadIdSystemBase(kip);
	}
	if (available<16)	// don't use the first 16 interrupts..?..
		return -1;	// no more free interrupts available
	if (available-16<nr)
		return available-16;
	if (nr>self->msixCount)
		return self->msixCount;
	int i,j;
	for (i = 0; i < nr; i++)
	{
		if (entries[i].entry >= self->msixCount)
			return -1;		/* invalid entry */
		for (j = i + 1; j < nr; j++) {
			if (entries[i].entry == entries[j].entry)
				return -1;	/* duplicate entry */
		}
	}
	// disable (normal) interrupts
	unsigned short cmd = self->read_config_word(self, 4);
	mb();
	unsigned short newcmd = cmd | 0x400;
	if (cmd!=newcmd)
	{
		self->write_config_word(self, 4, newcmd);
		mb();
	}
	// get cpu id
	int cpu_id;
	int b = 1;
	asm volatile ("cpuid; mov %%ebx, %0;" : "=r"(cpu_id) : "a"(b) : "%ebx","%ecx","%edx");
	cpu_id = (cpu_id >> 24) & 0xFF;
	unsigned short control = self->read_config_word(self, self->msixCap+2);
	mb();
	unsigned short newcontrol = control | (3 << 14);
	if (control!=newcontrol)
	{
		self->write_config_word(self, self->msixCap+2, newcontrol);
		control = newcontrol;
		mb();
	}
	// enable and mask msix
	for (i = 0; i < nr; i++)
	{
		--available;
		entries[i].vector = available;
		// set irq to device
		self->msixTable[entries[i].entry*4+0] = 0xFEE00000 | (cpu_id << 12);
		self->msixTable[entries[i].entry*4+1] = 0;
		self->msixTable[entries[i].entry*4+2] = ((0x44+available) & 0xFF);	// irq vector, HACK for L4 IO-APIC
		// defaults to masked..
		int control = self->msixTable[entries[i].entry*4+3];
		mb();
		self->msixTable[entries[i].entry*4+3] = control & (~1);	// unmask
		mb();
		//printf("%d(%d): %p %p %p %p\n", i, entries[i].entry, self->msixTable[entries[i].entry*4+3],self->msixTable[entries[i].entry*4+2], self->msixTable[entries[i].entry*4+1], self->msixTable[entries[i].entry*4+0]);
	}
	mb();
	// unmask msix?
	newcontrol = control & (~(1<<14));
	self->write_config_word(self, self->msixCap+2, newcontrol);
	//printf("cfg: %X -> %X\n", control, self->read_config_word(self, self->msixCap+2));
	return 0;
}

void disable_msix(struct pcie_device* dev)
{
	unsigned short control = dev->read_config_word(dev, dev->msixCap+2);
	unsigned short newcontrol = control & (~(3 << 14));
	if (control!=newcontrol)
	{
		dev->write_config_word(dev, dev->msixCap+2, newcontrol);
	}
	// enable (normal) interrupts
	unsigned short cmd = dev->read_config_word(dev, 4);
	unsigned short newcmd = cmd & (~0x400);
	if (cmd!=newcmd)
	{
		dev->write_config_word(dev, 4, newcmd);
	}
}

/*static u32 __msix_mask_irq(struct msi_desc *desc, u32 flag)
{
	u32 mask_bits = desc->masked;
	unsigned offset = desc->msi_attrib.entry_nr * PCI_MSIX_ENTRY_SIZE +
						PCI_MSIX_ENTRY_VECTOR_CTRL;
	mask_bits &= ~PCI_MSIX_ENTRY_CTRL_MASKBIT;
	if (flag)
		mask_bits |= PCI_MSIX_ENTRY_CTRL_MASKBIT;
	writel(mask_bits, desc->mask_base + offset);

	return mask_bits;
}

static void msix_mask_irq(struct msi_desc *desc, u32 flag)
{
	desc->masked = __msix_mask_irq(desc, flag);
}

static void msi_set_mask_bit(struct irq_data *data, u32 flag)
{
	struct msi_desc *desc = irq_data_get_msi(data);

	msix_mask_irq(desc, flag);
	readl(desc->mask_base);		// Flush write to device
}

void mask_msi_irq(struct irq_data *data)
{
	msi_set_mask_bit(data, 1);
}

void unmask_msi_irq(struct irq_data *data)
{
	msi_set_mask_bit(data, 0);
}

void __read_msi_msg(struct msi_desc *entry, struct msi_msg *msg)
{
	BUG_ON(entry->dev->current_state != PCI_D0);

	if (entry->msi_attrib.is_msix) {
		void __iomem *base = entry->mask_base +
			entry->msi_attrib.entry_nr * PCI_MSIX_ENTRY_SIZE;

		msg->address_lo = readl(base + PCI_MSIX_ENTRY_LOWER_ADDR);
		msg->address_hi = readl(base + PCI_MSIX_ENTRY_UPPER_ADDR);
		msg->data = readl(base + PCI_MSIX_ENTRY_DATA);
	} else {
		struct pci_dev *dev = entry->dev;
		int pos = entry->msi_attrib.pos;
		u16 data;

		pci_read_config_dword(dev, msi_lower_address_reg(pos),
					&msg->address_lo);
		if (entry->msi_attrib.is_64) {
			pci_read_config_dword(dev, msi_upper_address_reg(pos),
						&msg->address_hi);
			pci_read_config_word(dev, msi_data_reg(pos, 1), &data);
		} else {
			msg->address_hi = 0;
			pci_read_config_word(dev, msi_data_reg(pos, 0), &data);
		}
		msg->data = data;
	}
}

void read_msi_msg(unsigned int irq, struct msi_msg *msg)
{
	struct msi_desc *entry = irq_get_msi_desc(irq);

	__read_msi_msg(entry, msg);
}


void __write_msi_msg(struct msi_desc *entry, struct msi_msg *msg)
{
	if (entry->dev->current_state != PCI_D0) {
		// Don't touch the hardware now
	} else if (entry->msi_attrib.is_msix) {
		void __iomem *base;
		base = entry->mask_base +
			entry->msi_attrib.entry_nr * PCI_MSIX_ENTRY_SIZE;

		writel(msg->address_lo, base + PCI_MSIX_ENTRY_LOWER_ADDR);
		writel(msg->address_hi, base + PCI_MSIX_ENTRY_UPPER_ADDR);
		writel(msg->data, base + PCI_MSIX_ENTRY_DATA);
	} else {
		struct pci_dev *dev = entry->dev;
		int pos = entry->msi_attrib.pos;
		u16 msgctl;

		pci_read_config_word(dev, msi_control_reg(pos), &msgctl);
		msgctl &= ~PCI_MSI_FLAGS_QSIZE;
		msgctl |= entry->msi_attrib.multiple << 4;
		pci_write_config_word(dev, msi_control_reg(pos), msgctl);

		pci_write_config_dword(dev, msi_lower_address_reg(pos),
					msg->address_lo);
		if (entry->msi_attrib.is_64) {
			pci_write_config_dword(dev, msi_upper_address_reg(pos),
						msg->address_hi);
			pci_write_config_word(dev, msi_data_reg(pos, 1),
						msg->data);
		} else {
			pci_write_config_word(dev, msi_data_reg(pos, 0),
						msg->data);
		}
	}
	entry->msg = *msg;
}

void write_msi_msg(unsigned int irq, struct msi_msg *msg)
{
	struct msi_desc *entry = irq_get_msi_desc(irq);

	__write_msi_msg(entry, msg);
}

static void free_msi_irqs(struct pci_dev *dev)
{
	struct msi_desc *entry, *tmp;

	list_for_each_entry(entry, &dev->msi_list, list) {
		int i, nvec;
		if (!entry->irq)
			continue;
		nvec = 1 << entry->msi_attrib.multiple;
		for (i = 0; i < nvec; i++)
			BUG_ON(irq_has_action(entry->irq + i));
	}

	arch_teardown_msi_irqs(dev);

	list_for_each_entry_safe(entry, tmp, &dev->msi_list, list) {
		if (entry->msi_attrib.is_msix) {
			if (list_is_last(&entry->list, &dev->msi_list))
				iounmap(entry->mask_base);
		}
		list_del(&entry->list);
		kfree(entry);
	}
}

static struct msi_desc *alloc_msi_entry(struct pci_dev *dev)
{
	struct msi_desc *desc = kzalloc(sizeof(*desc), GFP_KERNEL);
	if (!desc)
		return NULL;

	INIT_LIST_HEAD(&desc->list);
	desc->dev = dev;

	return desc;
}

static void pci_intx_for_msi(struct pci_dev *dev, int enable)
{
	if (!(dev->dev_flags & PCI_DEV_FLAGS_MSI_INTX_DISABLE_BUG))
		pci_intx(dev, enable);
}

static int msix_setup_entries(struct pci_dev *dev, unsigned pos,
				void __iomem *base, struct msix_entry *entries,
				int nvec)
{
	struct msi_desc *entry;
	int i;

	for (i = 0; i < nvec; i++) {
		entry = alloc_msi_entry(dev);
		if (!entry) {
			if (!i)
				iounmap(base);
			else
				free_msi_irqs(dev);
			// No enough memory. Don't try again
			return -ENOMEM;
		}

		entry->msi_attrib.is_msix	= 1;
		entry->msi_attrib.is_64		= 1;
		entry->msi_attrib.entry_nr	= entries[i].entry;
		entry->msi_attrib.default_irq	= dev->irq;
		entry->msi_attrib.pos		= pos;
		entry->mask_base		= base;

		list_add_tail(&entry->list, &dev->msi_list);
	}

	return 0;
}

static void msix_program_entries(struct pci_dev *dev,
					struct msix_entry *entries)
{
	struct msi_desc *entry;
	int i = 0;

	list_for_each_entry(entry, &dev->msi_list, list) {
		int offset = entries[i].entry * PCI_MSIX_ENTRY_SIZE +
						PCI_MSIX_ENTRY_VECTOR_CTRL;

		entries[i].vector = entry->irq;
		irq_set_msi_desc(entry->irq, entry);
		entry->masked = readl(entry->mask_base + offset);
		msix_mask_irq(entry, 1);
		i++;
	}
}

static int msix_capability_init(struct pci_dev *dev,
				struct msix_entry *entries, int nvec)
{
	int pos, ret;
	u16 control;
	void __iomem *base;

	pos = pci_find_capability(dev, PCI_CAP_ID_MSIX);
	pci_read_config_word(dev, pos + PCI_MSIX_FLAGS, &control);

	// Ensure MSI-X is disabled while it is set up
	control &= ~PCI_MSIX_FLAGS_ENABLE;
	pci_write_config_word(dev, pos + PCI_MSIX_FLAGS, control);

	// Request & Map MSI-X table region
	base = msix_map_region(dev, pos, multi_msix_capable(control));
	if (!base)
		return -ENOMEM;

	ret = msix_setup_entries(dev, pos, base, entries, nvec);
	if (ret)
		return ret;

	ret = arch_setup_msi_irqs(dev, nvec, PCI_CAP_ID_MSIX);
	if (ret)
		goto error;

	control |= PCI_MSIX_FLAGS_MASKALL | PCI_MSIX_FLAGS_ENABLE;
	pci_write_config_word(dev, pos + PCI_MSIX_FLAGS, control);

	msix_program_entries(dev, entries);

	// Set MSI-X enabled bits and unmask the function
	pci_intx_for_msi(dev, 0);
	dev->msix_enabled = 1;

	control &= ~PCI_MSIX_FLAGS_MASKALL;
	pci_write_config_word(dev, pos + PCI_MSIX_FLAGS, control);

	return 0;

error:
	if (ret < 0) {
		struct msi_desc *entry;
		int avail = 0;

		list_for_each_entry(entry, &dev->msi_list, list) {
			if (entry->irq != 0)
				avail++;
		}
		if (avail != 0)
			ret = avail;
	}

	free_msi_irqs(dev);

	return ret;
}

int pci_enable_msix(struct pci_dev *dev, struct msix_entry *entries, int nvec)
{
	int status, nr_entries;
	int i, j;

	if (!entries)
		return -EINVAL;

	status = pci_msi_check_device(dev, nvec, PCI_CAP_ID_MSIX);
	if (status)
		return status;

	nr_entries = pci_msix_table_size(dev);
	if (nvec > nr_entries)
		return nr_entries;

	// Check for any invalid entries
	for (i = 0; i < nvec; i++) {
		if (entries[i].entry >= nr_entries)
			return -EINVAL;		// invalid entry
		for (j = i + 1; j < nvec; j++) {
			if (entries[i].entry == entries[j].entry)
				return -EINVAL;	// duplicate entry
		}
	}
	WARN_ON(!!dev->msix_enabled);

	// Check whether driver already requested for MSI irq
	if (dev->msi_enabled) {
		dev_info(&dev->dev, "can't enable MSI-X "
		       "(MSI IRQ already assigned)\n");
		return -EINVAL;
	}
	status = msix_capability_init(dev, entries, nvec);
	return status;
}*/
