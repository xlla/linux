/*
 * Driver for the Intel PSH IPC mechanism
 *
 * (C) Copyright 2012 Intel Corporation
 * Author: Yang Bin (bin.yang@intel.com)
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/platform_data/x86/intel_psh_ipc.h>
#include <linux/pm_runtime.h>
#include <linux/semaphore.h>
#include <linux/workqueue.h>

#define STATUS_PSH2IA(x)       (1 << ((x) + 6))
#define FLAG_BIND              (1 << 0)

#define PIMR_ADDR(x)           (&ipcdev->psh_regs->pimr##x)

#define PSH_REG_ADDR(x)                (&ipcdev->psh_regs->x)

#define PSH_CH_HANDLE(x)       (ipcdev->channel_handle[x])
#define PSH_CH_DATA(x)         (ipcdev->channel_data[x])
#define PSH_CH_FLAG(x)         (ipcdev->flags[x])

#define CMD_EXECUTE		"cmd exe"
#define CMD_DUMP		"cmd dump"

/* PSH registers */
struct psh_registers {
       u32             pimr0;          /* 00h */
       u32             csr;            /* 04h */
       u32             pmctl;          /* 08h */
       u32             pmstat;         /* 0Ch */
       u32             psh_msi_direct; /* 10h */
       u32             res1[59];       /* 14h ~ FCh + 3, padding */
       u32             pimr3;          /* 100h */
       struct psh_msg  scu2psh;        /* 104h ~ 108h + 3 */
       struct psh_msg  psh2scu;        /* 10Ch ~ 110h + 3 */
       u32             res2[187];      /* 114h ~ 3FCh + 3, padding */
       u32             pisr;           /* 400h */
       u32             scratchpad[2];  /* 404h ~ 407h */
       u32             res3[61];       /* 40Ch ~ 4FCh + 3, padding */
       u32             pimr1;          /* 500h */
       struct psh_msg  ia2psh[NUM_IA2PSH_IPC]; /* 504h ~ 520h + 3 */
       struct psh_msg  psh2ia[NUM_PSH2IA_IPC]; /* 524h ~ 540h + 3 */
       u32             res4[175];      /* 544h ~ 7FCh + 3, padding */
       u32             pimr2;          /* 800h */
       struct psh_msg  cry2psh;        /* 804h ~ 808h + 3 */
       struct psh_msg  psh2cry;        /* 80Ch ~ 810h + 3 */
} __packed;

struct ipc_controller_t {
       struct device *dev;
       struct psh_registers __iomem *psh_regs;
       int irq;

       spinlock_t              lock;
       int                     flags[NUM_ALL_CH];
       struct semaphore        ch_lock[NUM_ALL_CH];
       struct mutex            psh_mutex;
       psh_channel_handle_t    channel_handle[NUM_PSH2IA_IPC];
       void                    *channel_data[NUM_PSH2IA_IPC];
};

static struct ipc_controller_t *ipc_ctrl;

/**
 * intel_ia2psh_command - send IA to PSH command
 * Send ia2psh command and return psh message and status
 *
 * @in: input psh message
 * @out: output psh message
 * @ch: psh channel
 * @timeout: timeout for polling busy bit, in us
 */
int intel_ia2psh_command(struct psh_msg *in, struct psh_msg *out, int ch, int timeout)
{
       struct ipc_controller_t *ipcdev = ipc_ctrl;
       u32 status;
       int ret = 0;

       might_sleep();

       if (!ipcdev)
               return -ENODEV;

       if (ch < PSH_SEND_CH0 || ch > PSH_SEND_CH0 + NUM_IA2PSH_IPC - 1 || in == NULL)
               return -EINVAL;

       if (!in || in->msg & CHANNEL_BUSY)
               return -EINVAL;

       pm_runtime_get_sync(ipcdev->dev);
       down(&ipcdev->ch_lock[ch]);

       in->msg |= CHANNEL_BUSY;
       /* Check if channel is ready for IA sending command */

       if (readl(PSH_REG_ADDR(ia2psh[ch].msg)) & CHANNEL_BUSY) {
               ret = -EBUSY;
               goto end;
       }

       writel(in->param, PSH_REG_ADDR(ia2psh[ch].param));
       writel(in->msg, PSH_REG_ADDR(ia2psh[ch].msg));

       /* Input timeout is zero, do not check channel status */
       if (timeout == 0)
               goto end;

       /* Input timeout is nonzero, check channel status */
       while (((status = readl(PSH_REG_ADDR(ia2psh[ch].msg))) & CHANNEL_BUSY) && timeout) {
               usleep_range(100, 101);
               timeout -= 100;
       }

       if (timeout <= 0) {
               ret = -ETIMEDOUT;
               dev_err(ipcdev->dev, "ia2psh channel %d is always busy!\n", ch);
       } else if (out) {
               out->param = readl(PSH_REG_ADDR(ia2psh[ch].param));
               out->msg = status;
       }

end:
       up(&ipcdev->ch_lock[ch]);
       pm_runtime_put(ipcdev->dev);

       return ret;
}
EXPORT_SYMBOL(intel_ia2psh_command);

/**
 * intel_psh_ipc_bind - bind a handler to a psh channel
 *
 * @ch: psh channel
 * @handle: handle function called when IA received psh interrupt
 * @data: data passed to handle
 */
int intel_psh_ipc_bind(int ch, psh_channel_handle_t handle, void *data)
{
       struct ipc_controller_t *ipcdev = ipc_ctrl;
       unsigned long flags;

       if (!ipcdev)
               return -ENODEV;

       if (!handle ||
           ch < PSH_RECV_CH0 || ch > PSH_RECV_CH0 + NUM_PSH2IA_IPC - 1)
               return -EINVAL;

       mutex_lock(&ipcdev->psh_mutex);

       down(&ipcdev->ch_lock[ch]);
       if (PSH_CH_HANDLE(ch - PSH_RECV_CH0) != NULL) {
               up(&ipcdev->ch_lock[ch]);
               mutex_unlock(&ipcdev->psh_mutex);
               return -EBUSY;
       }

       PSH_CH_DATA(ch - PSH_RECV_CH0) = data;
       PSH_CH_HANDLE(ch - PSH_RECV_CH0) = handle;
       up(&ipcdev->ch_lock[ch]);

       pm_runtime_get_sync(ipcdev->dev);
       spin_lock_irqsave(&ipcdev->lock, flags);
       PSH_CH_FLAG(ch) |= FLAG_BIND;
       writel(readl(PIMR_ADDR(1)) | (1 << (ch - PSH_RECV_CH0)), PIMR_ADDR(1));
       spin_unlock_irqrestore(&ipcdev->lock, flags);
       pm_runtime_put(ipcdev->dev);

       mutex_unlock(&ipcdev->psh_mutex);
       return 0;
}
EXPORT_SYMBOL(intel_psh_ipc_bind);

/**
 * intel_psh_ipc_unbind - unbind a handler to a psh channel
 *
 * @ch: psh channel
 */
void intel_psh_ipc_unbind(int ch)
{
       struct ipc_controller_t *ipcdev = ipc_ctrl;
       unsigned long flags;

       if (!ipcdev)
               return;

       if (ch < PSH_RECV_CH0 || ch > PSH_RECV_CH0 + NUM_PSH2IA_IPC - 1)
               return;

       if (!(PSH_CH_FLAG(ch) & FLAG_BIND))
               return;

       mutex_lock(&ipcdev->psh_mutex);

       pm_runtime_get_sync(ipcdev->dev);
       spin_lock_irqsave(&ipcdev->lock, flags);
       PSH_CH_FLAG(ch) &= ~FLAG_BIND;
       writel(readl(PIMR_ADDR(1)) & (~(1 << (ch - PSH_RECV_CH0))), PIMR_ADDR(1));
       spin_unlock_irqrestore(&ipcdev->lock, flags);
       pm_runtime_put(ipcdev->dev);

       down(&ipcdev->ch_lock[ch]);
       PSH_CH_HANDLE(ch - PSH_RECV_CH0) = NULL;
       up(&ipcdev->ch_lock[ch]);

       mutex_unlock(&ipcdev->psh_mutex);
}
EXPORT_SYMBOL(intel_psh_ipc_unbind);

void intel_psh_ipc_disable_irq(void)
{
       struct ipc_controller_t *ipcdev = ipc_ctrl;

       disable_irq(ipcdev->irq);
}
EXPORT_SYMBOL(intel_psh_ipc_disable_irq);

void intel_psh_ipc_enable_irq(void)
{
       struct ipc_controller_t *ipcdev = ipc_ctrl;

       enable_irq(ipcdev->irq);
}
EXPORT_SYMBOL(intel_psh_ipc_enable_irq);

static void psh_recv_handle(struct ipc_controller_t *ipcdev, int i)
{
       int msg, param;

       down(&ipcdev->ch_lock[i + PSH_RECV_CH0]);

       msg = readl(PSH_REG_ADDR(psh2ia[i].msg)) & (~CHANNEL_BUSY);
       param = readl(PSH_REG_ADDR(psh2ia[i].param));
       /* write back to clear the busy bit */
       writel(msg, PSH_REG_ADDR(psh2ia[i].msg));

       if (PSH_CH_HANDLE(i) == NULL) {
               dev_err(ipcdev->dev, "Ignore message from channel %d\n", i + PSH_RECV_CH0);
               goto end;
       }

       PSH_CH_HANDLE(i)(msg, param, PSH_CH_DATA(i));
end:
       up(&ipcdev->ch_lock[i + PSH_RECV_CH0]);
}

static irqreturn_t psh_ipc_irq(int irq, void *data)
{
       struct ipc_controller_t *ipcdev = data;
       unsigned int i;
       u32 status;

       pm_runtime_get_sync(ipcdev->dev);
       status = readl(PSH_REG_ADDR(pisr));

       for (i = 0; i < NUM_PSH2IA_IPC; i++) {
               if (status & STATUS_PSH2IA(i))
                       psh_recv_handle(ipcdev, i);
       }

       pm_runtime_put(ipcdev->dev);
       return IRQ_HANDLED;
}

static void psh_regs_dump(struct ipc_controller_t *ipcdev)
{
       struct device *dev = ipcdev->dev;
       unsigned int i;

       pm_runtime_get_sync(dev);
       dev_err(dev, "\n<-------------start------------>\n");

       dev_err(dev, "csr:\t%#x\n", readl(PSH_REG_ADDR(csr)));
       dev_err(dev, "pisr:\t%#x\n", readl(PSH_REG_ADDR(pisr)));

       dev_err(dev, "pimr0:\t%#x\n", readl(PIMR_ADDR(0)));
       dev_err(dev, "pimr1:\t%#x\n", readl(PIMR_ADDR(1)));
       dev_err(dev, "pimr2:\t%#x\n", readl(PIMR_ADDR(2)));
       dev_err(dev, "pimr3:\t%#x\n", readl(PIMR_ADDR(3)));

       dev_err(dev, "pmctl:\t%#x\n", readl(PSH_REG_ADDR(pmctl)));
       dev_err(dev, "pmstat:\t%#x\n", readl(PSH_REG_ADDR(pmstat)));
       dev_err(dev, "scratchpad0:\t%#x\n", readl(PSH_REG_ADDR(scratchpad[0])));
       dev_err(dev, "scratchpad1:\t%#x\n", readl(PSH_REG_ADDR(scratchpad[1])));

       for (i = 0; i < NUM_IA2PSH_IPC; i++) {
               dev_err(dev, "ia2psh[%d].msg:\t%#x\n", i, readl(PSH_REG_ADDR(ia2psh[i].msg)));
               dev_err(dev, "ia2psh[%d].param:\t%#x\n", i, readl(PSH_REG_ADDR(ia2psh[i].param)));
       }

       dev_err(dev, "cry2psh.msg:\t%#x\n", readl(PSH_REG_ADDR(cry2psh.msg)));
       dev_err(dev, "cry2psh.param:\t%#x\n", readl(PSH_REG_ADDR(cry2psh.param)));
       dev_err(dev, "scu2psh.msg:\t%#x\n", readl(PSH_REG_ADDR(scu2psh.msg)));
       dev_err(dev, "scu2psh.param:\t%#x\n", readl(PSH_REG_ADDR(scu2psh.param)));

       for (i = 0; i < NUM_PSH2IA_IPC; i++) {
               dev_err(dev, "psh2ia[%d].msg:\t%#x\n", i, readl(PSH_REG_ADDR(psh2ia[i].msg)));
               dev_err(dev, "psh2ia[%d].param:\t%#x\n", i, readl(PSH_REG_ADDR(psh2ia[i].param)));
       }

       dev_err(dev, "psh2cry.msg:\t%#x\n", readl(PSH_REG_ADDR(psh2cry.msg)));
       dev_err(dev, "psh2cry.param:\t%#x\n", readl(PSH_REG_ADDR(psh2cry.param)));

       dev_err(dev, "\n<-------------end------------>\n");
       pm_runtime_put(dev);
}

static struct psh_msg psh_dbg_msg;
static int psh_ch;

static ssize_t psh_msg_show(struct device *dev, struct device_attribute *attr,
                           char *buf)
{
       return snprintf(buf, PAGE_SIZE,
                       "Last ia2psh command with msg: %#x\nparam: %#x\n",
                       psh_dbg_msg.msg, psh_dbg_msg.param);
}

static ssize_t psh_msg_store(struct device *dev, struct device_attribute *attr,
                            const char *buf, size_t size)
{
       u32 msg, param;
       int ret;

       memset(&psh_dbg_msg, 0, sizeof(psh_dbg_msg));

       ret = sscanf(buf, "%x %x", &msg, &param);
       if (ret != 2) {
               dev_err(dev, "Input two arguments as psh msg and param\n");
               return -EINVAL;
       }

       psh_dbg_msg.msg = msg;
       psh_dbg_msg.param = param;

       return size;
}

static ssize_t psh_ch_show(struct device *dev, struct device_attribute *attr,
                          char *buf)
{
       return snprintf(buf, PAGE_SIZE, "Last psh channel: %d\n", psh_ch);
}

static ssize_t psh_ch_store(struct device *dev, struct device_attribute *attr,
                           const char *buf, size_t size)
{
       int ret;

       ret = sscanf(buf, "%d", &psh_ch);
       if (ret != 1) {
               dev_err(dev, "Input one argument as psh channel\n");
               return -EINVAL;
       }

       return size;
}

static ssize_t psh_send_cmd_store(struct device *dev, struct device_attribute *attr,
                                 const char *buf, size_t size)
{
       struct ipc_controller_t *ipcdev = dev_get_drvdata(dev);
       struct psh_msg out_msg;
       int psh_dbg_err;
	int len = strlen(CMD_EXECUTE);
       	if (size >= len && strncmp(buf, CMD_EXECUTE, len) == 0) {
                memset(&out_msg, 0, sizeof(out_msg));

                psh_dbg_err = intel_ia2psh_command(&psh_dbg_msg, &out_msg, psh_ch, 3000000);
                if (psh_dbg_err) {
                        dev_err(dev, "Send ia2psh command failed, err %d\n", psh_dbg_err);
                        psh_regs_dump(ipcdev);
                        return psh_dbg_err;
                } else {
                        psh_dbg_msg.msg = out_msg.msg;
                        psh_dbg_msg.param = out_msg.param;
                }
		return size;
	} else {
                int len = strlen(CMD_DUMP);
                if (size >= len && strncmp(buf, CMD_DUMP, len) == 0) {
                        psh_regs_dump(ipcdev);
                        return size;
                }
        }
	pr_err("Please provide right string as [%s] | [%s]!\n", CMD_EXECUTE, CMD_DUMP);
	return -1;

}

static DEVICE_ATTR(psh_msg, S_IRUGO | S_IWUSR, psh_msg_show, psh_msg_store);
static DEVICE_ATTR(psh_ch, S_IRUGO | S_IWUSR, psh_ch_show, psh_ch_store);
static DEVICE_ATTR(ia2psh_cmd, S_IWUSR, NULL, psh_send_cmd_store);

static struct attribute *psh_attrs[] = {
       &dev_attr_psh_msg.attr,
       &dev_attr_psh_ch.attr,
       &dev_attr_ia2psh_cmd.attr,
       NULL,
};

static struct attribute_group psh_attr_group = {
       .name = "psh_debug",
       .attrs = psh_attrs,
};

static int intel_psh_debug_sysfs_create(struct device *dev)
{
       return sysfs_create_group(&dev->kobj, &psh_attr_group);
}

static void pmic_sysfs_remove(struct device *dev)
{
       sysfs_remove_group(&dev->kobj, &psh_attr_group);
}

static int __maybe_unused psh_ipc_suspend_noirq(struct device *dev)
{
       struct ipc_controller_t *ipcdev = dev_get_drvdata(dev);
       unsigned int i;

       for (i = 0; i < NUM_ALL_CH; i++) {
               if (down_trylock(&ipcdev->ch_lock[i]))
                       break;
       }
       if (i == NUM_ALL_CH) {
               while (i--)
                       up(&ipcdev->ch_lock[i]);
               return -EBUSY;
       }

       return 0;
}

static int __maybe_unused psh_ipc_resume_noirq(struct device *dev)
{
       struct ipc_controller_t *ipcdev = dev_get_drvdata(dev);
       unsigned int i;

       for (i = 0; i < NUM_ALL_CH; i++)
               up(&ipcdev->ch_lock[i]);

       return 0;
}

static int __maybe_unused psh_ipc_runtime_suspend(struct device *dev)
{
       dev_dbg(dev, "runtime suspend called\n");
       return 0;
}

static int __maybe_unused psh_ipc_runtime_resume(struct device *dev)
{
       dev_dbg(dev, "runtime resume called\n");
       return 0;
}

static int psh_ipc_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
       struct ipc_controller_t *ipcdev;
       unsigned int i;
       int ret;
        pr_info("intel psh ipc pci prepare...\n");
       ret = pcim_enable_device(pdev);
       if (ret)
               return ret;

       ret = pcim_iomap_regions(pdev, 1 << 0, pci_name(pdev));
       if (ret)
               return ret;

       ipcdev = devm_kzalloc(&pdev->dev, sizeof(*ipcdev), GFP_KERNEL);
       if (!ipcdev)
               return -ENOMEM;

       spin_lock_init(&ipcdev->lock);
       mutex_init(&ipcdev->psh_mutex);

       for (i = 0; i < NUM_ALL_CH; i++)
               sema_init(&ipcdev->ch_lock[i], 1);

       ipcdev->dev = &pdev->dev;
       ipcdev->psh_regs = pcim_iomap_table(pdev)[0];
       ipcdev->irq = pdev->irq;

       ret = devm_request_threaded_irq(&pdev->dev, pdev->irq, NULL, psh_ipc_irq,
                                       IRQF_ONESHOT, "intel_psh_ipc", ipcdev);
       if (ret) {
               dev_err(&pdev->dev, "Unable to register IRQ %d\n", pdev->irq);
               return ret;
       }

       irq_set_irq_wake(pdev->irq, 1);

       ipc_ctrl = ipcdev;
       pr_info("intel psh ipc ipc_ctrl ready.\n");
       pci_set_drvdata(pdev, ipcdev);

       intel_psh_debug_sysfs_create(&pdev->dev);

       pm_runtime_put_noidle(&pdev->dev);
       pm_runtime_allow(&pdev->dev);

       pr_info("intel psh ipc done.\n");
       return 0;
}

static void psh_ipc_remove(struct pci_dev *pdev)
{
       pm_runtime_forbid(&pdev->dev);
       pm_runtime_get_noresume(&pdev->dev);

       pmic_sysfs_remove(&pdev->dev);

       ipc_ctrl = NULL;
}

static const struct dev_pm_ops psh_ipc_drv_pm_ops = {
       SET_NOIRQ_SYSTEM_SLEEP_PM_OPS(psh_ipc_suspend_noirq, psh_ipc_resume_noirq)
       SET_RUNTIME_PM_OPS(psh_ipc_runtime_suspend, psh_ipc_runtime_resume, NULL)
};

static const struct pci_device_id psh_ipc_id_table[] = {
       { PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x11a3) },
       {}
};
MODULE_DEVICE_TABLE(pci, psh_ipc_id_table);

static struct pci_driver psh_ipc_driver = {
       .name = "intel_psh_ipc",
       .driver = {
               .pm = &psh_ipc_drv_pm_ops,
       },
       .id_table = psh_ipc_id_table,
       .probe = psh_ipc_probe,
       .remove = psh_ipc_remove,
};

module_pci_driver(psh_ipc_driver);

MODULE_AUTHOR("bin.yang@intel.com");
MODULE_DESCRIPTION("Intel PSH IPC driver");
MODULE_LICENSE("GPL v2");
