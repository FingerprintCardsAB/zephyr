
#include <kernel.h>
#include <misc/printk.h>
#include <device.h>
#include <init.h>
#include <arch/arm/cortex_m/cmsis.h>

extern void __printk_hook_install(int (*fn)(int));
extern void __stdout_hook_install(int (*fn)(int));

static int swo_output_char(int ch)
{
    return ITM_SendChar(ch);
}

static int swo_console_init(struct device *d)
{
	ARG_UNUSED(d);

	__printk_hook_install(swo_output_char);
	__stdout_hook_install(swo_output_char);

	return 0;
}

SYS_INIT(swo_console_init, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
