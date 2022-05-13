#include <drv_types.h>
#ifdef CONFIG_GPIO_WAKEUP
#include <linux/gpio.h>
#endif

#ifdef CONFIG_GPIO_WAKEUP
extern unsigned int oob_irq;
static int irq_flags, wakeup_enable;
#endif

#include <linux/mmc/host.h>
#include <linux/sunxi-gpio.h>
#include <linux/pm_wakeirq.h>

extern void sunxi_mmc_rescan_card(unsigned ids);
extern void sunxi_wlan_set_power(bool on);
extern int sunxi_wlan_get_bus_index(void);
extern int sunxi_wlan_get_oob_irq(int *, int *);
extern int sunxi_wlan_get_oob_irq_flags(void);

int platform_wifi_power_on(void)
{
	int wlan_bus_index = 0;
	sunxi_wlan_set_power(1);
	mdelay(100);

	wlan_bus_index = sunxi_wlan_get_bus_index();
	if(wlan_bus_index < 0){
		printk("get wifi_sdc_id failed\n");
		return -1;
	} else {
		printk("----- %s sdc_id: %d\n", __FUNCTION__, wlan_bus_index);
		sunxi_mmc_rescan_card(wlan_bus_index);
	}
// #ifdef CONFIG_GPIO_WAKEUP
// 	oob_irq = sunxi_wlan_get_oob_irq(&irq_flags, &wakeup_enable);
// #endif
	return 0;
}

void platform_wifi_power_off(void)
{
	int wlan_bus_index = 0;
	sunxi_wlan_set_power(0);
	mdelay(100);
	RTW_INFO("%s: remove card, power off.\n", __FUNCTION__);
	wlan_bus_index = sunxi_wlan_get_bus_index();
	sunxi_mmc_rescan_card(wlan_bus_index);
}