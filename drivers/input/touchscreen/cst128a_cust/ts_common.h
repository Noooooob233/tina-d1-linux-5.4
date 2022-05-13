#ifndef __LINUX_CST128A_CUST_COMMON_H__
#define __LINUX_CST128A_CUST_COMMON_H__

#include "ts_config.h"

#define ENABLE                              1
#define DISABLE                             0

/* i2c communication*/
int ts_i2c_write_reg(struct i2c_client *client, u8 regaddr, u8 regvalue);
int ts_i2c_read_reg(struct i2c_client *client, u8 regaddr, u8 *regvalue);
int ts_i2c_read(struct i2c_client *client, char *writebuf, int writelen, char *readbuf, int readlen);
int ts_i2c_write(struct i2c_client *client, char *writebuf, int writelen);
int ts_i2c_init(void);
int ts_i2c_exit(void);

/* Other */
int ts_reset_proc(int hdelayms);
void ts_irq_disable(void);
void ts_irq_enable(void);

/*****************************************************************************
* DEBUG function define here
*****************************************************************************/
#if CST128A_DEBUG_EN
#define TS_DEBUG_LEVEL     1

#if (TS_DEBUG_LEVEL == 2)
#define TS_DEBUG(fmt, args...) printk(KERN_ERR "[TS][%s]"fmt"\n", __func__, ##args)
#else
#define TS_DEBUG(fmt, args...) printk(KERN_ERR "[TS]"fmt"\n", ##args)
#endif

#define TS_FUNC_ENTER() printk(KERN_ERR "[TS]%s: Enter\n", __func__)
#define TS_FUNC_EXIT()  printk(KERN_ERR "[TS]%s: Exit(%d)\n", __func__, __LINE__)
#else
#define TS_DEBUG(fmt, args...)
#define TS_FUNC_ENTER()
#define TS_FUNC_EXIT()
#endif

#define TS_INFO(fmt, args...) do { \
		printk(KERN_ERR "[TS][Info]"fmt"\n", ##args); \
    }  while (0)

#define TS_ERROR(fmt, args...)  do { \
		printk(KERN_ERR "[TS][Error]"fmt"\n", ##args); \
    }  while (0)


#endif
