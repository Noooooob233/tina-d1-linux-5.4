#include "ts_core.h"

static DEFINE_MUTEX(i2c_rw_access);

int ts_i2c_read(struct i2c_client *client, char *writebuf, int writelen, char *readbuf, int readlen)
{
	int ret = 0;

	mutex_lock(&i2c_rw_access);

	if (readlen > 0) {
		if (writelen > 0) {
			struct i2c_msg msgs[] = {
				{
					.addr = client->addr,
					.flags = 0,
					.len = writelen,
					.buf = writebuf,
				},
				{
					.addr = client->addr,
					.flags = I2C_M_RD,
					.len = readlen,
					.buf = readbuf,
				},
			};
			ret = i2c_transfer(client->adapter, msgs, 2);
			if (ret < 0) {
				TS_ERROR("[IIC]: i2c_transfer(write) error, ret=%d!!", ret);
			}
		} else {
			struct i2c_msg msgs[] = {
				{
					.addr = client->addr,
					.flags = I2C_M_RD,
					.len = readlen,
					.buf = readbuf,
				},
			};
			ret = i2c_transfer(client->adapter, msgs, 1);
			if (ret < 0) {
				TS_ERROR("[IIC]: i2c_transfer(read) error, ret=%d!!", ret);
			}
		}
	}

	mutex_unlock(&i2c_rw_access);
	return ret;
}

int ts_i2c_write(struct i2c_client *client, char *writebuf, int writelen)
{
	int ret = 0;

	mutex_lock(&i2c_rw_access);
	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
				.addr = client->addr,
				.flags = 0,
				.len = writelen,
				.buf = writebuf,
			},
		};
		ret = i2c_transfer(client->adapter, msgs, 1);
		if (ret < 0) {
			TS_ERROR("%s: i2c_transfer(write) error, ret=%d", __func__, ret);
		}
	}
	mutex_unlock(&i2c_rw_access);

	return ret;
}

int ts_i2c_write_reg(struct i2c_client *client, u8 regaddr, u8 regvalue)
{
	u8 buf[2] = {0};

	buf[0] = regaddr;
	buf[1] = regvalue;
	return ts_i2c_write(client, buf, sizeof(buf));
}

int ts_i2c_read_reg(struct i2c_client *client, u8 regaddr, u8 *regvalue)
{
	return ts_i2c_read(client, &regaddr, 1, regvalue, 1);
}

int ts_i2c_init(void)
{
	TS_FUNC_ENTER();

	TS_FUNC_EXIT();
	return 0;
}

int ts_i2c_exit(void)
{
	TS_FUNC_ENTER();

	TS_FUNC_EXIT();
	return 0;
}

