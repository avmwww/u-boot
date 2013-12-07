/*
 * (C) Copyright 2013
 * Andrey Mitrofanov, <avmwww@gmail.com>
 *
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

/*
 * K5600BG1 Ethernet driver
 */
#include <config.h>

/*
 * Define DEBUG to enable debug() messages in this module
 */
#undef DEBUG

#include <common.h>
#include <malloc.h>
#include <net.h>
#include <miiphy.h>
#include <asm/errno.h>

/*
 * Device name
 */
#define K5600BG1_MAC_NAME			"bg"
#define FPGA_BASE				0x63800000

#define ETH_RX_DESC_NUM				32
#define ETH_TX_DESC_NUM				32

#define K56_MAC_TX_TIMEOUT			2 /* sec */

typedef volatile u32 vu32;

typedef struct
{
	vu32 MAC_CTRL;
	vu32 MinFrame;
	vu32 MaxFrame;
	vu32 CollConfig;
	vu32 IPGTx;
	vu32 MAC_ADDR_T;
	vu32 MAC_ADDR_M;
	vu32 MAC_ADDR_H;
	vu32 HASH0;
	vu32 HASH1;
	vu32 HASH2;
	vu32 HASH3;
	vu32 INT_MSK;
	vu32 INT_SRC;
	vu32 PHY_CTRL;
	vu32 PHY_STAT;
	vu32 RXBF_HEAD;
	vu32 RXBF_TAIL;
	vu32 RESERVED[2];
	vu32 STAT_RX_ALL;
	vu32 STAT_RX_OK;
	vu32 STAT_RX_OVF;
	vu32 STAT_RX_LOST;
	vu32 STAT_TX_ALL;
	vu32 STAT_TX_OK;
	vu32 BASE_RX_BUFF;
	vu32 BASE_TX_BUFF;
	vu32 BASE_RX_DESC;
	vu32 BASE_TX_DESC;
	vu32 BASE_RG;
	vu32 GCTRL;
} EthernetTypeDef;

typedef struct
{
	vu32 CTRL_STAT;
	vu32 LEN;
	vu32 ADDR_H;
	vu32 ADDR_L;
} EthDescTypeDef;

#define ETH_IPG_DEFAULT			96

#define ETH_GCTRL_RST			0x8000
#define ETH_GCTRL_RD_CLR_STAT		0x4000
#define ETH_GCTRL_SPI_RST		0x2000
#define ETH_GCTRL_ASYNC_MODE		0x1000

#define ETH_MAC_BCA_EN			0x0200
#define ETH_MAC_HALFD_EN		0x0004

#define ETH_COLLCONFIG_TRYN(n)	((u16)(n) << 8)
#define ETH_COLLCONFIG_CW(n)	((u16)(n) & 0xFF)

#define ETH_DESC_RDY			0x8000
#define ETH_DESC_WRAP			0x4000
#define ETH_DESC_IRQ_EN			0x2000

#define ETH_PHYC_RST			0x8000
#define ETH_PHYC_EXT_EN			0x4000
#define ETH_PHYC_TXEN			0x2000
#define ETH_PHYC_RXEN			0x1000
#define ETH_PHYC_LINK_PER(n)		(((u16)(n - 1) & 0x003F) << 6)
#define ETH_PHYC_BASE_2			0x0020
#define ETH_PHYC_DIR			0x0010
#define ETH_PHYC_EARLY_DV		0x0008
#define ETH_PHYC_HALFD			0x0004
#define ETH_PHYC_DLB			0x0002
#define ETH_PHYC_LB			0x0001
/*
 * K5600BG1 ETH device
 */
struct k56_eth_dev {
	/*
	 * Standard ethernet device
	 */
	struct eth_device		netdev;
	unsigned int			unit;
	/* Control registers */
	EthernetTypeDef			*reg;
	EthDescTypeDef			*desc_rx;
	unsigned int			desc_rx_num;
	EthDescTypeDef			*desc_tx;
	unsigned int			desc_tx_num;
	vu32				*buf_rx;
	vu32				*buf_tx;
	vu32				*cur_buf_tx;
	u8				rx_buf[1600];
};

#define to_k56_eth(_nd)	container_of(_nd, struct k56_eth_dev, netdev)

static inline void k56_eth_reset(struct k56_eth_dev *mac)
{
	*((vu32 *)(FPGA_BASE + 0x40)) &= ~(1 << mac->unit);
	udelay(1000);
	*((vu32 *) (FPGA_BASE + 0x40)) |= 1 << mac->unit;
	udelay(100000);
}

static inline void k56_eth_hw_setup(struct k56_eth_dev *mac)
{
	vu32	*p;
	int	i;

	k56_eth_reset(mac);

	mac->reg->GCTRL = ETH_GCTRL_ASYNC_MODE | ETH_GCTRL_RD_CLR_STAT;

	mac->reg->MAC_CTRL = ETH_MAC_BCA_EN;//| ETH_MAC_HALFD_EN;
	mac->reg->CollConfig = ETH_COLLCONFIG_TRYN (15) | ETH_COLLCONFIG_CW (64);
	mac->reg->MinFrame = 64;
	mac->reg->MaxFrame = PKTSIZE_ALIGN;
	mac->reg->IPGTx = ETH_IPG_DEFAULT;
	mac->reg->INT_MSK = 0;

	mac->reg->PHY_CTRL = ETH_PHYC_TXEN | ETH_PHYC_RXEN |
			     ETH_PHYC_LINK_PER (12) | ETH_PHYC_DIR; //| ETH_PHYC_HALFD;

	mac->cur_buf_tx = mac->buf_tx;
	mac->desc_tx_num = mac->desc_rx_num = 0;

	/* Clear buffer tx/rx, descriptors tx/rx */
	for (p = mac->buf_rx; p < (vu32 *)mac->reg; p++)
		*p = 0;

	for (i = 0; i < ETH_RX_DESC_NUM; i++) {
		mac->desc_rx[i].CTRL_STAT =
			ETH_DESC_RDY | ETH_DESC_IRQ_EN | ((i < (ETH_RX_DESC_NUM - 1)) ? 0 : ETH_DESC_WRAP);
	}
	for (i = 0; i < ETH_TX_DESC_NUM; i++)
		mac->desc_tx[i].CTRL_STAT = 0;
}

/*
 * Init
 */
static s32 k56_eth_init(struct eth_device *dev, bd_t *bd)
{
	struct k56_eth_dev	*mac = to_k56_eth(dev);

	debug("%s: mac is %02x:%02x:%02x:%02x:%02x:%02x.\n", __func__,
	      dev->enetaddr[0], dev->enetaddr[1],
	      dev->enetaddr[2], dev->enetaddr[3],
	      dev->enetaddr[4], dev->enetaddr[5]);

	mac->reg->MAC_ADDR_H = ((u32) dev->enetaddr[5] << 8)
			    | (u32) dev->enetaddr[4];
	mac->reg->MAC_ADDR_M = ((u32) dev->enetaddr[3] << 8)
			    | (u32) dev->enetaddr[2];
	mac->reg->MAC_ADDR_T = ((u32) dev->enetaddr[1] << 8)
			    | (u32) dev->enetaddr[0];

	return 0;
}

/*
 * Halt MAC
 */
static void k56_eth_halt(struct eth_device *dev)
{
	struct k56_eth_dev	*mac = to_k56_eth(dev);
	vu32			*p;
	int			i;

	debug("eth halt\n");
}

/*
 * Send frame
 */
static s32 k56_eth_send(struct eth_device *dev, volatile void *pkt, s32 len)
{
	struct k56_eth_dev	*mac = to_k56_eth(dev);
	s32			rv;
	ulong			start;
	int			i;
	vu32			*p;
	u16			*buf;
	EthDescTypeDef		*d = &mac->desc_tx[mac->desc_tx_num];

	debug("eth send %d bytes\n", len);
	if (len > PKTSIZE_ALIGN) {
		printf("%s: frame too long (%d).\n", __func__, len);
		rv = -EINVAL;
		goto out;
	}

	if (d->CTRL_STAT & ETH_DESC_RDY) {
		printf("%s: busy.\n", __func__);
		rv = -EBUSY;
		goto out;
	}

	p = mac->cur_buf_tx;
	/* copy packet */
	buf = (u16 *)pkt;
	for (i = 0; i < len && (u32)p < (u32)mac->desc_tx; i += 2)
		*p++ = *buf++;

	if (i < len) {
		for (p = mac->buf_tx; i < len; i += 2)
			*p++ = *buf++;
	}
	d->ADDR_H = 0;
	d->ADDR_L = ((u32)mac->cur_buf_tx - (u32)mac->buf_tx) >> 2;
	d->LEN = len;
	mac->cur_buf_tx = p;

	/* Fill current and go to next descriptor */
	d->CTRL_STAT = ETH_DESC_RDY | ETH_DESC_IRQ_EN | \
			((mac->desc_tx_num < (ETH_TX_DESC_NUM - 1)) ? 0 : ETH_DESC_WRAP);

	mac->desc_tx_num = (mac->desc_tx_num + 1) & (ETH_TX_DESC_NUM - 1);

	/*
	 * Wait until transmit completes
	 */
	rv = -ETIMEDOUT;
	start = get_timer(0);
	while (get_timer(start) < K56_MAC_TX_TIMEOUT * CONFIG_SYS_HZ) {
		if (!(d->CTRL_STAT & ETH_DESC_RDY)) {
			rv = 0;
			break;
		}
		udelay(1);
	}
	if (rv != 0) {
		printf("%s: timeout.\n", __func__);
		goto out;
	}

	/*
	 * Tx done.
	 */
	rv = 0;
out:
	return rv;
}

/*
 * Process received frames (if any)
 */
static s32 k56_eth_recv(struct eth_device *dev)
{
	struct k56_eth_dev	*mac = to_k56_eth(dev);
	int i;
	EthDescTypeDef *d;
	unsigned int pktlen;
	u16 s;
	vu32 *p;
	u16 *buf;

	while (1) {
		d = &mac->desc_rx[mac->desc_rx_num];
		pktlen = (u16)d->LEN;
		s = d->CTRL_STAT;
		/* no packets are available */
		if (s & ETH_DESC_RDY)
			break;

		if (pktlen > sizeof(mac->rx_buf))
			pktlen = sizeof(mac->rx_buf);

		//if (s & ETH_RX_DESC_EF) // corrupted frame

		p = mac->buf_rx + (u16)d->ADDR_L;
		/* copy packet */
		buf = (u16 *)mac->rx_buf;
		for (i = 0; i < pktlen && (u32)p < (u32)mac->desc_rx; i += 2)
			*buf++ = *p++;

		for (p = mac->buf_rx; i < pktlen; i += 2)
			*buf++ = *p++;

		mac->reg->RXBF_HEAD = ((u16)mac->reg->RXBF_HEAD + pktlen / 2) & 0x7ff;

		/*  Fill current and go to next descriptor */
		d->CTRL_STAT = ETH_DESC_RDY | ETH_DESC_IRQ_EN | \
		       ((mac->desc_rx_num < (ETH_RX_DESC_NUM - 1)) ? 0 : ETH_DESC_WRAP);
		mac->desc_rx_num = (mac->desc_rx_num + 1) & (ETH_RX_DESC_NUM - 1);

		/*
		 * Pass frame upper
		 */
		debug("eth recv %d bytes\n", pktlen);
		NetReceive(mac->rx_buf, pktlen);
	}

	return 0;
}

/*
 * Initialize driver
 */
s32 k5600bg1_eth_init(bd_t *bd, int base_addr)
{
	static int		net_dev_id = 0;
	struct k56_eth_dev	*mac;
	struct eth_device	*netdev;
	s32			rv;

	mac = malloc(sizeof(struct k56_eth_dev));
	if (!mac) {
		printf("Error: failed to allocate %dB of memory for k5600bg1\n",
			sizeof(struct k56_eth_dev));
		rv = -ENOMEM;
		goto out;
	}
	memset(mac, 0, sizeof(struct k56_eth_dev));

	netdev = &mac->netdev;

	/*
	 * Map registers
	 */
	netdev->iobase = base_addr;
	mac->buf_rx = (vu32 *)base_addr;
	mac->desc_rx = (EthDescTypeDef *)(base_addr + 0x2000);
	mac->buf_tx = mac->cur_buf_tx = (vu32 *)(base_addr + 0x4000);
	mac->desc_tx = (EthDescTypeDef *)(base_addr + 0x6000);
	mac->reg = (EthernetTypeDef *)(base_addr + 0x7f00);

	mac->unit = net_dev_id++;
	sprintf(netdev->name, "%s%d", K5600BG1_MAC_NAME, mac->unit);
	k56_eth_hw_setup(mac);

	netdev->init = k56_eth_init;
	netdev->halt = k56_eth_halt;
	netdev->send = k56_eth_send;
	netdev->recv = k56_eth_recv;

	rv = eth_register(netdev);
out:
	if (rv != 0 && mac)
		free(mac);

	return rv;
}


