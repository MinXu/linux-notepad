/****************************************************************
 * Project        Library support
 * (c) copyright  2013
 * Company        Min Xu
 *                All rights reserved
 * Secrecy Level  STRICTLY CONFIDENTIAL
 ****************************************************************/
/**
* @file          cmd.c
* @ingroup       testlib
* @author        Min Xu
*
* A TEST COMMAND
*/
#include <linux/module.h>
#include <linux/io.h>
#include <linux/ioport.h>

/****************************************************************
 *Defines
 ****************************************************************/
/*COMMAND PARAM*/
#define	DUMP_ETHERNET_STATISTIC 0
#define IOREMAP_READ						1
#define IOREMAP_WRITE						2
#define IOREMAP_TEST						3
#define CMD_INVALID						  100

/* CPSW_STATS @ CPSW + 0x400 */
#define RXGOODFRAMES                    0x00000400
#define RXBROADCASTFRAMES               0x00000404
#define RXMULTICASTFRAMES               0x00000408
#define RXPAUSEFRAMES                   0x0000040c
#define RXCRCERRORS                     0x00000410
#define RXALIGNCODEERRORS               0x00000414
#define RXOVERSIZEDFRAMES               0x00000418
#define RXJABBERFRAMES                  0x0000041c
#define RXUNDERSIZEDFRAMES              0x00000420
#define RXFRAGMENTS                     0x00000424
#define RXOCTETS                        0x00000430
#define TXGOODFRAMES                    0x00000434
#define TXBROADCASTFRAMES               0x00000438
#define TXMULTICASTFRAMES               0x0000043c
#define TXPAUSEFRAMES                   0x00000440
#define TXDEFERREDFRAMES                0x00000444
#define TXCOLLISIONFRAMES               0x00000448
#define TXSINGLECOLLFRAMES              0x0000044c
#define TXMULTCOLLFRAMES                0x00000450
#define TXEXCESSIVECOLLISIONS           0x00000454
#define TXLATECOLLISION                 0x00000458
#define TXUNDERRUN                      0x0000045c
#define TXCARRIERSENSEERRORS            0x00000460
#define TXOCTETS                        0x00000464
#define OCTETFRAMES64                   0x00000468
#define OCTETFRAMES65TO127              0x0000046c
#define OCTETFRAMES128TO255             0x00000470
#define OCTETFRAMES256TO511             0x00000474
#define OCTETFRAMES512TO1023            0x00000478
#define OCTETFRAMES1024TUP              0x0000047c
#define NETOCTETS                       0x00000480
#define RXSOFOVERRUNS                   0x00000484
#define RXMOFOVERRUNS                   0x00000488
#define RXDMAOVERRUNS                   0x0000048c

/****************************************************************
 *Globle Datas
 ****************************************************************/
static int cmd = CMD_INVALID;//cmd input from command prompt
static volatile void __iomem *base=NULL;//ioremap base vitrual address
static volatile void __iomem *PrcmBase=NULL;//prcm module base address
static volatile void __iomem *CtrlBase=NULL;//ctrl module base address

/*paramter input*/
module_param(cmd,int,S_IRUGO);

/**
 * a print switch port rx statistics.
 * @param[in] regs     register base address.
 */
void print_rx_stats(u8 *regs)
{
   printk("RX:\n");
   printk("RXGood Frames: %d\n", *(volatile u32 *)(regs + RXGOODFRAMES));
   printk("RX Broadcast Frames: %d\n", *(volatile u32 *)(regs + RXBROADCASTFRAMES));
   printk("RX Multicast Frames: %d\n", *(volatile u32 *)(regs + RXMULTICASTFRAMES));
   printk("RX Pause Frames: %d\n", *(volatile u32 *)(regs + RXPAUSEFRAMES));
   printk("RX CRC Errors: %d\n", *(volatile u32 *)(regs + RXCRCERRORS));
   printk("RX Over size Frames: %d\n", *(volatile u32 *)(regs + RXOVERSIZEDFRAMES));
   printk("RX Jabbers: %d\n", *(volatile u32 *)(regs + RXJABBERFRAMES));
   printk("RX Undersize Frames: %d\n", *(volatile u32 *)(regs + RXUNDERSIZEDFRAMES));
   printk("RX Fragments: %d\n", *(volatile u32 *)(regs + RXFRAGMENTS));
   printk("RX Octets: %d\n", *(volatile u32 *)(regs + RXOCTETS));
   printk("RX SOF Overruns: %d\n", *(volatile u32 *)(regs + RXSOFOVERRUNS));
   printk("RX Middle of Frame Overrun: %d\n", *(volatile u32 *)(regs + RXMOFOVERRUNS));
   printk("RX DMA Overrun: %d\n", *(volatile u32 *)(regs + RXDMAOVERRUNS));
}

/**
 * a print switch port tx statistics.
 * @param[in] regs     register base address.
 */
void print_tx_stats(u8 *regs)
{
   printk("TX:\n");
   printk("TXGood Frames: %d\n", *(volatile u32 *)(regs + TXGOODFRAMES));
   printk("TXBroadcast Frames: %d\n", *(volatile u32 *)(regs + TXBROADCASTFRAMES));
   printk("TXMulticast Frames: %d\n", *(volatile u32 *)(regs + TXMULTICASTFRAMES));
   printk("TXPause Frames: %d\n", *(volatile u32 *)(regs + TXPAUSEFRAMES));
   printk("TXDeferred Frames: %d\n", *(volatile u32 *)(regs + TXDEFERREDFRAMES));
   printk("TXCollision Frames: %d\n", *(volatile u32 *)(regs + TXCOLLISIONFRAMES));
   printk("TXSingle Col Frames: %d\n", *(volatile u32 *)(regs + TXSINGLECOLLFRAMES));
   printk("TXMulti Col Frames: %d\n", *(volatile u32 *)(regs + TXMULTCOLLFRAMES));
   printk("TXExcessive Collisions: %d\n", *(volatile u32 *)(regs + TXEXCESSIVECOLLISIONS));
   printk("TXLate Collisions: %d\n", *(volatile u32 *)(regs + TXLATECOLLISION));
   printk("TXUnderruns: %d\n", *(volatile u32 *)(regs + TXUNDERRUN));
   printk("TXCarrier Sense Errors: %d\n", *(volatile u32 *)(regs + TXCARRIERSENSEERRORS));
   printk("TX Octets: %d\n", *(volatile u32 *)(regs + TXOCTETS));
}

/**
 * a A8 chip ctrl register (gmii selcet) status print.
 * @param[in] regs     register base address.
 */
static void print_ctrl_stats(u8 *regs)
{
   printk("GMII SEL REG %x\n",*(volatile u32 *)(regs + 0x650));
}

/**
 * a A8 chip switch register (statistics enable|ale|mac control) status print.
 * @param[in] regs     register base address.
 */
static void print_cpsw_stats(u8 *regs)
{
   printk("statistics enable %x\n",*(volatile u32 *)(regs + 0x0c));
   printk("ale control   %x\n",*(volatile u32 *)(regs + 0x608));
   printk("MAC CONTROL    %x\n",*(volatile u32 *)(regs + 0x704));
}

/**
 * a inline delay function.
 * @param[in] loops     cycle numbers.
 */
static inline void delay1(unsigned long loops)
{
	int i;
	while((i--)>=0);
}

/**
 * a ioremap function,print registers firstly, set switch module clock and pll,print registers again.
 */
static void IORemap(void)
{
#define PRCM_BASE			                             0x48180000
#define CTRL_BASE			                             0x48140000
#define CM_ETHERNET_CLKSTCTRL		                   0x1404
#define CM_ALWON_ETHERNET_0_CLKCTRL                0x15D4
#define CM_ALWON_ETHERNET_1_CLKCTRL                0x15D8
#define CM_ALWON2_SATA_CLKCTRL                     0x0560

		PrcmBase = ioremap(PRCM_BASE, 0x2000);
		
		printk("before set the prcm register\n");
		printk("%s(): prcm virtual address = %p\n",__FUNCTION__,PrcmBase);
		printk("%s(): CM_ETHERNET_CLKSTCTRL  = %x\n",__FUNCTION__,*(volatile u32 *)(PrcmBase+0x1404));
		printk("%s(): CM_ALWON_ETHERNET_0_CLKCTRL  = %x\n",__FUNCTION__,*(volatile u32 *)(PrcmBase+0x15D4));
		printk("%s(): CM_ALWON_ETHERNET_1_CLKCTRL  = %x\n",__FUNCTION__,*(volatile u32 *)(PrcmBase+0x15D8));
		printk("%s(): CM_ALWON2_SATA_CLKCTRL  = %x\n",__FUNCTION__,*(volatile u32 *)(PrcmBase+0x0560));
		
		*(volatile u32 *)(PrcmBase+0x1404) = 2;
		*(volatile u32 *)(PrcmBase+0x15D4) = 2;
		*(volatile u32 *)(PrcmBase+0x15D8) = 2;
		*(volatile u32 *)(PrcmBase+0x0560) = 2;
		
		CtrlBase = ioremap(CTRL_BASE, 0x2000);
		*(volatile u32 *)(CtrlBase+0x724) = 0xC12C003C;
		*(volatile u32 *)(CtrlBase+0x72C) = 0x004008E0;delay1(0xFFFF);
		*(volatile u32 *)(CtrlBase+0x720) = 0x80000004;delay1(0xFFFF);
		*(volatile u32 *)(CtrlBase+0x720) = 0x80000014;delay1(0xFFFF);
		*(volatile u32 *)(CtrlBase+0x720) = 0x80000016;delay1(0xFFFF);
		*(volatile u32 *)(CtrlBase+0x720) = 0xC0000017;delay1(0xFFFF);
		while(((*(volatile u32 *)(CtrlBase + 0x734) & 0x01) == 0x0));

		printk("after set the prcm register\n");
		printk("%s(): CM_ETHERNET_CLKSTCTRL  = %x\n",__FUNCTION__,*(volatile u32 *)(PrcmBase+0x1404));
		printk("%s(): CM_ALWON_ETHERNET_0_CLKCTRL  = %x\n",__FUNCTION__,*(volatile u32 *)(PrcmBase+0x15D4));
		printk("%s(): CM_ALWON_ETHERNET_1_CLKCTRL  = %x\n",__FUNCTION__,*(volatile u32 *)(PrcmBase+0x15D8));
		printk("%s(): CM_ALWON2_SATA_CLKCTRL  = %x\n",__FUNCTION__,*(volatile u32 *)(PrcmBase+0x0560));

		iounmap(PrcmBase);
		iounmap(CtrlBase);
}

/**
 * test ioremap initialization,invoke inputed paramters,dump switch statistics,read switch register,write switch register,set switch clock and prcm.
 * @return 0.
 */
static int __init test_ioremap(void)
{
	int address;
	int size;
	volatile void __iomem * cpswbase;
	volatile void __iomem * cmrbase;	

	switch(cmd)
	{
	case DUMP_ETHERNET_STATISTIC:
		{
			address = 0x4a100000;size = 0x800;
//			volatile void __iomem * cpswbase = ioremap(address, size);
			cpswbase = ioremap(address, size);
			address = 0x48140000;size = 0x100;
//			volatile void __iomem * cmrbase = ioremap(address, size);
			cmrbase = ioremap(address, size);
			print_rx_stats((u8 *)cpswbase);
			print_tx_stats((u8 *)cpswbase);
			print_cpsw_stats((u8 *)cpswbase);
			print_ctrl_stats((u8 *)cmrbase);
			iounmap(cpswbase);
			iounmap(cmrbase);
		}
	break;
	case IOREMAP_READ:
		{
			address = 0x4a100000;size = 0x800;
      /*已经被cpsw 网络模块申请，无法再次申请，不过可以多次ioremap*/
      //if (!request_mem_region(address,size, "cpsw")) {
      //      printk("#%s(): request_mem_region error\n",__FUNCTION__);
      //      return -1;
      //}
      base = ioremap(address, size);
      printk("#%s(): test Read.......................\n",__FUNCTION__);
      printk("#%s(): Read register[%x] = %x \n",__FUNCTION__,address+0x03C,*(volatile u32 *)(base+0x03C));
      if(base != NULL)
      {
	      iounmap(base);
      }
      release_mem_region(address,size);
		}
	break;
	case IOREMAP_WRITE:
		{
			address = 0x4a100000;size = 0x800;
      /*已经被cpsw 网络模块申请，无法再次申请，不过可以多次ioremap*/
      //if (!request_mem_region(address,size, "cpsw")) {
      //      printk("#%s(): request_mem_region error\n",__FUNCTION__);
      //      return -1;
      //}
      base = ioremap(address, size);
      printk("#%s(): test Write.......................\n",__FUNCTION__);
      *(volatile u32 *)(base+0x03C) = 0x11223344;
      printk("#%s(): test Write...............complete\n",__FUNCTION__);     
      if(base != NULL)
      {
	      iounmap(base);
      }
      release_mem_region(address,size);
		}
	break;

	case IOREMAP_TEST:
		{
			IORemap();
		}
	break;
	default:
		printk("#%s(): test empty items.......................\n",__FUNCTION__);
	}
      return 0;
}

/**
 * .test ioremap destory
 */
static void __exit test_ioremap_exit(void)
{
      return;
}


module_init(test_ioremap);
module_exit(test_ioremap_exit);

MODULE_LICENSE("BSD|GPL");
MODULE_AUTHOR("Min <min.xu@harman.com>");
MODULE_DESCRIPTION("Generic configurable test driver");
