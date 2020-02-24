/******************************************************************************
**
** FILE NAME	: lgm_pp_hal.c
** PROJECT	: LGM
** MODULES	: PPA PPv4 HAL	
**
** DATE		: 29 Oct 2018
** AUTHOR	: Kamal Eradath
** DESCRIPTION	: PPv4 hardware abstraction layer
** COPYRIGHT	:	Copyright (c) 2014
**			Intel Corporation.
**
**	 For licensing information, see the file 'LICENSE' in the root folder of
**	 this software module.
**
** HISTORY
** $Date		$Author		 	$Comment
** 29 Oct 2018		Kamal Eradath		Initial Version
*******************************************************************************/
/*
 * ####################################
 *		Head File
 * ####################################
 */
/*
 *	Common Head File
 */
#include <linux/version.h>
#include <generated/autoconf.h>

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/netdevice.h>
#include <linux/ip.h>
#include <net/checksum.h>
#include <linux/clk.h>
#include <net/ip_tunnels.h>
#include <linux/if_arp.h>
#include <linux/in.h>
#include <linux/inet.h>
#include <asm/uaccess.h>
#include <net/ipv6.h>
#include <net/ip6_tunnel.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/hrtimer.h>
#include <linux/workqueue.h>

/*
 *	Chip Specific Head File
 */
#include <linux/pp_api.h>
#include <linux/learning_layer_api.h>
#include <net/datapath_api.h>
#include <net/intel_np_lro.h>
#include <net/datapath_api_qos.h>
#include <net/ppa/ppa_api.h>
#include <net/ppa/ppa_hal_api.h>
#include <net/ppa/qos_hal_api.h>
#include "../../ppa_api/ppa_api_netif.h"
#include "../../ppa_api/ppa_api_session.h"
#include "lgm_pp_hal.h"

/*
 * ####################################
 *			Version No.
 * ####################################
 */
#define VER_FAMILY	0x10
#define VER_DRTYPE	0x04 /* 4: Stack/System Adaption Layer driver*/
#define VER_INTERFACE	0x1  /*	bit 0: MII 1 */
#define VER_ACCMODE	0x11 /*	bit 0: Routing &Bridging */

#define VER_MAJOR	1
#define VER_MID		0
#define VER_MINOR	0

#define SESSION_RETRY_MAX	 8

/*
 *	Compilation Switch
 */
/*!
	\brief Turn on/off debugging message and disable inline optimization.
 */
#define ENABLE_DEBUG			1
/*!
	\brief Turn on/off ASSERT feature, print message while condition is not fulfilled.
 */
#define ENABLE_ASSERT			1
/*@}*/

#if defined(ENABLE_DEBUG) && ENABLE_DEBUG
	#define ENABLE_DEBUG_PRINT	1
	#define DISABLE_INLINE		1
#endif

#if defined(DISABLE_INLINE) && DISABLE_INLINE
	#define INLINE
#else
	#define INLINE			inline
#endif

#if defined(ENABLE_DEBUG_PRINT) && ENABLE_DEBUG_PRINT
	#undef	dbg
	static unsigned int lgm_pp_hal_dbg_enable = 0;
	#define dbg(format, arg...) do { if ( lgm_pp_hal_dbg_enable ) printk(KERN_WARNING ":%d:%s: " format "\n", __LINE__, __FUNCTION__, ##arg); } while ( 0 )
#else
	#if !defined(dbg)
	#define dbg(format, arg...)
	#endif
#endif

#if defined(ENABLE_ASSERT) && ENABLE_ASSERT
	#define ASSERT(cond, format, arg...) do { if ( !(cond) ) printk(KERN_ERR __FILE__ ":%d:%s: " format "\n", __LINE__, __FUNCTION__, ##arg); } while ( 0 )
#else
	#define ASSERT(cond, format, arg...)
#endif

#ifndef NUM_ENTITY
#define NUM_ENTITY(x)	(sizeof(x) / sizeof(*(x)))
#endif

#define PPA_API_PROC 		1

#define	DEFAULT_POLLING_TIME	4 /* 4 seconds */
#define DEFAULT_DB_SLICE		(1024U) /*number of db entries polled per iteration*/
#define DEFAULT_INACT_ARRAY_SIZE	(8192U)  /*8K*/

#define ETH_HLEN		14	/* Total octets in header.	 */
#define IPV6_HDR_LEN		40

/*
 * ####################################
 *		Declaration
 * ####################################
 */

#if defined(PPA_API_PROC)
static int proc_read_ppv4_rtstats_seq_open(struct inode *, struct file *);
static ssize_t proc_clear_ppv4_rtstats(struct file *, const char __user *, size_t , loff_t *);

static int proc_read_ppv4_accel_seq_open(struct inode *, struct file *);
static ssize_t proc_set_ppv4_accel(struct file *, const char __user *, size_t , loff_t *);

static int proc_read_ppv4_debug_seq_open(struct inode *, struct file *);
static ssize_t proc_set_ppv4_debug(struct file *, const char __user *, size_t , loff_t *);
#endif /*defined(PPA_API_PROC)*/

/*Each client will have one session in pp*/
typedef struct mc_db_nod {
	bool		used;	/*index is in use*/
	void 		*node;	/*mc session node pointer */
	struct pp_stats	stats;	/*statistics per group*/
	uint32_t 	session_id[MAX_MC_CLIENT_PER_GRP]; /*session id of all the dst sessions */
} MC_DB_NODE;

/*Each unicast entry in pp will need following information kept in hal*/
typedef struct uc_db_node{
	bool			used;	/*index is in use*/
	struct pp_stats		stats;	/*statistics*/
	void 			*node; 	/*unicast session node pointer*/
} PP_HAL_DB_NODE;

struct mc_dev_priv{
	struct module	*owner;
};
#if IS_ENABLED(CONFIG_LGM_TOE)
extern int32_t ppa_bypass_lro(PPA_SESSION *p_session);
#endif /*IS_ENABLED(CONFIG_LGM_TOE)*/
/*
 * ####################################
 *	 Global Variable
 * ####################################
 */

/* gswip port bitmap map */
static uint32_t g_port_map = 0x8FFF; /*{ 1 0 0 0 1 1 1 1 1 1 1 1 1 1 1 1 }*/

static uint32_t g_us_accel_enabled = 1;
static uint32_t g_ds_accel_enabled = 1; 
static uint32_t g_max_hw_sessions = MAX_UC_SESSION_ENTRIES;

static uint16_t g_gswip_qid;
static struct nf_node g_mcast_nf;
static struct nf_node g_frag_nf;

/*multicast group DB*/
static MC_DB_NODE mc_db[MAX_MC_GROUP_ENTRIES]={0};
/*DB to maintain pp status in pp hal */
static PP_HAL_DB_NODE	*pp_hal_db=NULL;
/*Session delete callback */
static void (*del_routing_session_cb)(void *p_item)=NULL;

static	spinlock_t		g_hal_db_lock;
static	spinlock_t		g_hal_mc_db_lock;
static	struct hrtimer		g_gc_timer;
static struct workqueue_struct	*g_workq=0;
static  uint32_t		g_db_index = 0;
static  uint32_t		g_iter_cnt = 0;

/* Global ppv4 hal counters */
static uint64_t nsess_add=0;
static uint64_t nsess_del=0;
static uint64_t nsess_del_fail=0;
static uint64_t nsess_add_succ=0;
static uint64_t nsess_add_fail_rt_tbl_full=0;
static uint64_t nsess_add_fail_coll_full=0;
static uint32_t nsess_add_fail_oth=0;

#if defined(PPA_API_PROC)
static struct dentry *g_ppa_ppv4hal_debugfs_dir = NULL;
static int g_ppa_ppv4hal_debugfs_flag = 0;

static struct file_operations g_proc_file_ppv4_rtstats_seq_fops = {
	.owner		= THIS_MODULE,
	.open		= proc_read_ppv4_rtstats_seq_open,
	.read		= seq_read,
	.write		= proc_clear_ppv4_rtstats,
	.llseek	 	= seq_lseek,
	.release	= seq_release,
};

static struct file_operations g_proc_file_ppv4_accel_seq_fops = {
	.owner		= THIS_MODULE,
	.open		= proc_read_ppv4_accel_seq_open,
	.read		= seq_read,
	.write		= proc_set_ppv4_accel,
	.llseek	 	= seq_lseek,
	.release	= seq_release,
};

static struct file_operations g_proc_file_ppv4_debug_seq_fops = {
	.owner		= THIS_MODULE,
	.open		= proc_read_ppv4_debug_seq_open,
	.read		= seq_read,
	.write		= proc_set_ppv4_debug,
	.llseek	 	= seq_lseek,
	.release	= seq_release,
};
#endif /*defined(PPA_API_PROC)*/

/*
 * ####################################
 *		Extern Variable
 * ####################################
 */
#if defined(PPA_API_PROC)
extern int g_ppa_debugfs_flag;
extern struct dentry *g_ppa_debugfs_dir;
#endif /*defined(PPA_API_PROC)*/

/*
 * ####################################
 *		Extern Function
 * ####################################
 */

extern uint32_t ppa_drv_generic_hal_register(uint32_t hal_id, ppa_generic_hook_t generic_hook);
extern void ppa_drv_generic_hal_deregister(uint32_t hal_id);

extern uint32_t ppa_drv_register_cap(PPA_API_CAPS cap, uint8_t wt, PPA_HAL_ID hal_id);
extern uint32_t ppa_drv_deregister_cap(PPA_API_CAPS cap, PPA_HAL_ID hal_id);

/*
 * ####################################
 *			Local Function
 * ####################################
 */
static inline uint32_t is_valid_session(uint32_t session_id)
{
	if(session_id && (session_id <= g_max_hw_sessions)) {
		return 1;
	}
	return 0;
}

static inline uint32_t is_lansession(uint32_t flags)
{
	return ((flags & SESSION_LAN_ENTRY) ? 1 : 0);
}

static int mcdev_xmit(struct sk_buff *skb, struct net_device *netdev)
{
	consume_skb(skb);
	return NETDEV_TX_OK;
}

static struct net_device_ops mcdev_ops = {
	.ndo_start_xmit	= mcdev_xmit,

};

static void mcdev_setup(struct net_device *dev)
{
	ether_setup(dev);/*     assign some members */
	return;
}

/*Calback invoked by dp when packets are received on g_mc_gpid */
int32_t mc_dp_rx_handler(struct net_device *rxif, struct net_device *txif,
        struct sk_buff *skb, int32_t len)
{
	struct pp_desc *ppdesc = NULL;
	struct packet_type ptype = {0};
	int16_t groupid = -1, dev_idx = -1;
	struct mc_session_node *p_item = NULL;
	PPA_NETIF *tx_netif = NULL;

	skb_reset_mac_header(skb);
	skb_set_network_header(skb, ETH_HLEN);

	/*1. Call the learning ingress hook for the skb*/
	ETH_P_CUSTOM_HOOK_TYPE_SET(ptype.type, HOOK_TYPE_RX);
	learning_packet_handler(skb, NULL, &ptype, NULL);

	/*2. Read the ud parameters from the descriptor*/
	/*   Based on the mc groupid and dev_idx we need to find the eg_netdev from the mc_db*/
	ppdesc = pp_pkt_desc_get(skb);
	if (ppdesc) {
		groupid = ppdesc->ps & MC_GRP_MASK; /*BITS: [8:0] */
		dev_idx = ppdesc->ps & MC_DST_MASK; /* BITS: [18:21] */
	}

	spin_lock_bh(&g_hal_mc_db_lock);
	if ((groupid >= 0) && (groupid < MAX_MC_GROUP_ENTRIES) && mc_db[groupid].used) {
		p_item = (struct mc_session_node *) mc_db[groupid].node;
		spin_unlock_bh(&g_hal_mc_db_lock);
		if (p_item && (dev_idx >= 0) && (dev_idx < MAX_MC_CLIENT_PER_GRP)) {
			tx_netif = p_item->grp.txif[dev_idx].netif;
		}
	} else {
		spin_unlock_bh(&g_hal_mc_db_lock);
	}

	/*3. set skb->dev as the eg_netdev and call dev_queue_xmit*/
	/*   learning driver's egress hook will be invoked automatically*/
	if (tx_netif) {
		skb->dev = tx_netif;
		dev_queue_xmit(skb);
	} else {
		consume_skb(skb);
	}

	return PPA_SUCCESS;
}

static dp_cb_t mc_dp_cb = {
        .rx_fn = mc_dp_rx_handler,
};

static inline int32_t init_mc_nf(void)
{
	int32_t ret = PPA_SUCCESS;
	struct pp_nf_info nf_info = {0};
	struct dp_spl_cfg dp_con = {0};
	struct dp_qos_q_logic q_logic = {0};
	struct pp_port_cfg pcfg = {0};
	struct mc_dev_priv *priv = NULL;
	char ifname[IFNAMSIZ];

	dbg("%s %d\n",__FUNCTION__,__LINE__);

	ppa_memset(&g_mcast_nf, 0, sizeof(g_mcast_nf));
	/*Allocate netdevice */
	snprintf(ifname, sizeof(ifname), "mcast_dev%d", 0);

	g_mcast_nf.dev = alloc_netdev(sizeof(struct mc_dev_priv), ifname, NET_NAME_UNKNOWN, mcdev_setup);
	if(!g_mcast_nf.dev) {
		dbg("alloc_netdev failed for %s\n",ifname);
		return PPA_FAILURE;
	}

	priv = netdev_priv(g_mcast_nf.dev);
	priv->owner = THIS_MODULE;

	g_mcast_nf.dev->netdev_ops = &mcdev_ops;
	/*Register netdevice*/
	if (register_netdev(g_mcast_nf.dev)) {
		free_netdev(g_mcast_nf.dev);
                g_mcast_nf.dev = NULL;
		dbg("register device \"%s\" failed\n", ifname);
		return PPA_FAILURE;
	}
	/*call the dp to allocate special connection */
	/*******************************************************/

	/*dp connection for multicast uC */
	dp_con.flag = 0;/*DP_F_REGISTER;*/
	dp_con.type = DP_SPL_PP_NF;
	dp_con.f_subif = 1;
        dp_con.f_gpid = 1;

	/*assign the netdevice */
	dp_con.dev = g_mcast_nf.dev;

	/*callback to be invoked by dp when packet is received for this GPID*/
	dp_con.dp_cb = &mc_dp_cb;

	if ((ret = dp_spl_conn(0, &dp_con))) {
		dbg("Regsiter spl conn for mc failed\n");
		return PPA_FAILURE;
	}

	/*enable rp_rx*/
	if ((ret = dp_rx_enable(g_mcast_nf.dev, g_mcast_nf.dev->name, 1))) {
		dbg("Enable rx_fn for mc failed\n");
		return PPA_FAILURE;
	}

	/*******************************************************/
	dbg("%s %d dp_spl_conn success dp_con.gpid=%d,  dp_con.subif=%d dp_con.spl_id=%d, dp_con.egp[0].qid=%d, dp_con.egp[1].qid=%d\n",
		__FUNCTION__, __LINE__,dp_con.gpid, dp_con.subif, dp_con.spl_id, dp_con.egp[0].qid, dp_con.egp[1].qid);

	/*Store the gpid and uc_id*/
	g_mcast_nf.uc_id = dp_con.spl_id;
	g_mcast_nf.gpid = dp_con.gpid;
	g_mcast_nf.subif = (dp_con.subif >> 9 ) & 0X0F; /*remove 9 bits of mc groupid*/

	/*Get the port settings and change the ingress classification parameters*/
	if((ret=pp_port_get(g_mcast_nf.gpid, &pcfg))) {
		dbg("pp_port_get failed in %s %d\n", __FUNCTION__, __LINE__);
		return PPA_FAILURE;
	}

	/*Fields to copy from STW*/
	pcfg.rx.cls.n_flds = 2;
        pcfg.rx.cls.cp[0].stw_off = 0;       /* multicast group index in ps0 form bit 0*/
        pcfg.rx.cls.cp[0].copy_size = 9;     /* 9 bits field */
        pcfg.rx.cls.cp[1].stw_off = 18;       /* multicast dst bitmap in ps0 from bit 18 */
        pcfg.rx.cls.cp[1].copy_size = 4;     /* 4 bits field */

	/*Set the modified port configuration */
	if((ret=pp_port_update(g_mcast_nf.gpid, &pcfg))) {
		dbg("pp_port_update failed in %s %d\n", __FUNCTION__, __LINE__);
		return PPA_FAILURE;
	}

	/*Egress port qid*/;
	q_logic.q_id = dp_con.egp[0].qid;

	/* physical to logical qid */
	if (dp_qos_get_q_logic(&q_logic, 0) == DP_FAILURE) {
		dbg("%s:%d ERROR Failed to Logical Queue Id\n", __func__, __LINE__);
		return PPA_FAILURE;
	}

	/*Store the logical qid */
	g_mcast_nf.qid = q_logic.q_logic_id;

	if(!g_gswip_qid) {
		/* Qid to be used from NF to send the backet back to Gswip */
		ppa_memset(&q_logic, 0, sizeof(q_logic));
		q_logic.q_id = dp_con.egp[1].qid;
		/* physical to logical qid */
		if (dp_qos_get_q_logic(&q_logic, 0) == DP_FAILURE) {
			dbg("%s:%d ERROR Failed to Logical Queue Id\n", __func__, __LINE__);
			return PPA_FAILURE;
		}
		g_gswip_qid = q_logic.q_logic_id;
	}

	nf_info.cycl2_q = g_gswip_qid;
	nf_info.pid = g_mcast_nf.gpid;
	nf_info.q = g_mcast_nf.qid;

	dbg("%s %d calling pp_nf_set gpid=%d qid=%d cycl2_qid=%d\n", __FUNCTION__, __LINE__, nf_info.pid, nf_info.q, nf_info.cycl2_q);

	/*Setup the uC path */
	if ((ret = pp_nf_set(PP_NF_MULTICAST, &nf_info))) {
		dbg("pp_nf_set failed for PP_NF_MULTICAST\n");
	}

	dbg("%s %d g_mcast_nf.gpid=%d, dp_con.subif=%d, dp_con.spl_id=%d, g_mcast_nf.qid=%d, g_gswip_qid=%d\n",
		__FUNCTION__, __LINE__, g_mcast_nf.gpid, dp_con.subif, dp_con.spl_id, g_mcast_nf.qid, g_gswip_qid);

	return ret;
}

static int init_frag_nf(void)
{
	int32_t ret = PPA_SUCCESS;
	struct pp_nf_info nf_info = {0};
	struct dp_spl_cfg dp_con = {0};
	struct dp_qos_q_logic q_logic = {0};

	/*TBD: remove next line when fragmentor in implemented in pp*/
	ppa_memset(&g_frag_nf, 0, sizeof(g_frag_nf));

	/*dp connection for fragmenter uC */
	dp_con.flag = 0;/*DP_F_REGISTER;*/
	dp_con.type = DP_SPL_PP_NF;
	dp_con.f_subif = 1;
        dp_con.f_gpid = 0;

	if ((ret = dp_spl_conn(0, &dp_con))) {
		dbg("Regsiter spl conn for mc failed\n");
		return PPA_FAILURE;
	}
	g_frag_nf.uc_id = dp_con.spl_id;

	dbg("%s %d dp_spl_conn success dp_con.gpid=%d, dp_con.spl_id=%d, dp_con.egp[0].qid=%d, dp_con.igp[0].egp.qid=%d\n",
		__FUNCTION__, __LINE__,dp_con.gpid, dp_con.spl_id, dp_con.egp[0].qid, dp_con.egp[1].qid);
	/*******************************************************/

	g_frag_nf.gpid = dp_con.gpid;

	/*Egress port qid*/
	q_logic.q_id = dp_con.egp[0].qid;

	/* physical to logical qid */
	if (dp_qos_get_q_logic(&q_logic, 0) == DP_FAILURE) {
		dbg("%s:%d ERROR Failed to Logical Queue Id\n", __func__, __LINE__);
		return PPA_FAILURE;
	}

	nf_info.pid = g_frag_nf.gpid;
	nf_info.q = g_frag_nf.qid = q_logic.q_logic_id;

	if(!g_gswip_qid) {
		/* Qid to be used from NF to send the backet back to Gswip */
		ppa_memset(&q_logic, 0, sizeof(q_logic));
		q_logic.q_id = dp_con.egp[1].qid;
		/* physical to logical qid */
		if (dp_qos_get_q_logic(&q_logic, 0) == DP_FAILURE) {
			dbg("%s:%d ERROR Failed to Logical Queue Id\n", __func__, __LINE__);
			return PPA_FAILURE;
		}
		g_gswip_qid = q_logic.q_logic_id;
	}

	nf_info.cycl2_q = g_gswip_qid;
	dbg("%s %d calling pp_nf_set gpid=%d qid=%d cycl2_qid=%d\n", __FUNCTION__, __LINE__, nf_info.pid, nf_info.q, nf_info.cycl2_q);

	/*Setup the uC path */
	if ((ret = pp_nf_set(PP_NF_FRAGMENTER, &nf_info))) {
		dbg("pp_nf_set failed for PP_NF_FRAGMENTER\n");
	}

	printk(KERN_INFO"%s success\n", __FUNCTION__);
	return ret;
}

static inline void uninit_mc_nf(void)
{
	/*******************************************************/
	struct dp_spl_cfg dp_con={0};
	dp_con.flag = DP_F_DEREGISTER;
	dp_con.type = DP_SPL_PP_NF;
	dp_con.spl_id = g_mcast_nf.uc_id;

	if (dp_spl_conn(0, &dp_con)) {
		dbg("Deregister of dp spl conn for mc failed\n");
	}
	/*******************************************************/
}

static inline void uninit_frag_nf(void)
{
	/*******************************************************/
	struct dp_spl_cfg dp_con={0};
	dp_con.flag = DP_F_DEREGISTER;
	dp_con.type = DP_SPL_PP_NF;
	dp_con.spl_id = g_frag_nf.uc_id;

	if (dp_spl_conn(0, &dp_con)) {
		dbg("Deregister of dp spl conn for mc failed\n");
	}
	/*******************************************************/
}

#if defined(PPA_API_PROC)
static int proc_read_ppv4_rtstats(struct seq_file *seq, void *v)
{
	if (!capable(CAP_SYSLOG)) {
		printk ("Read Permission denied");
		return 0;
	}

	seq_printf(seq,	"=====================================================================\n");
	seq_printf(seq,	"Total Number of Routing session entrys			: %lu\n", nsess_add_succ - nsess_del + nsess_del_fail);
	seq_printf(seq,	"Total Number of Routing session add requests		: %lu\n", nsess_add);
	seq_printf(seq,	"Total Number of Routing session delete			: %lu\n", nsess_del);
	seq_printf(seq,	"Total Number of Routing session delete fail		: %lu\n", nsess_del_fail);
	seq_printf(seq,	"Total Number of Routing session add fails		: %lu\n", nsess_add_fail_rt_tbl_full + nsess_add_fail_coll_full + nsess_add_fail_oth);
	seq_printf(seq,	"Total Number of Routing session add fail rt tbl full	: %lu\n", nsess_add_fail_rt_tbl_full);
	seq_printf(seq,	"Total Number of Routing session add fail coll full	: %lu\n", nsess_add_fail_coll_full);
	seq_printf(seq,	"Total Number of Routing session add fail others	: %lu\n", nsess_add_fail_oth);
	seq_printf(seq,	"=====================================================================\n");
	return 0;
}

static int proc_read_ppv4_rtstats_seq_open(struct inode *inode, struct file *file)
{
	return single_open(file, proc_read_ppv4_rtstats, NULL);
}

static ssize_t proc_clear_ppv4_rtstats(struct file *file, const char __user *buf, size_t count, loff_t *data)
{
	int len;
	char str[40];
	char *p;

        if (!capable(CAP_NET_ADMIN)) {
                printk ("Write Permission denied");
                return 0;
        }

	len = min(count, (size_t)(sizeof(str) - 1));
	len -= ppa_copy_from_user(str, buf, len);
	while ( len && str[len - 1] <= ' ' )
		len--;
	str[len] = 0;
	for ( p = str; *p && *p <= ' '; p++, len-- );
	if ( !*p )
		return count;

	if (strncmp(p, "clear", 5) == 0) {
		nsess_add = 0;
		nsess_del = 0;
		nsess_del_fail = 0;
		nsess_add_succ = 0;
		nsess_add_fail_rt_tbl_full = 0;
		nsess_add_fail_coll_full = 0;
		nsess_add_fail_oth = 0;
		printk(KERN_INFO "PPv4 HAL stats cleared!!!\n"); 
	} else {
		printk(KERN_INFO "usage : echo clear > /proc/ppa/ppv4/rtstats\n"); 
	}	
	
	return len; 
}

static int proc_read_ppv4_accel(struct seq_file *seq, void *v)
{
	if (!capable(CAP_SYSLOG)) {
		printk ("Read Permission denied");
		return 0;
	}
	seq_printf(seq,	"PPv4 Upstream Acceleration	: %s\n", g_us_accel_enabled ? "enabled" : "disabled");
	seq_printf(seq,	"PPv4 Downstream Acceleration	: %s\n", g_ds_accel_enabled ? "enabled" : "disabled");
	return 0;
}
 
static int proc_read_ppv4_accel_seq_open(struct inode *inode, struct file *file)
{
	return single_open(file, proc_read_ppv4_accel, NULL);
}

static ssize_t proc_set_ppv4_accel(struct file *file, const char __user *buf, size_t count, loff_t *data)
{
	int len;
	char str[40];
	char *p;
        if (!capable(CAP_NET_ADMIN)) {
                printk ("Write Permission denied");
                return 0;
        }

	len = min(count, (size_t)(sizeof(str) - 1));
	len -= ppa_copy_from_user(str, buf, len);
	while ( len && str[len - 1] <= ' ' )
		len--;
	str[len] = 0;
	for ( p = str; *p && *p <= ' '; p++, len-- );
	if ( !*p )
		return count;

	if (strncmp(p, "enable", 6) == 0) {
		if (len > 6) {
			if (strncmp(p + 7, "us", 2) == 0) {
				g_us_accel_enabled = 3;
			} else if (strncmp(p + 7, "ds", 2) == 0) {
				g_ds_accel_enabled = 3;
			}
		} else {
			g_us_accel_enabled = 3;
			g_ds_accel_enabled = 3;
		}
		printk(KERN_INFO "Acceleration Enabled!!!\n");
	} else if (strncmp(p, "disable", 7) == 0) {
		if (len > 7) {
			if (strncmp(p + 8, "us", 2) == 0) {
				g_us_accel_enabled=0;
			} else if (strncmp(p + 8, "ds", 2) == 0) {
				g_ds_accel_enabled=0;
			}
		} else {
			g_us_accel_enabled = 0;
			g_ds_accel_enabled = 0;
		} 
		printk(KERN_INFO "Acceleration Disabled!!!\n"); 
	} else {
		printk(KERN_INFO "usage : echo <enable/disable> [us/ds] > /proc/ppa/ppv4/accel\n"); 
	}
	
	return len; 
}

static int proc_read_ppv4_debug(struct seq_file *seq, void *v)
{
	if (!capable(CAP_SYSLOG)) {
		printk ("Read Permission denied");
		return 0;
	}
	seq_printf(seq,	"PPv4 Debug	: %s\n", lgm_pp_hal_dbg_enable ? "enabled" : "disabled");
	return 0;
}

static int proc_read_ppv4_debug_seq_open(struct inode *inode, struct file *file)
{
	return single_open(file, proc_read_ppv4_debug, NULL);
}

static ssize_t proc_set_ppv4_debug(struct file *file, const char __user *buf, size_t count, loff_t *data)
{
	int len;
	char str[40];
	char *p;
        if (!capable(CAP_NET_ADMIN)) {
                printk ("Write Permission denied");
                return 0;
        }

	len = min(count, (size_t)(sizeof(str) - 1));
	len -= ppa_copy_from_user(str, buf, len);
	while ( len && str[len - 1] <= ' ' )
		len--;
	str[len] = 0;
	for ( p = str; *p && *p <= ' '; p++, len-- );
	if ( !*p )
		return count;

	if (strncmp(p, "enable", 6) == 0) {
		lgm_pp_hal_dbg_enable = 1;
		printk(KERN_INFO"Debug Enabled!!!\n");
	} else if (strncmp(p, "disable", 7) == 0) {
		lgm_pp_hal_dbg_enable = 0;
		printk(KERN_INFO"Debug Disbled!!!\n");
	} else {
		printk(KERN_INFO "usage : echo <enable/disable> > /proc/ppa/ppv4/dbg\n"); 
	}
	
	return len; 
}

void ppv4_proc_file_create(void)
{
	struct dentry *res;
	
	/*TBD: replace this with necessary data to be passed to the fn*/
	void *dummy_data = NULL;
	
	if (!g_ppa_debugfs_flag) {
		g_ppa_debugfs_dir = debugfs_create_dir("ppa", NULL);
		g_ppa_debugfs_flag = 1;
	}

	g_ppa_ppv4hal_debugfs_dir = debugfs_create_dir("pp_hal", g_ppa_debugfs_dir);
	g_ppa_ppv4hal_debugfs_flag = 1;

	res = debugfs_create_file("accel", 0600,
			g_ppa_ppv4hal_debugfs_dir,
			dummy_data,	
			&g_proc_file_ppv4_accel_seq_fops);	
	if (!res)
                goto err;

	res = debugfs_create_file("dbg", 0600,
			g_ppa_ppv4hal_debugfs_dir,
			dummy_data,	
			&g_proc_file_ppv4_debug_seq_fops);

	if (!res)
                goto err;

	res = debugfs_create_file("rtstats", 0600,
			g_ppa_ppv4hal_debugfs_dir,
			dummy_data,	
			&g_proc_file_ppv4_rtstats_seq_fops);
	if (!res)
                goto err;

	return;

err:
	debugfs_remove_recursive(g_ppa_ppv4hal_debugfs_dir);
	g_ppa_ppv4hal_debugfs_flag = 0;
}

void ppv4_proc_file_remove(void)
{
	if (g_ppa_ppv4hal_debugfs_flag) {
		debugfs_remove_recursive(g_ppa_ppv4hal_debugfs_dir);
		g_ppa_ppv4hal_debugfs_flag = 0;
	}
	
}
#endif /*defined(PPA_API_PROC)*/
/*
 * ####################################
 *		 Global Function
 * ####################################
 */

/*!
	\fn uint32_t get_hal_id(uint32_t *p_family,
			uint32_t *p_type,
			uint32_t *p_if,
			uint32_t *p_mode,
			uint32_t *p_major,
			uint32_t *p_mid,
			uint32_t *p_minor)
			char *name,
			char *version)
	\ingroup PPA_lgm_pp_hal_GLOBAL_FUNCTIONS
	\brief read HAL ID
	\param p_family	get family code
	\param p_type	get driver type
	\param p_if	get interface type
	\param p_mode	get driver mode
	\param p_major	get major number
	\param p_mid	get mid number
	\param p_minor	get minor number 
	\return no return value
 */
void get_lgm_pp_hal_id(uint32_t *p_family,
			uint32_t *p_type,
			uint32_t *p_if,
			uint32_t *p_mode,
			uint32_t *p_major,
			uint32_t *p_mid,
			uint32_t *p_minor)
{
	if ( p_family )
		*p_family = VER_FAMILY;

	if ( p_type )
		*p_type = VER_DRTYPE;

	if ( p_if )
		*p_if = VER_INTERFACE;

	if ( p_mode )
		*p_mode = VER_ACCMODE;

	if ( p_major )
		*p_major = VER_MAJOR;

	if ( p_mid )
		*p_mid = VER_MID;

	if ( p_minor )
		*p_minor = VER_MINOR;
}

/*!
	\fn uint32_t get_firmware_id(uint32_t *id,
					 char *name,
					 char *version)
	\ingroup PPA_lgm_pp_hal_GLOBAL_FUNCTIONS
	\brief read firmware ID
	\param p_family	 get family code
	\param p_type	 get firmware type
	\return no return value
 */
int32_t get_firmware_id(uint32_t *id,
				char *p_name,
				char *p_version)
{
	/* TBD */
	/* Get the firmware version from PPv4 */
	
	/*
	*id=sw_version.nId;
	ppa_memcpy(p_name,sw_version.cName, PPA_VERSION_LEN);
	ppa_memcpy(p_version,sw_version.cVersion, PPA_VERSION_LEN);
	*/

	return PPA_SUCCESS;
}

/*!
	\fn uint32_t get_number_of_phys_port(void)
	\ingroup PPA_lgm_pp_hal_GLOBAL_FUNCTIONS
	\brief get max number of physical ports
	\return get max number of physical ports
 */
uint32_t get_number_of_phys_port(void)
{	
	/* TBD?? needed ? get the port number dynamically from the system*/

	return MAX_LGM_PORTS;	
}

/*!
	\fn void get_phys_port_info(uint32_t port, uint32_t *p_flags,
				PPA_IFNAME ifname[PPA_IF_NAME_SIZE])
	\ingroup PPA_lgm_pp_hal_GLOBAL_FUNCTIONS
	\brief get physical port information
	\param port	 in port id
	\param p_flags	 get flags
	\param ifname	 get inteface name [ depricated ]
	\return no return value
 */
void get_phys_port_info(uint32_t port, uint32_t *p_flags,
				PPA_IFNAME ifname[PPA_IF_NAME_SIZE])
{
	/* This function can only set the flags based on GSWIP-O configuration
	Interface name needs to be retrieved from the dp */

	if ( port >= MAX_LGM_PORTS) {
		if (p_flags)
			*p_flags = 0;
		if (ifname)
			*ifname = 0;
		return;
	}

	if (p_flags) {
		*p_flags = 0;
		switch(port) {
		case 0: /*CPU port */
			*p_flags = PPA_PHYS_PORT_FLAGS_MODE_CPU_VALID;
			*ifname = 0;
			break;
		case 2: /*LAN side ports */
		case 3:
		case 4:
		case 5:
		case 6:
			if (g_port_map & (1 << port)) {
				*p_flags = PPA_PHYS_PORT_FLAGS_MODE_ETH_LAN_VALID;
			}
			break;
		case 7: /* dynamic ports */
		case 8:
		case 9:
		case 10:
		case 11:
		case 12:
		case 13:
		case 14:
		case 15:
			if (g_port_map & (1 << port)) {
				*p_flags = PPA_PHYS_PORT_FLAGS_MODE_ETH_MIX_VALID;
			}
			break;
		case 1: /* ethernet wan port*/
			if (g_port_map & (1 << port)) {
				*p_flags = PPA_PHYS_PORT_FLAGS_MODE_ETH_WAN_VALID;
			}
			break;
		default:
			*p_flags = 0;
			break;
		}
	}
}

/*!
	\fn void get_max_route_entries(uint32_t *p_entry,
					uint32_t *p_mc_entry)
	\ingroup PPA_lgm_pp_hal_GLOBAL_FUNCTIONS
	\brief get maximum number of routing entries
	\param p_entry	get maximum number of uni-cast routing entries. 
	\param p_mc_entry get maximum number of multicast routing entries.
	\return no return value
 */
void get_max_route_entries(uint32_t *p_entry,
				uint32_t *p_mc_entry)
{
	if ( p_entry )
		*p_entry = g_max_hw_sessions;

	if ( p_mc_entry )
		*p_mc_entry = MAX_MC_GROUP_ENTRIES;
}

/*!
	\fn void get_acc_mode(uint32_t f_is_lan, uint32_t *p_acc_mode)
	\ingroup PPA_lgm_pp_hal_GLOBAL_FUNCTIONS
	\brief get acceleration mode for interfaces (LAN/WAN)
	\param f_is_lan		 0: WAN interface, 1: LAN interface
	\param p_acc_mode	 a u32 data pointer to get acceleration mode (PPA_ACC_MODE_ROUTING / PPA_ACC_MODE_NONE)
	\return no return value
 */
void get_acc_mode(uint32_t f_is_lan, uint32_t *p_acc_mode)
{
	if (f_is_lan)
		*p_acc_mode = g_us_accel_enabled;
	else
		*p_acc_mode = g_ds_accel_enabled;
}

/*!
	\fn void set_acc_mode(uint32_t f_is_lan,
					uint32_t acc_mode)
	\ingroup PPA_lgm_pp_hal_GLOBAL_FUNCTIONS
	\brief set acceleration mode for interfaces (LAN/WAN)
	\param f_is_lan		 0: WAN interface, 1: LAN interface
	\param p_acc_mode	 acceleration mode (PPA_ACC_MODE_ROUTING / PPA_ACC_MODE_NONE/ PPA_ACC_MODE_BRIDGING/ PPA_ACC_MODE_HYBRID)
	\return no return value
*/
void set_acc_mode(uint32_t f_is_lan, uint32_t acc_mode)
{
	if (f_is_lan)
		g_us_accel_enabled = acc_mode;
	else
		g_ds_accel_enabled = acc_mode;
}

static inline uint32_t lgm_get_session_color(struct uc_session_node *p_item)
{
	return PP_COLOR_GREEN;; /*FIXME: 1 = green, 2 = orange, 3 = red */
}

static inline uint16_t dp_get_cpu_portinfo(void)
{
	/*TBD: this api will be implemented by dp later to return all the 8 CPU GPIds and qids*/
	return 16;
}
/*!
	\fn int32_t add_routing_entry(PPA_ROUTING_INFO *route_info) 
	\ingroup PPA_lgm_pp_hal_GLOBAL_FUNCTIONS
	\brief add one routing entry
 */
int32_t add_routing_entry(PPA_ROUTING_INFO *route)
{
	int32_t ret = 0, i;
	uint32_t session_id = 0;
	struct pp_sess_create_args rt_entry;
	struct learn_superset *suprset = NULL;
	PPA_NETIF *txif = NULL;
	PPA_IFNAME phys_netif_name[PPA_IF_NAME_SIZE];
	PPA_SUBIF dp_port = {0};
	struct qos_mgr_match_qid eg_tc = {0};
#if IS_ENABLED(CONFIG_LGM_TOE)
	PPA_LRO_INFO lro_entry = {0};
#endif /*IS_ENABLED(CONFIG_LGM_TOE)*/
	
	struct uc_session_node *p_item = (struct uc_session_node *)route->p_item;
	if (!p_item) {
		dbg("uc_session_node is null!!!\n");
		return PPA_FAILURE;
	}

	dbg("%s %d\n", __FUNCTION__, __LINE__);
	suprset = (struct learn_superset *) p_item->session_meta;	
	if (!suprset) {
		dbg("learn_superset is null!!!\n");
		return PPA_FAILURE;
	}

	ppa_memset(&rt_entry, 0, sizeof(struct pp_sess_create_args));

	nsess_add++;	
	
	rt_entry.in_port = p_item->ig_gpid;
	if(!rt_entry.in_port) {
		if(!(p_item->flag2 |= SESSION_FLAG2_CPU_OUT)) {
			dbg("Ingress port in null!\n");
			return PPA_FAILURE;
		} else {
		/* Local out session */
		/*TBD: ingress port needs to be set as the litepath device gpid when the litepath HW acceleration is enabled 
		until the hw acceleration is enabled; we keep this session not accelable*/
			p_item->flags |= SESSION_NOT_ACCELABLE;
			return PPA_FAILURE;
		}
	}

	if (p_item->tx_if) {
	/* get the physical tx netdevice*/
		txif = p_item->tx_if;
		if ((p_item->flags & (SESSION_VALID_PPPOE | SESSION_TUNNEL_DSLITE | SESSION_TUNNEL_6RD
			| SESSION_VALID_PPPOL2TP | SESSION_VALID_VLAN_INS | SESSION_VALID_OUT_VLAN_INS))
			|| (p_item->flag2 & SESSION_FLAG2_GRE) || netif_is_macvlan(txif)) {
			if (ppa_get_physical_if(txif, NULL, phys_netif_name) == PPA_SUCCESS) {
				txif = ppa_get_netif(phys_netif_name);
			}
			dbg("txif=%s\n", txif->name);
		}

		/* Get the egress gpid from the tx netdevice */
		if (dp_get_netif_subifid(txif, NULL, NULL, NULL, &dp_port, 0)) {
			dbg("Unable to get tx netdevice GPID!!!\n");
			return PPA_FAILURE;
		}
		rt_entry.eg_port = dp_port.gpid;
		dbg("%s %d ig_gpid=%d eg_port=%d eg_gpid=%d\n", __FUNCTION__, __LINE__, rt_entry.in_port, dp_port.port_id, rt_entry.eg_port);

		/*Set the dest_q */
		eg_tc.class = p_item->pkt.priority;
		rt_entry.dst_q = qos_mgr_get_mapped_queue(txif, rt_entry.eg_port, 0, &eg_tc, 0);
		dbg("%s %d rt_entry.dst_q=%d\n", __FUNCTION__, __LINE__, rt_entry.dst_q);
	} else {
		/* Locally terminated session we need to get the CPU GPID/queueid or LRO GPID/queueid*/
		if ((p_item->flag2 & SESSION_FLAG2_CPU_IN)) {
#if IS_ENABLED(CONFIG_LGM_TOE)
			if(!ppa_bypass_lro(p_item->session)) {
				/*add lro entry in PP and lro engine */
				ppa_memset(&lro_entry,0,sizeof(lro_entry));

				if(p_item->flags & SESSION_IS_TCP) {
					lro_entry.lro_type = LRO_TYPE_TCP;
					/*check mptcp options and if yes set
					lro_entry.lro_type = LRO_TYPE_MPTCP;
					*/
				} else if(p_item->flags & SESSION_TUNNEL_ESP) {
					dbg("ESP not supported in LRO\n");
				} else {
					lro_entry.lro_type = LRO_TYPE_UDP;
				}

				if ((add_lro_entry(&lro_entry)) == PPA_SUCCESS) {
					dbg("lro entry added\n");
					p_item->flag2 |= SESSION_FLAG2_LRO;
					p_item->lro_sessid = lro_entry.session_id;
					rt_entry.eg_port = dp_get_cpu_portinfo(); /*TBD: CPU_GPID*/
					/*LRO qid as returned from the lro conn*/
					rt_entry.dst_q = lro_entry.dst_q;
					/*set the lro flowid */
					rt_entry.lro_info = lro_entry.session_id;
					/*Set the session flags */
					rt_entry.flags |=  PP_SESS_FLAG_LRO_INFO_BIT;
				} else {
					dbg("lro entry add failed\n");
					/*TBD: ingress port needs to be set as the CPU gpid when the litepath HW acceleration is enabled 
					until the hw acceleration is enabled; we keep this session not accelable*/
					p_item->flags |= SESSION_NOT_ACCELABLE;
					return PPA_FAILURE;
				}
			}
#endif /*CONFIG_LGM_TOE*/
#if IS_ENABLED(CONFIG_LITEPATH_OFFLOAD)
			//rt_entry.eg_port = ?; TBD: LITEPATH_GPID
			if(!(p_item->flag2 & SESSION_FLAG2_LRO)) {
				/*Hardware offload of litepath sessions*/
				//rt_entry.dst_q = ?; TBD: CPU_QID'
			}
#endif /*IS_ENABLED(CONFIG_LITEPATH_OFFLOAD)*/
		} else {
			dbg("Unable to get tx netdevice GPID!!!\n");
			return PPA_FAILURE;
		}
	}
#if 0  // FIXME: needs more clarity

	/*Set the fsqm priority*/
	rt_entry.fsqm_prio = lgm_get_fsqm_prio(suprset->skb, p_item->tx_if); /* TBD: where to fetch this info */


#endif 

	/*Set Color for the session */
	rt_entry.color = lgm_get_session_color(p_item); /* TBD: Based on what we are supposed to set color*/

	/*TBD:Set the session group counters */
	for (i = 0; i < ARRAY_SIZE(rt_entry.sgc); i++)
		rt_entry.sgc[i] = PP_SGC_INVALID;

	/*TBD: Set the token bucket metering */
	for (i = 0; i < ARRAY_SIZE(rt_entry.tbm); i++)
		rt_entry.tbm[i] = PP_TBM_INVALID;
	
 	/*TBD: Set the UD parameters */		
	/* set UD parameters for egress session */
	if (!p_item->is_loopback) {
		rt_entry.ud_sz = 0;
		ppa_memset(rt_entry.ps,0,6); /* PS-B 6 bytes*/
		if(p_item->flag2 & SESSION_FLAG2_LRO) {
#if IS_ENABLED(CONFIG_LGM_TOE)
			/*Set lro type in UD*/
			rt_entry.ps[5] |= (lro_entry.lro_type << 6) ; /*set the egress flag in the SI UD bit 14-15 of PS-A*/
#if IS_ENABLED(CONFIG_LITEPATH_OFFLOAD)
			/* if litepath is enabled packet needs to be received on litepath netdev*/
			rt_entry.ps[3] = 0x8; /*set the egress flag in the SI UD bit 27 of PS-B*/
#endif /*IS_ENABLED(CONFIG_LITEPATH_OFFLOAD)*/
#endif /*IS_ENABLED(CONFIG_LGM_TOE)*/
		} else {
			rt_entry.ps[0] = dp_port.subif & 0x00FF;  /* sub_if_id */
			rt_entry.ps[1] = (dp_port.subif >> 8) & 0x00FF;
			rt_entry.ps[3] = 0x8; /*set the egress flag in the SI UD bit 27 of PS-B*/
		}
		rt_entry.ps_sz = 6; /* 6 bytes of PS-B data */
		rt_entry.ps_off = 0; /* copy ud0 to begining of the STW */
		/*Set the session flags */
		rt_entry.flags |=  PP_SESS_FLAG_PS_COPY_BIT;
	} else {
		/*TBD: Fill in the pp_port_cls_data in case we have a second cycle through PPv4 */
	}

	/* Fill in the fv information */
	rt_entry.in_pkt = &suprset->pkt[SUPRST_IN];
	rt_entry.eg_pkt = &suprset->pkt[SUPRST_EG];
	rt_entry.nhdr 	= &suprset->nhdr; 
		
	/* Fill in the Hash information */
	rt_entry.hash.h1 = p_item->hwhash.h1;
	rt_entry.hash.h2 = p_item->hwhash.h2;
	rt_entry.hash.sig = p_item->hwhash.sig;

	dbg("%s %d ig_gpid=%d eg_port=%d eg_gpid=%d\n", __FUNCTION__, __LINE__, rt_entry.in_port, dp_port.port_id, rt_entry.eg_port);
	dbg("%s %d rt_entry.dst_q=%d\n", __FUNCTION__, __LINE__, rt_entry.dst_q);

	/* Callin the API */
	if ((ret = pp_session_create(&rt_entry, &session_id, NULL))) {
		switch(ret) {
		/* TBD: handle the correct errorcode */
		case GSW_ROUTE_ERROR_RT_SESS_FULL:
			nsess_add_fail_rt_tbl_full++;
			break;
		case GSW_ROUTE_ERROR_RT_COLL_FULL:
			nsess_add_fail_coll_full++;
			break;
		default:
			nsess_add_fail_oth++;
			break;
		}
		dbg("pp_session_create returned failure!! %s %d ret=%d\n", __FUNCTION__, __LINE__,ret);
		return PPA_FAILURE;
	}	

	dbg("%s %d pp_session_create succeeded id=%d\n", __FUNCTION__, __LINE__,session_id);
	route->entry = p_item->routing_entry = session_id;
	p_item->flags |= SESSION_ADDED_IN_HW;

	/*set used flag in hal db*/
	if ((session_id >= 0) && (session_id < g_max_hw_sessions)) {
		spin_lock_bh(&g_hal_db_lock);
		pp_hal_db[session_id].used = 1;
		pp_hal_db[session_id].node = (void*)p_item;
		spin_unlock_bh(&g_hal_db_lock);
		dbg("%s %d session id=%d p_item=%px\n", __FUNCTION__, __LINE__, p_item->routing_entry, p_item);
	} else {
		dbg("invalid session_id %d!!\n", session_id);
	}

	return PPA_SUCCESS;
}

/*!
	\fn int32_t del_routing_entry(PPA_ROUTING_INFO *route_info) 
	\ingroup PPA_lgm_pp_hal_GLOBAL_FUNCTIONS
	\brief add one routing entry
*/
int32_t del_routing_entry(PPA_ROUTING_INFO *route)
{
	int32_t ret=0;

	struct uc_session_node *p_item = (struct uc_session_node *)route->p_item;
	if (!p_item)
		return PPA_FAILURE;

	if (p_item->flags & SESSION_ADDED_IN_HW) {

		dbg("%s %d deleting p_item=%px sessionid=%d\n", __FUNCTION__, __LINE__, p_item, p_item->routing_entry);

		if ((ret = pp_session_delete(p_item->routing_entry, NULL))) {
			dbg("pp_session delete returned Error:%d\n",ret);
		} else {
			/*Reset set used flag in hal db*/
			spin_lock_bh(&g_hal_db_lock);
			ppa_memset(&pp_hal_db[p_item->routing_entry],0,sizeof(PP_HAL_DB_NODE));
			spin_unlock_bh(&g_hal_db_lock);
		}
	}
	p_item->flags &= !SESSION_ADDED_IN_HW;
	return ret;	
}

/*!
	\fn void del_wan_mc_entry(uint32_t entry)
	\ingroup PPA_lgm_pp_hal_GLOBAL_FUNCTIONS
	\brief delete one multicast routing entry
	\param entry	entry number got from function call "add_wan_mc_entry"
	\return no return value
 */
int32_t del_wan_mc_entry(PPA_MC_INFO *mc_route)
{
	int32_t ret = 0, i;

	if (mc_route->p_item) {
		struct mc_session_node *p_item = (struct mc_session_node *)mc_route->p_item;

		/*If the session is not a DROP session */
		if (!(p_item->flags & SESSION_DROP)) {
			spin_lock_bh(&g_hal_mc_db_lock);
			/*1. Delete the multicast Mc session */
			for (i = 0; i < MAX_MC_CLIENT_PER_GRP; i++) {
				if (mc_db[p_item->grp.group_id].used &&
					is_valid_session(mc_db[p_item->grp.group_id].session_id[i])) {
					/*1. Delete all the group member PPv4sessions*/
					/* session_id always stored +1 to handle valid sessionid 0*/
					if ((ret = pp_session_delete(mc_db[p_item->grp.group_id].session_id[i]-1, NULL))) {
						dbg("pp_session delete multicast client session returned Error:%d\n", ret);
					}
				}
				/*If the interface bitmask is set have to remove the uC entry */
				if (p_item->grp.if_mask & (1 << i)) {
					/* 2.Reset the bit in uC multicast group*/
					if ((ret = pp_mcast_dst_set(p_item->grp.group_id, i, PPA_IF_DEL))) {
						dbg("pp_mcast_dst_set returned Error:%d\n", ret);
					}
				}
			}
			/* 3.Clear the hal db entry */
			if (mc_db[p_item->grp.group_id].node != NULL) {
				ppa_memset(&mc_db[p_item->grp.group_id], 0, sizeof(MC_DB_NODE));
			} else {
				dbg("Multicast db for groupid %d is already in empty \n", p_item->grp.group_id);
			}
			spin_unlock_bh(&g_hal_mc_db_lock);
		}
		p_item->flags &= !SESSION_ADDED_IN_HW;
	}

	/*4. Delete the multicast group PPv4 session*/
	if(is_valid_session(mc_route->p_entry)) {
		if ((ret = pp_session_delete(mc_route->p_entry - 1, NULL))) { /*always stored +1 to handle index 0*/
			dbg("pp_session delete multicast group session returned Error:%d\n", ret);
		}
	}

	return ret;
}

/*!
	\fn int32_t update_wan_mc_entry(PPA_MC_INFO mc_route)
	\ingroup PPA_lgm_pp_hal_GLOBAL_FUNCTIONS
*/
int32_t update_wan_mc_entry(PPA_MC_INFO *mc_route)
{
	struct pp_sess_create_args pp_args = {0};
	struct lgm_mc_args *mc_args = NULL;
	struct qos_mgr_match_qid eg_tc = {0};
	uint32_t session_id = 0, dev_idx = 0;
	PPA_SUBIF dp_port = {0};
	int32_t ret = PPA_FAILURE, i = 0;

	struct mc_session_node *p_item = (struct mc_session_node *)mc_route->p_item;
	if (!p_item) {
		dbg("mc_session_node is null!!!\n");
		return PPA_FAILURE;
	}

	/*If the call is invoked from the learning driver p_item->session action will not be NULL*/
	if (!p_item->session_action) {

		/*Call is invoked from multicast daemon
		Following actions needs to be invoked
		1. update the HAL MC DB
		2. based on the current operation mc_route->cop we need to update the pp uC */
		if (mc_route->cop) {
			spin_lock_bh(&g_hal_mc_db_lock);
			mc_db[p_item->grp.group_id].node = (void *)p_item;
			mc_db[p_item->grp.group_id].used = 1;
			spin_unlock_bh(&g_hal_mc_db_lock);
			dev_idx = mc_route->cop->index; /* [valid index 0 -7] */
			if ((dev_idx >= 0) && (dev_idx < MAX_MC_GROUP_ENTRIES)) {
				/*Update the bit in uC multicast group*/
				if ((ret = pp_mcast_dst_set(p_item->grp.group_id, dev_idx, mc_route->cop->flag))) {
					dbg("pp_mcast_dst_set returned Error:%d group = %d index = %d op = %d\n",
						ret, p_item->grp.group_id, dev_idx, mc_route->cop->flag);
				} else {
					dbg("uC mc %s for group=%d index=%d dev=%s\n, "
						, (mc_route->cop->flag ? "reset" : "set"), p_item->grp.group_id, dev_idx,
						p_item->grp.txif[dev_idx].netif->name);
					/*If the operation is delete we need to delete the correspoinding session from the pp*/
					if (mc_route->cop->flag == PPA_IF_DEL) {
						spin_lock_bh(&g_hal_mc_db_lock);
						if (is_valid_session(mc_db[p_item->grp.group_id].session_id[dev_idx])) {
							/*Delete the group member PPv4sessions*/
							/* session_id always stored +1 to handle valid sessionid 0*/
							if ((ret = pp_session_delete(
								mc_db[p_item->grp.group_id].session_id[dev_idx] - 1, NULL))) {
								dbg("pp_session delete multicast client session %d returned Error:%d\n",
									mc_db[p_item->grp.group_id].session_id[dev_idx] - 1, ret);
							}
							dbg("pp_session delete multicast client session %d succeeded\n",
								mc_db[p_item->grp.group_id].session_id[dev_idx] - 1);
							/*Clear the index in the db*/
							mc_db[p_item->grp.group_id].session_id[dev_idx] = 0;
						}
						spin_unlock_bh(&g_hal_mc_db_lock);
					}
				}
			}
		} else {
			/*operation not specified nothing needs to be done */
			dbg("Multicast operation not specified !!!\n");
		}
		return ret;
	}

	/*IF the control reaches here the call is invoked from the learning driver */
	mc_args = (struct lgm_mc_args *) p_item->session_action;
	if (mc_args && !mc_args->superset) {
		dbg("learn_superset is null!!!\n");
		return PPA_FAILURE;
	}

	if (!mc_args->ig_gpid) {
		dbg("Session ingress gpid is not valid\n");
		return PPA_FAILURE;
	}

	memset(&pp_args, 0, sizeof(pp_args));

	pp_args.color = PP_COLOR_GREEN;
	/* Fill in the Hash information */
	pp_args.hash.h1 = mc_args->hwhash.h1;
	pp_args.hash.h2 = mc_args->hwhash.h2;
	pp_args.hash.sig = mc_args->hwhash.sig;

	/*TBD:Set the session group counters */
	for (i = 0; i < ARRAY_SIZE(pp_args.sgc); i++)
		pp_args.sgc[i] = PP_SGC_INVALID;
	/*TBD: Set the token bucket metering */
	for (i = 0; i < ARRAY_SIZE(pp_args.tbm); i++)
		pp_args.tbm[i] = PP_TBM_INVALID;
	/*End TBD Set*/

	/*************Getting ingress and egress ports***************/
	pp_args.in_port = mc_args->ig_gpid;

	/*Compare the ingress gpid with the multicast GPID; which is stored*/
	/*if the gpids match this is session for a specific destination.*/
	if (pp_args.in_port == g_mcast_nf.gpid) {
		/* Find the tx netdevice */
		if (p_item->grp.group_id == mc_args->groupid &&
			((p_item->grp.if_mask >> mc_args->dst_idx) & 1)) {
	
			dev_idx = mc_args->dst_idx; /* [valid value returned is [0-15] */
			if((dev_idx >= 0) && dev_idx < MAX_MC_CLIENT_PER_GRP) {
				/* if session already exists return */
				if(mc_db[p_item->grp.group_id].session_id[dev_idx]) {
					/*dbg("%s %d Session exists \n",__FUNCTION__,__LINE__);*/
					return PPA_SUCCESS;
				}

				if (p_item->grp.txif[dev_idx].netif) {
					/*************Get the egress gpid from the tx netdevice******/
					if (dp_get_netif_subifid(p_item->grp.txif[dev_idx].netif,
						NULL, NULL, NULL, &dp_port, 0)){

						dbg("Unable to get tx netdevice GPID!!!\n");
						return PPA_FAILURE;
					}
					/* egress gpid */
					pp_args.eg_port = dp_port.gpid;
					/*egress qid */
					pp_args.dst_q = qos_mgr_get_mapped_queue(p_item->grp.txif[dev_idx].netif,
						pp_args.eg_port, 0, &eg_tc, 0);
					dbg("%s %d dst_dev=%s\n",__FUNCTION__,__LINE__,p_item->grp.txif[dev_idx].netif->name);
					/************************************************************/
				}

				/*Setup the classification parameters*/
				/*classification not based on FV: Duplicate packets pass through PPv4*/
				pp_args.in_pkt  = &mc_args->superset->pkt[SUPRST_IN];
				pp_args.eg_pkt  = &mc_args->superset->pkt[SUPRST_EG];
				pp_args.nhdr    = &mc_args->superset->nhdr;
				/*End of FV information*/

				/*Classification is based on the Groupid and the dst_index present in the UD0*/
				pp_args.cls.n_flds = 2;
				pp_args.cls.fld_data[0] = mc_args->groupid;
				pp_args.cls.fld_data[1] = mc_args->dst_idx;

				/*Set the egress UD parameters */
				pp_args.ud_sz = 0;
				ppa_memset(pp_args.ps, 0, 6); /* PS-B 6 bytes*/
				/*For wlan stations we need to pass the multicast gpid to the fw for reliable multicast*/
				if ((p_item->grp.txif[dev_idx].if_flags & NETIF_DIRECTCONNECT)){
					pp_args.ps[0] = p_item->grp.group_id & 0xFF;  /* multicast group id bit[0-7]*/
					pp_args.ps[1] = (p_item->grp.group_id >> 8) & 0x01;	/* multicast group id bit[8]*/
				} else {
					pp_args.ps[0] = dp_port.subif & 0x00FF;  /* sub_if_id */
					pp_args.ps[1] = (dp_port.subif >> 8) & 0x00FF;
				}
				pp_args.ps[3] = 0x8; /*set the egress flag in the SI UD bit 27 of PS-B*/
				pp_args.ps_sz = 6; /* 6 bytes of PS-B data */
				pp_args.ps_off = 0; /* copy ud0 to begining of the STW */

				pp_args.mcast.dst_idx = mc_args->dst_idx;
				/*Set the session flags */
				pp_args.flags |=  PP_SESS_FLAG_MCAST_DST_BIT;

				/*add hardware session */
				if ((ret = pp_session_create(&pp_args, &session_id, NULL))) {
					dbg("pp_session_create returned failure!! %s %d ret=%d\n", __FUNCTION__, __LINE__, ret);
				} else {
					/* store session_id against the corresponding dev_idx in the hal mc db */
					if(is_valid_session(session_id + 1)) {
						spin_lock_bh(&g_hal_mc_db_lock);
						mc_db[p_item->grp.group_id].session_id[dev_idx] = session_id + 1; /*always stored +1 to handle index 0*/
						spin_unlock_bh(&g_hal_mc_db_lock);
						dbg("%s %d session_id=%d\n",__FUNCTION__,__LINE__, session_id + 1);
					}
				}
			}
		} else {
			dbg("Device index %d not valid for multicast group %d\n", dev_idx, mc_args->groupid);
		}
	} else if(!p_item->mc_entry) {
		/*It is multicast group session*/
		/* In case of multicast group session the egress gpid must be set as ingress gpid for the uC to identify correct IGP*/
		/*pp_args.eg_port = mc_args->ig_gpid;*/
		pp_args.eg_port = g_mcast_nf.gpid;

		/*egress qid static qid allocated during mc NF creation*/
		pp_args.dst_q = g_mcast_nf.qid;

		/*Filling in the FV information*/
		pp_args.in_pkt  = &mc_args->superset->pkt[SUPRST_IN];
		pp_args.eg_pkt  = &mc_args->superset->pkt[SUPRST_EG];
		pp_args.nhdr    = &mc_args->superset->nhdr;
		/*End of FV information*/

		/*Set the egress UD parameters */
		pp_args.ud_sz = 0;
		ppa_memset(pp_args.ps, 0, 6); /* PS-B 6 bytes*/
		pp_args.ps[0] = p_item->grp.group_id & 0xFF;  /* multicast group id bit[0-7]*/
		pp_args.ps[1] = (p_item->grp.group_id >> 8) & 0x01;	/* multicast group id bit[8]*/
		pp_args.ps[1] |= (g_mcast_nf.subif << 1) & 0xFE;	/* multicast NF subif id [12:9]*/
		pp_args.ps_sz = 6; /* 6 bytes of PS-B data */
		pp_args.ps_off = 0; /* copy ud0 to begining of the STW */

		pp_args.mcast.grp_idx = p_item->grp.group_id;
		/*Set the session flags */
		pp_args.flags |=  PP_SESS_FLAG_MCAST_GRP_BIT;

		/*add hardware session */
		if ((ret = pp_session_create(&pp_args, &session_id, NULL))) {
			dbg("pp_session_create returned failure!! %s %d ret=%d\n", __FUNCTION__, __LINE__, ret);
		} else {
			p_item->flags |= SESSION_ADDED_IN_HW;

			/* Multicast group session */
			if(is_valid_session(session_id + 1)) {
				mc_route->p_entry = p_item->mc_entry = session_id + 1; /*always stored +1 to handle index 0*/
				dbg("%s %d session_id=%d\n",__FUNCTION__,__LINE__, session_id + 1);
			}
		}
	}
	dbg("%s %d ig_gpid=%d eg_gpid=%d dst_q=%d g_mcast_nf.subif=%d\n", __FUNCTION__, __LINE__, pp_args.in_port, pp_args.eg_port, pp_args.dst_q, g_mcast_nf.subif);

	return ret;
}

void get_itf_mib(uint32_t itf, struct ppe_itf_mib *p)
{
}

/*!
	\fn uint32_t get_routing_entry_bytes(uint32_t session_index, uint32_t *f_hit, uint32_t reset_flag)
	\brief get one routing entry's byte counter
	\param entry	entry number got from function call "add_routing_entry"
	\param f_hit hit status
	\param count counter value
	\return error code from switch API
 */
int32_t get_routing_entry_bytes(uint32_t session_id, uint8_t *f_hit, uint64_t *bytes, uint64_t *packets, uint8_t flag)
{
	int ret = PPA_SUCCESS;

	spin_lock_bh(&g_hal_db_lock);
	if ((session_id >= 0) && (session_id < g_max_hw_sessions) && (pp_hal_db[session_id].used)) {
		*f_hit = 1;
		if (flag) {
			/* if the flag is set when the hw needs to be read immidiately */
			/* when the sessions is about to get removed from hardware */
			pp_session_stats_get(session_id, &pp_hal_db[session_id].stats);
		}
		*bytes = pp_hal_db[session_id].stats.bytes;
		*packets = pp_hal_db[session_id].stats.packets;
	} else {
		ret = PPA_FAILURE;
	}
	spin_unlock_bh(&g_hal_db_lock);
	return ret;
}

static inline uint32_t ppa_drv_get_phys_port_num(void)
{
	return MAX_LGM_PORTS;
}

/* All the capabilities currently supported	are hardcoded 
// register all the capabilities supported by PPV4 HAL*/
static int32_t lgm_pp_hal_register_caps(void)
{
	int32_t res = PPA_SUCCESS;	

	if ((res = ppa_drv_register_cap(SESS_IPV4, 1, PPV4_HAL)) != PPA_SUCCESS) {
		dbg("ppa_drv_register_cap returned failure for capability SESS_IPV4!!!\n");	
		goto PP_HAL_FAIL;
	}	
	
	if ((res = ppa_drv_register_cap(SESS_IPV6, 1, PPV4_HAL)) != PPA_SUCCESS) {
		dbg("ppa_drv_register_cap returned failure for capability SESS_IPV6!!!\n");	
		goto PP_HAL_FAIL;
	}	
	
	if ((res = ppa_drv_register_cap(SESS_MC_DS, 1, PPV4_HAL)) != PPA_SUCCESS) {
		dbg("ppa_drv_register_cap returned failure for capability SESS_MC_DS!!!\n");	
		goto PP_HAL_FAIL;
	}	
			
	if ((res = ppa_drv_register_cap(TUNNEL_6RD, 1, PPV4_HAL)) != PPA_SUCCESS) {
		dbg("ppa_drv_register_cap returned failure for capability TUNNEL_6RD!!!\n");	
		goto PP_HAL_FAIL;
	}	
	
	if ((res = ppa_drv_register_cap(TUNNEL_DSLITE, 1, PPV4_HAL)) != PPA_SUCCESS) {
		dbg("ppa_drv_register_cap returned failure for capability TUNNEL_DSLITE!!!\n");	
		goto PP_HAL_FAIL;
	}

	if ((res = ppa_drv_register_cap(TUNNEL_L2TP_US, 1, PPV4_HAL)) != PPA_SUCCESS) {
		dbg("ppa_drv_register_cap returned failure for capability TUNNEL_L2TP_US!!!\n");
		goto PP_HAL_FAIL;
	}

	if ((res = ppa_drv_register_cap(TUNNEL_L2TP_DS, 1, PPV4_HAL)) != PPA_SUCCESS) {
		dbg("ppa_drv_register_cap returned failure for capability TUNNEL_L2TP_DS!!!\n");	
		goto PP_HAL_FAIL;
	}

	if ((res = ppa_drv_register_cap(TUNNEL_GRE_US, 1, PPV4_HAL)) != PPA_SUCCESS) {
		dbg("ppa_drv_register_cap returned failure for capability TUNNEL_GRE_US!!\n");
		goto PP_HAL_FAIL;
	}

	if ((res = ppa_drv_register_cap(TUNNEL_GRE_DS, 1, PPV4_HAL)) != PPA_SUCCESS) {
		dbg("ppa_drv_register_cap returned failure for capability TUNNEL_GRE_DS!!\n");	
		goto PP_HAL_FAIL;
	}

	if ((res = ppa_drv_register_cap(SESS_LOCAL_IN, 1, PPV4_HAL)) != PPA_SUCCESS) {
		pr_err("ppa_drv_register_cap returned failure for capability SESS_LOCAL_IN!!!\n");
		return res;
	}

	if ((res = ppa_drv_register_cap(SESS_LOCAL_OUT, 1, PPV4_HAL)) != PPA_SUCCESS) {
		pr_err("ppa_drv_register_cap returned failure for capability SESS_LOCAL_OUT!!!\n");
		return res;
	}

#if IS_ENABLED(CONFIG_LGM_TOE)
	/*LRO table init */
	init_lro_table();
#endif /*IS_ENABLED(CONFIG_LGM_TOE)*/

	return res;

PP_HAL_FAIL:
	ppa_drv_deregister_cap(SESS_IPV4,PPV4_HAL);
	ppa_drv_deregister_cap(SESS_IPV6,PPV4_HAL);
	ppa_drv_deregister_cap(SESS_MC_DS,PPV4_HAL);
	ppa_drv_deregister_cap(TUNNEL_6RD,PPV4_HAL);
	ppa_drv_deregister_cap(TUNNEL_DSLITE,PPV4_HAL);
	ppa_drv_deregister_cap(TUNNEL_L2TP_DS,PPV4_HAL);
	ppa_drv_deregister_cap(TUNNEL_GRE_DS,PPV4_HAL);
	return res;
}

static int32_t lgm_pp_hal_deregister_caps(void)
{
	ppa_drv_deregister_cap(SESS_BRIDG,PPV4_HAL);
	ppa_drv_deregister_cap(SESS_IPV4,PPV4_HAL);
	ppa_drv_deregister_cap(SESS_IPV6,PPV4_HAL);
	ppa_drv_deregister_cap(SESS_MC_DS,PPV4_HAL);
	ppa_drv_deregister_cap(TUNNEL_6RD,PPV4_HAL);
	ppa_drv_deregister_cap(TUNNEL_DSLITE,PPV4_HAL);
	ppa_drv_deregister_cap(TUNNEL_L2TP_DS,PPV4_HAL);
	ppa_drv_deregister_cap(QOS_CLASSIFY,PPV4_HAL); 
	ppa_drv_deregister_cap(TUNNEL_GRE_DS,PPV4_HAL); 

	return PPA_SUCCESS;
}

static int32_t lgm_pp_hal_generic_hook(PPA_GENERIC_HOOK_CMD cmd, void *buffer, uint32_t flag)
{
	/*dbg("lgm_pp_hal_generic_hook cmd 0x%x_%s\n", cmd, ENUM_STRING(cmd) );*/
	switch (cmd) {
	case PPA_GENERIC_HAL_GET_PORT_MIB: {
			int i=0;
			int num;
			PPA_PORT_MIB *mib = (PPA_PORT_MIB*) buffer;
			num = NUM_ENTITY(mib->mib_info) > ppa_drv_get_phys_port_num() ? ppa_drv_get_phys_port_num() : NUM_ENTITY(mib->mib_info) ;
			for (i = 0; i < num; i++) {
			/* port mib needs to be read from dp library ?? or PP ?? */
			/*	
				mib->mib_info[i].ig_fast_rt_ipv4_udp_pkts = pae_port_mib.nRxUCv4UDPPktsCount;
				mib->mib_info[i].ig_fast_rt_ipv4_tcp_pkts = pae_port_mib.nRxUCv4TCPPktsCount;
				mib->mib_info[i].ig_fast_rt_ipv4_mc_pkts = pae_port_mib.nRxMCv4PktsCount;
				mib->mib_info[i].ig_fast_rt_ipv4_bytes = pae_port_mib.nRxIPv4BytesCount;
				mib->mib_info[i].ig_fast_rt_ipv6_udp_pkts = pae_port_mib.nRxUCv6UDPPktsCount;
				mib->mib_info[i].ig_fast_rt_ipv6_tcp_pkts = pae_port_mib.nRxUCv6TCPPktsCount;
				mib->mib_info[i].ig_fast_rt_ipv6_mc_pkts = pae_port_mib.nRxMCv6PktsCount;
				mib->mib_info[i].ig_fast_rt_ipv6_bytes = pae_port_mib.nRxIPv6BytesCount;
				mib->mib_info[i].ig_cpu_pkts = pae_port_mib.nRxCpuPktsCount;
				mib->mib_info[i].ig_cpu_bytes = pae_port_mib.nRxCpuBytesCount;
				mib->mib_info[i].ig_drop_pkts = pae_port_mib.nRxPktsDropCount;
				mib->mib_info[i].ig_drop_bytes = pae_port_mib.nRxBytesDropCount;
				mib->mib_info[i].eg_fast_pkts = pae_port_mib.nTxPktsCount;
				mib->mib_info[i].eg_fast_bytes = pae_port_mib.nTxBytesCount;
			*/
				if ((i >= 1) && (i <= 6))
					mib->mib_info[i].port_flag = PPA_PORT_MODE_ETH;
				else if (i == 13)
					mib->mib_info[i].port_flag = PPA_PORT_MODE_DSL;
				else if (i == 0)	/* 0 is CPU port*/
					mib->mib_info[i].port_flag = PPA_PORT_MODE_CPU;
				else
					mib->mib_info[i].port_flag = PPA_PORT_MODE_EXT;
			}
			mib->port_num = num;
			/*dbg("port_num=%d\n", mib->port_num);*/
			return PPA_SUCCESS;
		}
	case PPA_GENERIC_HAL_SET_DEBUG:	{

			lgm_pp_hal_dbg_enable = ((PPA_CMD_GENERAL_ENABLE_INFO*)buffer)->enable;
			dbg("Set lgm_pp_hal_dbg_enable to 0x%x\n", lgm_pp_hal_dbg_enable );

			return PPA_SUCCESS;
		}
	 case PPA_GENERIC_HAL_GET_MAX_ENTRIES: {

			PPA_MAX_ENTRY_INFO *entry = (PPA_MAX_ENTRY_INFO *)buffer;
			entry->max_lan_entries = g_max_hw_sessions;
			entry->max_wan_entries = g_max_hw_sessions;
			entry->max_mc_entries = MAX_MC_GROUP_ENTRIES;

			return PPA_SUCCESS;
		}
	case PPA_GENERIC_HAL_GET_HAL_VERSION: {

			PPA_VERSION *v = (PPA_VERSION *)buffer;
			get_lgm_pp_hal_id(&v->family, &v->type, &v->itf, &v->mode, &v->major, &v->mid, &v->minor);

			return PPA_SUCCESS;
		}
	case PPA_GENERIC_HAL_GET_PPE_FW_VERSION: {

			PPA_VERSION *v = (PPA_VERSION *)buffer;
			get_lgm_pp_hal_id(&v->family, &v->type, &v->itf, &v->mode, &v->major, &v->mid, &v->minor);

			return get_firmware_id(&v->id, v->name, v->version);
		}
	case PPA_GENERIC_HAL_GET_PHYS_PORT_NUM: {

			PPA_COUNT_CFG *count = (PPA_COUNT_CFG *)buffer;
			count->num = get_number_of_phys_port();

			return PPA_SUCCESS;
		 }
	case PPA_GENERIC_HAL_GET_PHYS_PORT_INFO: {

			PPE_IFINFO *info = (PPE_IFINFO *) buffer;
			get_phys_port_info(info->port, &info->if_flags, info->ifname);

			return PPA_SUCCESS;
		}
	case PPA_GENERIC_HAL_SET_ROUT_CFG: 
	case PPA_GENERIC_HAL_SET_BRDG_CFG: {

			/* not supported */
			return PPA_SUCCESS;
		}
	case PPA_GENERIC_HAL_SET_ACC_ENABLE: {

			/*Enable/disable upstream/downstream acceleration */
			PPA_ACC_ENABLE *cfg = (PPA_ACC_ENABLE *)buffer;
			set_acc_mode(cfg->f_is_lan, cfg->f_enable);

			return PPA_SUCCESS;
		}
	 case PPA_GENERIC_HAL_GET_ACC_ENABLE: {

			PPA_ACC_ENABLE *cfg = (PPA_ACC_ENABLE *)buffer;
			get_acc_mode(cfg->f_is_lan, &cfg->f_enable);

			return PPA_SUCCESS;
		}
	case PPA_GENERIC_HAL_GET_IPV6_FLAG: {

			/*Always returns enabled*/
			return PPA_ENABLED;
		}
	case PPA_GENERIC_HAL_UPDATE_SESS_META:
	case PPA_GENERIC_HAL_CLEAR_SESS_META: {

			/* No special metadate needed for PPv4 HAL*/
			if (g_us_accel_enabled || g_ds_accel_enabled)
				return PPA_SUCCESS;
			else
				return PPA_FAILURE;
		} 
	case PPA_GENERIC_HAL_ADD_ROUTE_ENTRY: {

			PPA_ROUTING_INFO *route = (PPA_ROUTING_INFO *)buffer;
			struct uc_session_node *p_item = (struct uc_session_node *)route->p_item;
			if (!p_item)
				return PPA_FAILURE;
			if (is_lansession(p_item->flags)) {
				if (!g_us_accel_enabled) {
					dbg("\n PPv4 HAL US Acceleration is disabled!!! \n"); 
					return PPA_FAILURE; 
				} 
			} else {
				if (!g_ds_accel_enabled) {
					dbg("\n PPv4 HAL DS Acceleration is disabled!!! \n"); 
					return PPA_FAILURE;
				}	
			}
			return add_routing_entry(route);
		}
	case PPA_GENERIC_HAL_DEL_ROUTE_ENTRY: {

			PPA_ROUTING_INFO *route = (PPA_ROUTING_INFO *)buffer;
			while ((del_routing_entry(route)) < 0) {
				return PPA_FAILURE;
			}
			return PPA_SUCCESS;
		}
	case PPA_GENERIC_HAL_ADD_MC_ENTRY:
	case PPA_GENERIC_HAL_UPDATE_MC_ENTRY: {

			PPA_MC_INFO *mc = (PPA_MC_INFO *)buffer;
			return update_wan_mc_entry(mc);
		}
	case PPA_GENERIC_HAL_DEL_MC_ENTRY: {

			PPA_MC_INFO *mc = (PPA_MC_INFO *)buffer;
			del_wan_mc_entry(mc);
			return PPA_SUCCESS;
		}
	 case PPA_GENERIC_HAL_GET_ROUTE_ACC_BYTES: {

			PPA_ROUTING_INFO *route = (PPA_ROUTING_INFO *)buffer;

			get_routing_entry_bytes(route->entry, &route->f_hit, &route->bytes, &route->packets, flag);
			return PPA_SUCCESS;
		}
	case PPA_GENERIC_HAL_GET_MC_ACC_BYTES: {

			PPA_MC_INFO *mc = (PPA_MC_INFO *)buffer;

			get_routing_entry_bytes(mc->p_entry, &mc->f_hit, &mc->bytes, &mc->packets, flag);
		
			return PPA_SUCCESS;
		}
	case PPA_GENERIC_HAL_GET_ITF_MIB: {

			PPE_ITF_MIB_INFO *mib = (PPE_ITF_MIB_INFO *)buffer;
			get_itf_mib( mib->itf, &mib->mib);
			return PPA_SUCCESS;
		}
	case PPA_GENERIC_HAL_INIT: {

			PPA_HAL_INIT_CFG *cfg = (PPA_HAL_INIT_CFG*)buffer;
			del_routing_session_cb = cfg->del_cb;

			return lgm_pp_hal_register_caps();
		}
	case PPA_GENERIC_HAL_EXIT: {

			del_routing_session_cb = NULL;
			return lgm_pp_hal_deregister_caps();
		}
	default:
		return PPA_FAILURE;
	}
	return PPA_FAILURE;
}

/**************************************************************************************************
* Each entry in the inactivity list is being inactive in HW for more than 420 seconds.
* For each entry in the inactivity list
* Call the ppa callback for hw session delete.
* it will internally call the hal session delete entry
***************************************************************************************************/
void pphal_session_inact_cb(struct pp_cb_args *args)
{
	struct pp_inactive_list_cb_args *inact_args;
	int i;
	uint32_t session_id = 0;

	dbg("%s invoked\n",__FUNCTION__);

	if (args->ev != PP_INACTIVE_LIST) {
		dbg("%s %d empty list\n", __FUNCTION__, __LINE__);
		return;
	}

	inact_args = container_of(args, struct pp_inactive_list_cb_args, base);

	if (inact_args->base.ret) {
		dbg("failed to get inactive sessions list %d\n", inact_args->base.ret);
        } else {
		dbg("%s %d inact_args->n_sessions=%d\n", __FUNCTION__, __LINE__, inact_args->n_sessions);
		/*for each entry in the pp hal_db update the inactiviry status*/
		for (i = 0; i < inact_args->n_sessions; i++) {
			session_id = inact_args->inact_sess[i];
			dbg("%s %d inact_args->inact_sess[i]=%d pp_hal_db[%d].used=%d\n", __FUNCTION__, __LINE__,
				session_id, session_id, pp_hal_db[session_id].used);
			if (((session_id >= 0) && (session_id < g_max_hw_sessions)) && pp_hal_db[session_id].used) {
				dbg("%s %d deleting %d p_item=%px p_item->sessiond=%d\n",
					__FUNCTION__, __LINE__, session_id, pp_hal_db[session_id].node,
					((struct uc_session_node *)pp_hal_db[session_id].node)->routing_entry);
				del_routing_session_cb(pp_hal_db[session_id].node);
			}
		}
        }

	/*free the inact_list after processing*/
	ppa_free(inact_args->inact_sess);
}

/* work queue handler thread function */
static void pphal_get_stats_and_inactive_sessions(struct work_struct *w)
{
	struct pp_request req;
	int ret = 0;
	uint32_t *inact_list = NULL;
	uint32_t arr_sz = 0, i;

	/*Iterate through maximum 1K entries in the HW*/
	for (i = 0; i < DEFAULT_DB_SLICE; g_db_index++) {
		if (g_db_index >= g_max_hw_sessions) {
			g_db_index = 0;
			break;
		} else {
			spin_lock_bh(&g_hal_db_lock);
			if ( (g_db_index >= 0) && (g_db_index < g_max_hw_sessions) && (pp_hal_db[g_db_index].used)) {
			/*If the entry is valid read the statistics from HW*/
				pp_session_stats_get(g_db_index, &pp_hal_db[g_db_index].stats);
				i++;
			}
			spin_unlock_bh(&g_hal_db_lock);
		}
	}

	g_iter_cnt++;
	if ((g_iter_cnt * DEFAULT_DB_SLICE) >= MAX_UC_SESSION_ENTRIES) { /*4x64=256 seconds completed */
		dbg("%s %d starting crawler\n",__FUNCTION__, __LINE__);
		/*start the crawler*/
		g_iter_cnt = 0;
		arr_sz = (g_max_hw_sessions < DEFAULT_INACT_ARRAY_SIZE) ? g_max_hw_sessions : DEFAULT_INACT_ARRAY_SIZE;

		/*Call the pp crawler to get all the inactive sessions*/
		req.cb = pphal_session_inact_cb;
		req.req_prio = 0;
		req.req_id   = get_jiffies_64();

		inact_list = (uint32_t *)ppa_malloc(sizeof(uint32_t) * arr_sz);
		if (inact_list) {
			dbg("%s %d calling pp_inactive_sessions_get arr_sz=%d\n",__FUNCTION__, __LINE__, arr_sz);
			ret = pp_inactive_sessions_get(&req, inact_list, arr_sz);
			if (ret) {
				ppa_free(inact_list);
			}
		}
	}

}

/* Declare work queue handler thread */
static DECLARE_DELAYED_WORK(stats_n_gc_thread, pphal_get_stats_and_inactive_sessions);

/**************************************************************************************************
* This function is invoked by the timer at an interval of 4 seconds.
* Each time it goes through a maximum of 1K entries in HW and reads the session statistics.
* It takes 4X64 = 256 Seconds (4.3 minutes to complete one pass through all the 64 k sessions).
* After collecting the statistics of all sessions it triggers HW thread (crawler) for getting incative sessions.
* Crawler will complete its action in less than 500ms and return a set of inactive sessions.
* The crawler will call callback function with a set of inactive sessions.
* One cycle of these operations is expected to complete in 5min in worst case (all hw sessions are full).
***************************************************************************************************/
static enum hrtimer_restart pphal_get_stats_and_inactive_sessions_timer(struct hrtimer *hrtimer)
{

	/* schedule work queue */
	if (g_workq)
		queue_delayed_work(g_workq, &stats_n_gc_thread, 0);

	/*re start timer*/
	ppa_hrt_forward(hrtimer, DEFAULT_POLLING_TIME);
	return HRTIMER_RESTART;
}

/*
 * ####################################
 *		 Init/Cleanup API
 * ####################################
 */
static inline void hal_init(void)
{
	int32_t ret=0;
	u32 flags = WQ_MEM_RECLAIM | WQ_UNBOUND;
	/* register callback with the hal selector*/
	ppa_drv_generic_hal_register(PPV4_HAL, lgm_pp_hal_generic_hook);

	ret = pp_max_sessions_get(&g_max_hw_sessions);
	if (unlikely(ret)) {
		return;
	}

	/*initialize the HAL DB*/
	pp_hal_db = (PP_HAL_DB_NODE *) ppa_malloc(sizeof(PP_HAL_DB_NODE) * g_max_hw_sessions);
	if (!pp_hal_db) {
		pr_err("Failed to allocate hal db\n");
		return;
	}
	ppa_memset(pp_hal_db, 0, sizeof(PP_HAL_DB_NODE) * g_max_hw_sessions);

	/*initalize the HAL MC DB*/
	ppa_memset(mc_db, 0, sizeof(MC_DB_NODE) * MAX_MC_GROUP_ENTRIES);

	/*Initialize the MC NF and Register the MC callback*/
	init_mc_nf();

	/*Initialize the Fragmenter NF*/
	init_frag_nf();

	/*Init the lock*/
	spin_lock_init(&g_hal_db_lock);
	/*Init the mc db lock*/
	spin_lock_init(&g_hal_mc_db_lock);

	/*initialize workq*/
	g_workq = alloc_workqueue("%s", flags, 2, "PP_HAL wq");
	if (!g_workq) {
		pr_err("Failed to create work queue");
        }

	/*init the timer*/
	ppa_hrt_init(&g_gc_timer, pphal_get_stats_and_inactive_sessions_timer);
	/*start timer*/
	ppa_hrt_start(&g_gc_timer, DEFAULT_POLLING_TIME);
}

static inline void hal_exit(void)
{
	if (g_workq)
		destroy_workqueue(g_workq);
	
	uninit_frag_nf();
	uninit_mc_nf();

	ppa_drv_generic_hal_deregister(PPV4_HAL);
}

static int __init lgm_pp_hal_init(void)
{
	hal_init();
#if defined(PPA_API_PROC)
	ppv4_proc_file_create();
#endif
	printk(KERN_INFO"lgm_pp_hal loaded successfully MAX_HW_SESSIONS=%d\n", g_max_hw_sessions);
	return 0;
}

static void __exit lgm_pp_hal_exit(void)
{
#if defined(PPA_API_PROC)
	ppv4_proc_file_remove();
#endif
	hal_exit();
}

module_init(lgm_pp_hal_init);
module_exit(lgm_pp_hal_exit);
MODULE_LICENSE("GPL");
