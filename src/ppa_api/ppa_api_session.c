/*******************************************************************************
**
** FILE NAME	: ppa_api_session.c
** PROJECT	: PPA
** MODULES	: PPA API(Routing/Bridging Acceleration APIs)
**
** DATE		: 4 NOV 2008
** AUTHOR	: Xu Liang
** DESCRIPTION  : PPA Protocol Stack Hook API Session Operation Functions
** COPYRIGHT	:		  Copyright(c) 2009
**				  Lantiq Deutschland GmbH
**			   Am Campeon 3; 85579 Neubiberg, Germany
**
**   For licensing information, see the file 'LICENSE' in the root folder of
**   this software module.
**
** HISTORY
** $Date		$Author				$Comment
** 04 NOV 2008  Xu Liang			   Initiate Version
** 10 DEC 2012  Manamohan Shetty	   Added the support for RTP,MIB mode  
**									 Features
** 01 AUG 2018  Kamal Eradath		Added external packet learning 
*******************************************************************************/
/*
 * ####################################
 *			  Head File
 * ####################################
 */
/*
 *  Common Head File
 */
#include <linux/version.h>
#include <generated/autoconf.h>
#include <linux/swap.h>
#include <linux/netfilter.h>
#include <linux/byteorder/generic.h>
#include <linux/netfilter/nf_conntrack_common.h>
#include <linux/workqueue.h>

#if IS_ENABLED(CONFIG_SKB_EXTENSION)
#include <linux/skb_extension.h>
#endif /*IS_ENABLED(CONFIG_SKB_EXTENSION)*/

#if IS_ENABLED(CONFIG_PPA_EXT_PKT_LEARNING)
#include <linux/learning_layer_api.h>
#endif /*CONFIG_PPA_EXT_PKT_LEARNING*/

/*
 *  PPA Specific Head File
 */
#if IS_ENABLED(CONFIG_LTQ_DATAPATH)
#include <net/datapath_api.h>
#endif
#include <net/ppa/ppa_api.h>
#include <net/ppa/ppa_drv_wrapper.h>
#include <net/ppa/ppa_hal_wrapper.h>
#include <net/ppa/ppa_stack_al.h>

#include "ppa_api_misc.h"
#include "ppa_api_netif.h"
#include "ppa_api_session.h"
#if defined(CONFIG_PPA_HANDLE_CONNTRACK_SESSIONS)
#include "ppa_api_session_limit.h"
#endif
#include "ppa_api_mib.h"
#include "ppa_api_hal_selector.h"
#include "ppa_api_tools.h"
#if defined(CONFIG_LTQ_CPU_FREQ) || IS_ENABLED(CONFIG_LTQ_PMCU)
#include "ppa_api_pwm.h"
#endif
#include "ppa_api_sess_helper.h"
#include "ppa_api_debug.h"

#if defined(CONFIG_PPA_API_SW_FASTPATH)
#include "ppa_sae_hal.h"
#endif

#if IS_ENABLED(CONFIG_PPA_MPE_IP97)
#include <crypto/ltq_ipsec_ins.h>
#endif
/*
 * ####################################
 *			  Definition
 * ####################################
 */
#if IS_ENABLED(CONFIG_PPA_EXT_PKT_LEARNING)
static s32 learning_done_event(struct notifier_block *unused,
				unsigned long event,
				void *learned_superset);

static  PPA_NOTIFIER_BLOCK learning_done_notifier = {
		.notifier_call = learning_done_event,
};
#endif /*CONFIG_PPA_EXT_PKT_LEARNING*/

/*
 *  hash calculation
 */

#define MIN_POLLING_INTERVAL 1
uint32_t sys_mem_check_flag=0;
uint32_t stop_session_add_threshold_mem=500; //unit is K
#define K(x)((x) << (PAGE_SHIFT - 10))

#ifndef LROSESS_START_BIT_POS
#define LROSESS_START_BIT_POS 31
#define LROSESS_NO_OF_BITS 1
#define LROSESS_MASK 0x80000000
#endif
/*
 * ####################################
 *		  Data Type
 * ####################################
 */
/* session delete workqueue parameter */
struct ppa_sess_del_dwork{
	struct delayed_work dwork;
	PPA_SESSION *session;
	uint32_t flags;
};

/*
 * ####################################
 *		 Declaration
 * ####################################
 */

bool only_SW_acc = 0;
int32_t g_mac_threshold_time = 100;
EXPORT_SYMBOL(g_mac_threshold_time);
/*
 *  implemented in PPA Low Level Driver
 */
/* multicast routing group list item operation*/
void ppa_init_mc_session_node(struct mc_session_node *);
static INLINE struct mc_session_node *ppa_alloc_mc_session_node(void);
static INLINE void ppa_free_mc_session_node(struct mc_session_node *);
static INLINE void ppa_insert_mc_group_item(struct mc_session_node *);
static INLINE void ppa_remove_mc_group_item(struct mc_session_node *);
static void ppa_free_mc_group_list(void);

/*  routing session timeout help function */
#if defined(CONFIG_PPA_ACCEL)
static INLINE uint32_t ppa_get_default_session_timeout(void);
#endif /*CONFIG_PPA_ACCEL*/
#if IS_ENABLED(CONFIG_PPA_RT_SESS_LEARNING)
/* to ckeck the various criterias for avoiding hw acceleration */
static INLINE int32_t ppa_hw_accelerate(PPA_BUF *ppa_buf, struct uc_session_node *p_item);
#endif /*CONFIG_PPA_RT_SESS_LEARNING*/
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,15,0)
static void ppa_check_hit_stat(unsigned long);
static void ppa_mib_count(unsigned long);
#if defined(PPA_INTF_MIB_TIMER) && IS_ENABLED(CONFIG_SOC_GRX500)
static void ppa_check_intf_mib(unsigned long dummy);
#endif /*defined(PPA_INTF_MIB_TIMER) && IS_ENABLED(CONFIG_SOC_GRX500)*/
#else
static PPA_HRT_RESTART ppa_check_hit_stat(PPA_HRT *hrt);
static PPA_HRT_RESTART ppa_mib_count(PPA_HRT *hrt);
#if defined(PPA_INTF_MIB_TIMER) && IS_ENABLED(CONFIG_SOC_GRX500)
static PPA_HRT_RESTART ppa_check_intf_mib(PPA_HRT *hrt);
#endif /*defined(PPA_INTF_MIB_TIMER) && IS_ENABLED(CONFIG_SOC_GRX500)*/
extern void ppa_hrt_stop(PPA_HRT *p_timer);
extern void ppa_hrt_restart(PPA_HRT *p_timer);
#endif

/*  bridging session list item operation*/
static INLINE void ppa_bridging_init_uc_session_node(struct br_mac_node *);
static INLINE struct br_mac_node *ppa_bridging_alloc_uc_session_node(void);
static INLINE void ppa_bridging_free_uc_session_node(struct br_mac_node *);
static INLINE void ppa_bridging_insert_session_item(struct br_mac_node *);
static INLINE void ppa_bridging_remove_session_item(struct br_mac_node *);
static void ppa_free_bridging_session_list(void);

/*  bridging session timeout help function */
static INLINE uint32_t ppa_bridging_get_default_session_timeout(void);

/*  help function for special function*/
static INLINE void ppa_remove_mc_groups_on_netif (PPA_IFNAME *);
static INLINE void ppa_remove_bridging_sessions_on_netif (PPA_IFNAME *);

/*
 * ####################################
 *			Local variables
 * ####################################
 */

static struct workqueue_struct	*g_timer_workq=0;
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,15,0)
static PPA_TIMER	g_hit_stat_timer;
static PPA_TIMER	g_mib_cnt_timer;
#else
static PPA_HRT		g_hit_stat_timer;
static PPA_HRT		g_mib_cnt_timer;
#endif /*LINUX_VERSION_CODE < KERNEL_VERSION(4,15,0)*/
static uint32_t		g_hit_polling_time = DEFAULT_HIT_POLLING_TIME;
static uint32_t		g_mib_polling_time = DEFAULT_MIB_POLLING_TIME;
#if defined(PPA_INTF_MIB_TIMER) && IS_ENABLED(CONFIG_SOC_GRX500)
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,15,0)
static PPA_TIMER	g_intf_mib_timer;
#else
static PPA_HRT          g_intf_mib_timer;
#endif /*LINUX_VERSION_CODE < KERNEL_VERSION(4,15,0)*/
static uint32_t		g_intf_mib_polling_time = DEFAULT_MIB_POLLING_TIME;

static void ppa_get_intf_mib(struct work_struct *w);
void ppa_update_intf_mib_hw(void);
#endif

PPA_ATOMIC		g_hw_session_cnt; /*including unicast & multicast sessions */
/*
 *  multicast routing session table
 */
static PPA_LOCK		g_mc_group_list_lock;
PPA_HLIST_HEAD		g_mc_group_list_hash_table[SESSION_LIST_MC_HASH_TABLE_SIZE];
static PPA_MEM_CACHE	*g_mc_group_item_cache = NULL;

#if IS_ENABLED(CONFIG_PPA_MPE_IP97)
extern PPE_LOCK		g_tunnel_table_lock;
static PPA_LOCK		g_ipsec_group_list_lock;
static PPA_INIT_LIST_HEAD(ipsec_list_head);
static PPA_MEM_CACHE	*g_ipsec_group_item_cache = NULL;
#endif

/*
 *  bridging session table
 */
static PPA_LOCK		g_bridging_session_list_lock;
static PPA_HLIST_HEAD	g_bridging_session_list_hash_table[BRIDGING_SESSION_LIST_HASH_TABLE_SIZE];
static PPA_MEM_CACHE	*g_bridging_session_item_cache = NULL;
/*
 * ####################################
 *		   Extern Variable
 * ####################################
 */
#if defined(CONFIG_PPA_API_SW_FASTPATH)
extern volatile uint8_t	g_sw_fastpath_enabled;
#endif

#if defined(CONFIG_PPA_MPE_IP97)
extern ppa_tunnel_entry	*g_tunnel_table[MAX_TUNNEL_ENTRIES];
extern uint32_t 	 g_tunnel_counter[MAX_TUNNEL_ENTRIES];
#endif
INLINE void ppa_free_ipsec_group_list_item(struct uc_session_node *p_item);
extern int32_t ppa_get_pppoa_base_if (PPA_IFNAME *ifname, PPA_IFNAME base_ifname[PPA_IF_NAME_SIZE]);
extern int32_t ppa_bypass_lro(PPA_SESSION *p_session);
/*
 * ####################################
 *			Local Function
 * ####################################
 */

#if IS_ENABLED(CONFIG_PPA_EXT_PKT_LEARNING)
/* Functions to extract data from the struct learn_superset */
/* Get ingress l3 protocol */
static inline void get_correct_header(struct learn_superset *superset, uint32_t dir, uint32_t *bitmap, struct fv_headers **header)
{
	if (superset->pkt[dir].fv_tunnel_bitmap) {
		*bitmap = superset->pkt[dir].fv_inner_bitmap;
		*header = &superset->pkt[dir].fv.inner;
	} else {
		*bitmap = superset->pkt[dir].fv_outer_bitmap;
		*header = &superset->pkt[dir].fv.outer;
	}
}
static inline uint16_t superset_is_pkt_mc(struct learn_superset *superset)
{
	return(superset->pkt[SUPRST_IN].proto_bitmap & PKT_MCAST);
}
static inline uint16_t superset_l3_proto(struct learn_superset *superset, uint32_t dir)
{
	uint32_t bitmap=0;
	struct fv_headers *header=NULL;
	
	get_correct_header(superset, dir, &bitmap, &header);
	
	if (!header)
		return 0;
	if (bitmap)
		return((bitmap & FV_L3_IPV6) ? ETH_P_IPV6 : ETH_P_IP);
	return 0;
}
/* Get ingress l4 protocol */
static inline uint16_t superset_l4_proto(struct learn_superset *superset, uint32_t dir)
{
	uint32_t bitmap=0;
	struct fv_headers *header=NULL;
	
	get_correct_header(superset, dir, &bitmap, &header);
	if (!header)
		return 0;
	if (bitmap) {
		return((bitmap & FV_L4_TCP) ? PPA_IPPROTO_TCP : (bitmap & FV_L4_UDP)
			? PPA_IPPROTO_UDP : UNKNOWN_L3);
	}	
	return 0; 
}
/* Get ingress ip tos */
static inline uint8_t superset_ip_tos(struct learn_superset *superset, uint32_t dir)
{
	uint8_t tos=0;
	uint32_t bitmap=0;
	struct fv_headers *header=NULL;

	get_correct_header(superset, dir, &bitmap, &header);

	if (!header)
		return 0;
	if ((bitmap & FV_L3_IPV6)) {
	/*TBD: endienness */
		tos = header->l3.v6.priority;
		tos = ((tos << 4) & 0xF0);
		tos |= header->l3.v6.flow_lbl[0];
	} else {
		tos = header->l3.v4.tos;
	}
	return tos; 
}
/* Get ingress source IP */
static inline void superset_src_ip(union nf_inet_addr *srcip, struct learn_superset *superset, uint32_t dir)
{
	uint32_t bitmap=0;
	struct fv_headers *header=NULL;

	get_correct_header(superset, dir, &bitmap, &header);
	
	if (!header)
		return;
	if ((bitmap & FV_L3_IPV6)) {
		memcpy(srcip->ip6, header->l3.v6.saddr.in6_u.u6_addr32, sizeof(PPA_IPADDR));
	} else {
		srcip->ip = header->l3.v4.saddr;
	}	
}
/* Get ingress destinantion IP*/
static inline void superset_dst_ip(union nf_inet_addr *dstip, struct learn_superset *superset, uint32_t dir)
{
	uint32_t bitmap=0;
	struct fv_headers *header=NULL;

	get_correct_header(superset, dir, &bitmap, &header);
	
	if (!header)
		return;
	if ((bitmap & FV_L3_IPV6)) {
		memcpy(dstip->ip6, header->l3.v6.daddr.in6_u.u6_addr32, sizeof(PPA_IPADDR));
	} else {
		dstip->ip = header->l3.v4.daddr;
	}	
}
static inline int32_t superset_is_loopback_pkt(struct learn_superset *superset, uint32_t dir)
{
	uint32_t bitmap=0;
        struct fv_headers *header=NULL;

        get_correct_header(superset, dir, &bitmap, &header);

	if (!header)
		return 0;
	if ((bitmap & FV_L3_IPV6)) {
		if (ppa_memcmp(header->l3.v6.saddr.in6_u.u6_addr32,
			header->l3.v6.daddr.in6_u.u6_addr32, sizeof(PPA_IPADDR)) == 0) {
			return 1;
		}
	} else {
		if (header->l3.v4.saddr == header->l3.v4.daddr) {
			return 1;
		}
	}
	return 0;
}

/* Get ingress source tcp/udp port */
static inline uint16_t superset_src_port(struct learn_superset *superset, uint32_t dir)
{
	__be16 port=0;
	uint32_t bitmap=0;
	struct fv_headers *header=NULL;

	get_correct_header(superset, dir, &bitmap, &header);

	if (!header)
		return 0;
	if (bitmap & FV_L4_TCP) {
		port = header->l4.tcp.source;
	} else if (bitmap & FV_L4_UDP) {
		port = header->l4.udp.source;
	}

	return be16_to_cpup(&port);
} 
/* Get ingress destination tcp/udp port */
static inline uint16_t superset_dst_port(struct learn_superset *superset, uint32_t dir)
{
	__be16 port=0;
	uint32_t bitmap=0;
	struct fv_headers *header=NULL;

	get_correct_header(superset, dir, &bitmap, &header);

	if (!header)
		return 0;
	if (bitmap & FV_L4_TCP) {
		port = header->l4.tcp.dest;
	} else if (bitmap & FV_L4_UDP) {
		port = header->l4.udp.dest;
	}

	return be16_to_cpup(&port);
}

static inline void superset_src_mac(uint8_t *smac, struct learn_superset *superset, uint32_t dir)
{
	memcpy(smac, superset->pkt[dir].fv.outer.l2.h_source, PPA_ETH_ALEN);
}

static inline void superset_dst_mac(uint8_t *dmac, struct learn_superset *superset, uint32_t dir)
{
	memcpy(dmac, superset->pkt[dir].fv.outer.l2.h_dest, PPA_ETH_ALEN);
}

static uint32_t superset_l2tp_info(struct learn_superset *superset, struct uc_session_node* p_item, uint32_t dir)
{
	if (superset->pkt[dir].fv_tunnel_bitmap & FV_TUNNEL_L2TPUDP) {
		p_item->pkt.pppol2tp_session_id = superset->pkt[dir].fv.tunnel.l2tpoudp.session_id;
		p_item->pkt.pppol2tp_tunnel_id = superset->pkt[dir].fv.tunnel.l2tpoudp.tunnel_id;
		p_item->flags |= SESSION_VALID_PPPOL2TP;
		return 1;
	} else {
		return 0;
	}
}

static uint32_t superset_pppoe_info(struct learn_superset *superset, struct uc_session_node* p_item, uint32_t dir)
{
	uint32_t bitmap=0;
	struct fv_headers *header=NULL;
	__be16 sess_id=0;

	get_correct_header(superset, dir, &bitmap, &header);

	if (!header)
		return 0;
	if (bitmap & FV_L2_PPPOE) {
		sess_id = header->l2.pppoe_id;
		p_item->pkt.pppoe_session_id = be16_to_cpup(&sess_id);
		p_item->flags |= SESSION_VALID_PPPOE;	
		return 1;
	} else {
		return 0;
	}
}

static uint32_t superset_dslite_info(struct learn_superset *superset, struct uc_session_node* p_item, uint32_t dir)
{
	if (superset->pkt[dir].fv_tunnel_bitmap & FV_TUNNEL_DSLITE) {
		p_item->flags |= SESSION_TUNNEL_DSLITE;
		return 1;
	} else {
		return 0;
	}
}

static uint32_t superset_6rd_info(struct learn_superset *superset, struct uc_session_node* p_item, uint32_t dir)
{
	if (superset->pkt[dir].fv_tunnel_bitmap & FV_TUNNEL_6RD) {
		p_item->flags |= SESSION_TUNNEL_6RD;
		return 1;
	} else {
		return 0;
	}
}

static uint32_t superset_gre_info(struct learn_superset *superset, struct uc_session_node* p_item, uint32_t dir)
{
	if (superset->pkt[dir].fv_tunnel_bitmap & (FV_TUNNEL_GRE_IP | FV_TUNNEL_GRE_ETH)) {
		p_item->flag2 |= SESSION_FLAG2_GRE;
		return 1;
	} else {
		return 0;
	}
}

static uint32_t superset_esp_info(struct learn_superset *superset, struct uc_session_node* p_item, uint32_t dir)
{
	if (superset->pkt[dir].fv_tunnel_bitmap & FV_TUNNEL_IPSEC) {
		p_item->flags |= SESSION_TUNNEL_ESP;
		return 1;
	} else {
		return 0;
	}
}

static void superset_vlan_info(struct learn_superset *superset, struct uc_session_node* p_item)
{
	uint32_t bitmapin=0, bitmapeg=0;
	struct fv_headers *headerin=NULL, *headereg=NULL;
	__be16 vlan=0;

	get_correct_header(superset, SUPRST_IN, &bitmapin, &headerin);
	get_correct_header(superset, SUPRST_EG, &bitmapeg, &headereg);

	if (!headerin || !headereg)
		return;
	
	if (bitmapin & FV_L2_INT_VLAN) {
	/* if the packet has internal vlan we need to remove internal as well as external vlan */
		p_item->flags |= SESSION_VALID_VLAN_RM;
		if ((bitmapin & FV_L2_EXT_VLAN))
			p_item->flags |= SESSION_VALID_OUT_VLAN_RM;
	} else if (bitmapin & FV_L2_EXT_VLAN) {
		/* only external vlan; that means single vlan */
		p_item->flags |= SESSION_VALID_VLAN_RM;
	}

	if (bitmapeg & FV_L2_INT_VLAN) {
	/* EG packet has internal vlan. 
	1. check whether the ingress external vlan is equal to the internal vlan(QinQ)
	2. if yes clear the flag SESSION_VALID_VLAN_RM
	3. if no add the internal vlan*/
		
		if (headerin->l2.ext_vlan == headereg->l2.int_vlan) {
			p_item->flags &= ~SESSION_VALID_VLAN_RM;
		} else {
			p_item->flags |= SESSION_VALID_VLAN_INS;
			vlan = headereg->l2.int_vlan;
			p_item->pkt.new_vci = be16_to_cpup(&vlan);	
		
		}
		if (bitmapeg & FV_L2_EXT_VLAN) { 	
			p_item->flags |= SESSION_VALID_OUT_VLAN_INS;
			vlan = headereg->l2.ext_vlan;
			p_item->pkt.out_vlan_tag = be16_to_cpup(&vlan);
		}
		
	} else if (bitmapeg & FV_L2_EXT_VLAN) { 
		vlan = headereg->l2.ext_vlan;
		p_item->pkt.new_vci = be16_to_cpup(&vlan);
		p_item->flags |= SESSION_VALID_VLAN_INS;
		
	}
}
/* Get base interface */
static inline PPA_NETIF *superset_netif (struct learn_superset *superset, uint32_t dir)
{
	PPA_NETIF* netif = ppa_dev_get_by_index(superset->pkt[dir].ifindex);
	ppa_put_netif (netif);
	return netif;
}

static inline uint8_t superset_eg_header_len(struct learn_superset *superset)
{
	return superset->nhdr.len;
} 

#endif /*IS_ENABLED(CONFIG_PPA_EXT_PKT_LEARNING)*/
/*
 *  routing session list item operation
 */
/*
 *  PPA Session routines
 */
#if IS_ENABLED(CONFIG_PPA_MPE_IP97)
extern PPA_HLIST_HEAD		  g_session_list_hash_table[SESSION_LIST_HASH_TABLE_SIZE];

int32_t ppa_session_delete_ipsec_mpe(uint32_t tunnel_index);

int32_t ppa_session_delete_ipsec(uint32_t tunnel_index)
{
	int32_t ret = PPA_SUCCESS;
	struct uc_session_node *p_item;
	int i =0;

	ppa_session_list_lock();

	for (i = 0; i < NUM_ENTITY(g_session_list_hash_table); i++) {

		ppa_hlist_for_each_entry(p_item, &g_session_list_hash_table[i], hlist)
		if ((p_item->flag2 & SESSION_FLAG2_VALID_IPSEC_OUTBOUND) || (p_item->flag2 & SESSION_FLAG2_VALID_IPSEC_TRANS)) {
			__ppa_session_put(p_item);
			__ppa_session_delete_item(p_item); 
		}
	}
	ppa_session_list_unlock();
	return ret;
}

int32_t ppa_session_delete_ipsec_mpe(uint32_t tunnel_index)
{
	int32_t ret = PPA_SUCCESS;
	struct uc_session_node *p_item;
	int i =0;

	ppa_session_list_lock();

	for (i = 0; i < NUM_ENTITY(g_session_list_hash_table); i++) {

		ppa_hlist_for_each_entry(p_item, &g_session_list_hash_table[i], hlist)
		if (((p_item->flag2 & (SESSION_FLAG2_VALID_IPSEC_OUTBOUND|SESSION_FLAG2_VALID_IPSEC_OUTBOUND_SA))
			== (SESSION_FLAG2_VALID_IPSEC_OUTBOUND|SESSION_FLAG2_VALID_IPSEC_OUTBOUND_SA))) {

			printk("\n%s,%d Tunel Index=0x%x,routing entry=%d\n",__FUNCTION__,__LINE__,
					p_item->tunnel_idx,p_item->routing_entry);
			__ppa_session_put(p_item);
			__ppa_session_delete_item(p_item); 
  		} 
  	} 
 	ppa_session_list_unlock();
  	return ret;
}
#endif

int32_t ppa_session_delete(PPA_SESSION *p_session, uint32_t flags)
{
	int32_t ret = PPA_FAILURE;
	uint32_t hash;
	PPA_TUPLE tuple;

	if (p_session) {
		/* find forward session hash */
		hash = ppa_get_hash_from_ct(p_session, 0, &tuple);
		ret = ppa_session_del(hash, p_session);

		/*find return session hash */
		hash = ppa_get_hash_from_ct(p_session, 1, &tuple);
		ret = ppa_session_del(hash, p_session);
	}
	return ret;
}

#if defined(SESSION_STATISTIC_DEBUG) && SESSION_STATISTIC_DEBUG 
static int32_t inline __update_session_hash(struct uc_session_node *p_item)
{
	PPA_SESSION_HASH hash;
	int32_t res;

	if (p_item->flag2 & SESSION_FLAG2_HASH_INDEX_DONE)
		return PPA_SUCCESS;

	hash.f_is_lan = (p_item->flags & SESSION_LAN_ENTRY) ? 1 : 0;
	if (p_item->flags & SESSION_IS_IPV6) {

		hash.src_ip = p_item->src_ip6_psuedo_ip;
		hash.dst_ip = p_item->dst_ip6_psuedo_ip;
	} else {
		hash.src_ip = p_item->pkt.src_ip.ip;
		hash.dst_ip = p_item->pkt.dst_ip.ip;
	}

	hash.src_port = p_item->pkt.src_port;
	hash.dst_port = p_item->pkt.dst_port;

	res = ppa_get_session_hash(&hash, 0);
	p_item->hash_index = hash.hash_index;
	p_item->hash_table_id = hash.hash_table_id;
	p_item->flag2 |= SESSION_FLAG2_HASH_INDEX_DONE;
	return res;
}
#endif

int32_t ppa_update_session_extra(PPA_SESSION_EXTRA *p_extra, struct uc_session_node *p_item, uint32_t flags)
{
	if ((flags & PPA_F_SESSION_NEW_DSCP)) {
		if (p_extra->dscp_remark) {
			p_item->flags |= SESSION_VALID_NEW_DSCP;
			p_item->pkt.new_dscp = p_extra->new_dscp;
		} else {
			p_item->flags &= ~SESSION_VALID_NEW_DSCP;
		}
	}

	if ((flags & PPA_F_SESSION_VLAN)) {
		if (p_extra->vlan_insert) {
			p_item->flags |= SESSION_VALID_VLAN_INS;
			p_item->pkt.new_vci = ((uint32_t)p_extra->vlan_prio << 13)
				|((uint32_t)p_extra->vlan_cfi << 12) | p_extra->vlan_id;
		} else {
			p_item->flags &= ~SESSION_VALID_VLAN_INS;
			p_item->pkt.new_vci = 0;
		}

		if (p_extra->vlan_remove)
			p_item->flags |= SESSION_VALID_VLAN_RM;
		else
			p_item->flags &= ~SESSION_VALID_VLAN_RM;
	}

	if ((flags & PPA_F_MTU)) {
		p_item->flags |= SESSION_VALID_MTU;
		p_item->mtu = p_extra->mtu;
	}

	if ((flags & PPA_F_SESSION_OUT_VLAN)) {
		if (p_extra->out_vlan_insert) {
			p_item->flags |= SESSION_VALID_OUT_VLAN_INS;
			p_item->pkt.out_vlan_tag = p_extra->out_vlan_tag;
			} else {
			p_item->flags &= ~SESSION_VALID_OUT_VLAN_INS;
			p_item->pkt.out_vlan_tag = 0;
		}

		if (p_extra->out_vlan_remove)
			p_item->flags |= SESSION_VALID_OUT_VLAN_RM;
		else
			p_item->flags &= ~SESSION_VALID_OUT_VLAN_RM;
	}

	/*only for hook and ioctl, not for ppe fw and HAL, It is mainly for session management*/
	if ((flags & PPA_F_ACCEL_MODE)) {
		if (p_extra->accel_enable) {
		/* If it needs to be added to h/w we simply reset flags which will make session go through linux stack */
			p_item->flags &= ~SESSION_NOT_ACCEL_FOR_MGM;
#if IS_ENABLED(CONFIG_PPA_API_SW_FASTPATH)
			if (p_item->flags & SESSION_ADDED_IN_SW) {
#if defined(PPA_IF_MIB) && PPA_IF_MIB
				sw_del_session_mgmt_stats(p_item);
#endif
				/* software session management is handled by SAE */
				ppa_sw_session_enable(p_item, 0, p_item->flags);
			}
#endif
		} else {
#if IS_ENABLED(CONFIG_PPA_API_SW_FASTPATH)
			/* software session management is handled by SAE */
			ppa_sw_session_enable(p_item, 1, p_item->flags);
#else
			p_item->flags |= SESSION_NOT_ACCEL_FOR_MGM;
#endif
		}
	}

	/*only for hook and ioctl, not for ppe fw and HAL. At present it only for test purpose*/
	if ((flags & PPA_F_PPPOE)) {
		/*need to add or replace old pppoe*/
		if (p_extra->pppoe_id) {
			p_item->flags |= SESSION_VALID_PPPOE;
			p_item->pkt.pppoe_session_id = p_extra->pppoe_id;
		} else {
		/*need to change pppoe termination to transparent*/
			p_item->flags &= ~SESSION_VALID_PPPOE;
			p_item->pkt.pppoe_session_id = 0;
		}
	}
	return PPA_SUCCESS;
}

#if defined(CONFIG_PPA_ACCEL)
static INLINE uint32_t ppa_get_default_session_timeout(void)
{
	return DEFAULT_TIMEOUT_IN_SEC;
}
#endif /*CONFIG_PPA_ACCEL*/

#if IS_ENABLED(CONFIG_PPA_RT_SESS_LEARNING)
int32_t ppa_postrouting_speed_handle_frame(PPA_BUF *ppa_buf,
				struct uc_session_node *p_item,
				uint32_t flags)
{
#if defined(CONFIG_PPA_MPE_IP97)
	uint32_t tunnel_index = 0;
	PPA_XFRM_STATE *xfrm_sa = NULL;
	uint32_t dir;
#endif
	uint8_t proto = ppa_get_pkt_ip_proto(ppa_buf);
	uint16_t sport, dport;

	if (p_item->flags & (SESSION_ADDED_IN_HW | SESSION_NOT_ACCEL_FOR_MGM | SESSION_NOT_ACCELABLE ) ) {
		/*
		 * Session exists, but this packet will take s/w path nonetheless.
		 * Can happen for individual pkts of a session, or for complete sessions!
		 * For eg., if WAN upstream acceleration is disabled.
		 */
#if defined(CONFIG_PPA_HANDLE_CONNTRACK_SESSIONS)
		ppa_update_ewma(ppa_buf,p_item,1);
#endif
		goto __PPA_FILTERED;
	}

#if defined(CONFIG_PPA_MPE_IP97)
/* Check for the IPSec packets and get the tunnel index  */
	if (proto == IP_PROTO_ESP) {
		if (p_item->tunnel_idx == ~0) {
			if (__ppa_update_ipsec_tunnelindex(ppa_buf,&tunnel_index) == PPA_SUCCESS) {
				p_item->tunnel_idx = tunnel_index;
				ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,"\n%s,%d Tunel Index=0x%x,routing entry=%d\n", __func__, __LINE__,p_item->tunnel_idx,p_item->routing_entry);
			} else {
				pr_err("\n%s,%d Tunel Index not found\n", __func__, __LINE__);
				goto __PPA_FILTERED;
			}
		} else
			tunnel_index = p_item->tunnel_idx; /*avoiding the lookup again */

		if (tunnel_index >= MAX_TUNNEL_ENTRIES) {
			pr_err("<%s> tunnel idx incorrect\n", __func__);
			goto __PPA_FILTERED;
		}
		if (g_tunnel_counter[tunnel_index] != 2) { /* one for inbound the other for outbound, so 2*/
			pr_err("<%s> tunnel counter incorrect\n", __func__);
			goto __PPA_FILTERED;
		}

		dir = g_tunnel_table[tunnel_index]->tunnel_info.ipsec_hdr.dir;

		if (dir == LTQ_SAB_DIRECTION_INBOUND)
			xfrm_sa = g_tunnel_table[tunnel_index]->tunnel_info.ipsec_hdr.inbound;
		else if (dir == LTQ_SAB_DIRECTION_OUTBOUND)
			xfrm_sa = g_tunnel_table[tunnel_index]->tunnel_info.ipsec_hdr.outbound;
		else {
			pr_err("<%s> Invalid direction\n", __func__);
			return PPA_FAILURE;
		}

		if (xfrm_sa->props.mode == XFRM_MODE_TRANSPORT) {
			p_item->flag2 |= (SESSION_FLAG2_VALID_IPSEC_TRANS);
			goto __PPA_FILTERED;
		} else if (xfrm_sa->props.mode == XFRM_MODE_TUNNEL) {
			p_item->flag2 |= (SESSION_FLAG2_VALID_IPSEC_OUTBOUND);
		} else
			pr_err("<%s> mode not tunnel nor transport!\n", __func__);
	}
#endif
	/*
	 * Skip skb from update session,
	 * specifically when it traverses through l2tp tunnel segment.
	 */
	if (proto == PPA_IPPROTO_UDP) {
		sport = ppa_get_pkt_src_port(ppa_buf);
		dport = ppa_get_pkt_dst_port(ppa_buf);

		if ((sport == 1701) && (dport == 1701)) {
			SET_DBG_FLAG(p_item, SESSION_DBG_ALG);
			goto __PPA_FILTERED;
		}
	}

/*Incrementing the hit count moved to post-routing part */
	p_item->num_adds++;

#if defined(CONFIG_PPA_HANDLE_CONNTRACK_SESSIONS)
	if (ppa_session_pass_criteria(ppa_buf,p_item,0) == PPA_FAILURE)
#else
	//  not enough hit
	if (ppa_session_entry_pass(p_item->num_adds, g_ppa_min_hits))
#endif
	{
		SET_DBG_FLAG(p_item, SESSION_DBG_NOT_REACH_MIN_HITS);
		goto __PPA_FILTERED;
	} else {
		CLR_DBG_FLAG(p_item, SESSION_DBG_NOT_REACH_MIN_HITS);
	}

	if (proto == PPA_IPPROTO_TCP) {
		p_item->flags |= SESSION_IS_TCP;
		CLR_DBG_FLAG(p_item, SESSION_DBG_TCP_NOT_ESTABLISHED);
	}

	if (!(flags & PPA_F_BRIDGED_SESSION)) {
		if (!p_item->session) {
			ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,"no connection track for this p_item!!!\n");
			goto __PPA_FILTERED;
		}
		//  check if session needs to be handled in MIPS for conntrack handling (e.g. ALG)
		if (ppa_check_is_special_session(ppa_buf, p_item->session)) {
			p_item->flags |= SESSION_NOT_ACCELABLE;
			SET_DBG_FLAG(p_item, SESSION_DBG_ALG);
			goto __PPA_FILTERED;
		}
		//  for TCP, check whether connection is established
		if (p_item->flags & SESSION_IS_TCP) {
			if (!ppa_is_tcp_established(p_item->session)) {
				SET_DBG_FLAG(p_item, SESSION_DBG_TCP_NOT_ESTABLISHED);
				goto __PPA_FILTERED;
			}
			else
				CLR_DBG_FLAG(p_item, SESSION_DBG_TCP_NOT_ESTABLISHED);
#if IS_ENABLED(CONFIG_PPA_TCP_LITEPATH)
		} else {
		// local_in/out non tcp session not accelerated
			if (flags & (PPA_F_SESSION_LOCAL_IN|PPA_F_SESSION_LOCAL_OUT)) {
				p_item->flags |= SESSION_NOT_ACCELABLE;
				SET_DBG_FLAG(p_item, SESSION_DBG_ALG);
				goto __PPA_FILTERED;
			}
#endif // CONFIG_PPA_TCP_LITEPATH
		}
	}
#if defined(CONFIG_PPA_HANDLE_CONNTRACK_SESSIONS)
	// If session added successfully to the PPA then only reset ewma stats
	ppa_reset_ewma_stats(p_item,0);
#endif
	return PPA_SESSION_NOT_FILTED;

__PPA_FILTERED:

	return PPA_SESSION_FILTED;
}

int32_t ppa_prerouting_speed_handle_frame(PPA_BUF *ppa_buf,
				struct uc_session_node *p_item,
				uint32_t flags)
{
	/* Every packet will come to here two times(before nat and after nat).
	* so make sure only count once. In case of bridged session handling,
	* packets will hit twice from PRE routing hook.
	*/
	/* Set skb extmark bit to tell the stack packet is not classified by Flow rule */
#ifdef CONFIG_NETWORK_EXTMARK
	if (ppa_get_pkt_priority(ppa_buf) == 0) {
		//ppa_buf->extmark |= SESSION_FLAG_DSCP_REMARK;
		ppa_set_skb_extmark(ppa_buf, SESSION_FLAG_DSCP_REMARK);
#if IS_ENABLED(CONFIG_INTEL_IPQOS_MPE_DS_ACCEL)
		ppa_buf->extmark = ppa_buf->extmark & (~(SESSION_DS_QOS_SET));
#endif
	}
#else /* CONFIG_NETWORK_EXTMARK */
	if (ppa_get_pkt_priority(ppa_buf) == 0) {
		ppa_buf->mark |= SESSION_FLAG_DSCP_REMARK;
#if IS_ENABLED(CONFIG_INTEL_IPQOS_MPE_DS_ACCEL)
		ppa_buf->mark = ppa_buf->mark & (~(SESSION_DS_QOS_SET));
#endif
	}

#endif /* CONFIG_NETWORK_EXTMARK */
	p_item->host_bytes += ppa_skb_len(ppa_buf) + PPA_ETH_HLEN + PPA_ETH_CRCLEN;
#if defined(CONFIG_PPA_HANDLE_CONNTRACK_SESSIONS)
	ppa_session_record_time(ppa_buf,p_item,0);
#endif
#ifdef CONFIG_PPA_PP_LEARNING
	if (ppa_hw_accelerate(ppa_buf, p_item))
		ppa_pp_ingress_handler(ppa_buf, p_item);
#endif

	return PPA_SESSION_FILTED;
}

int32_t ppa_update_session(PPA_BUF *ppa_buf, struct uc_session_node *p_item, uint32_t flags)
{
	uint32_t ret = PPA_SESSION_NOT_ADDED;

	/*if ppa session is already added returning PPA_SESSION_ADDED*/
	if (p_item->flags & (SESSION_ADDED_IN_HW | SESSION_ADDED_IN_SW)) {
		ret = PPA_SESSION_ADDED;
		goto __UPDATE_FAIL;
	}
#ifdef CONFIG_PPA_PUMA7
	/*if ppa session info is already updated then just go and try to add the session*/
	if (p_item->flag2 & SESSION_FLAG2_UPDATE_INFO_PROCESSED)
		goto __UPDATE_PROCESSED;
#endif

	p_item->pkt.mark = ppa_get_skb_mark(ppa_buf);
#if IS_ENABLED(CONFIG_NETWORK_EXTMARK)
	p_item->pkt.extmark = ppa_get_skb_extmark(ppa_buf);
#endif
/*printk(KERN_INFO"in %s %d\n",__FUNCTION__, __LINE__);*/

#if (IS_ENABLED(CONFIG_PPA_TCP_LITEPATH) || IS_ENABLED(CONFIG_LTQ_TOE_DRIVER))
	/* session_add hook called from local_in
	// tcp traffic terminating on CPE will hit this point from local_in hook & PPA_F_SESSION_LOCAL_IN is passed with add_session
	// here we need to mark this session item as SESSION_FLAG2_CPU_IN as that it can be checked later for taking actions*/
	if ((flags & PPA_F_SESSION_LOCAL_IN)) {
		p_item->flag2 |= SESSION_FLAG2_CPU_IN;
#if IS_ENABLED(CONFIG_LTQ_TOE_DRIVER)
			/*if SESSION_FLAG2_LRO is not already set try adding the session to LRO*/
		if (!(p_item->flag2 & SESSION_FLAG2_LRO)) {
			if (ppa_lro_entry_criteria(p_item, ppa_buf, flags) == PPA_SUCCESS) {
				if ((ppa_add_lro_entry(p_item, 0)) == PPA_SUCCESS) {
					ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,"lro entry added\n");
					p_item->flag2 |= SESSION_FLAG2_LRO;  /*session added to LRO */
				} else {
					ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,"lro entry add failed\n");
					p_item->flag2 |= SESSION_FLAG2_ADD_HW_FAIL;  /*LRO session table full..*/
				}
			}
		}
#endif /*CONFIG_LTQ_TOE_DRIVER*/
	}
#endif /*CONFIG_PPA_TCP_LITEPATH || CONFIG_LTQ_TOE_DRIVER*/

#if defined(SKB_PRIORITY_DEBUG) && SKB_PRIORITY_DEBUG
	if (g_ppa_dbg_enable & DBG_ENABLE_MASK_MARK_TEST) {
		/*for test qos queue only depends on mark last 4 bits value*/
		p_item->pkt.priority = ppa_api_set_test_qos_priority_via_mark(ppa_buf);
	} else if (g_ppa_dbg_enable & DBG_ENABLE_MASK_PRI_TEST) {
		/*for test qos queue only depends on tos last 4 bits value*/
		p_item->pkt.priority = ppa_api_set_test_qos_priority_via_tos(ppa_buf);
	} else 
#endif
	{
		p_item->pkt.priority = ppa_get_pkt_priority(ppa_buf);
	}

	if (!((p_item->flag2 & SESSION_FLAG2_CONTAINER) && !(p_item->flag2 & SESSION_FLAG2_VETH))) {

		/* get information needed by hardware/firmware*/
		if ((ret = ppa_update_session_info(ppa_buf, p_item, flags)) != PPA_SUCCESS) {
			ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,"update session fail\n");
			goto __UPDATE_FAIL;
		}
		/*printk(KERN_INFO"%s %d update_session success\n", __FUNCTION__, __LINE__);*/
	}
	/*dump_list_item(p_item, "after ppa_update_session");*/

	/*  protect before ARP*/
	if ((!(p_item->flags & SESSION_TX_ITF_IPOA_PPPOA_MASK)
			&& p_item->pkt.dst_mac[0] == 0
		&& p_item->pkt.dst_mac[1] == 0
		&& p_item->pkt.dst_mac[2] == 0
		&& p_item->pkt.dst_mac[3] == 0
		&& p_item->pkt.dst_mac[4] == 0
		&& p_item->pkt.dst_mac[5] == 0)
		&& !(flags & PPA_F_SESSION_LOCAL_IN)) {
		SET_DBG_FLAG(p_item, SESSION_DBG_ZERO_DST_MAC);
		ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,"sesion exist for zero mac\n");
		goto __UPDATE_FAIL;
	}
	CLR_DBG_FLAG(p_item, SESSION_DBG_ZERO_DST_MAC);

#if defined(SESSION_STATISTIC_DEBUG) && SESSION_STATISTIC_DEBUG  
	if (__update_session_hash(p_item) != PPA_SUCCESS) {
		SET_DBG_FLAG(p_item, SESSION_DBG_UPDATE_HASH_FAIL);
		ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,"__update_session_hash failed\n");
			goto __UPDATE_FAIL;
	}
	CLR_DBG_FLAG(p_item, SESSION_DBG_UPDATE_HASH_FAIL);
	/*ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,"hash table/id:%d:%d\n", p_item->hash_table_id, p_item->hash_index);*/
#endif

#if IS_ENABLED(CONFIG_PPA_MPE_IP97)
	if (ppa_get_pkt_ip_proto(ppa_buf) == IP_PROTO_ESP) {
		p_item->flag2 |= SESSION_FLAG2_VALID_IPSEC_OUTBOUND_LAN;
	}
#endif
	/* place holder for some explicit criteria based on which the session 
	must go through SW acceleration */
#ifdef CONFIG_PPA_PP_LEARNING
	p_item->pkt.protocol	  = ppa_get_pkt_protocol(ppa_buf);
#endif

#ifdef CONFIG_PPA_PUMA7
	p_item->flag2 |= SESSION_FLAG2_UPDATE_INFO_PROCESSED;
#endif

#ifdef CONFIG_PPA_PUMA7
__UPDATE_PROCESSED:
#endif
	if (ppa_hw_accelerate(ppa_buf, p_item)) {
#ifdef CONFIG_PPA_PP_LEARNING
		ppa_pp_session_create(ppa_buf, p_item);
#endif
		only_SW_acc = 0;
	} else {
		only_SW_acc = 1;
	}
	if ((ret = ppa_hsel_add_routing_session(p_item, ppa_buf, 0)) != PPA_SUCCESS) {
		/*PPE hash full in this hash index, or IPV6 table full ,...*/
		p_item->flag2 |= SESSION_FLAG2_ADD_HW_FAIL;
		ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,"ppa_hw_add_session(p_item) fail\n");
		if (ppa_is_hal_registered_for_all_caps(p_item) != PPA_SUCCESS) {
		/* TODO: In future, need to handle the case in legacy platforms
		* protocol that supports hardware acceleration and not software acceleration
		* but adding session to hardware failed this time, to try again in next cycle*/
			p_item->flags |= SESSION_NOT_ACCELABLE;
		}
		ret = PPA_FAILURE;
		goto __UPDATE_FAIL;
	} else {
		p_item->flag2 &= ~SESSION_FLAG2_ADD_HW_FAIL;
		p_item->flags &= ~SESSION_NOT_ACCEL_FOR_MGM;
	}
	/*ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,"hardware/firmware entry added\n");*/
	ret = PPA_SESSION_ADDED;
	
__UPDATE_FAIL:
	return ret;
}

int32_t ppa_update_session_info(PPA_BUF *ppa_buf, struct uc_session_node *p_item, uint32_t flags)
{
	int32_t ret = PPA_SUCCESS;
	PPA_NETIF *netif;
	PPA_IPADDR ip;
	uint32_t port;
	uint8_t *nat_src_mac;
	uint32_t dscp;
  	struct netif_info *rx_ifinfo=NULL, *tx_ifinfo=NULL;
  	int f_is_ipoa_or_pppoa = 0;
	int qid;
#if defined(L2TP_CONFIG) && L2TP_CONFIG
	PPA_IFNAME new_rxifname[PPA_IF_NAME_SIZE];
	PPA_IFNAME new_txifname[PPA_IF_NAME_SIZE];
	PPA_NETIF *new_rxnetif;
	PPA_NETIF *new_txnetif;
#endif
	int extra_hdr_bytes = 0;
	uint16_t tunnel_hdr_len;
	bool isIgEoGre = false;
	bool isEgEoGre = false;
	uint8_t rx_local=0, tx_local=0;   

#if IS_ENABLED(CONFIG_NAT_LOOP_BACK)
	uint8_t nat_loopback_src = 0;
#endif
	PPA_NETIF *base_netif=NULL;
	PPA_NETIF *temp_netif=NULL;
	uint8_t is_eogre=0;

  	/* If the update session is called for more than 2 X the min hits 
	 	// that means the session cannot be accelerated at this point of time.
	// set the flag to speed up the CPU path traffic. This session will get chance 
	// again when the p_item->num_adds wraps around or by the session management */
	if (p_item->num_adds > 2*g_ppa_min_hits) {
		p_item->flags |= SESSION_NOT_ACCEL_FOR_MGM;
	}

	p_item->tx_if = ppa_get_pkt_dst_if (ppa_buf);
	p_item->mtu  = g_ppa_ppa_mtu; /*reset MTU since later it will be updated*/

#if IS_ENABLED(CONFIG_PPA_TCP_LITEPATH)
  	/*local session = CPU_BOUND */
  	/* or local in sessions txif will be null and for local out session txif will be null*/
	if ((p_item->flag2 & SESSION_FLAG2_CPU_BOUND)) {
		if (flags & PPA_F_SESSION_LOCAL_IN) { 
			rx_local = 1;
		} else { 
			tx_local = 1 ;
		}
	} 
	/*printk(KERN_INFO"p_item->rx_if = %s p_item->tx_if = %s rx_local =%d tx_local =%d\n",(p_item->rx_if ? p_item->rx_if->name : "NULL"),
		(p_item->tx_if ? p_item->tx_if->name : "NULL"), rx_local, tx_local);*/
#endif

  /*
   *  update and get rx/tx information
   */
	if (!tx_local && p_item->rx_if && (ppa_netif_lookup(ppa_get_netif_name(p_item->rx_if), &rx_ifinfo) != PPA_SUCCESS)) {
		ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,
			"failed in getting info structure of rx_if (%s)\n", ppa_get_netif_name(p_item->rx_if));
		SET_DBG_FLAG(p_item, SESSION_DBG_RX_IF_NOT_IN_IF_LIST);
		return PPA_ENOTPOSSIBLE;
	}
	if (!rx_local && p_item->tx_if && (ppa_netif_lookup(ppa_get_netif_name(p_item->tx_if), &tx_ifinfo) != PPA_SUCCESS)) {
		ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,
			"failed in getting info structure of tx_if (%s)\n", ppa_get_netif_name(p_item->tx_if));
		SET_DBG_FLAG(p_item, SESSION_DBG_TX_IF_NOT_IN_IF_LIST);
		ppa_netif_put(rx_ifinfo);
		return PPA_ENOTPOSSIBLE;
	}

	if (!(p_item->rx_if) && p_item->tx_if && ppa_dev_is_loopback(p_item->tx_if)) {
		ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT, "Loopback session , cannot add to sw acceleration\n");
		ppa_netif_put(tx_ifinfo);
		return PPA_ENOTPOSSIBLE;
	}

#if IS_ENABLED(CONFIG_SOC_GRX500)
#if defined(PPA_IF_MIB) && PPA_IF_MIB
	if (rx_ifinfo && rx_ifinfo->sub_if_index > 0)
		p_item->is_rx_logical = true;
	if (tx_ifinfo && tx_ifinfo->sub_if_index > 0)
		p_item->is_tx_logical = true;
#endif
#endif

  /*
   *  PPPoE is highest level, collect PPPoE information
   */
	p_item->flags &= ~SESSION_VALID_PPPOE;
	if (rx_ifinfo && ((rx_ifinfo->flags & (NETIF_WAN_IF | NETIF_PPPOE)) == (NETIF_WAN_IF | NETIF_PPPOE))) {
	/*src interface is WAN and PPPoE*/
		p_item->pkt.pppoe_session_id = rx_ifinfo->pppoe_session_id;
		p_item->flags |= SESSION_VALID_PPPOE;
		SET_DBG_FLAG(p_item, SESSION_DBG_RX_PPPOE);
	}

	/*if destination interface is PPPoE, it covers the previous setting*/
	if (tx_ifinfo && ((tx_ifinfo->flags & (NETIF_WAN_IF | NETIF_PPPOE)) == (NETIF_WAN_IF | NETIF_PPPOE))) {
		if ((p_item->flags & SESSION_VALID_PPPOE))
			ppa_debug(DBG_ENABLE_MASK_ASSERT, "both interfaces are WAN PPPoE interface, not possible\n");
			p_item->pkt.pppoe_session_id = tx_ifinfo->pppoe_session_id;
			p_item->flags |= SESSION_VALID_PPPOE;
			SET_DBG_FLAG(p_item, SESSION_DBG_TX_PPPOE);
			/*adjust MTU to ensure ethernet frame size does not exceed 1518(without VLAN)*/
			extra_hdr_bytes += 8;
	}
#if defined(L2TP_CONFIG) && L2TP_CONFIG
	if (rx_ifinfo && ((rx_ifinfo->flags & (NETIF_WAN_IF | NETIF_PPPOL2TP)) == (NETIF_WAN_IF | NETIF_PPPOL2TP))) {
	/* src interface is WAN and PPPOL2TP */
		p_item->pkt.pppol2tp_session_id = rx_ifinfo->pppol2tp_session_id; /*Add tunnel id also*/
		p_item->pkt.pppol2tp_tunnel_id = rx_ifinfo->pppol2tp_tunnel_id;
		p_item->flags |= SESSION_VALID_PPPOL2TP;
		SET_DBG_FLAG(p_item, SESSION_DBG_RX_PPPOL2TP);
		if (ppa_pppol2tp_get_base_netif(p_item->rx_if, new_rxifname) == PPA_SUCCESS) {
			new_rxnetif = ppa_get_netif(new_rxifname);
			if (ppa_check_is_ppp_netif(new_rxnetif)) {
				if (ppa_check_is_pppoe_netif(new_rxnetif)) {
					p_item->pkt.pppoe_session_id = rx_ifinfo->pppoe_session_id;
					p_item->flags |= SESSION_VALID_PPPOE;
					SET_DBG_FLAG(p_item, SESSION_DBG_RX_PPPOE);
				}
			}
		}
	}

	if (tx_ifinfo && ((tx_ifinfo->flags & (NETIF_WAN_IF | NETIF_PPPOL2TP)) == (NETIF_WAN_IF | NETIF_PPPOL2TP))) {
		if ((p_item->flags & SESSION_VALID_PPPOL2TP))
			ppa_debug(DBG_ENABLE_MASK_ASSERT, "both interfaces are WAN PPPoL2TP interface, not possible\n");
		p_item->pkt.pppol2tp_session_id = tx_ifinfo->pppol2tp_session_id;
		p_item->pkt.pppol2tp_tunnel_id = tx_ifinfo->pppol2tp_tunnel_id;
		p_item->flags |= SESSION_VALID_PPPOL2TP;
		SET_DBG_FLAG(p_item, SESSION_DBG_TX_PPPOL2TP);
		extra_hdr_bytes += 40; 

		if (ppa_pppol2tp_get_base_netif(p_item->tx_if, new_txifname) == PPA_SUCCESS) {
			new_txnetif = ppa_get_netif(new_txifname);
			if (ppa_check_is_ppp_netif(new_txnetif)) {
				if (ppa_check_is_pppoe_netif(new_txnetif)) {
					p_item->pkt.pppoe_session_id = tx_ifinfo->pppoe_session_id;
					p_item->flags |= SESSION_VALID_PPPOE;
					SET_DBG_FLAG(p_item, SESSION_DBG_TX_PPPOE);
					/*adjust MTU to ensure ethernet frame size does not exceed 1518(without VLAN)*/
				}
			}
		}
	}
#endif /*defined(L2TP_CONFIG) && L2TP_CONFIG*/

	/**  detect bridge and get the real effective device under this bridge
	*  do not support VLAN interface created on bridge*/
	if (rx_ifinfo && ((rx_ifinfo->flags & (NETIF_BRIDGE | NETIF_PPPOE)) == NETIF_BRIDGE)) {
  	/*  can't handle PPPoE over bridge properly, because src mac info is corrupted*/
		if (!(rx_ifinfo->flags & NETIF_PHY_IF_GOT)
				|| (netif = ppa_get_netif_by_net(rx_ifinfo->net, rx_ifinfo->phys_netif_name)) == NULL) {
			ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT, "failed in get underlying interface of PPPoE interface(RX)\n");
			ret = PPA_ENOTPOSSIBLE;
			goto PPA_UPDATE_SESSION_DONE_SHOTCUT;
		}
		while ((rx_ifinfo->flags & NETIF_BRIDGE)) {
			if ((ret = ppa_get_br_dst_port_with_mac(netif, p_item->pkt.src_mac, &netif)) != PPA_SUCCESS) {
				SET_DBG_FLAG(p_item, SESSION_DBG_SRC_BRG_IF_NOT_IN_BRG_TBL);
				ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,"ppa_get_br_dst_port_with_mac fail\n");
				if (ret != PPA_EAGAIN)
					ret = PPA_FAILURE;
				goto PPA_UPDATE_SESSION_DONE_SHOTCUT;
			} else {
				p_item->br_rx_if = netif;
				CLR_DBG_FLAG(p_item, SESSION_DBG_SRC_BRG_IF_NOT_IN_BRG_TBL);
			}
			ppa_netif_put(rx_ifinfo);

			if (ppa_netif_lookup(ppa_get_netif_name(netif), &rx_ifinfo) != PPA_SUCCESS) {
				ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,
					"failed in getting info structure of dst_rx_if (%s)\n", ppa_get_netif_name(netif));
				SET_DBG_FLAG(p_item, SESSION_DBG_SRC_IF_NOT_IN_IF_LIST);
				ppa_netif_put(tx_ifinfo);
				return PPA_ENOTPOSSIBLE;
			}
		}
	}

	if (tx_ifinfo && (tx_ifinfo->flags & NETIF_BRIDGE)) {
		if (!(tx_ifinfo->flags & NETIF_PHY_IF_GOT) ||
				(netif = ppa_get_netif_by_net(tx_ifinfo->net, tx_ifinfo->phys_netif_name)) == NULL) {
			ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,
				"failed in get underlying interface of PPPoE interface(TX)\n");
			ret = PPA_ENOTPOSSIBLE;
			goto PPA_UPDATE_SESSION_DONE_SHOTCUT;
		}
		while ((tx_ifinfo->flags & NETIF_BRIDGE)) {
			if ((ret = ppa_get_br_dst_port(netif, ppa_buf, &netif)) != PPA_SUCCESS) {
				SET_DBG_FLAG(p_item, SESSION_DBG_DST_BRG_IF_NOT_IN_BRG_TBL);
				ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,"ppa_get_br_dst_port fail\n");
				if (ret != PPA_EAGAIN)
					ret = PPA_FAILURE;
				goto PPA_UPDATE_SESSION_DONE_SHOTCUT;
			} else {
				p_item->br_tx_if = netif;
				CLR_DBG_FLAG(p_item, SESSION_DBG_DST_BRG_IF_NOT_IN_BRG_TBL);
			}

			ppa_netif_put(tx_ifinfo);

			if (ppa_netif_lookup(ppa_get_netif_name(netif), &tx_ifinfo) != PPA_SUCCESS) {
				ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,
					"failed in getting info structure of dst_tx_if (%s)\n", ppa_get_netif_name(netif));
				SET_DBG_FLAG(p_item, SESSION_DBG_DST_IF_NOT_IN_IF_LIST);
			 				ppa_netif_put(rx_ifinfo);
				return PPA_ENOTPOSSIBLE;
			}
		}
	}

	/**  check whether physical port is determined or not */
	if (((tx_ifinfo && !(tx_ifinfo->flags & NETIF_PHYS_PORT_GOT)) ||
		(rx_ifinfo && !(rx_ifinfo->flags & NETIF_PHYS_PORT_GOT)))) {
		p_item->flags |= SESSION_NOT_VALID_PHY_PORT;
		ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,"tx no NETIF_PHYS_PORT_GOT\n");
	}

	if (p_item->flag2 & SESSION_FLAG2_VETH) {
		if ((tx_ifinfo && !(tx_ifinfo->flags & NETIF_VETH)) || (p_item->tx_if && ppa_dev_is_loopback(p_item->tx_if))) {
			p_item->flag2 &= ~SESSION_FLAG2_VETH;
			p_item->flags &= ~SESSION_LAN_ENTRY;
			p_item->flags &= ~SESSION_WAN_ENTRY;
			p_item->flags &= ~SESSION_NOT_VALID_PHY_PORT;
			ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,"unsetting Veth NETIF_PHYS_PORT_GOT\n");
			ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT, "Tx is not a VETH device. So Unsetting it. Session Can now be SW accelerated\n");
		}
	} else {
		if ((tx_ifinfo && (tx_ifinfo->flags & NETIF_VETH))) {
			p_item->flag2 |= SESSION_FLAG2_VETH;
			p_item->flag2 |= SESSION_FLAG2_CONTAINER;
			ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT, "Tx is a VETH device. So It should not be SW accelerated. Container Session\n");
		}
	}

	/**  decide which table to insert session, LAN side table or WAN side table */
	if (rx_ifinfo) {
		if ((rx_ifinfo->flags & (NETIF_LAN_IF | NETIF_WAN_IF)) == (NETIF_LAN_IF | NETIF_WAN_IF)) {
			if (tx_ifinfo) {
				switch (tx_ifinfo->flags & (NETIF_LAN_IF | NETIF_WAN_IF)) {
				case NETIF_LAN_IF:
					p_item->flags |= SESSION_WAN_ENTRY;
					break;
				case NETIF_WAN_IF:
					p_item->flags |= SESSION_LAN_ENTRY;
					break;
				default:
					ret = PPA_FAILURE;
					ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT, "tx_ifinfo->flags wrong LAN/WAN flag\n");
					goto PPA_UPDATE_SESSION_DONE_SHOTCUT;
				}
			} else {
				p_item->flags |= SESSION_WAN_ENTRY;
			}	
		} else {
			switch (rx_ifinfo->flags & (NETIF_LAN_IF | NETIF_WAN_IF)) {
			case NETIF_LAN_IF:
				p_item->flags |= SESSION_LAN_ENTRY;
				break;
			case NETIF_WAN_IF:
				p_item->flags |= SESSION_WAN_ENTRY;
				break;
			default:
				ret = PPA_FAILURE;
				ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT, "rx_ifinfo->flags wrong LAN/WAN flag\n");
				goto PPA_UPDATE_SESSION_DONE_SHOTCUT;
			}
		}
	} else {
		if (tx_ifinfo) {
			switch (tx_ifinfo->flags & (NETIF_LAN_IF | NETIF_WAN_IF)) {
			case NETIF_LAN_IF:
				p_item->flags |= SESSION_WAN_ENTRY;
				break;
			case NETIF_WAN_IF:
				p_item->flags |= SESSION_LAN_ENTRY;
				break;
			default:
				ret = PPA_FAILURE;
				ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT, "tx_ifinfo->flags wrong LAN/WAN flag\n");
				goto PPA_UPDATE_SESSION_DONE_SHOTCUT;
			}
		}
	}
#if defined(CONFIG_PPA_HANDLE_CONNTRACK_SESSIONS)
	ppa_decide_collision(p_item);
#endif
 
	/**  Check for tunnel */
	tunnel_hdr_len = 0; 
	if ((!rx_local && ppa_netif_type(p_item->tx_if) == ARPHRD_SIT)
		|| (!tx_local && ppa_netif_type(p_item->rx_if) == ARPHRD_SIT)) {

		p_item->flags |= SESSION_TUNNEL_6RD;
		tunnel_hdr_len = 20; 
#if defined(PPA_DSLITE) && PPA_DSLITE
	} else if ((!rx_local && ppa_netif_type(p_item->tx_if) == ARPHRD_TUNNEL6) ||
		(!tx_local && ppa_netif_type(p_item->rx_if) == ARPHRD_TUNNEL6)) {

		p_item->flags |= SESSION_TUNNEL_DSLITE;
		tunnel_hdr_len = 40; 
#endif
	}

	if (rx_ifinfo && (rx_ifinfo->flags & NETIF_GRE_TUNNEL)) {
		ppa_set_GreSession(p_item);
		p_item->flag2 |= SESSION_FLAG2_IG_GRE;
		isIgEoGre = ((rx_ifinfo->greInfo.flowId == FLOWID_IPV4_EoGRE) ||
			     (rx_ifinfo->greInfo.flowId == FLOWID_IPV6_EoGRE)) ? true : false;
		temp_netif = p_item->rx_if;
		if (ppa_get_underlying_vlan_interface_if_eogre(p_item->rx_if,
			&base_netif, &is_eogre) != PPA_SUCCESS) {
			SET_DBG_FLAG(p_item, SESSION_DBG_RX_VLAN_CANT_SUPPORT);
		}
		//ppa_get_gre_hdrlen(base_netif,&tunnel_hdr_len);
		p_item->rx_if = temp_netif;
	}

	if( tx_ifinfo && (tx_ifinfo->flags & NETIF_GRE_TUNNEL)) {
		ppa_set_GreSession(p_item);
		p_item->flag2 |= SESSION_FLAG2_EG_GRE;
		isEgEoGre = ((tx_ifinfo->greInfo.flowId == FLOWID_IPV4_EoGRE) ||
			     (tx_ifinfo->greInfo.flowId == FLOWID_IPV6_EoGRE)) ? true : false;
		temp_netif = p_item->tx_if;
		if(ppa_get_underlying_vlan_interface_if_eogre(p_item->tx_if, 
					&base_netif, &is_eogre) != PPA_SUCCESS) {
			SET_DBG_FLAG(p_item, SESSION_DBG_RX_VLAN_CANT_SUPPORT);
		}
		ppa_get_gre_hdrlen(base_netif,&tunnel_hdr_len);
		if (isEgEoGre) {
			tunnel_hdr_len += 14;
		}
		p_item->tx_if = temp_netif;
	}

	if (ppa_is_LanSession(p_item) && tunnel_hdr_len)
		extra_hdr_bytes += tunnel_hdr_len;

	/**  collect VLAN information(outer/inner)*/
	/* do not support VLAN interface created on bridge*/
	if ((rx_ifinfo && (rx_ifinfo->flags & NETIF_VLAN_CANT_SUPPORT)) || 
		(tx_ifinfo && (tx_ifinfo->flags & NETIF_VLAN_CANT_SUPPORT))) {
		ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,"physical interface has limited VLAN support\n");
		p_item->flags |= SESSION_NOT_ACCELABLE;

		if (rx_ifinfo && (rx_ifinfo->flags & NETIF_VLAN_CANT_SUPPORT))
			SET_DBG_FLAG(p_item, SESSION_DBG_RX_VLAN_CANT_SUPPORT);
		else
			SET_DBG_FLAG(p_item, SESSION_DBG_TX_VLAN_CANT_SUPPORT);
		goto PPA_UPDATE_SESSION_DONE_SHOTCUT;
	}
	CLR_DBG_FLAG(p_item, SESSION_DBG_RX_VLAN_CANT_SUPPORT);
	CLR_DBG_FLAG(p_item, SESSION_DBG_TX_VLAN_CANT_SUPPORT);

	if (rx_ifinfo && (rx_ifinfo->flags & NETIF_VLAN_OUTER))
		p_item->flags |= SESSION_VALID_OUT_VLAN_RM;
	if (tx_ifinfo && (tx_ifinfo->flags & NETIF_VLAN_OUTER)) {
		if (tx_ifinfo->out_vlan_netif == NULL) {
			p_item->pkt.out_vlan_tag = tx_ifinfo->outer_vid; /*  ignore prio and cfi */
		} else {
			p_item->pkt.out_vlan_tag = (tx_ifinfo->outer_vid & PPA_VLAN_TAG_MASK)
				| ppa_vlan_dev_get_egress_qos_mask(tx_ifinfo->out_vlan_netif, ppa_buf);
		}
		p_item->flags |= SESSION_VALID_OUT_VLAN_INS;
	}

	if (rx_ifinfo && (rx_ifinfo->flags & NETIF_VLAN_INNER))
		p_item->flags |= SESSION_VALID_VLAN_RM;
	if (tx_ifinfo && (tx_ifinfo->flags & NETIF_VLAN_INNER)) {
		if (tx_ifinfo->in_vlan_netif == NULL) {
			p_item->pkt.new_vci = tx_ifinfo->inner_vid;/*  ignore prio and cfi*/
		} else {
			p_item->pkt.new_vci = (tx_ifinfo->inner_vid & PPA_VLAN_TAG_MASK)
				| ppa_vlan_dev_get_egress_qos_mask(tx_ifinfo->in_vlan_netif, ppa_buf);
		}
		p_item->flags |= SESSION_VALID_VLAN_INS;
	}
	/*  decide destination list
	**  if tx interface is based on DSL, determine which PVC it is(QID)*/
	if (tx_ifinfo) {
		PPA_SKBUF fake_skb;
		p_item->dest_ifid = tx_ifinfo->phys_port;
#if defined(CONFIG_SOC_GRX500) && CONFIG_SOC_GRX500
		if (rx_ifinfo)
			p_item->src_ifid = rx_ifinfo->phys_port;
		if ((tx_ifinfo->flags & NETIF_VLAN) ||
			(tx_ifinfo->flags & NETIF_PHY_ATM) || (tx_ifinfo->flags & NETIF_DIRECTPATH)) {
			p_item->dest_subifid = tx_ifinfo->subif_id;
		}
#endif
		if ((tx_ifinfo->flags & NETIF_PHY_ATM)) {
			fake_skb.priority = ppa_get_pkt_priority(ppa_buf);
			qid = ppa_drv_get_netif_qid_with_pkt(&fake_skb, tx_ifinfo->vcc, 1);
			if (qid >= 0)
				p_item->dslwan_qid = qid;
			else
				p_item->dslwan_qid = tx_ifinfo->dslwan_qid;
			p_item->flags |= SESSION_VALID_DSLWAN_QID;
			if ((tx_ifinfo->flags & NETIF_EOA)) {
				SET_DBG_FLAG(p_item, SESSION_DBG_TX_BR2684_EOA);
			} else if ((tx_ifinfo->flags & NETIF_IPOA)) {
				p_item->flags |= SESSION_TX_ITF_IPOA;
				SET_DBG_FLAG(p_item, SESSION_TX_ITF_IPOA);
				f_is_ipoa_or_pppoa = 1;
			} else if ((tx_ifinfo->flags & NETIF_PPPOATM)) {
				p_item->flags |= SESSION_TX_ITF_PPPOA;
				SET_DBG_FLAG(p_item, SESSION_TX_ITF_PPPOA);
				f_is_ipoa_or_pppoa = 1;
			}
		} else {
			netif = ppa_get_netif_by_net(tx_ifinfo->net, tx_ifinfo->phys_netif_name);
			fake_skb.priority = ppa_get_pkt_priority(ppa_buf);
			qid = ppa_drv_get_netif_qid_with_pkt(&fake_skb, netif, 0);
			if (qid >= 0) {
				p_item->dslwan_qid = qid;
				p_item->flags |= SESSION_VALID_DSLWAN_QID;
			}
		}
	}
	/**  collect src IP/Port, dest IP/Port information*/
	/* only port change with same IP not supported here, not really useful*/
	ppa_memset(&p_item->pkt.natip, 0, sizeof(p_item->pkt.natip));
	p_item->pkt.nat_port= 0;
	ppa_get_pkt_src_ip(&ip, ppa_buf);
	if (ppa_memcmp(&ip, &p_item->pkt.src_ip, ppa_get_pkt_ip_len(ppa_buf)) != 0) {
#if IS_ENABLED(CONFIG_NAT_LOOP_BACK)
		p_item->pkt.nat_src_ip = ip;
		nat_loopback_src = 1;
#endif
		if (p_item->flags & SESSION_LAN_ENTRY) {
			p_item->pkt.nat_ip = ip;
			p_item->flags |= SESSION_VALID_NAT_IP;
		} else {  /*cannot acclerate*/
			ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,"WAN Session cannot edit src ip\n");
			/*p_item->flags |= SESSION_NOT_ACCELABLE;
			SET_DBG_FLAG(p_item, SESSION_DBG_PPE_EDIT_LIMIT);*/
			ret = PPA_EAGAIN;
			goto PPA_UPDATE_SESSION_DONE_SHOTCUT;
		}
	}

	port = ppa_get_pkt_src_port(ppa_buf);
	if (port != p_item->pkt.src_port) {
		if (p_item->flags & SESSION_LAN_ENTRY) {
			p_item->pkt.nat_port = port;
			p_item->flags |= SESSION_VALID_NAT_PORT | SESSION_VALID_NAT_SNAT;
		} else {  /*cannot acclerate*/
			ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,"WAN Session cannot edit src port\n");
			/*p_item->flags |= SESSION_NOT_ACCELABLE;
			SET_DBG_FLAG(p_item, SESSION_DBG_PPE_EDIT_LIMIT);*/
			ret = PPA_EAGAIN;
			goto PPA_UPDATE_SESSION_DONE_SHOTCUT;
		}
	}	

#if IS_ENABLED(CONFIG_PPA_MPE_IP97)
	if (ppa_get_pkt_ip_proto(ppa_buf) != IP_PROTO_ESP) {
#endif
		ppa_get_pkt_dst_ip(&ip, ppa_buf);
		if (ppa_memcmp(&ip, &p_item->pkt.dst_ip, ppa_get_pkt_ip_len(ppa_buf)) != 0) {
#if IS_ENABLED(CONFIG_NAT_LOOP_BACK)
			p_item->pkt.nat_dst_ip = ip;
			if (nat_loopback_src) {
				p_item->flag2 |= SESSION_FLAG2_VALID_NAT_LOOPBACK; 
				p_item->flags &= (~SESSION_VALID_NAT_IP);
			} else
#endif
			{
				if ((p_item->flags & SESSION_WAN_ENTRY) && (ppa_zero_ip(p_item->pkt.natip.natsrcip.ip))) {
					p_item->pkt.natip.natsrcip.ip = ip;
					p_item->flags |= SESSION_VALID_NAT_IP;
				} else { /*cannot accelerate*/
					ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,"LAN Session cannot edit dst ip\n");
					p_item->flags |= SESSION_NOT_ACCELABLE;
					SET_DBG_FLAG(p_item, SESSION_DBG_PPE_EDIT_LIMIT);
					ret = PPA_EAGAIN;
					goto PPA_UPDATE_SESSION_DONE_SHOTCUT;
				}
			}
		}
		port = ppa_get_pkt_dst_port(ppa_buf);
		if ((port  != p_item->pkt.dst_port) && (!p_item->pkt.nat_port)) {
			if (p_item->flags & SESSION_WAN_ENTRY) {
				p_item->pkt.nat_port = port;
				p_item->flags |= SESSION_VALID_NAT_PORT;
			} else { /*cannot accelerate*/
				ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,"LAN Session cannot edit dst port\n");
				p_item->flags |= SESSION_NOT_ACCELABLE;
				SET_DBG_FLAG(p_item, SESSION_DBG_PPE_EDIT_LIMIT);
				ret = PPA_EAGAIN;
				goto PPA_UPDATE_SESSION_DONE_SHOTCUT;
			}
		}

		/** check for l2 source nat in bridged sessions*/
		if (ppa_is_BrSession(p_item)) {
			nat_src_mac = ppa_get_pkt_src_mac_ptr(ppa_buf);
			if (nat_src_mac && ppa_memcmp(nat_src_mac, p_item->pkt.src_mac, ETH_ALEN)) {
				ppa_memcpy(p_item->pkt.nat_src_mac, nat_src_mac, ETH_ALEN);
				p_item->flag2 |= SESSION_FLAG2_VALID_L2_SNAT;
			}
		}
		CLR_DBG_FLAG(p_item, SESSION_DBG_PPE_EDIT_LIMIT);
#if defined(CONFIG_PPA_MPE_IP97)
	}
#endif
	if (tx_ifinfo && (tx_ifinfo->flags & (NETIF_LAN_IF | NETIF_PPPOE))
		== (NETIF_LAN_IF | NETIF_PPPOE)) { /*cannot accelerate*/
		ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,"Non-WAN Session cannot add ppp header\n");
		p_item->flags |= SESSION_NOT_ACCELABLE;
		ret = PPA_EAGAIN;
		goto PPA_UPDATE_SESSION_DONE_SHOTCUT;
	}

	/**  calculate new DSCP value if necessary*/
	dscp = ppa_get_pkt_ip_tos(ppa_buf);
	if (dscp != p_item->pkt.ip_tos) {
		p_item->pkt.new_dscp = dscp >> 2;
		p_item->flags |= SESSION_VALID_NEW_DSCP;
	}

	/**  IPoA/PPPoA does not have MAC address*/
	if (f_is_ipoa_or_pppoa)
		goto PPA_UPDATE_SESSION_DONE_SKIP_MAC;

	/* Updated the 6rd destination address */
	if (p_item->flags & SESSION_LAN_ENTRY  && (p_item->flags & SESSION_TUNNEL_6RD)) {
		if (!rx_local && p_item->tx_if)
			p_item->pkt.sixrd_daddr = ppa_get_6rdtunel_dst_ip(ppa_buf,p_item->tx_if);
		}


	/**  get new dest MAC address for ETH, EoA*/
  	CLR_DBG_FLAG(p_item, SESSION_DBG_GET_DST_MAC_FAIL);
	if ( (isIgEoGre && ppa_is_BrSession(p_item)) || isEgEoGre ) {
	/*
	* It is special handling for EoGRE. The destination MAC should be outer MAC
	* Note: The IPv6 EoGRE sessions have conntrack, so session is not marked
	* as bridged sessions. The general rule- All bridged sessions conntrack
	* is NULL (that is p_item->session) needs to be relooked. */ 
		if ( ppa_is_EgressGreSession(p_item) ) {
			temp_netif = ppa_buf->dev;
			if (ppa_is_BrSession(p_item)) {
				if(ppa_get_underlying_vlan_interface_if_eogre(p_item->tx_if,
					&base_netif, &is_eogre) == PPA_SUCCESS)
					ppa_buf->dev = base_netif;

				if(ppa_get_dst_mac(ppa_buf, NULL, p_item->pkt.dst_mac, 0) != PPA_SUCCESS ) {
					ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,
						"session:%x can not get dst mac!\n", (u32)ppa_get_session(ppa_buf));
					SET_DBG_FLAG(p_item, SESSION_DBG_GET_DST_MAC_FAIL);
					ret = PPA_EAGAIN;
				}
			} else {
				if(ppa_get_underlying_vlan_interface_if_eogre(p_item->br_tx_if,
					&base_netif, &is_eogre) == PPA_SUCCESS)
					ppa_buf->dev = base_netif;

				if (ppa_get_gre_dmac(p_item->pkt.dst_mac, p_item->br_tx_if, ppa_buf)) {
					ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,
						"session:%x can not get dst mac!\n", (u32)ppa_get_session(ppa_buf));
					SET_DBG_FLAG(p_item, SESSION_DBG_GET_DST_MAC_FAIL);
					ret = PPA_EAGAIN;
				}
			}
			ppa_buf->dev = temp_netif;
		} else {
			ppa_get_pkt_rx_dst_mac_addr(ppa_buf,p_item->pkt.dst_mac);
		}
	} else if (p_item->session == NULL) {
		ppa_get_pkt_rx_dst_mac_addr(ppa_buf,p_item->pkt.dst_mac);
	} else if (ppa_get_dst_mac(ppa_buf, p_item->session, p_item->pkt.dst_mac, p_item->pkt.sixrd_daddr)
			!= PPA_SUCCESS) {
		ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,
			"session:%x can not get dst mac!\n", (u32)ppa_get_session(ppa_buf));
		SET_DBG_FLAG(p_item, SESSION_DBG_GET_DST_MAC_FAIL);
		ret = PPA_EAGAIN;
	}

	/* For VETH sessions(tx or rx is VETH device) Return Success */
	if (p_item->flag2 & SESSION_FLAG2_VETH)  {
		ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT, "Skipping veth session\n");
		ret = PPA_SUCCESS;
		goto PPA_UPDATE_SESSION_DONE_SHOTCUT;
	}

#if IS_ENABLED(CONFIG_INTEL_IPQOS_MPE_DS_ACCEL)
	if (p_item->flags & SESSION_WAN_ENTRY) {
#if IS_ENABLED(CONFIG_NETWORK_EXTMARK)
		/* DS_QOS valid only for WAN sessions */
		if(p_item->pkt.extmark & SESSION_DS_QOS_SET) {
			p_item->flag2 |= SESSION_FLAG2_DS_MPE_QOS;
		}
#else /* CONFIG_NETWORK_EXTMARK */
		/* DS_QOS valid only for WAN sessions */
		if(p_item->pkt.mark & SESSION_DS_QOS_SET) {
			p_item->flag2 |= SESSION_FLAG2_DS_MPE_QOS;
		}
#endif /* CONFIG_NETWORK_EXTMARK */
	}
#endif /* CONFIG_INTEL_IPQOS_MPE_DS_ACCEL */

#if defined(CONFIG_PPA_API_DIRECTCONNECT) && CONFIG_PPA_API_DIRECTCONNECT
	/* For directconnect destif, get destination SubifId={VapId, StationId} corresponding to dst_mac */
	if(tx_ifinfo && (tx_ifinfo->flags & NETIF_DIRECTCONNECT) ) {
		PPA_NETIF *dp_netif = tx_ifinfo->netif;
		PPA_DP_SUBIF dp_subif = {0};

		/* Use phys interface <wlanX> for any logical interface on top of it. */
		if (tx_ifinfo->flags & NETIF_PHY_IF_GOT)
			dp_netif = ppa_get_netif(tx_ifinfo->phys_netif_name);

		dp_subif.port_id = tx_ifinfo->phys_port;
		dp_subif.subif = tx_ifinfo->subif_id;
		if(DP_SUCCESS == dp_get_netif_subifid(dp_netif, NULL, NULL, p_item->pkt.dst_mac, &dp_subif, 0)) {
			p_item->dest_subifid = dp_subif.subif;
		} else {
			ret = PPA_FAILURE;
			ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,"unable to find the subif info\n");
			goto PPA_UPDATE_SESSION_DONE_SHOTCUT;
		}
	}
#endif
	if (!rx_local && p_item->tx_if)
		p_item->mtu =  ppa_get_mtu(p_item->tx_if);
  	p_item->mtu -= extra_hdr_bytes;

PPA_UPDATE_SESSION_DONE_SKIP_MAC:
PPA_UPDATE_SESSION_DONE_SHOTCUT:
	ppa_netif_put(rx_ifinfo);
  	ppa_netif_put(tx_ifinfo);
  	return ret;
}

int32_t ppa_alloc_session( PPA_BUF *ppa_buf, PPA_SESSION *p_session,

			uint32_t flags, struct uc_session_node **pp_item)
{
	struct uc_session_node *p_item;
	int32_t ret = PPA_FAILURE;
	PPA_SYSINFO sysinfo;
	PPA_TUPLE tuple;
	uint32_t is_reply = (flags & PPA_F_SESSION_REPLY_DIR) ? 1 : 0;

	struct netif_info *rx_ifinfo = NULL;
	int32_t hdr_offset, flg2=0; 
	uint16_t hdrlen =0;

	PPA_NETIF *base_netif = NULL, *rxif=NULL;
	PPA_IFNAME underlying_intname[PPA_IF_NAME_SIZE];

	if (sys_mem_check_flag)  {
		ppa_si_meminfo(&sysinfo);		

#ifndef CONFIG_SWAP
		ppa_si_swapinfo(&sysinfo);
		if( K(ppa_si_freeram(&sysinfo)) <= stop_session_add_threshold_mem) {
			err("System memory too low: %lu K bytes\n", K(ppa_si_freeram(&sysinfo));
			return PPA_ENOMEM;
		}
#endif
	}

	ppa_session_list_lock();
	rxif = ppa_get_pkt_src_if (ppa_buf);
  
#if IS_ENABLED(CONFIG_PPA_TCP_LITEPATH)
	/* local tcp traffic originated from CPE.
	** we need to mark this session item as SESSION_FLAG2_CPU_OUT as that it can be checked later for taking action*/
	if (ppa_is_pkt_host_output(ppa_buf) && (ppa_get_pkt_ip_proto(ppa_buf) == PPA_IPPROTO_TCP)) { 
		flg2 |= SESSION_FLAG2_CPU_OUT | SESSION_FLAG2_ADD_HW_FAIL;   /*this is a local session which cant be LRO accelerated*/
	} else 
#endif /* CONFIG_PPA_TCP_LITEPATH*/
	if (rxif) {
		if (PPA_SUCCESS !=
			ppa_netif_lookup(ppa_get_netif_name(rxif), &rx_ifinfo)) {

			ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,
				"failed in getting info structure of rx_if (%s)\n", ppa_get_netif_name(rxif));
			ret = PPA_ENOTPOSSIBLE;
			goto __ADD_SESSION_DONE;
		}
		if (rx_ifinfo->netif == NULL) {
			ppa_debug(DBG_ENABLE_MASK_ERR,"failed in fetch interface\n");
			ppa_netif_put(rx_ifinfo);
			goto __ADD_SESSION_DONE;
		}

		if (rx_ifinfo->flags & NETIF_VETH) {
			ppa_netif_put(rx_ifinfo);
			goto __ADD_SESSION_DONE;
		}
	}

	if (p_session)
		ret = __ppa_session_find_by_ct(p_session, is_reply, pp_item);
	else
		ret = __ppa_find_session_from_skb(ppa_buf,0, pp_item);

	if (PPA_SESSION_EXISTS == ret) {
		ret = PPA_SUCCESS;
		ppa_netif_put(rx_ifinfo);
		goto __ADD_SESSION_DONE;
	}

	p_item = ppa_session_alloc_item();
	if (!p_item) {
		ret = PPA_ENOMEM;
		ppa_debug(DBG_ENABLE_MASK_ERR,"failed in memory allocation\n");
		ppa_netif_put(rx_ifinfo);
		goto __ADD_SESSION_DONE;
	}
	/*dump_list_item(p_item, "ppa_alloc_session (after init)");*/

	p_item->session = p_session;
	if ((flags & PPA_F_SESSION_REPLY_DIR))
		p_item->flags	|= SESSION_IS_REPLY;

	/*ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,
		"ppa_get_session(ppa_buf) = %08X\n",(uint32_t)ppa_get_session(ppa_buf));*/

#ifdef CONFIG_PPA_PP_LEARNING
	p_item->pkt.protocol	  = ppa_get_pkt_protocol(ppa_buf);
#endif
	p_item->pkt.ip_proto	  = ppa_get_pkt_ip_proto(ppa_buf);
#if defined(CONFIG_PPA_MPE_IP97)
	if (p_item->pkt.ip_proto == PPA_IPPROTO_TCP)
		p_item->flags |= SESSION_IS_TCP;
#endif
	p_item->pkt.ip_tos	= ppa_get_pkt_ip_tos(ppa_buf);
	ppa_get_pkt_src_ip(&p_item->pkt.src_ip, ppa_buf);
	p_item->pkt.src_port	= ppa_get_pkt_src_port(ppa_buf);
	ppa_get_pkt_dst_ip(&p_item->pkt.dst_ip, ppa_buf);
	p_item->pkt.dst_port	= ppa_get_pkt_dst_port(ppa_buf);
	p_item->rx_if		= rxif; 
	p_item->timeout	   	= ppa_get_default_session_timeout();
	p_item->last_hit_time 	= ppa_get_time_in_sec();

	p_item->flag2 |= flg2;

	if (p_session)
		p_item->hash = ppa_get_hash_from_ct(p_session, is_reply?1:0,&tuple);
	else {
		ppa_get_hash_from_packet(ppa_buf,0, &p_item->hash, &tuple);
		ppa_set_BrSession(p_item);
		ppa_session_br_timer_init(p_item);
	}

	if (p_item->rx_if && rx_ifinfo) {
		hdr_offset = PPA_ETH_HLEN; 
		if (rx_ifinfo->flags & NETIF_WAN_IF) {
			if (rx_ifinfo->phys_port != ETH_WAN_PORT_ID) {
			    p_item->flag2 |= SESSION_FLAG2_NON_ETHWAN_SESSION;
			}

			if (rx_ifinfo->flags & NETIF_PPPOE) {
				hdr_offset += PPPOE_SES_HLEN ; /*PPPoE*/
			}
			if (rx_ifinfo->flags & NETIF_PPPOL2TP) {
				hdr_offset += 38; 
			}
			/* 6rd interface*/
			if (ppa_netif_type(rx_ifinfo->netif) == ARPHRD_SIT) {
				hdr_offset += 20;
#if defined(PPA_DSLITE) && PPA_DSLITE
			} else if (ppa_netif_type(rx_ifinfo->netif) == ARPHRD_TUNNEL6) {
				hdr_offset += 40;
#endif
			}

#ifdef CONFIG_PPA_PP_LEARNING
			ppa_get_src_mac_addr(ppa_buf, p_item->pkt.src_mac, -hdr_offset);
#endif
			if (rx_ifinfo->flags & NETIF_GRE_TUNNEL) {
			/** Note:
			* For EoGRE the inner MAC addresses are required to be configured
			* in RT table */
				if(rx_ifinfo->greInfo.flowId == FLOWID_IPV4_EoGRE ||
					rx_ifinfo->greInfo.flowId == FLOWID_IPV6_EoGRE ) {
					ppa_get_src_mac_addr(ppa_buf, p_item->pkt.src_mac,
						-(PPA_ETH_HLEN));
					hdr_offset += PPA_ETH_HLEN; /*To skip inner MAC hdr*/
				}
				base_netif = p_item->rx_if;
				if(ppa_if_is_vlan_if(p_item->rx_if, NULL)) {
					if(ppa_vlan_get_underlying_if(p_item->rx_if, NULL,
						underlying_intname) == PPA_SUCCESS) {
						base_netif = ppa_get_netif (underlying_intname);
						hdr_offset += 4;
					}
				}
				ppa_get_gre_hdrlen(base_netif, &hdrlen);
				hdr_offset += hdrlen;
			}
#if IS_ENABLED(CONFIG_SOC_GRX500)
			ppa_get_src_mac_addr(ppa_buf, p_item->s_mac,-hdr_offset);
#endif
		} else {
			ppa_get_pkt_rx_src_mac_addr(ppa_buf, p_item->pkt.src_mac);
#if IS_ENABLED(CONFIG_SOC_GRX500)
			ppa_memcpy(p_item->s_mac, p_item->pkt.src_mac, PPA_ETH_ALEN);
			if ((flags & PPA_F_BRIDGED_SESSION) && (rx_ifinfo->flags & NETIF_GRE_TUNNEL)) {
			/** Note:
			* For EoGRE the inner MAC addresses are required to be configured
			* in RT table */
				if ( (rx_ifinfo->greInfo.flowId == FLOWID_IPV4_EoGRE) ||
				     (rx_ifinfo->greInfo.flowId == FLOWID_IPV6_EoGRE) ) {
					hdr_offset += PPA_ETH_HLEN; /*To skip inner MAC hdr*/
				}
				base_netif = p_item->rx_if;
				if (ppa_if_is_vlan_if(p_item->rx_if, NULL)) {
					if (ppa_vlan_get_underlying_if(p_item->rx_if, NULL,
						underlying_intname) == PPA_SUCCESS) {
						base_netif = ppa_get_netif(underlying_intname);
						hdr_offset += 4;
					}
				}
				ppa_get_gre_hdrlen(base_netif, &hdrlen);
				hdr_offset += hdrlen;
				ppa_get_src_mac_addr(ppa_buf, p_item->s_mac, -hdr_offset);
			}
#endif
		}
		ppa_netif_put(rx_ifinfo);
	}

#if IS_ENABLED(CONFIG_IPV6)
	if (ppa_is_pkt_ipv6(ppa_buf)) {
		p_item->flags	|= SESSION_IS_IPV6;
	} else 
#endif
	{
	/** ppa_get_pkt_src_ip/ppa_get_pkt_dst_ip returns IP address 
	* referenced from skb, so IPv4 address contains garbage value
	* from 5th byte*/

#if IS_ENABLED(CONFIG_IPV6)	 
	p_item->pkt.src_ip.ip6[1] = p_item->pkt.src_ip.ip6[2] = p_item->pkt.src_ip.ip6[3] = 0;
	p_item->pkt.dst_ip.ip6[1] = p_item->pkt.dst_ip.ip6[2] = p_item->pkt.dst_ip.ip6[3] = 0;
#endif
	}

	p_item->host_bytes   += ppa_skb_len(ppa_buf) + PPA_ETH_HLEN + PPA_ETH_CRCLEN;
	p_item->num_adds++;
#if defined(CONFIG_PPA_HANDLE_CONNTRACK_SESSIONS)
	ppa_session_store_ewma(ppa_buf,p_item,0);
#endif
	ppa_atomic_set(&p_item->used,2);
/* Store the pointer to p_item in skb and increment the reference counter */
	ppa_atomic_inc_not_zero(&p_item->used);
	ppa_buf->ptr_ppa_pitem = (void *)p_item;
	__ppa_session_insert_item(p_item);
	*pp_item = p_item; 
	/*dump_list_item(p_item, "ppa_alloc_session (after Add)");*/

	/* If bridged session, add timer */
	if (!p_session) {
		ppa_session_br_timer_add(p_item);
	}
	ret = PPA_SUCCESS; 

__ADD_SESSION_DONE:
	/*release table lock*/
  	ppa_session_list_unlock();
	return ret;
}
#endif /*CONFIG_PPA_RT_SESS_LEARNING*/

/*
 *  multicast routing group list item operation
 */
void ppa_init_mc_session_node(struct mc_session_node *p_item)
{
	ppa_memset(p_item, 0, sizeof(*p_item));
	PPA_INIT_HLIST_NODE(&p_item->mc_hlist);
	p_item->mc_entry		= ~0;
}

static INLINE struct mc_session_node *ppa_alloc_mc_session_node(void)
{
	struct mc_session_node *p_item;
	p_item = ppa_mem_cache_alloc(g_mc_group_item_cache);
	if (p_item)
		ppa_init_mc_session_node(p_item);
	return p_item;
}

static INLINE void ppa_free_mc_session_node(struct mc_session_node *p_item)
{
	ppa_hsel_del_wan_mc_group(p_item);
	ppa_mem_cache_free(p_item, g_mc_group_item_cache);
}

static INLINE void ppa_insert_mc_group_item(struct mc_session_node *p_item)
{
	uint32_t idx;
	idx = SESSION_LIST_MC_HASH_VALUE(p_item->grp.ip_mc_group.ip.ip);
	ppa_hlist_add_head(&p_item->mc_hlist, &g_mc_group_list_hash_table[idx]);
}

static INLINE void ppa_remove_mc_group_item(struct mc_session_node *p_item)
{
	ppa_hlist_del(&p_item->mc_hlist);
}

static void ppa_free_mc_group_list(void)
{
	struct mc_session_node *p_mc_item;
	PPA_HLIST_NODE *tmp;
	int i;

	ppa_mc_get_htable_lock();
	for (i = 0; i < NUM_ENTITY(g_mc_group_list_hash_table); i++) {
		ppa_hlist_for_each_entry_safe(p_mc_item, tmp, 
			&g_mc_group_list_hash_table[i], mc_hlist) {
			ppa_remove_mc_group_item(p_mc_item);
			ppa_free_mc_session_node(p_mc_item);
		}
	}
	ppa_mc_release_htable_lock();
}

/*
 *  routing session timeout help function
 */
#if defined(PPA_IF_MIB) && PPA_IF_MIB
int32_t update_netif_hw_mib(PPA_IFNAME *ifname, uint64_t new_mib, uint8_t is_rx)
{
	struct netif_info *ifinfo;
	int i;

	if (!ifname) {
		return PPA_SUCCESS;
	}

	if (ppa_netif_lookup(ifname, &ifinfo) == PPA_SUCCESS) {
		struct netif_info *sub_ifinfo;
		/*
		 * If CONFIG_IPV4_IPV6_COUNTER_SUPPORT is enabled, update packet counts
		 * Else update the byte counts
		 */
#if IS_ENABLED(CONFIG_IPV4_IPV6_COUNTER_SUPPORT)
		if (is_rx) {
			ifinfo->hw_accel_stats.rx_pkts += new_mib;
		} else {
			ifinfo->hw_accel_stats.tx_pkts += new_mib;
		}

		for (i = 0; i < ifinfo->sub_if_index; i++) {
			if (ppa_netif_lookup(ifinfo->sub_if_name[i], &sub_ifinfo) == PPA_SUCCESS) {
				if (is_rx) {
					sub_ifinfo->hw_accel_stats.rx_pkts += new_mib;
				} else {
					sub_ifinfo->hw_accel_stats.tx_pkts += new_mib;
				}
				ppa_netif_put(sub_ifinfo);
			}
		}
#else /* CONFIG_IPV4_IPV6_COUNTER_SUPPORT */
		if (is_rx) {
			ifinfo->hw_accel_stats.rx_bytes += new_mib;
		} else {
			ifinfo->hw_accel_stats.tx_bytes += new_mib;
		}

		for (i = 0; i < ifinfo->sub_if_index; i++) {
			if (ppa_netif_lookup(ifinfo->sub_if_name[i], &sub_ifinfo) == PPA_SUCCESS) {
				if (is_rx) {
					sub_ifinfo->hw_accel_stats.rx_bytes += new_mib;
				} else {
					sub_ifinfo->hw_accel_stats.tx_bytes += new_mib;
				}
				ppa_netif_put(sub_ifinfo);
			}
		}
#endif /* CONFIG_IPV4_IPV6_COUNTER_SUPPORT*/
		ppa_netif_put(ifinfo);
	}

	return PPA_SUCCESS;
}

int32_t update_netif_sw_mib(PPA_IFNAME *ifname, uint64_t new_mib, uint8_t is_rx)
{
	struct netif_info *ifinfo;
	int i;

	if (!ifname) {
		return PPA_SUCCESS;
	}
	if (ppa_netif_lookup(ifname, &ifinfo) == PPA_SUCCESS) {
		struct netif_info *sub_ifinfo;
		/*
		 * If CONFIG_IPV4_IPV6_COUNTER_SUPPORT is enabled, update packet counts
		 * Else update the byte counts
		 */
#if IS_ENABLED(CONFIG_IPV4_IPV6_COUNTER_SUPPORT)
		if (is_rx) {
			ifinfo->sw_accel_stats.rx_pkts += new_mib;
		} else {
			ifinfo->sw_accel_stats.tx_pkts += new_mib;
		}
		for (i = 0; i < ifinfo->sub_if_index; i++) {
			if (ppa_netif_lookup(ifinfo->sub_if_name[i], &sub_ifinfo) == PPA_SUCCESS) {
				if (is_rx) {
					sub_ifinfo->sw_accel_stats.rx_pkts += new_mib;
				} else {
					sub_ifinfo->sw_accel_stats.tx_pkts += new_mib;
				}
				ppa_netif_put(sub_ifinfo);
			}
		}
#else /* CONFIG_IPV4_IPV6_COUNTER_SUPPORT*/
		if (is_rx) {
			ifinfo->sw_accel_stats.rx_bytes += new_mib;
		} else {
			ifinfo->sw_accel_stats.tx_bytes += new_mib;
		}
		for (i = 0; i < ifinfo->sub_if_index; i++) {
			if (ppa_netif_lookup(ifinfo->sub_if_name[i], &sub_ifinfo) == PPA_SUCCESS) {
				if (is_rx) {
					sub_ifinfo->sw_accel_stats.rx_bytes += new_mib;
				} else {
					sub_ifinfo->sw_accel_stats.tx_bytes += new_mib;
				}
				ppa_netif_put(sub_ifinfo);
			}
		}
#endif /* CONFIG_IPV4_IPV6_COUNTER_SUPPORT*/
		ppa_netif_put(ifinfo);
	}
	return PPA_SUCCESS;
}

void clear_all_netif_mib(void)
{
	struct netif_info *p_netif;
	uint32_t pos = 0;

	if (ppa_netif_start_iteration(&pos, &p_netif) == PPA_SUCCESS) {
		do { /* clear all the interface mib */
			p_netif->hw_accel_stats.rx_bytes = 0;
			p_netif->hw_accel_stats.tx_bytes = 0;
			p_netif->sw_accel_stats.rx_bytes = 0;
			 p_netif->sw_accel_stats.tx_bytes = 0;
#if IS_ENABLED(CONFIG_IPV4_IPV6_COUNTER_SUPPORT)
			p_netif->hw_accel_stats.rx_pkts_ipv4 = 0;
			p_netif->hw_accel_stats.tx_pkts_ipv4 = 0;
			p_netif->hw_accel_stats.rx_pkts_ipv6 = 0;
			p_netif->hw_accel_stats.tx_pkts_ipv6 = 0;
			p_netif->sw_accel_stats.rx_pkts_ipv4 = 0;
			p_netif->sw_accel_stats.tx_pkts_ipv4 = 0;
			p_netif->sw_accel_stats.rx_pkts_ipv6 = 0;
			p_netif->sw_accel_stats.tx_pkts_ipv6 = 0;
#endif /* CONFIG_IPV4_IPV6_COUNTER_SUPPORT */
		} while (ppa_netif_iterate_next(&pos, &p_netif) == PPA_SUCCESS);
	}
	ppa_netif_stop_iteration();
}

void sw_del_session_mgmt_stats(struct uc_session_node *p_item)
{
	uint64_t tmp;

	/* collect the updated accelerated counters */
	if (p_item->acc_bytes >= (uint64_t)p_item->last_bytes)
		tmp = (p_item->acc_bytes - (uint64_t)p_item->last_bytes);
	else
		tmp = (p_item->acc_bytes + ((uint64_t)WRAPROUND_64BITS - (uint64_t)p_item->last_bytes));

	/* reset the accelerated counters, as it is going to be deleted */
	p_item->acc_bytes = 0;
	p_item->last_bytes = 0;

	/* update mib interfaces */
	update_netif_sw_mib(ppa_get_netif_name(p_item->rx_if), tmp, 1);
	if (p_item->br_rx_if)
		update_netif_sw_mib(ppa_get_netif_name(p_item->br_rx_if), tmp, 1);
	update_netif_sw_mib(ppa_get_netif_name(p_item->tx_if), tmp, 0);
	if (p_item->br_tx_if)
		update_netif_sw_mib(ppa_get_netif_name(p_item->br_tx_if), tmp, 0);

#if IS_ENABLED(CONFIG_IPV4_IPV6_COUNTER_SUPPORT)
	if (p_item->rx_if) {
		if (p_item->flags & SESSION_IS_IPV6)
			update_netif_hw_mib_ip_based(ppa_get_netif_name(p_item->rx_if), tmp, 1, 0);
		else
			update_netif_hw_mib_ip_based(ppa_get_netif_name(p_item->rx_if), tmp, 1, 1);
	}
	if (p_item->br_rx_if) {
		if (p_item->flags & SESSION_IS_IPV6)
			update_netif_hw_mib_ip_based(ppa_get_netif_name(p_item->br_rx_if), tmp, 1, 0);
		else
			update_netif_hw_mib_ip_based(ppa_get_netif_name(p_item->br_rx_if), tmp, 1, 1);
	}
	if (p_item->tx_if) {
		if (p_item->flags & SESSION_IS_IPV6)
			update_netif_hw_mib_ip_based(ppa_get_netif_name(p_item->tx_if), tmp, 0, 0);
		else
			update_netif_hw_mib_ip_based(ppa_get_netif_name(p_item->tx_if), tmp, 0, 1);
	}
	if (p_item->br_tx_if) {
		if (p_item->flags & SESSION_IS_IPV6)
			update_netif_hw_mib_ip_based(ppa_get_netif_name(p_item->br_tx_if), tmp, 0, 0);
		else
			update_netif_hw_mib_ip_based(ppa_get_netif_name(p_item->br_tx_if), tmp, 0, 1);
	}
#endif /* CONFIG_IPV4_IPV6_COUNTER_SUPPORT */

}

#if IS_ENABLED(CONFIG_IPV4_IPV6_COUNTER_SUPPORT)
int32_t update_netif_hw_mib_ip_based(PPA_IFNAME *ifname, uint64_t new_mib, uint8_t is_rx, uint8_t is_ipv4)
{
	struct netif_info *ifinfo;
	int i;

	if (!ifname) {
		return PPA_SUCCESS;
	}

	if (ppa_netif_lookup(ifname, &ifinfo) == PPA_SUCCESS) {
		struct netif_info *sub_ifinfo;
		if (is_rx) {
			if (is_ipv4) {
				ifinfo->hw_accel_stats.rx_pkts_ipv4 += new_mib;
			} else {
				ifinfo->hw_accel_stats.rx_pkts_ipv6 += new_mib;
			}
		} else {
			if (is_ipv4) {
				ifinfo->hw_accel_stats.tx_pkts_ipv4 += new_mib;
			} else {
				ifinfo->hw_accel_stats.tx_pkts_ipv6 += new_mib;
			}
		}

		for (i = 0; i < ifinfo->sub_if_index; i++) {
			if (ppa_netif_lookup(ifinfo->sub_if_name[i], &sub_ifinfo) == PPA_SUCCESS) {
				if (is_rx) {
					if (is_ipv4) {
						sub_ifinfo->hw_accel_stats.rx_pkts_ipv4 += new_mib;
					} else {
						sub_ifinfo->hw_accel_stats.rx_pkts_ipv6 += new_mib;
					}
				} else {
					if (is_ipv4) {
						sub_ifinfo->hw_accel_stats.tx_pkts_ipv4 += new_mib;
					} else {
						sub_ifinfo->hw_accel_stats.tx_pkts_ipv6 += new_mib;
					}
				}
				ppa_netif_put(sub_ifinfo);
			}
		}
		ppa_netif_put(ifinfo);
	}

	return PPA_SUCCESS;
}

int32_t update_netif_sw_mib_ip_based(PPA_IFNAME *ifname, uint64_t new_mib, uint8_t is_rx, uint8_t is_ipv4)
{
	struct netif_info *ifinfo;
	int i;

	if (!ifname) {
		return PPA_SUCCESS;
	}

	if (ppa_netif_lookup(ifname, &ifinfo) == PPA_SUCCESS) {
		struct netif_info *sub_ifinfo;
		if (is_rx) {
			if (is_ipv4) {
				ifinfo->sw_accel_stats.rx_pkts_ipv4 += new_mib;
			} else {
				ifinfo->sw_accel_stats.rx_pkts_ipv6 += new_mib;
			}
		} else {
			if (is_ipv4) {
				ifinfo->sw_accel_stats.tx_pkts_ipv4 += new_mib;
			} else {
				ifinfo->sw_accel_stats.tx_pkts_ipv6 += new_mib;
			}
		}

		for (i = 0; i < ifinfo->sub_if_index; i++) {
			if (ppa_netif_lookup(ifinfo->sub_if_name[i], &sub_ifinfo) == PPA_SUCCESS) {
				if (is_rx) {
					if (is_ipv4) {
						sub_ifinfo->sw_accel_stats.rx_pkts_ipv4 += new_mib;
					} else {
						sub_ifinfo->sw_accel_stats.rx_pkts_ipv6 += new_mib;
					}
				} else {
					if (is_ipv4) {
						sub_ifinfo->sw_accel_stats.tx_pkts_ipv4 += new_mib;
					} else {
						sub_ifinfo->sw_accel_stats.tx_pkts_ipv6 += new_mib;
					}
				}
				ppa_netif_put(sub_ifinfo);
			}
		}
		ppa_netif_put(ifinfo);
	}

	return PPA_SUCCESS;
}
#endif /* CONFIG_IPV4_IPV6_COUNTER_SUPPORT */

#endif /* PPA_IF_MIB */

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,15,0)
static void ppa_mib_count(unsigned long dummy)
{
#if defined(PPA_IF_MIB) && PPA_IF_MIB
	PPA_PORT_MIB	 *port_mib = NULL;
#if IS_ENABLED(CONFIG_PPA_QOS)
	PPA_QOS_STATUS   *qos_mib  = NULL;
#endif
	uint32_t   i;

	/*update port mib without update rate_mib*/
	port_mib = ppa_malloc(sizeof(PPA_PORT_MIB));
	if (port_mib) {
		ppa_update_port_mib(port_mib,0, 0);
		ppa_free(port_mib);
	}

	/*update qos queue mib without update rate_mib*/
#if IS_ENABLED(CONFIG_PPA_QOS)
	qos_mib = ppa_malloc(sizeof(PPA_QOS_STATUS));
	if (qos_mib) {
		for (i = 0; i < PPA_MAX_PORT_NUM; i++) {
			ppa_memset(qos_mib, 0, sizeof(qos_mib));
			qos_mib->qos_queue_portid = i;
			ppa_update_qos_mib(qos_mib, 0, 0);
		}
		ppa_free(qos_mib);
	}
#endif

	/*restart timer*/
	ppa_timer_add(&g_mib_cnt_timer, g_mib_polling_time);
#endif
}
#else
static void get_mib_counters(struct work_struct *w)
{
#if defined(PPA_IF_MIB) && PPA_IF_MIB
	PPA_PORT_MIB	 *port_mib = NULL;
#if IS_ENABLED(CONFIG_PPA_QOS)
	PPA_QOS_STATUS   *qos_mib  = NULL;
#endif
	uint32_t   i;
	/*update port mib without update rate_mib*/
	port_mib = ppa_malloc(sizeof(PPA_PORT_MIB));
	if (port_mib) {
		ppa_update_port_mib(port_mib,0, 0);
		ppa_free(port_mib);
	}

	/*update qos queue mib without update rate_mib*/
#if IS_ENABLED(CONFIG_PPA_QOS)
	qos_mib = ppa_malloc(sizeof(PPA_QOS_STATUS));
	if (qos_mib) {
		for (i = 0; i < PPA_MAX_PORT_NUM; i++) {
			ppa_memset(qos_mib, 0, sizeof(qos_mib));
			qos_mib->qos_queue_portid = i;
			ppa_update_qos_mib(qos_mib, 0, 0);
		}
		ppa_free(qos_mib);
	}
#endif
#endif
}
/* Declare work queue handler thread */
static DECLARE_DELAYED_WORK(mib_counter_dwork, get_mib_counters);

static PPA_HRT_RESTART ppa_mib_count(PPA_HRT *hrt)
{
	/* schedule work queue */
	if (g_timer_workq)
		queue_delayed_work(g_timer_workq, &mib_counter_dwork, 0);

	ppa_hrt_forward(&g_mib_cnt_timer, g_mib_polling_time);
	return 1; /*HRTIMER_RESTART*/
}
#endif

extern PPA_HLIST_HEAD	g_session_list_hash_table[SESSION_LIST_HASH_TABLE_SIZE];
extern void ppa_nf_ct_refresh_acct(PPA_SESSION *ct, PPA_CTINFO ctinfo, 
		unsigned long extra_jiffies, unsigned long bytes, unsigned int pkts);

void ppa_check_hit_stat_clear_mib(int32_t flag)
{
	struct uc_session_node *p_item	= NULL;
	struct mc_session_node *p_mc_item = NULL;
	
	uint32_t i=0;
	PPA_ROUTING_INFO route = {0};
	PPA_MC_INFO	  mc	= {0};
	uint64_t		 tmp   = 0;

	PPA_CTINFO ctinfo;   
 
#if IS_ENABLED(CONFIG_PPA_MPE_IP97)
	IPSEC_TUNNEL_MIB_INFO mib_info={0,0,0,0,0}; 
	uint32_t  index=0;
	ppa_tunnel_entry *t_entry = NULL;
#endif /*IS_ENABLED(CONFIG_PPA_MPE_IP97)*/

#if defined(PPA_IF_MIB) && PPA_IF_MIB
	uint32_t j=0;
	if (flag & PPA_CMD_CLEAR_PORT_MIB) {
		clear_all_netif_mib();
	}
#endif

	for (i = 0; i < NUM_ENTITY(g_session_list_hash_table); i++) {
		ppa_session_list_lock();
		ppa_hlist_for_each_entry(p_item, &g_session_list_hash_table[i], hlist) {
			tmp = 0;
#if IS_ENABLED(CONFIG_SOC_GRX500)
			route.entry = p_item->sess_hash;
			route.p_item = p_item;
#else
			route.entry = p_item->routing_entry;	 
#endif
			if (flag & PPA_CMD_CLEAR_PORT_MIB) {
				p_item->acc_bytes = 0;
				p_item->host_bytes = 0;
				p_item->last_bytes = 0;
				p_item->prev_sess_bytes=0;
				p_item->prev_clear_acc_bytes=0;
				p_item->prev_clear_host_bytes = 0;
				if (p_item->flags & SESSION_ADDED_IN_HW) {
					route.bytes = 0; route.f_hit = 0;
/* TBD: we have to read the routing entry bytes from the HAL that does capability SESS_IPV4 or SESS_IPV6 for this session
// the first entry in the p_item->caps_list will give this information; get will automatically clear the hw counters*/
					ppa_hsel_get_routing_entry_bytes(&route, flag, 
						p_item->caps_list[0].hal_id);
				}
				continue;
			}

			if (p_item->flags & SESSION_ADDED_IN_HW) {
				route.bytes = 0; route.f_hit = 0;
				ppa_hsel_get_routing_entry_bytes(&route, flag, p_item->caps_list[0].hal_id);
#if IS_ENABLED(CONFIG_SOC_GRX500)
				if (route.bytes > WRAPROUND_SESSION_MIB) {
					err("why route.bytes(%llu) > WRAPROUND_SESSION_MIB(%llu)\n",
						route.bytes, (uint64_t)WRAPROUND_SESSION_MIB);
				}
#endif /*IS_ENABLED(CONFIG_SOC_GRX500)*/

				if (route.f_hit) {
					p_item->last_hit_time = ppa_get_time_in_sec();
#if IS_ENABLED(CONFIG_SOC_GRX500)
					tmp = route.bytes;

					if ((p_item->acc_bytes + route.bytes) > (uint64_t)WRAPROUND_64BITS) {
						err(" p_item->acc_bytes %llu wrapping around \n", p_item->acc_bytes);
						p_item->acc_bytes = route.bytes;
					} else {
						p_item->last_bytes = p_item->acc_bytes;
						p_item->acc_bytes += route.bytes;
					}	
#else
					p_item->last_bytes = p_item->acc_bytes;
					p_item->acc_bytes = route.bytes;	
			
					tmp = p_item->acc_bytes - p_item->last_bytes;
#endif /*IS_ENABLED(CONFIG_SOC_GRX500)*/
					/*refresh the ct timeout with g_hit_polling_time*/
					ctinfo = (p_item->flags & SESSION_IS_REPLY) ? IP_CT_IS_REPLY : 0;
					ppa_nf_ct_refresh_acct(p_item->session, ctinfo, 2*g_hit_polling_time*HZ, tmp, route.packets);

#if defined(PPA_IF_MIB) && PPA_IF_MIB
					if (p_item->rx_if)
						update_netif_hw_mib(ppa_get_netif_name(p_item->rx_if), tmp, 1);
					if (p_item->br_rx_if)
						update_netif_hw_mib(ppa_get_netif_name(p_item->br_rx_if), tmp, 1);
					if (p_item->tx_if)
						update_netif_hw_mib(ppa_get_netif_name(p_item->tx_if), tmp, 0);
					if (p_item->br_tx_if)
						update_netif_hw_mib(ppa_get_netif_name(p_item->br_tx_if), tmp, 0);
#if IS_ENABLED(CONFIG_IPV4_IPV6_COUNTER_SUPPORT)
					if (p_item->rx_if) {
						if (p_item->flags & SESSION_IS_IPV6)
							update_netif_hw_mib_ip_based(ppa_get_netif_name(p_item->rx_if), tmp, 1, 0);
						else
							update_netif_hw_mib_ip_based(ppa_get_netif_name(p_item->rx_if), tmp, 1, 1);
					}
					if (p_item->br_rx_if) {
						if (p_item->flags & SESSION_IS_IPV6)
							update_netif_hw_mib_ip_based(ppa_get_netif_name(p_item->br_rx_if), tmp, 1, 0);
						else
							update_netif_hw_mib_ip_based(ppa_get_netif_name(p_item->br_rx_if), tmp, 1, 1);
					}
					if (p_item->tx_if) {
						if (p_item->flags & SESSION_IS_IPV6)
							update_netif_hw_mib_ip_based(ppa_get_netif_name(p_item->tx_if), tmp, 0, 0);
						else
							update_netif_hw_mib_ip_based(ppa_get_netif_name(p_item->tx_if), tmp, 0, 1);
					}
					if (p_item->br_tx_if) {
						if (p_item->flags & SESSION_IS_IPV6)
							update_netif_hw_mib_ip_based(ppa_get_netif_name(p_item->br_tx_if), tmp, 0, 0);
						else
							update_netif_hw_mib_ip_based(ppa_get_netif_name(p_item->br_tx_if), tmp, 0, 1);
					}
#endif /* CONFIG_IPV4_IPV6_COUNTER_SUPPORT */
#endif /* PPA_IF_MIB */
				}
			} else if (p_item->flags & SESSION_ADDED_IN_SW) {
				if (p_item->acc_bytes >= (uint64_t)p_item->last_bytes)
					tmp = p_item->acc_bytes - (uint64_t)p_item->last_bytes;
				else
					tmp = p_item->acc_bytes + 
						((uint64_t)WRAPROUND_64BITS - (uint64_t)p_item->last_bytes);
				p_item->last_bytes = p_item->acc_bytes;
				if (tmp > 0) {
					p_item->last_hit_time = ppa_get_time_in_sec();
					/*refresh the ct timeout with g_hit_polling_time*/
					ctinfo = (p_item->flags & SESSION_IS_REPLY) ? IP_CT_IS_REPLY : 0;
					ppa_nf_ct_refresh_acct(p_item->session, ctinfo,
							2*g_hit_polling_time*HZ, tmp, route.packets);
#if defined(PPA_IF_MIB) && PPA_IF_MIB
					if (p_item->rx_if)
						update_netif_sw_mib(ppa_get_netif_name(p_item->rx_if), tmp, 1);
					if (p_item->br_rx_if)
						update_netif_sw_mib(ppa_get_netif_name(p_item->br_rx_if), tmp, 1);
					if (p_item->tx_if)
						update_netif_sw_mib(ppa_get_netif_name(p_item->tx_if), tmp, 0);
					if (p_item->br_tx_if)
						update_netif_sw_mib(ppa_get_netif_name(p_item->br_tx_if), tmp, 0);
#if IS_ENABLED(CONFIG_IPV4_IPV6_COUNTER_SUPPORT)
					if (p_item->rx_if) {
						if (p_item->flags & SESSION_IS_IPV6)
							update_netif_sw_mib_ip_based(ppa_get_netif_name(p_item->rx_if), tmp, 1, 0);
						else
							update_netif_sw_mib_ip_based(ppa_get_netif_name(p_item->rx_if), tmp, 1, 1);
					}
					if (p_item->br_rx_if) {
						if (p_item->flags & SESSION_IS_IPV6)
							update_netif_sw_mib_ip_based(ppa_get_netif_name(p_item->br_rx_if), tmp, 1, 0);
						else
							update_netif_sw_mib_ip_based(ppa_get_netif_name(p_item->br_rx_if), tmp, 1, 1);
					}
					if (p_item->tx_if) {
						if (p_item->flags & SESSION_IS_IPV6)
							update_netif_sw_mib_ip_based(ppa_get_netif_name(p_item->tx_if), tmp, 0, 0);
						else
							update_netif_sw_mib_ip_based(ppa_get_netif_name(p_item->tx_if), tmp, 0, 1);
					}
					if (p_item->br_tx_if) {
						if (p_item->flags & SESSION_IS_IPV6)
							update_netif_sw_mib_ip_based(ppa_get_netif_name(p_item->br_tx_if), tmp, 0, 0);
						else
							update_netif_sw_mib_ip_based(ppa_get_netif_name(p_item->br_tx_if), tmp, 0, 1);
					}
#endif /* CONFIG_IPV4_IPV6_COUNTER_SUPPORT */
#endif /* PPA_IF_MIB */
				}
#if IS_ENABLED(CONFIG_X86_INTEL_LGM)
			} else {
				/*Session timed out and removed from HW */
				if (!p_item->session) {
					/* bridged session */
					if ((ppa_get_time_in_sec()-p_item->last_hit_time) > DEFAULT_BRIDGING_TIMEOUT_IN_SEC) {
					/* delete the session*/	
						__ppa_session_delete_item(p_item);
					}	
				}	
#endif /*IS_ENABLED(CONFIG_X86_INTEL_LGM)*/
			}
		}
		ppa_session_list_unlock();
	}

	for (i = 0; i < NUM_ENTITY(g_mc_group_list_hash_table); i++) {
		ppa_mc_get_htable_lock();
		ppa_hlist_for_each_entry(p_mc_item, &g_mc_group_list_hash_table[i], mc_hlist) {
			tmp = 0;
			mc.p_entry = p_mc_item->mc_entry;
			mc.p_item = p_mc_item;

			if (flag & PPA_CMD_CLEAR_PORT_MIB)   {
				/* flag means clear mib counter or not */
				if (p_mc_item->flags & SESSION_ADDED_IN_HW)
					ppa_hsel_get_mc_entry_bytes(&mc, flag, p_mc_item->caps_list[0].hal_id);
				p_mc_item->acc_bytes = 0;
				p_mc_item->last_bytes = 0;
				mc.f_hit = 0; mc.bytes = 0;
						continue;
			}

			if (p_mc_item->flags & SESSION_ADDED_IN_HW) {
				mc.f_hit = 0; mc.bytes = 0;
				ppa_hsel_get_mc_entry_bytes(&mc, flag, p_mc_item->caps_list[0].hal_id);
				if (mc.f_hit) {
					p_mc_item->last_hit_time = ppa_get_time_in_sec();
					tmp = mc.bytes; 
					if ((p_mc_item->acc_bytes + mc.bytes) >
							(uint64_t)WRAPROUND_64BITS) {
						err(" p_mc_item->acc_bytes  %llu wrapping around \n", 
							p_mc_item->acc_bytes);
						p_mc_item->acc_bytes = mc.bytes;
					} else {
						p_mc_item->last_bytes = p_mc_item->acc_bytes;
						p_mc_item->acc_bytes += mc.bytes;
					}   
#if defined(PPA_IF_MIB) && PPA_IF_MIB
					if (p_mc_item->grp.src_netif)
						update_netif_hw_mib(ppa_get_netif_name
							(p_mc_item->grp.src_netif), tmp, 1);

					for (j = 0; j < PPA_MAX_MC_IFS_NUM; j++)
						if (p_mc_item->grp.txif[j].netif)
							update_netif_hw_mib(ppa_get_netif_name
								(p_mc_item->grp.txif[j].netif), tmp, 0);
#endif
				}
			}
		}
		ppa_mc_release_htable_lock();
	}

#if IS_ENABLED(CONFIG_PPA_MPE_IP97)

	for (index = 0; index < MAX_TUNNEL_ENTRIES; index++) {
		ppe_lock_get(&g_tunnel_table_lock);
		t_entry = g_tunnel_table[index];
		if (t_entry != NULL && (t_entry->tunnel_type == TUNNEL_TYPE_IPSEC)) {
			mib_info.tunnel_id= index;
			mib_info.rx_pkt_count= mib_info.rx_byte_count= mib_info.tx_pkt_count= mib_info.tx_byte_count=0x0;
#if IS_ENABLED(CONFIG_SOC_GRX500) && CONFIG_SOC_GRX500
			ppa_hsel_get_ipsec_tunnel_mib(&mib_info, 0, MPE_HAL);
			if (t_entry->tunnel_info.ipsec_hdr.inbound != NULL) {
				spin_lock_bh(&t_entry->tunnel_info.ipsec_hdr.inbound->lock);
				t_entry->tunnel_info.ipsec_hdr.inbound->curlft.packets += 
					(mib_info.rx_pkt_count - t_entry->tunnel_info.ipsec_hdr.inbound_pkt_cnt);
				t_entry->tunnel_info.ipsec_hdr.inbound->curlft.bytes +=
					(mib_info.rx_byte_count - t_entry->tunnel_info.ipsec_hdr.inbound_byte_cnt);
				t_entry->tunnel_info.ipsec_hdr.inbound_pkt_cnt = mib_info.rx_pkt_count;
				t_entry->tunnel_info.ipsec_hdr.inbound_byte_cnt = mib_info.rx_byte_count;
				spin_unlock_bh(&t_entry->tunnel_info.ipsec_hdr.inbound->lock);
			}

			if (t_entry->tunnel_info.ipsec_hdr.outbound != NULL) {
				spin_lock_bh(&t_entry->tunnel_info.ipsec_hdr.outbound->lock);
				t_entry->tunnel_info.ipsec_hdr.outbound->curlft.packets +=
					(mib_info.tx_pkt_count - t_entry->tunnel_info.ipsec_hdr.outbound_pkt_cnt);
				t_entry->tunnel_info.ipsec_hdr.outbound->curlft.bytes +=
					(mib_info.tx_byte_count - t_entry->tunnel_info.ipsec_hdr.outbound_byte_cnt);
				t_entry->tunnel_info.ipsec_hdr.outbound_pkt_cnt = mib_info.tx_pkt_count;
				t_entry->tunnel_info.ipsec_hdr.outbound_byte_cnt = mib_info.tx_byte_count;
				spin_unlock_bh(&t_entry->tunnel_info.ipsec_hdr.outbound->lock); 
			}
#endif
		}
		ppe_lock_release(&g_tunnel_table_lock);
	}
#endif
}
EXPORT_SYMBOL(ppa_check_hit_stat_clear_mib);

static void ppa_chk_hit_stat(struct work_struct *w)
{
	ppa_check_hit_stat_clear_mib(0);
}
/* Declare work queue handler thread */
static DECLARE_DELAYED_WORK(check_hit_stat_dwork, ppa_chk_hit_stat);

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,15,0)
static void ppa_check_hit_stat(unsigned long dummy)
{
	/*schedule work queue */
	if (g_timer_workq)
                queue_delayed_work(g_timer_workq, &check_hit_stat_dwork, 0);

   	/*restart timer*/
	ppa_timer_add(&g_hit_stat_timer, g_hit_polling_time);
}
#else
static PPA_HRT_RESTART ppa_check_hit_stat(PPA_HRT *hrt)
{
	/*schedule work queue */
	if (g_timer_workq)
                queue_delayed_work(g_timer_workq, &check_hit_stat_dwork, 0);

   	/*restart timer*/
	ppa_hrt_forward(hrt, g_hit_polling_time);
	return 1; /*HRTIMER_RESTART*/
}
#endif


#if defined(PPA_INTF_MIB_TIMER) && IS_ENABLED(CONFIG_SOC_GRX500)
static void ppa_get_intf_mib(struct work_struct *w)
{
	ppa_update_intf_mib_hw();

}
/* Declare work queue handler thread */
static DECLARE_DELAYED_WORK(get_intf_mib_dwork, ppa_get_intf_mib);

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,15,0)
static void ppa_check_intf_mib(unsigned long dummy)
{
	/*schedule work queue */
	if (g_timer_workq)
                queue_delayed_work(g_timer_workq, &get_intf_mib_dwork, 0);

	/*restart timer*/
	ppa_timer_add(&g_intf_mib_timer, g_intf_mib_polling_time);
}
#else
static PPA_HRT_RESTART ppa_check_intf_mib(PPA_HRT *hrt)
{
	/*schedule work queue */
	if (g_timer_workq)
	queue_delayed_work(g_timer_workq, &get_intf_mib_dwork, 0);

	/*restart timer*/
	ppa_hrt_forward(hrt, g_intf_mib_polling_time);
	return 1; /*HRTIMER_RESTART*/
}
#endif

extern ppa_generic_hook_t  ppa_drv_hal_hook[MAX_HAL];
/*
 * Update the interface MIB for the logical port and also the under-laying interfaces 
 */
int ppa_update_base_inf_mib(PPA_ITF_MIB_INFO *mib_tmp,uint32_t hal_id)
{
	struct netif_info *ifinfo_tmp =  mib_tmp->ifinfo;
	struct intf_mib *p = &(mib_tmp->mib);
	uint32_t diff_rx_pkts=0,diff_tx_pkts=0;
	PPA_NETIF *netif_tmp = NULL;
	struct netif_info *ifinfo_base = NULL;
	int i = 0;

	if (p->rx_packets == 0)
		ifinfo_tmp->hw_accel_stats.rx_pkts_prev[hal_id] = 0;
		
	diff_rx_pkts = (p->rx_packets - ifinfo_tmp->hw_accel_stats.rx_pkts_prev[hal_id]);
	ifinfo_tmp->hw_accel_stats.rx_pkts += diff_rx_pkts;
	ifinfo_tmp->hw_accel_stats.rx_pkts_prev[hal_id] = p->rx_packets;

	if (p->tx_packets == 0)
		ifinfo_tmp->hw_accel_stats.tx_pkts_prev[hal_id] = 0;

	diff_tx_pkts = (p->tx_packets - ifinfo_tmp->hw_accel_stats.tx_pkts_prev[hal_id]);
	ifinfo_tmp->hw_accel_stats.tx_pkts += diff_tx_pkts;
	ifinfo_tmp->hw_accel_stats.tx_pkts_prev[hal_id] = p->tx_packets;

		/* Check if the interface is in bridge, if yes update the packet count for the bridge interface  */	
	netif_tmp = ifinfo_tmp->netif;
	if (ppa_is_netif_bridged(netif_tmp)) {
		if (ifinfo_tmp->brif != NULL) {
			__ppa_netif_lookup(ifinfo_tmp->brif->name,&ifinfo_base);
			if (ifinfo_base != NULL) {
				ifinfo_base->hw_accel_stats.rx_pkts += diff_rx_pkts;
				ifinfo_base->hw_accel_stats.tx_pkts += diff_tx_pkts;
				ppa_netif_put(ifinfo_base);
			}
		}
	}

	/* Handle the byte count adjustment */
	ifinfo_tmp->hw_accel_stats.rx_bytes -=  diff_rx_pkts * (1500 - ifinfo_tmp->mtu);
	ifinfo_tmp->hw_accel_stats.tx_bytes -=  diff_tx_pkts * (1500 - ifinfo_tmp->mtu);

	for (i = 0; i < ifinfo_tmp->sub_if_index; i++) {
		ifinfo_base = NULL;
		if (ifinfo_tmp->sub_if_name[i] != NULL) {
			__ppa_netif_lookup(ifinfo_tmp->sub_if_name[i],&ifinfo_base);
			if ((ifinfo_base != NULL) && (ifinfo_base->sub_if_index > 0)) {
				ifinfo_base->hw_accel_stats.rx_pkts += diff_rx_pkts;
				ifinfo_base->hw_accel_stats.tx_pkts += diff_tx_pkts;
				/* Handle the byte count adjustment */
				ifinfo_base->hw_accel_stats.rx_bytes -=  diff_rx_pkts * (1500 - ifinfo_base->mtu);
				ifinfo_base->hw_accel_stats.tx_bytes -=  diff_tx_pkts * (1500 - ifinfo_base->mtu);
			}
			ppa_netif_put(ifinfo_base);
		}
	}
	return PPA_SUCCESS;
}


/*
 * Get the interface MIB for each of the interfaces registered in the PPA from the respective HAL 
 */
void ppa_update_intf_mib_hw(void)
{
	struct netif_info *ifinfo;
	uint32_t pos = 0;
	uint32_t hal_id = 0;

	PPA_ITF_MIB_INFO itf_mib;

	if (ppa_netif_start_iteration(&pos, &ifinfo) == PPA_SUCCESS) {
		do {
			itf_mib.ifinfo = ifinfo;
			for (hal_id=1; hal_id< MAX_HAL;hal_id++) {
				ppa_memset(&(itf_mib.mib),0, sizeof(struct intf_mib));
				if (hal_id == TMU_HAL)
					continue;
				if (ppa_drv_hal_hook[hal_id] != NULL) {
					ppa_hsel_get_generic_itf_mib(&itf_mib, 0,hal_id);
					ppa_update_base_inf_mib(&itf_mib,hal_id);
				}
			}
		} while (ppa_netif_iterate_next(&pos, &ifinfo) == PPA_SUCCESS);
	}
	ppa_netif_stop_iteration();
}
#endif

/*
 *  bridging session list item operation
 */
static INLINE void ppa_bridging_init_uc_session_node(struct br_mac_node *p_item)
{
	ppa_memset(p_item, 0, sizeof(*p_item));
	p_item->bridging_entry = ~0;
	PPA_INIT_HLIST_NODE(&p_item->br_hlist);
}

static INLINE struct br_mac_node *ppa_bridging_alloc_uc_session_node(void)
{
	struct br_mac_node *p_item;
	p_item = ppa_mem_cache_alloc(g_bridging_session_item_cache);
	if (p_item)
		ppa_bridging_init_uc_session_node(p_item);
	return p_item;
}

static INLINE void ppa_bridging_free_uc_session_node(struct br_mac_node *p_item)
{
	ppa_bridging_hw_del_mac(p_item);
	ppa_mem_cache_free(p_item, g_bridging_session_item_cache);
}

static INLINE void ppa_bridging_insert_session_item(struct br_mac_node *p_item)
{
	uint32_t idx;
	idx = PPA_BRIDGING_SESSION_LIST_HASH_VALUE(p_item->mac);
	ppa_hlist_add_head(&p_item->br_hlist, &g_bridging_session_list_hash_table[idx]);
}

static INLINE void ppa_bridging_remove_session_item(struct br_mac_node *p_item)
{
	ppa_hlist_del(&p_item->br_hlist);
}

static void ppa_free_bridging_session_list(void)
{
	struct br_mac_node *p_br_item;
	PPA_HLIST_NODE *tmp;
	int i;

	ppa_br_get_htable_lock();
	for (i = 0; i < NUM_ENTITY(g_bridging_session_list_hash_table); i++) {
		ppa_hlist_for_each_entry_safe(p_br_item, tmp, &g_bridging_session_list_hash_table[i], br_hlist) {
			ppa_bridging_remove_mac(p_br_item);
		}
	}
	ppa_br_release_htable_lock();
}

/*
 *  bridging session timeout help function
 */
static INLINE uint32_t ppa_bridging_get_default_session_timeout(void)
{
	return DEFAULT_BRIDGING_TIMEOUT_IN_SEC;
}

uint32_t ppa_get_br_count(uint32_t count_flag)
{
	PPA_HLIST_NODE *node;
	uint32_t count = 0;
	uint32_t idx;

	ppa_br_get_htable_lock();
	for (idx = 0; idx < NUM_ENTITY(g_bridging_session_list_hash_table); idx++) {
		ppa_hlist_for_each(node, &g_bridging_session_list_hash_table[idx]) {
			count++;
		}
	}
	ppa_br_release_htable_lock();
	return count;
}

/*
 *  help function for special function
 */
static INLINE void ppa_remove_mc_groups_on_netif (PPA_IFNAME *ifname)
{
	uint32_t idx;
	struct mc_session_node *p_mc_item;
	PPA_HLIST_NODE *tmp;
	int i;
#if !IS_ENABLED(CONFIG_SOC_GRX500)
	PPA_C_OP cop;
#endif /*!IS_ENABLED(CONFIG_SOC_GRX500)*/

	ppa_lock_get(&g_mc_group_list_lock);
	for (idx = 0; idx < NUM_ENTITY(g_mc_group_list_hash_table); idx++) {
		ppa_hlist_for_each_entry_safe(p_mc_item, tmp, &g_mc_group_list_hash_table[idx],mc_hlist) { 
			for (i = 0; i < PPA_MAX_MC_IFS_NUM; i++) {
				if (ppa_is_netif_name(p_mc_item->grp.txif[i].netif, ifname)) {
					p_mc_item->grp.txif[i].netif = NULL;
					/*Reset the if bitmap index*/
					p_mc_item->grp.if_mask = p_mc_item->grp.if_mask & ~(1<<i);
					p_mc_item->grp.num_ifs--;
					break;
				}
			}
			if (i >= p_mc_item->grp.num_ifs)
				continue;
			if (p_mc_item->grp.num_ifs == 0)
				ppa_remove_mc_group(p_mc_item);

#if !IS_ENABLED(CONFIG_SOC_GRX500)
			cop.index = i;
			cop.flag = PPA_IF_DEL;
			/*Update the existing HW session */
			if (ppa_hsel_update_wan_mc_group(p_mc_item, &cop, 0) != PPA_SUCCESS) {
				ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT, "can not update %s %d\n", __FUNCTION__, __LINE__);
			}
#endif /*!IS_ENABLED(CONFIG_SOC_GRX500)*/
		}
	}
	ppa_lock_release(&g_mc_group_list_lock);
}

static INLINE void ppa_remove_bridging_sessions_on_netif (PPA_IFNAME *ifname)
{
	uint32_t idx;
	struct br_mac_node *p_br_item;
	PPA_HLIST_NODE *tmp;

	ppa_br_get_htable_lock();
	for (idx = 0; idx < NUM_ENTITY(g_bridging_session_list_hash_table); idx++) {
		ppa_hlist_for_each_entry_safe(p_br_item, tmp, &g_bridging_session_list_hash_table[idx], br_hlist) {
			if (ppa_is_netif_name(p_br_item->netif, ifname)) {
				ppa_bridging_remove_mac(p_br_item);
			}
		}
	}
	ppa_br_release_htable_lock();
	return;
}

#if defined(CONFIG_PPA_API_DIRECTCONNECT) && CONFIG_PPA_API_DIRECTCONNECT
static INLINE void ppa_remove_bridging_sessions_on_subif (PPA_DP_SUBIF *subif)
{
	uint32_t idx;
	struct br_mac_node *p_br_item;
	PPA_HLIST_NODE *tmp;
	
	ppa_br_get_htable_lock();
	for (idx = 0; idx < NUM_ENTITY(g_bridging_session_list_hash_table); idx++) {
		ppa_hlist_for_each_entry_safe(p_br_item, tmp, &g_bridging_session_list_hash_table[idx], br_hlist) {
			if ((p_br_item->dest_ifid == subif->port_id)
#if IS_ENABLED(CONFIG_SOC_GRX500)
				&& (p_br_item->sub_ifid == subif->subif)
#endif
			) {
				ppa_bridging_remove_mac(p_br_item);
			}
		}
	}
	ppa_br_release_htable_lock();
	return;
}

static INLINE void ppa_remove_bridging_sessions_on_macaddr(uint8_t *mac)
{
	uint32_t idx;
	struct br_mac_node *p_br_item;
	PPA_HLIST_NODE *tmp;

	ppa_br_get_htable_lock();
	idx = PPA_BRIDGING_SESSION_LIST_HASH_VALUE(mac);
	ppa_hlist_for_each_entry_safe(p_br_item, tmp, &g_bridging_session_list_hash_table[idx], br_hlist) {
		if (0 == ppa_memcmp(p_br_item->mac, mac, sizeof(p_br_item->mac))) {
			ppa_bridging_remove_mac(p_br_item);
		}
	}
	ppa_br_release_htable_lock();
	return;
}
#endif
/*
 * ####################################
 *		   Global Function
 * ####################################
 */
int32_t ppa_get_underlying_vlan_interface_if_eogre(PPA_NETIF *netif, PPA_NETIF **base_netif, uint8_t *isEoGre)
{
	uint8_t isIPv4Gre = 0;
	PPA_IFNAME underlying_intname[PPA_IF_NAME_SIZE];
			
	*base_netif = NULL;
	
	if (ppa_if_is_vlan_if (netif, NULL)) {
		if (ppa_vlan_get_underlying_if (netif, NULL, underlying_intname) == PPA_SUCCESS) {
			netif = ppa_get_netif (underlying_intname);
		} else {
			return PPA_FAILURE;
		}
	}  
	
	if (ppa_is_gre_netif_type(netif, &isIPv4Gre, isEoGre)) {
		*base_netif = netif;
		return PPA_SUCCESS;
	}

	return PPA_FAILURE;
}

#if IS_ENABLED(CONFIG_PPA_RT_SESS_LEARNING)
void ppa_session_not_bridged(struct uc_session_node *p_item,
							 PPA_SESSION* p_session)
{
	p_item->session = p_session;
	ppa_reset_BrSession(p_item);
	ppa_session_br_timer_del(p_item);
}
#endif /*IS_ENABLED(CONFIG_PPA_RT_SESS_LEARNING)*/

static void print_flags(uint32_t flags)
{
	static const char *str_flag[] = {
		"IS_REPLY",		/*  0x00000001*/
		"Reserved",
		"SESSION_IS_TCP",
		"STAMPING",
		"ADDED_IN_HW",		/*  0x00000010*/
		"NOT_ACCEL_FOR_MGM",
		"STATIC",
		"DROP",
		"VALID_NAT_IP",		/*  0x00000100*/
		"VALID_NAT_PORT",
		"VALID_NAT_SNAT",
		"NOT_ACCELABLE",
		"VALID_VLAN_INS",	/*  0x00001000*/
		"VALID_VLAN_RM",
		"SESSION_VALID_OUT_VLAN_INS",
		"SESSION_VALID_OUT_VLAN_RM",
		"VALID_PPPOE",		/*  0x00010000*/
		"VALID_NEW_SRC_MAC",
		"VALID_MTU",
		"VALID_NEW_DSCP",
		"SESSION_VALID_DSLWAN_QID", /*  0x00100000*/
		"SESSION_TX_ITF_IPOA",
		"SESSION_TX_ITF_PPPOA",
		"Reserved",
		"SRC_MAC_DROP_EN",	/*  0x01000000*/
		"SESSION_TUNNEL_6RD",
		"SESSION_TUNNEL_DSLITE",
		"Reserved",
		"LAN_ENTRY",		/*  0x10000000*/
		"WAN_ENTRY",
		"IPV6",
		"ADDED_IN_SW",
	};

	int flag;
	unsigned long bit;
	int i;
	flag = 0;

	for (bit = 1, i = 0; bit; bit <<= 1, i++)
		if ((flags & bit)) {
			if (flag++)
				printk(KERN_INFO  "| ");
			printk(KERN_INFO "%s", str_flag[i]);
		}
	
	if (flag)
		printk(KERN_INFO  "\n");
	else
		printk(KERN_INFO  "NULL\n");
}

void dump_list_item(struct uc_session_node *p_item, char *comment)
{
#if defined(DEBUG_DUMP_LIST_ITEM) && DEBUG_DUMP_LIST_ITEM
	int8_t strbuf[64];
	if (!(g_ppa_dbg_enable & DBG_ENABLE_MASK_DUMP_ROUTING_SESSION))
		return;

	if (max_print_num == 0)
		return;

	if (comment)
		printk("dump_list_item - %s\n", comment);
	else
		printk("dump_list_item\n");
	
	printk("  hlist			= %px\n", &p_item->hlist);
	printk("  session		  = %px\n", p_item->session);
	printk("  ip_proto		 = %08X\n", p_item->pkt.ip_proto);
	printk("  src_ip		   = %s\n",   ppa_get_pkt_ip_string(p_item->pkt.src_ip, 
				p_item->flags & SESSION_IS_IPV6, strbuf));
	printk("  src_port		 = %d\n",   p_item->pkt.src_port);
	printk("  src_mac[6]	   = %s\n",   ppa_get_pkt_mac_string(p_item->pkt.src_mac, strbuf));
	printk("  dst_ip		   = %s\n",   ppa_get_pkt_ip_string(p_item->pkt.dst_ip, 
				p_item->flags & SESSION_IS_IPV6, strbuf));
	printk("  dst_port		 = %d\n",   p_item->pkt.dst_port);
	printk("  dst_mac[6]	   = %s\n",   ppa_get_pkt_mac_string(p_item->pkt.dst_mac, strbuf));
	printk("  natip		   = %s\n",   ppa_get_pkt_ip_string(p_item->pkt.natip.natsrcip.ip,
				p_item->flags & SESSION_IS_IPV6, strbuf));
	printk("  nat_port		 = %d\n",   p_item->pkt.nat_port);
	printk("  rx_if			= %s\n", (p_item->rx_if ? p_item->rx_if->name : ""));
	printk("  tx_if			= %s\n", (p_item->tx_if ? p_item->tx_if->name : ""));
	printk("  timeout		  = %d\n",   p_item->timeout);
	printk("  last_hit_time	= %d\n",   p_item->last_hit_time);
	printk("  num_adds		 = %d\n",   p_item->num_adds);
	printk("  pppoe_session_id = %d\n",   p_item->pkt.pppoe_session_id);
#if defined(L2TP_CONFIG) && L2TP_CONFIG
	printk("  pppol2tp_session_id = %d\n",   p_item->pkt.pppol2tp_session_id);
	printk("  pppol2tp_tunnel_id = %d\n",   p_item->pkt.pppol2tp_tunnel_id);
#endif
	printk("  new_dscp		 = %d\n",   p_item->pkt.new_dscp);
	printk("  new_vci		  = %08X\n",  p_item->pkt.new_vci);
	printk("  mtu			  = %d\n",   p_item->mtu);
	printk("  flags			= %08X\n", p_item->flags);
	print_flags(p_item->flags);
	printk("  routing_entry	= %08X\n", p_item->routing_entry);

	if (max_print_num != ~0)
		max_print_num--;
#endif
}

#if defined(L2TP_CONFIG) && L2TP_CONFIG
uint16_t  checksum_l2tp(uint8_t* ip, int len)
{
	uint32_t chk_sum = 0;

	while (len > 1) {
		chk_sum += *((uint16_t*) ip);
		ip +=2;
		if (chk_sum & 0x80000000)
			chk_sum = (chk_sum & 0xFFFF) + (chk_sum >> 16);
		len -= 2;
	}

	if (len)
		chk_sum += (uint16_t) *((uint8_t *)ip);
	while (chk_sum >> 16)
		chk_sum = (chk_sum & 0xFFFF) + (chk_sum >> 16);

	return ~chk_sum;
}

void ppa_pppol2tp_get_l2tpinfo(struct net_device *dev, PPA_L2TP_INFO *l2tpinfo)
{
	uint32_t outer_srcip = 0;
	uint32_t outer_dstip = 0;
	uint32_t ip_chksum;
	uint32_t udp_chksum;
	struct l2tp_ip_hdr iphdr;
	struct l2tp_udp_hdr udphdr;

	ppa_memset(&iphdr, 0, sizeof(iphdr));
	ppa_memset(&udphdr, 0, sizeof(udphdr));

	ppa_pppol2tp_get_src_addr(dev,&outer_srcip);
	ppa_pppol2tp_get_dst_addr(dev,&outer_dstip);

	iphdr.ihl = 0x05;
	iphdr.version = 0x4;
	iphdr.tos = 0x0;
	iphdr.tot_len = 0x0;
	iphdr.id = 0x0;
	iphdr.frag_off = 0x0;
	iphdr.ttl = 0x40;
	iphdr.protocol = 0x11;
	iphdr.saddr = outer_srcip;
	iphdr.daddr = outer_dstip;
	iphdr.check = 0x0;
	ip_chksum = checksum_l2tp((uint8_t *)(&iphdr),sizeof(struct l2tp_ip_hdr));

	udphdr.saddr = outer_srcip;
	udphdr.daddr = outer_dstip;
	udphdr.src_port = 0x06a5;
	udphdr.dst_port = 0x06a5;
	udphdr.protocol = 0x0011;
	udp_chksum = checksum_l2tp((uint8_t *)(&udphdr), (sizeof(struct l2tp_udp_hdr) - 2));

	l2tpinfo->ip_chksum = ip_chksum;
	l2tpinfo->udp_chksum = udp_chksum;
	l2tpinfo->source_ip = outer_srcip;
	l2tpinfo->dest_ip = outer_dstip;
}
#endif

#if IS_ENABLED(CONFIG_PPA_RT_SESS_LEARNING)
static INLINE int32_t ppa_hw_accelerate(PPA_BUF *ppa_buf, struct uc_session_node *p_item)
{
	/* evaluate the condition based on some logic */
	/* current logic to check is ppe fastpath is enabled */
	if ((p_item->flags & SESSION_NOT_VALID_PHY_PORT)) {
		return 0;
	}
	return g_ppe_fastpath_enabled;
}
#endif /*CONFIG_PPA_RT_SESS_LEARNING*/

int32_t ppa_hw_update_session_extra(struct uc_session_node *p_item, uint32_t flags)
{	
/*Modify operation is not currently supported GRX500 PAE routing sessions*/
/*This support can be added later If the session is accelerated by MPE */
	return PPA_FAILURE;
}

uint32_t ip_equal(IP_ADDR_C *dst_ip, IP_ADDR_C *src_ip)
{
	if (dst_ip->f_ipv6) {
		return(((dst_ip->ip.ip6[0] ^ src_ip->ip.ip6[0]) |
				(dst_ip->ip.ip6[1] ^ src_ip->ip.ip6[1]) |
				(dst_ip->ip.ip6[2] ^ src_ip->ip.ip6[2]) |
				(dst_ip->ip.ip6[3] ^ src_ip->ip.ip6[3])) == 0);
	}else{
		return((dst_ip->ip.ip ^ src_ip->ip.ip) == 0);
	}
}

unsigned int is_ip_allbit1(IP_ADDR_C *ip)
{
	if (ip->f_ipv6) {
		return((~ip->ip.ip6[0] | ~ip->ip.ip6[1] | ~ip->ip.ip6[2] | ~ip->ip.ip6[3]) == 0);
	}else{
		return(~ip->ip.ip == 0);
	}
}
/*
 *  multicast routing operation
 */
int32_t __ppa_lookup_mc_group(IP_ADDR_C *ip_mc_group, IP_ADDR_C *src_ip, struct mc_session_node **pp_item)
{
	uint32_t idx;
	struct mc_session_node *p_mc_item = NULL;

	if (!pp_item) {
		ppa_debug(DBG_ENABLE_MASK_ASSERT,"pp_item == NULL\n");
		return PPA_SESSION_NOT_ADDED;
	}

	idx = SESSION_LIST_MC_HASH_VALUE(ip_mc_group->ip.ip);
	ppa_hlist_for_each_entry(p_mc_item,&g_mc_group_list_hash_table[idx],mc_hlist) {
			if (ip_equal(&p_mc_item->grp.ip_mc_group, ip_mc_group)) { /*mc group ip match*/
			if (ip_equal(&p_mc_item->grp.source_ip, src_ip)) {/*src ip match*/
				*pp_item = p_mc_item;
				return PPA_SESSION_EXISTS;
			}else{
				if (is_ip_zero(&p_mc_item->grp.source_ip) || is_ip_zero(src_ip)) {
					*pp_item = NULL;
					return PPA_MC_SESSION_VIOLATION;
				}
			}
		}
	}
	return PPA_SESSION_NOT_ADDED;
}
/*
  *  delete mc groups
  *  if SSM flag(Sourceip Specific Mode) is 1 , then match both dest ip and source ip
  *  if SSM flag is 0, then only match dest ip
  */
void ppa_delete_mc_group(PPA_MC_GROUP *ppa_mc_entry)
{
	struct mc_session_node *p_mc_item = NULL;
	uint32_t idx;
	PPA_HLIST_NODE *node_next;

	idx = SESSION_LIST_MC_HASH_VALUE(ppa_mc_entry->ip_mc_group.ip.ip);
	ppa_mc_get_htable_lock();
	ppa_hlist_for_each_entry_safe(p_mc_item,node_next,&g_mc_group_list_hash_table[idx],mc_hlist) {
		if (ip_equal(&p_mc_item->grp.ip_mc_group, &ppa_mc_entry->ip_mc_group)) {/*mc group ip match*/
			if (!ppa_mc_entry->SSM_flag || ip_equal(&p_mc_item->grp.source_ip, &ppa_mc_entry->source_ip)) {
				ppa_remove_mc_group_item(p_mc_item);
				ppa_free_mc_session_node(p_mc_item);
				if (ppa_mc_entry->SSM_flag) {/*src ip specific, so only one can get matched*/
					break;
				}
			}
		}
	}
	ppa_mc_release_htable_lock();
	return;
}

static int32_t ppa_mc_itf_get(char *itf, struct netif_info **pp_netif_info)
{
	if (!itf) {
		return PPA_FAILURE;
	}
	return ppa_netif_lookup(itf, pp_netif_info);
}

static int32_t ppa_mc_check_src_itf(char *itf, struct mc_session_node *p_item)
{
	struct netif_info *p_netif_info = NULL;

	if (itf != NULL) {
		if (ppa_mc_itf_get(itf, &p_netif_info) != PPA_SUCCESS)
			return PPA_FAILURE;
		if (!(p_netif_info->flags & NETIF_PHYS_PORT_GOT)) {
			ppa_netif_put(p_netif_info);
			return PPA_FAILURE;
		}
		p_item->grp.src_netif = p_netif_info->netif;
		if (p_netif_info->flags & NETIF_VLAN_INNER) {
			p_item->flags |= SESSION_VALID_VLAN_RM;
		}
		if (p_netif_info->flags & NETIF_VLAN_OUTER) {
			p_item->flags |= SESSION_VALID_OUT_VLAN_RM;
		}
		if (p_netif_info->netif->type == ARPHRD_SIT) {/*6rd Device*/
			p_item->flags |= SESSION_TUNNEL_6RD;
		}
		if (p_netif_info->netif->type == ARPHRD_TUNNEL6) {/*dslite Device*/
			p_item->flags |= SESSION_TUNNEL_DSLITE;
		}
		ppa_netif_put(p_netif_info);
	} else {
		p_item->grp.src_netif = NULL;
	}
	return PPA_SUCCESS;
}

static int32_t ppa_mc_check_dst_itf(PPA_MC_GROUP *p_mc_group, struct mc_session_node *p_item)
{
	int i = 0;
	int first_dst = 1;
	struct netif_info *p_netif_info = NULL;
	PPA_NETIF *br_dev;

	for (i = 0; i < PPA_MAX_MC_IFS_NUM; i++) {
		if (p_mc_group->array_mem_ifs != NULL) {

			if (strcmp(p_mc_group->array_mem_ifs[i].ifname, "NULL") == 0)
			{
				continue;
			}

			if (ppa_mc_itf_get(p_mc_group->array_mem_ifs[i].ifname, &p_netif_info) != PPA_SUCCESS)
			{
				return PPA_FAILURE;
			}

		}

		if (!(p_netif_info->flags & NETIF_PHYS_PORT_GOT)) {
			ppa_netif_put(p_netif_info);
			return PPA_FAILURE;
		}

		if (first_dst) {
			first_dst = 0;
			if (!p_item->bridging_flag) {/*route mode, need replace the src mac address*/
				/*if the dst device is in the bridge, we try to get bridge's src mac address.  */
				br_dev = ppa_get_br_dev(p_netif_info->netif);
				p_item->flags |= SESSION_VALID_SRC_MAC; 
			}
			/*if no vlan,reset value to zero in case it is update*/
			p_item->grp.new_vci = p_netif_info->flags & NETIF_VLAN_INNER ? p_netif_info->inner_vid : 0;
			p_item->grp.out_vlan_tag = p_netif_info->flags & NETIF_VLAN_OUTER ? p_netif_info->outer_vid : ~0;
			p_item->flags |= p_netif_info->flags & NETIF_VLAN_INNER ? SESSION_VALID_VLAN_INS : 0;
			p_item->flags |= p_netif_info->flags & NETIF_VLAN_OUTER ? SESSION_VALID_OUT_VLAN_INS : 0;
		} else {
			if ((p_netif_info->flags & NETIF_VLAN_INNER && !(p_item->flags & SESSION_VALID_VLAN_INS))
			|| (!(p_netif_info->flags & NETIF_VLAN_INNER) && p_item->flags & SESSION_VALID_VLAN_INS)
			|| (p_netif_info->flags & NETIF_VLAN_OUTER && !(p_item->flags & SESSION_VALID_OUT_VLAN_INS))
			|| (!(p_netif_info->flags & NETIF_VLAN_OUTER) && p_item->flags & SESSION_VALID_OUT_VLAN_INS)
			|| ((p_item->flags & SESSION_VALID_VLAN_INS) && p_item->grp.new_vci != p_netif_info->inner_vid) 
			|| ((p_item->flags & SESSION_VALID_OUT_VLAN_INS) && p_item->grp.out_vlan_tag != p_netif_info->outer_vid)) {
				goto DST_VLAN_FAIL;
			}
		}
#if IS_ENABLED(CONFIG_SOC_GRX500)
		if ((p_netif_info->flags & NETIF_VLAN_INNER || p_netif_info->flags & NETIF_VLAN_OUTER)
			|| (p_netif_info->flags & NETIF_DIRECTPATH)
#if defined(CONFIG_PPA_API_DIRECTCONNECT) && CONFIG_PPA_API_DIRECTCONNECT
			/* For directconnect destif, find out the num of VAP and dest subif */
			|| (p_netif_info->flags & NETIF_DIRECTCONNECT)
#endif
		) {
		/* For the 1st Virtual interface, update the session subifid */
			if (p_item->num_vap == 0) {
				p_item->num_vap++;
				p_item->dest_subifid = p_netif_info->subif_id;
			} else if (p_item->num_vap == 1) {
			/* For the next Virtual interface, if it is different subifid, then ignore session subifid(set it to 0) */
				if (p_item->dest_subifid != p_netif_info->subif_id) {
					p_item->num_vap++;
					p_item->dest_subifid = 0;
				}
			}
		}
#endif
		p_item->dest_ifid |= 1 << p_netif_info->phys_port;
		p_item->grp.txif[i].netif = p_netif_info->netif;
		p_item->grp.txif[i].ttl   = p_mc_group->array_mem_ifs[i].ttl;
		p_item->grp.txif[i].if_flags = p_netif_info->flags;
		p_item->grp.if_mask |= 1 << i;
		ppa_netif_put(p_netif_info);
	}
	p_item->grp.num_ifs = p_mc_group->num_ifs;
	return PPA_SUCCESS;

DST_VLAN_FAIL:
	ppa_netif_put(p_netif_info);
	return PPA_FAILURE;
}


int32_t ppa_mc_group_setup(PPA_MC_GROUP *p_mc_group, struct mc_session_node *p_item, uint32_t flags)
{
	struct netif_info *rx_ifinfo=NULL;
	PPA_NETIF *rx_netif = NULL;
	PPA_NETIF *tmp_src_netif = NULL;
	const PPA_IFNAME *const_ifname;
	bool isIgEoGre = false;
	int i;

	p_item->bridging_flag = p_mc_group->bridging_flag;

	/*Initializing p_item->grp.netif pointers with NULL...All 16 values */
	for (i = 0; i < PPA_MAX_MC_IFS_NUM; i++) {
		p_item->grp.txif[i].netif = NULL;
	}

	/*Routed multicast session */
	if (!p_item->bridging_flag) {
		/*Get the source netdev info*/
		if (ppa_netif_lookup(p_mc_group->src_ifname, &rx_ifinfo) != PPA_SUCCESS) {
			ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,"failed in getting info structure of rx_if (%s)\n", p_mc_group->src_ifname);
			SET_DBG_FLAG(p_item, SESSION_DBG_RX_IF_NOT_IN_IF_LIST);
			return PPA_FAILURE;
		}

		if ((rx_ifinfo->flags & (NETIF_WAN_IF | NETIF_PPPOE)) == (NETIF_WAN_IF | NETIF_PPPOE)) {
			p_item->flags |= SESSION_VALID_PPPOE;
		}

		if ((rx_ifinfo->flags & NETIF_GRE_TUNNEL) == NETIF_GRE_TUNNEL) {
			p_item->flag2 |= SESSION_FLAG2_GRE;
			p_item->flag2 |= SESSION_FLAG2_IG_GRE;
			isIgEoGre = ((rx_ifinfo->greInfo.flowId == FLOWID_IPV4_EoGRE) ||
				    (rx_ifinfo->greInfo.flowId == FLOWID_IPV6_EoGRE)) ? true : false;
		}

		if ((rx_ifinfo->flags & NETIF_PPPOL2TP) ==  NETIF_PPPOL2TP) {
			p_item->flags |= SESSION_VALID_PPPOL2TP;
		}

		const_ifname= p_mc_group->src_ifname;
		if ((rx_netif = ppa_dev_get_by_name(const_ifname)) != NULL) {
			if (ppa_netif_type(rx_netif) == ARPHRD_SIT) {
				p_item->flags |= SESSION_TUNNEL_6RD;
			}
			if (ppa_netif_type(rx_netif) == ARPHRD_TUNNEL6) {
				p_item->flags |= SESSION_TUNNEL_DSLITE;
			}
			ppa_put_netif (rx_netif);
		}
		tmp_src_netif = rx_ifinfo->netif;
		ppa_netif_put((struct netif_info *)rx_ifinfo);
	}

	p_item->SSM_flag = p_mc_group->SSM_flag;

	p_item->grp.group_id = p_mc_group->group_id;
	p_item->grp.if_mask = p_mc_group->if_mask;

#if IS_ENABLED(CONFIG_SOC_GRX500)
	ppa_memcpy(p_item->s_mac, p_mc_group->src_mac, PPA_ETH_ALEN);

	/** Note:
	 * For EoGRE Routing, GRE outer MAC address is required to be configured in PAE MAC table.
	 *
	 * If source MAC address is bridged(say, inner MAC), extract GRE outer MAC.
	 * Otherwise, explicitely ignore to avoid PAE/EoGRE routing disruption.
	 */
	if (isIgEoGre) {
		PPA_NETIF *br_memif = NULL;
		PPA_NETIF *br_if = NULL;

		if (tmp_src_netif && br_port_exists(tmp_src_netif)) {
			br_if = ppa_get_br_dev(tmp_src_netif);
			if ((ppa_get_br_dst_port_with_mac(br_if, p_item->s_mac, &br_memif)) == PPA_SUCCESS) {
				if (ppa_get_gre_dmac(p_item->s_mac, tmp_src_netif, NULL) != PPA_SUCCESS) {
					ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT, "failed to update s_mac(%pM) for rx_if (%s)\n",
							p_mc_group->src_mac, p_mc_group->src_ifname);
					ppa_memset(p_item->s_mac, 0, PPA_ETH_ALEN);
				}
			}
		}
	}
#endif

	/*check src interface*/
	if (ppa_mc_check_src_itf(p_mc_group->src_ifname, p_item) != PPA_SUCCESS)
		return PPA_FAILURE;

	/*check dst interface*/
	if (ppa_mc_check_dst_itf(p_mc_group, p_item) != PPA_SUCCESS) {
		return PPA_FAILURE;
	}

	/*multicast  address*/
	ppa_memcpy(&p_item->grp.ip_mc_group, &p_mc_group->ip_mc_group, sizeof(p_mc_group->ip_mc_group));
	/*source ip address*/
	ppa_memcpy(&p_item->grp.source_ip, &p_mc_group->source_ip, sizeof(p_mc_group->source_ip));
	p_item->dslwan_qid = p_mc_group->dslwan_qid;

	if ((flags & PPA_F_SESSION_VLAN)) {
		if (p_mc_group->vlan_insert) {
			p_item->flags |= SESSION_VALID_VLAN_INS;
			p_item->grp.new_vci = ((uint32_t)p_mc_group->vlan_prio << 13) |
				((uint32_t)p_mc_group->vlan_cfi << 12) | (uint32_t)p_mc_group->vlan_id;
		}
		if (p_mc_group->vlan_remove)
			p_item->flags |= SESSION_VALID_VLAN_RM;
	}

	if ((flags & PPA_F_SESSION_OUT_VLAN)) {
		if (p_mc_group->out_vlan_insert) {
			p_item->flags |= SESSION_VALID_OUT_VLAN_INS;
			p_item->grp.out_vlan_tag = p_mc_group->out_vlan_tag;
		}
		if (p_mc_group->out_vlan_remove)
			p_item->flags |= SESSION_VALID_OUT_VLAN_RM;
	}

	if ((flags & PPA_F_SESSION_NEW_DSCP)) {
		if (p_mc_group->new_dscp_en) {
			p_item->grp.new_dscp = p_mc_group->new_dscp;
			p_item->flags |= SESSION_VALID_NEW_DSCP;
		}
	}
	return PPA_SUCCESS;
}

#if 0
/*ppa_mc_group_checking: check whether it is valid acceleration session. the result value :
	1) if not found any valid downstream interface, includes num_ifs is zero: return PPA_FAILURE
	2) if downstream interface's VLAN tag not complete same: return PPA_FAILURE
	3) if p_item is NULL: return PPA_ENOMEM; */
int32_t ppa_mc_group_checking(PPA_MC_GROUP *p_mc_group, struct mc_session_node *p_item, uint32_t flags)
{
	struct netif_info *p_netif_info;
	uint32_t bit;
	uint32_t idx;
	uint32_t i, bfAccelerate=1, tmp_flag = 0, tmp_out_vlan_tag=0;
	uint16_t  tmp_new_vci=0, bfFirst = 1 ;
	uint8_t netif_mac[PPA_ETH_ALEN], tmp_mac[PPA_ETH_ALEN];

	if (!p_item)
		return PPA_ENOMEM;

		/*before updating p_item, need to clear some previous values, 
	but cannot memset all p_item esp for p_item's next pointer.*/
	p_item->dest_ifid = 0;
	p_item->grp.if_mask = 0;
	p_item->grp.new_dscp = 0;
	p_item->bridging_flag = p_mc_group->bridging_flag;

	for (i = 0, bit = 1, idx = 0; i < PPA_MAX_MC_IFS_NUM && idx < p_mc_group->num_ifs; i++, bit <<= 1) {
		if (p_mc_group->if_mask & bit) {
			ppa_debug(DBG_ENABLE_MASK_DUMP_MC_GROUP,"group checking itf: %s\n",
				p_mc_group->array_mem_ifs[i].ifname);
			if (ppa_netif_lookup(p_mc_group->array_mem_ifs[i].ifname, &p_netif_info) == PPA_SUCCESS) {
				if (!(p_netif_info->flags & NETIF_PHYS_PORT_GOT)) {
							/* dest interface*/
					ppa_netif_put(p_netif_info);
					ppa_debug(DBG_ENABLE_MASK_DUMP_MC_GROUP, "Warning: No PHYS found for interface %s\n",
						p_mc_group->array_mem_ifs[i].ifname);
					bfAccelerate = 0;
					break;
				}
				if (bfFirst) {
				/* keep the first interface's flag. Make sure all interface's vlan action 
				should be same, otherwise PPE FW cannot do it */
					tmp_flag = p_netif_info->flags;
					tmp_new_vci = p_netif_info->inner_vid;
					tmp_out_vlan_tag = p_netif_info->outer_vid;
					/*if the multicast entry has multiple output interface, 
					make sure they must has same MAC address
					the devices in the bridge will get the bridge's mac address.*/
					ppa_get_netif_hwaddr(p_netif_info->netif,netif_mac, 1);
					bfFirst = 0;
				} else {
					if ((tmp_flag & (NETIF_VLAN_OUTER | NETIF_VLAN_INNER)) !=
						(p_netif_info->flags & (NETIF_VLAN_OUTER | NETIF_VLAN_INNER))) {
						bfAccelerate = 0;
						ppa_debug(DBG_ENABLE_MASK_DUMP_MC_GROUP, 
							"ppa_add_mc_group not same flag(%0x_%0x)\n",
							tmp_flag & (NETIF_VLAN_OUTER | NETIF_VLAN_INNER),
							p_netif_info->flags & (NETIF_VLAN_OUTER | NETIF_VLAN_INNER));
						ppa_netif_put(p_netif_info);
						break;
					} else if (tmp_out_vlan_tag != p_netif_info->outer_vid) {
						bfAccelerate = 0;
						ppa_debug(DBG_ENABLE_MASK_DUMP_MC_GROUP, 
							"ppa_add_mc_group not same out vlan tag(%0x_%0x)\n",
							tmp_out_vlan_tag, p_netif_info->outer_vid);
						ppa_netif_put(p_netif_info);
						break;
					} else if (tmp_new_vci != p_netif_info->inner_vid) {
						bfAccelerate = 0;
						ppa_debug(DBG_ENABLE_MASK_DUMP_MC_GROUP,
							"ppa_add_mc_group not same inner vlan(%0x_%0x)\n",
							tmp_new_vci , p_netif_info->inner_vid);
						ppa_netif_put(p_netif_info);
						break;
					}

					ppa_get_netif_hwaddr(p_netif_info->netif,tmp_mac, 1);
					if (ppa_memcmp(netif_mac, tmp_mac, sizeof(tmp_mac))) {
						bfAccelerate = 0;
						ppa_debug(DBG_ENABLE_MASK_DUMP_MC_GROUP,
							"ppa_add_mc_group not same mac address\n");
						ppa_netif_put(p_netif_info);
						break;
					}
				}
				p_item->dest_ifid |= 1 << p_netif_info->phys_port;

				if (!(p_netif_info->flags & NETIF_MAC_ENTRY_CREATED))
					ppa_debug(DBG_ENABLE_MASK_ASSERT,"ETH physical interface must have MAC address\n");
				if (!p_mc_group->bridging_flag)
					p_item->flags |= SESSION_VALID_SRC_MAC;
				else { 
					/*otherwise clear this bit in case it is set beofre calling this API*/
					p_item->flags &= ~SESSION_VALID_SRC_MAC;
				}

				p_item->grp.txif[idx].netif = p_netif_info->netif;
				p_item->grp.txif[idx].ttl  = p_mc_group->array_mem_ifs[i].ttl;
				p_item->grp.txif[idx].if_flags = tmp_flag;
				p_item->grp.if_mask |= 1 << idx;

				ppa_netif_put(p_netif_info);
				idx++;
			} else {
				bfAccelerate = 0;
				ppa_debug(DBG_ENABLE_MASK_DUMP_MC_GROUP, "ppa_add_mc_group cannot find the interface:%s)\n",
					p_mc_group->array_mem_ifs[i].ifname);
				break;
			}
		}
		}

	if (bfAccelerate == 0 || idx == 0 || (!p_mc_group->bridging_flag &&
				!(p_item->flags & SESSION_VALID_SRC_MAC))) {
		return PPA_FAILURE;
	}

	/*VLAN*/
	if (!(tmp_flag & NETIF_VLAN_CANT_SUPPORT))
		ppa_debug(DBG_ENABLE_MASK_ASSERT,"MC processing can support two layers of VLAN only\n");

	if ((tmp_flag & NETIF_VLAN_OUTER)) {
		p_item->grp.out_vlan_tag = tmp_out_vlan_tag;
		p_item->flags |= SESSION_VALID_OUT_VLAN_INS;
		ppa_debug(DBG_ENABLE_MASK_DUMP_MC_GROUP, "set SESSION_VALID_OUT_VLAN_INS:%x_%x\n", 
				p_item->grp.out_vlan_tag, tmp_new_vci);
	} else { 
	/*otherwise clear this bit in case it is set beofre calling this API*/
		p_item->flags &= ~SESSION_VALID_OUT_VLAN_INS;
		ppa_debug(DBG_ENABLE_MASK_DUMP_MC_GROUP, "unset SESSION_VALID_OUT_VLAN_INS\n");
	}

	if ((tmp_flag & NETIF_VLAN_INNER)) {
		p_item->grp.new_vci = tmp_new_vci;
		p_item->flags |= SESSION_VALID_VLAN_INS;
		ppa_debug(DBG_ENABLE_MASK_DUMP_MC_GROUP, "set SESSION_VALID_VLAN_INS:%x\n", p_item->grp.new_vci);
	} else {
	/*otherwise clear this bit in case it is set beofre calling this API*/
		p_item->flags &= ~SESSION_VALID_VLAN_INS;
		ppa_debug(DBG_ENABLE_MASK_DUMP_MC_GROUP, "unset SESSION_VALID_VLAN_INS\n");
	}

   	/*PPPOE*/
	if (!p_mc_group->bridging_flag)
		p_item->flags |= SESSION_VALID_PPPOE;
	else {
		p_item->flags &= ~SESSION_VALID_PPPOE;
	}

	/*multicast  address*/
	ppa_memcpy(&p_item->grp.ip_mc_group, &p_mc_group->ip_mc_group, sizeof(p_mc_group->ip_mc_group));
	ppa_memcpy(&p_item->grp.source_ip, &p_mc_group->source_ip, sizeof(p_mc_group->source_ip));

	if (p_mc_group->ip_mc_group.f_ipv6)
		p_item->flags |= SESSION_IS_IPV6;

	if (p_mc_group->src_ifname && ppa_netif_lookup(p_mc_group->src_ifname, &p_netif_info) == PPA_SUCCESS) {
		if (p_netif_info->flags & NETIF_PHYS_PORT_GOT) {
			if (!p_mc_group->bridging_flag && (p_netif_info->flags & NETIF_PPPOE))
				p_item->flags |= SESSION_VALID_PPPOE;
			else
				p_item->flags &= ~SESSION_VALID_PPPOE;

			if (!(p_netif_info->flags & NETIF_VLAN_CANT_SUPPORT))
				ppa_debug(DBG_ENABLE_MASK_ASSERT, 
					"MC processing can support two layers of VLAN only\n");

			if ((p_netif_info->flags & NETIF_VLAN_OUTER))
				p_item->flags |= SESSION_VALID_OUT_VLAN_RM;
			else
				p_item->flags &= ~SESSION_VALID_OUT_VLAN_RM;

			if ((p_netif_info->flags & NETIF_VLAN_INNER))
				p_item->flags |= SESSION_VALID_VLAN_RM;
			else
				p_item->flags &= ~SESSION_VALID_VLAN_RM;

			if (p_netif_info->netif->type == ARPHRD_SIT)
				p_item->flags |= SESSION_TUNNEL_6RD;
			else
				p_item->flags &= ~SESSION_TUNNEL_6RD;

			if (p_netif_info->netif->type == ARPHRD_TUNNEL6)
				p_item->flags |= SESSION_TUNNEL_DSLITE;
			else
				p_item->flags &= ~SESSION_TUNNEL_DSLITE;
																										} else {
			/*not allowed to support non-physical interfaces,like bridge */
			ppa_netif_put(p_netif_info);
			return PPA_FAILURE;
		}
		p_item->grp.src_netif = p_netif_info->netif;
		ppa_netif_put(p_netif_info);
	}
	p_item->grp.num_ifs = idx;
	p_item->dslwan_qid = p_mc_group->dslwan_qid;

	/*force update some status by hook itself*/
	if ((flags & PPA_F_SESSION_VLAN)) {
		if (p_mc_group->vlan_insert) {
			p_item->flags |= SESSION_VALID_VLAN_INS;
			p_item->grp.new_vci = ((uint32_t)p_mc_group->vlan_prio << 13) |
				((uint32_t)p_mc_group->vlan_cfi << 12) | (uint32_t)p_mc_group->vlan_id;
		} else {
			p_item->flags &= ~SESSION_VALID_VLAN_INS;
			p_item->grp.new_vci = 0;
		}

		if (p_mc_group->vlan_remove)
			p_item->flags |= SESSION_VALID_VLAN_RM;
		else
			p_item->flags &= ~SESSION_VALID_VLAN_RM;
	}

	if ((flags & PPA_F_SESSION_OUT_VLAN)) {
		if (p_mc_group->out_vlan_insert) {
			p_item->flags |= SESSION_VALID_OUT_VLAN_INS;
			p_item->grp.out_vlan_tag = p_mc_group->out_vlan_tag;
		} else {
			p_item->flags &= ~SESSION_VALID_OUT_VLAN_INS;
			p_item->grp.out_vlan_tag = 0;
		}

		if (p_mc_group->out_vlan_remove)
			p_item->flags |= SESSION_VALID_OUT_VLAN_RM;
		else
			p_item->flags &= ~SESSION_VALID_OUT_VLAN_RM;
	}

	if ((flags & PPA_F_SESSION_NEW_DSCP)) {
		if (p_mc_group->new_dscp_en) {
			p_item->grp.new_dscp = p_mc_group->new_dscp;
			p_item->flags |= SESSION_VALID_NEW_DSCP;
		} else
		p_item->grp.new_dscp &= ~SESSION_VALID_NEW_DSCP;
	}

	ppa_debug(DBG_ENABLE_MASK_DUMP_MC_GROUP, "mc flag:%x\n", p_item->flags);
	return PPA_SUCCESS;
}
#endif

int32_t ppa_add_mc_group(PPA_MC_GROUP *p_mc_group, struct mc_session_node **pp_item, uint32_t flags)
{
	struct mc_session_node *p_item;

	/*Allocate memory for mc db node */
	p_item = ppa_alloc_mc_session_node();
	if (!p_item)
		return PPA_ENOMEM;

	/*Extract and fill in the necessary information in the mc db node */
	if (ppa_mc_group_setup(p_mc_group, p_item, flags) != PPA_SUCCESS) {
		ppa_mem_cache_free(p_item, g_mc_group_item_cache);
		return PPA_FAILURE;
	}

	/*Inset the node in mc db*/
	ppa_insert_mc_group_item(p_item);
	*pp_item = p_item;
	return PPA_SUCCESS;
}

int32_t ppa_update_mc_group_extra(PPA_SESSION_EXTRA *p_extra, 
			struct mc_session_node *p_item, uint32_t flags)
{
	if ((flags & PPA_F_SESSION_NEW_DSCP)) {
		if (p_extra->dscp_remark) {
			p_item->flags |= SESSION_VALID_NEW_DSCP;
			p_item->grp.new_dscp = p_extra->new_dscp;
		} else
			p_item->flags &= ~SESSION_VALID_NEW_DSCP;
	}

	if ((flags & PPA_F_SESSION_VLAN)) {
			if (p_extra->vlan_insert) {
			p_item->flags |= SESSION_VALID_VLAN_INS;
			p_item->grp.new_vci = ((uint32_t)p_extra->vlan_prio << 13)
				| ((uint32_t)p_extra->vlan_cfi << 12) | p_extra->vlan_id;
		} else {
			p_item->flags &= ~SESSION_VALID_VLAN_INS;
			p_item->grp.new_vci = 0;
		}

		if (p_extra->vlan_remove)
			p_item->flags |= SESSION_VALID_VLAN_RM;
		else
			p_item->flags &= ~SESSION_VALID_VLAN_RM;
		}

	if ((flags & PPA_F_SESSION_OUT_VLAN)) {
		if (p_extra->out_vlan_insert) {
 			p_item->flags |= SESSION_VALID_OUT_VLAN_INS;
			p_item->grp.out_vlan_tag = p_extra->out_vlan_tag;
			} else {
					p_item->flags &= ~SESSION_VALID_OUT_VLAN_INS;
					p_item->grp.out_vlan_tag = 0;
			}

		if (p_extra->out_vlan_remove)
					p_item->flags |= SESSION_VALID_OUT_VLAN_RM;
		else
			p_item->flags &= ~SESSION_VALID_OUT_VLAN_RM;
	}

	if (p_extra->dslwan_qid_remark)
		p_item->dslwan_qid = p_extra->dslwan_qid;

	return PPA_SUCCESS;
}

void ppa_remove_mc_group(struct mc_session_node *p_item)
{
	ppa_debug(DBG_ENABLE_MASK_DUMP_MC_GROUP,
		"ppa_remove_mc_group:  remove %d from PPA\n",p_item->mc_entry);
	ppa_remove_mc_group_item(p_item);

	ppa_free_mc_session_node(p_item);
}

int32_t ppa_mc_group_start_iteration(uint32_t *ppos, struct mc_session_node **pp_item)
{
	struct mc_session_node *p = NULL;
	int idx;
	uint32_t l;

	l = *ppos + 1;

	ppa_lock_get(&g_mc_group_list_lock);
	if (!ppa_is_init()) {
		*pp_item = NULL;
		return PPA_FAILURE; 
	}
	for (idx = 0; l && idx < NUM_ENTITY(g_mc_group_list_hash_table); idx++) {
		ppa_hlist_for_each_entry(p, &g_mc_group_list_hash_table[idx], mc_hlist) {
			if (!--l)
				break;
		}
	}

	if (l == 0 && p) {
		++*ppos;
		*pp_item = p;
		return PPA_SUCCESS;
	} else {
		*pp_item = NULL;
		return PPA_FAILURE;
	}
}
EXPORT_SYMBOL(ppa_mc_group_start_iteration);

int32_t ppa_mc_group_iterate_next(uint32_t *ppos, struct mc_session_node **pp_item)
{
	uint32_t idx;

	if (*pp_item == NULL)
		return PPA_FAILURE;

	if ((*pp_item)->mc_hlist.next != NULL) {
		++*ppos;
		*pp_item = ppa_hlist_entry((*pp_item)->mc_hlist.next,
				struct mc_session_node, mc_hlist);
		return PPA_SUCCESS;
	} else {
		for (idx = SESSION_LIST_MC_HASH_VALUE((*pp_item)->grp.ip_mc_group.ip.ip) + 1;
			idx < NUM_ENTITY(g_mc_group_list_hash_table); idx++)
			if (g_mc_group_list_hash_table[idx].first != NULL) {
				++*ppos;
				*pp_item = ppa_hlist_entry(g_mc_group_list_hash_table[idx].first,
						struct mc_session_node, mc_hlist);
				return PPA_SUCCESS;
			}
		*pp_item = NULL;
		return PPA_FAILURE;
	}
}
EXPORT_SYMBOL(ppa_mc_group_iterate_next);

#if IS_ENABLED(CONFIG_PPA_MPE_IP97)
void ppa_ipsec_get_session_lock(void)
{
	ppa_lock_get(&g_ipsec_group_list_lock);
}

void ppa_ipsec_release_session_lock(void)
{
	ppa_lock_release(&g_ipsec_group_list_lock);
}

static INLINE void ppa_remove_ipsec_group_item(struct uc_session_node *p_item)
{
	ppa_list_del((struct list_head *)&p_item->hlist);
}

INLINE void ppa_free_ipsec_group_list_item(struct uc_session_node *p_item)
{
	ppa_mem_cache_free(p_item, g_ipsec_group_item_cache);
}

void ppa_remove_ipsec_group(struct uc_session_node *p_item)
{
	ppa_remove_ipsec_group_item(p_item);
	ppa_free_ipsec_group_list_item(p_item);
}

static INLINE struct uc_session_node *ppa_alloc_ipsec_group_list_item(void)
{
	struct uc_session_node *p_item;

	p_item = ppa_mem_cache_alloc(g_ipsec_group_item_cache);
	if (p_item) {
		ppa_memset(p_item, 0, sizeof(*p_item));
	}
	return p_item;
}

int32_t ppa_ipsec_add_entry(uint32_t tunnel_index)
{
	struct uc_session_node *p_item;
	int32_t ret = PPA_SESSION_NOT_ADDED;
	PPA_IFNAME phys_netif_name[PPA_IF_NAME_SIZE];
	dp_subif_t dp_subif={0};
	struct netif_info *p_ifinfo;

	p_item = ppa_alloc_ipsec_group_list_item();
	if (!p_item)
		return PPA_ENOMEM;

	if (ppa_is_ipv4_ipv6(g_tunnel_table[tunnel_index]->tunnel_info.ipsec_hdr.inbound) == SESSION_IPV4)
		p_item->flags	|= 0x0;/*SESSION_IS_IPV4*/
	else
		p_item->flags	|= SESSION_IS_IPV6;

	p_item->flag2		|= SESSION_FLAG2_VALID_IPSEC_INBOUND;
	p_item->flags		|= SESSION_WAN_ENTRY;
	p_item->pkt.ip_proto	= IP_PROTO_ESP;
	p_item->hash		= g_tunnel_table[tunnel_index]->tunnel_info.ipsec_hdr.inbound->props.family;

	p_item->pkt.src_ip	= *(PPA_IPADDR *)(&(g_tunnel_table[tunnel_index]->tunnel_info.ipsec_hdr.inbound->props.saddr));
	p_item->pkt.src_port	= g_tunnel_table[tunnel_index]->tunnel_info.ipsec_hdr.inbound->id.spi >> 16;
	p_item->pkt.dst_ip	= *(PPA_IPADDR *)(&(g_tunnel_table[tunnel_index]->tunnel_info.ipsec_hdr.inbound->id.daddr));
	p_item->pkt.dst_port	= g_tunnel_table[tunnel_index]->tunnel_info.ipsec_hdr.inbound->id.spi & 0xFFFF;
	p_item->tunnel_idx	= tunnel_index;

	if (ppa_get_physical_if (ipsec_tnl_info.tx_if, NULL, phys_netif_name) == PPA_SUCCESS) {
		if (ppa_netif_lookup(phys_netif_name, &p_ifinfo) == PPA_SUCCESS) {
			dp_subif.port_id =p_ifinfo->phys_port;
			if ((dp_get_netif_subifid(ppa_get_netif (p_ifinfo->manual_lower_ifname),
					NULL, p_ifinfo->vcc, NULL, &dp_subif, 0)) == PPA_SUCCESS) {
				p_item->dest_subifid  = (dp_subif.subif & 0xF00) >> 7;
			} else {
				printk("\nfailed to get sub interface id\n");
			}
		}
		ppa_netif_put(p_ifinfo);
	}

	ppa_list_add((struct list_head *)&p_item->hlist,&ipsec_list_head);
  
	/* place holder for some explicit criteria based on which the session 
	 must go through SW acceleration */
	if (ppa_hw_accelerate(NULL, p_item)) {
		if ((ret = ppa_hsel_add_routing_session(p_item, NULL, 0)) != PPA_SUCCESS) {
			p_item->flag2 |= SESSION_FLAG2_ADD_HW_FAIL;/*PPE hash full in this hash index, or IPV6 table full ,...*/
			ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,"ppa_hw_add_session(p_item) fail\n");
		} else {
			p_item->flag2 &= ~SESSION_FLAG2_ADD_HW_FAIL;
		}
	}

	ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,"hardware/firmware entry added\n");
	ret = PPA_SESSION_ADDED;

	return ret;
}

int32_t ppa_ipsec_add_entry_outbound(uint32_t tunnel_index)
{
	struct uc_session_node *p_item;
	int32_t ret = PPA_SESSION_NOT_ADDED;
	
	p_item = ppa_alloc_ipsec_group_list_item();
	if (!p_item)
		return PPA_ENOMEM;

	p_item->flag2	= SESSION_FLAG2_VALID_IPSEC_OUTBOUND_SA;

	p_item->tunnel_idx = tunnel_index;

	/* place holder for some explicit criteria based on which the session 
	 must go through SW acceleration */
	if (ppa_hw_accelerate(NULL, p_item)) {
		if ((ret = ppa_hsel_add_routing_session(p_item, NULL, 0)) != PPA_SUCCESS) {
			/*PPE hash full in this hash index, or IPV6 table full ,...*/
			p_item->flag2 |= SESSION_FLAG2_ADD_HW_FAIL;
			ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,"ppa_hw_add_session(p_item) fail\n");
		} else {
			p_item->flag2 &= ~SESSION_FLAG2_ADD_HW_FAIL;
		}
	}

	ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,"hardware/firmware entry added\n");
	ret = PPA_SESSION_ADDED;

	ppa_free_ipsec_group_list_item(p_item);
	return ret;
}

int32_t __ppa_lookup_ipsec_group(PPA_XFRM_STATE *ppa_x, struct uc_session_node **pp_item)
{
	PPA_LIST_NODE *node;
	struct uc_session_node *p_ipsec_item = NULL;

	if (!pp_item) {
		ppa_debug(DBG_ENABLE_MASK_ASSERT,"pp_item == NULL\n");
		return PPA_SESSION_NOT_ADDED;
	}

	ppa_list_for_each(node, &ipsec_list_head) {
		p_ipsec_item = ppa_list_entry((PPA_HLIST_NODE *)node, struct uc_session_node , hlist);
		if (p_ipsec_item->hash == ppa_x->props.family) {
			if (ppa_ipsec_addr_equal((PPA_XFRM_ADDR *)&(p_ipsec_item->pkt.src_ip),
							&(ppa_x->props.saddr), ppa_x->props.family)) {
				if (ppa_ipsec_addr_equal((PPA_XFRM_ADDR *)&(p_ipsec_item->pkt.dst_ip),
								&(ppa_x->id.daddr), ppa_x->props.family)) {
					if (p_ipsec_item->pkt.src_port == (ppa_x->id.spi >> 16)) {
						if (p_ipsec_item->pkt.dst_port == (ppa_x->id.spi & 0xFFFF)) {
							*pp_item = p_ipsec_item;
							return PPA_IPSEC_EXISTS;
						}
					}
				}
			}
		}
	}
	return PPA_IPSEC_NOT_ADDED;
}

int32_t ppa_ipsec_del_entry(struct uc_session_node *p_item)
{
	int32_t ret = PPA_SESSION_DELETED;

	ppa_hsel_del_routing_session(p_item);
	ppa_list_delete((PPA_LIST_HEAD*)&p_item->hlist);
	ppa_free_ipsec_group_list_item(p_item);

	return ret;
}

int32_t ppa_ipsec_del_entry_outbound(uint32_t tunnel_index)
{
	struct uc_session_node *p_item;
	int32_t ret = PPA_SESSION_DELETED;
	
	p_item = ppa_alloc_ipsec_group_list_item();
	if (!p_item)
		return PPA_ENOMEM;

	p_item->flag2	= SESSION_FLAG2_VALID_IPSEC_OUTBOUND_SA;
	p_item->tunnel_idx = tunnel_index;

	ppa_hsel_del_routing_session(p_item);

	ppa_free_ipsec_group_list_item(p_item);
	return ret;
}
#endif

void ppa_mc_group_stop_iteration(void)
{
	ppa_lock_release(&g_mc_group_list_lock);
}
EXPORT_SYMBOL(ppa_mc_group_stop_iteration);

void ppa_mc_get_htable_lock(void)
{
	ppa_lock_get(&g_mc_group_list_lock);
}

void ppa_mc_release_htable_lock(void)
{
	ppa_lock_release(&g_mc_group_list_lock);
}

void ppa_br_get_htable_lock(void)
{
	ppa_lock_get(&g_bridging_session_list_lock);
}

void ppa_br_release_htable_lock(void)
{
	ppa_lock_release(&g_bridging_session_list_lock);
}

static uint32_t get_time_in_msec(void)
{
	return((jiffies + HZ / 2) * 1000 / HZ);
}

uint32_t ppa_del_hw_bridging_session_by_mac(uint8_t *src_mac, short *is_drop)
{
	struct br_mac_node *br_p_item = NULL;
	uint32_t cur_time = 0;

	ppa_br_get_htable_lock();

	if (__ppa_bridging_lookup_mac(src_mac, 0, NULL, &br_p_item) == PPA_SESSION_EXISTS) {
		cur_time = get_time_in_msec();
		if (cur_time - br_p_item->last_mac_violation_time > g_mac_threshold_time) {
			br_p_item->last_mac_violation_time = cur_time;
#if IS_ENABLED(CONFIG_SOC_GRX500)
			ppa_bridging_hw_del_mac(br_p_item);
#endif
		} else {
			*is_drop = 0;
		}
	}
	ppa_br_release_htable_lock();
	return 0;
}
EXPORT_SYMBOL(ppa_del_hw_bridging_session_by_mac); 

uint32_t ppa_get_mc_group_count(uint32_t count_flag)
{
	uint32_t count = 0, idx;
	PPA_HLIST_NODE *node;

	ppa_lock_get(&g_mc_group_list_lock);
	for (idx = 0; idx < NUM_ENTITY(g_mc_group_list_hash_table); idx++) {
		ppa_hlist_for_each(node, &g_mc_group_list_hash_table[idx]) {
			count++;
		}
	}
	ppa_lock_release(&g_mc_group_list_lock);

	return count;
}

static void ppa_mc_group_replace(struct mc_session_node *old, struct mc_session_node *new)
{
	ppa_hlist_replace(&old->mc_hlist,&new->mc_hlist);
}

/*
	1. Create a new mc_item & replace the original one
	2. Delete the old one
	3. Add the entry to PPE
*/
int32_t __ppa_mc_group_update(PPA_MC_GROUP *p_mc_entry, struct mc_session_node *p_item, uint32_t flags)
{
	struct mc_session_node *p_mc_item;

	/*Allocate a new db node */
	p_mc_item = ppa_alloc_mc_session_node();
	if (!p_mc_item)
		goto UPDATE_FAIL;

	/*Extract the information and store in db node */
	if (ppa_mc_group_setup(p_mc_entry, p_mc_item, flags) != PPA_SUCCESS) {
		goto UPDATE_FAIL;
	}

#if defined(RTP_SAMPLING_ENABLE) && RTP_SAMPLING_ENABLE
	p_mc_item->RTP_flag =p_item->RTP_flag;
#endif

	/*Replace the db node with the new node */
	ppa_mc_group_replace(p_item, p_mc_item);

#if IS_ENABLED(CONFIG_SOC_GRX500)
	/*Delete the HW entry in case of legacy platform*/
	ppa_hsel_del_wan_mc_group(p_item);
	/*Copy mecesary information from the original p_item */
	p_mc_item->last_hit_time = p_item->last_hit_time;
	p_mc_item->acc_bytes = p_item->acc_bytes;
	p_mc_item->acc_pkts = p_item->acc_pkts;
	/*Free the original p_item */
	ppa_mem_cache_free(p_item, g_mc_group_item_cache);
	if (!p_mc_item->grp.src_netif) {/*NO src itf, cannot add to ppe*/
		return PPA_SUCCESS;
	}
	/*Add the new p_mc_item to HW */
	if (ppa_hsel_add_wan_mc_group(p_mc_item, &p_mc_entry->cop) != PPA_SUCCESS) {
		ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT, "can not update %s %d\n", __FUNCTION__, __LINE__);
	}
#else
	/*Copy mecesary information from the original p_item */
	p_mc_item->mc_entry = p_item->mc_entry;
	p_mc_item->last_hit_time = p_item->last_hit_time;
	p_mc_item->acc_bytes = p_item->acc_bytes;
	p_mc_item->acc_pkts = p_item->acc_pkts;
	p_mc_item->session_action = p_item->session_action;
	p_mc_item->flags |= (p_item->flags & SESSION_ADDED_IN_HW);
	p_mc_item->num_caps = p_item->num_caps;
	ppa_memcpy(p_mc_item->caps_list, p_item->caps_list, sizeof(PPA_HSEL_CAP_NODE)*p_item->num_caps);

	/*Update the existing HW session */
	if (ppa_hsel_update_wan_mc_group(p_mc_item, &p_mc_entry->cop, flags) != PPA_SUCCESS) {
		ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT, "can not update %s %d\n", __FUNCTION__, __LINE__);
	}

	/*Free the original p_item */
	ppa_mem_cache_free(p_item, g_mc_group_item_cache);
#endif /*IS_ENABLED(CONFIG_SOC_GRX500)*/

	return PPA_SUCCESS;

UPDATE_FAIL:
	if (p_mc_item) {
		ppa_remove_mc_group(p_mc_item);
	}

	return PPA_FAILURE;
}

/*
  * multicast routing fw entry update
  */
int32_t ppa_update_mc_hw_group(struct mc_session_node *p_item)
{
	return PPA_SUCCESS;
}

/*
 *  multicast routing hardware/firmware operation
 */
int32_t ppa_hw_add_mc_group(struct mc_session_node *p_item)
{
	return PPA_SUCCESS;
}

int32_t ppa_hw_update_mc_group_extra(struct mc_session_node *p_item, uint32_t flags)
{
	return PPA_SUCCESS;
}

void ppa_hw_del_mc_group(struct mc_session_node *p_item)
{
	return;
}

#if defined(RTP_SAMPLING_ENABLE) && RTP_SAMPLING_ENABLE
int32_t ppa_mc_entry_rtp_set(PPA_MC_GROUP *ppa_mc_entry)
{
	struct mc_session_node *p_mc_item = NULL;
	uint32_t idx;
	PPA_HLIST_NODE *node_next;
	int32_t entry_found = 0;

	idx = SESSION_LIST_MC_HASH_VALUE(ppa_mc_entry->ip_mc_group.ip.ip);

	ppa_mc_get_htable_lock();
	ppa_hlist_for_each_entry_safe(p_mc_item, node_next, &g_mc_group_list_hash_table[idx], mc_hlist) {
		if (ip_equal(&p_mc_item->grp.ip_mc_group, &ppa_mc_entry->ip_mc_group)) {/*mc group ip match*/
			if (is_ip_zero(&ppa_mc_entry->source_ip) || ip_equal(&p_mc_item->grp.source_ip, &ppa_mc_entry->source_ip)) {
				entry_found = 1;
				if (ppa_mc_entry->RTP_flag != p_mc_item->RTP_flag) {
					p_mc_item->RTP_flag = ppa_mc_entry->RTP_flag;
					if (ppa_hw_set_mc_rtp(p_mc_item) != PPA_SUCCESS) {
						ppa_mc_release_htable_lock();
						return PPA_FAILURE;
					}
				}
				if (ip_equal(&p_mc_item->grp.source_ip, &ppa_mc_entry->source_ip)) {
					break;
				}
			}
		}
	}		
	ppa_mc_release_htable_lock();

	if (entry_found == 1)
		 return PPA_SUCCESS;
	else
		 return PPA_FAILURE;
}

int32_t ppa_hw_set_mc_rtp(struct mc_session_node *p_item)
{
	PPA_MC_INFO mc={0};
	mc.p_entry = p_item->mc_entry;
	ppa_hsel_set_wan_mc_rtp(&mc, p_item->caps_list[0].hal_id);

	return PPA_SUCCESS;
}

int32_t ppa_hw_get_mc_rtp_sampling_cnt(struct mc_session_node *p_item)
{
	PPA_MC_INFO mc={0};
	mc.p_entry = p_item->mc_entry;

	ppa_hsel_get_mc_rtp_sampling_cnt(&mc, p_item->caps_list[0].hal_id);
	return PPA_SUCCESS;
}
#endif
/*
 *  routing polling timer
 */
void ppa_set_polling_timer(uint32_t polling_time, uint32_t force)
{
	if (g_hit_polling_time <= MIN_POLLING_INTERVAL) {
		/*already using minimal interval already*/
		return;
	} else if (polling_time < g_hit_polling_time) {
		/*remove timer*/
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,15,0)
		ppa_timer_del(&g_hit_stat_timer);
#else
		ppa_hrt_stop(&g_hit_stat_timer);
#endif
		/*  timeout can not be zero*/
		g_hit_polling_time = polling_time < MIN_POLLING_INTERVAL
				? MIN_POLLING_INTERVAL : polling_time;

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,15,0)
		/*check hit stat in case the left time is less then the new timeout*/
		ppa_check_hit_stat(0);  /*timer is added in this function*/
#else
		
		ppa_hrt_restart(&g_hit_stat_timer);
#endif
	} else if ((polling_time > g_hit_polling_time) && force) {
		g_hit_polling_time = polling_time;
	}
}
/*
 *  bridging mac operation
 */
int32_t __ppa_bridging_lookup_mac(uint8_t *mac, uint16_t fid, PPA_NETIF *netif, struct br_mac_node **pp_item)
{
	int32_t ret;
	struct br_mac_node *p_br_item;
	uint32_t idx;

	if (!pp_item) {
		ppa_debug(DBG_ENABLE_MASK_ASSERT,"pp_item == NULL\n");
		return PPA_SESSION_NOT_ADDED;
	}

	ret = PPA_SESSION_NOT_ADDED;

	idx = PPA_BRIDGING_SESSION_LIST_HASH_VALUE(mac);

	*pp_item = NULL;

	ppa_hlist_for_each_entry(p_br_item, &g_bridging_session_list_hash_table[idx],br_hlist) {
		if ((ppa_memcmp(mac, p_br_item->mac, PPA_ETH_ALEN) == 0)
#if IS_ENABLED(CONFIG_SOC_GRX500)
			&& (p_br_item->fid == fid)
#endif
		) {
			*pp_item = p_br_item;
			ret = PPA_SESSION_EXISTS;
		}
	}

	return ret;
}

int32_t ppa_bridging_add_mac(uint8_t *mac, uint16_t fid, PPA_NETIF *netif, struct br_mac_node **pp_item, uint32_t flags)
{
	struct br_mac_node *p_item;
	struct netif_info *ifinfo;

#if IS_ENABLED(CONFIG_SOC_GRX500)
#if defined(CONFIG_PPA_API_DIRECTCONNECT) && CONFIG_PPA_API_DIRECTCONNECT
	PPA_DP_SUBIF dp_subif = {0};
#endif
#endif
	if ( ppa_netif_lookup(ppa_get_netif_name(netif), &ifinfo) != PPA_SUCCESS )
		return PPA_FAILURE;

	p_item = ppa_bridging_alloc_uc_session_node();
	if (!p_item) {
		ppa_netif_put(ifinfo);
		return PPA_ENOMEM;
	}

	dump_bridging_list_item(p_item, "ppa_bridging_add_session(after init)");

	ppa_memcpy(p_item->mac, mac, PPA_ETH_ALEN);
	p_item->netif		 = netif;
	p_item->timeout	   = ppa_bridging_get_default_session_timeout();
	p_item->last_hit_time = ppa_get_time_in_sec();
#if IS_ENABLED(CONFIG_SOC_GRX500)
	p_item->fid = fid;
	p_item->ref_count++;
#endif

	/*decide destination list*/
	if (!(ifinfo->flags & NETIF_PHYS_PORT_GOT)) {
		PPA_COUNT_CFG count={0};
		ppa_drv_get_number_of_phys_port(&count, 0);
		p_item->dest_ifid =  count.num + 1;
	} else {
		p_item->dest_ifid = ifinfo->phys_port;
	}

	if (ifinfo->flags & NETIF_PHY_ATM)
		p_item->dslwan_qid = ifinfo->dslwan_qid;

	if (flags & PPA_F_STATIC_ENTRY) {
		p_item->flags	|= SESSION_STATIC;
		p_item->timeout   = ~0; /* max timeout*/
#if IS_ENABLED(CONFIG_SOC_GRX500)
	} else {
		p_item->flags |= SESSION_LAN_ENTRY; /*dynamic entry learned by bridge learning */
#endif
	}

	if (flags & PPA_F_DROP_PACKET)
		p_item->flags	|= SESSION_DROP;


#if IS_ENABLED(CONFIG_SOC_GRX500)
#if IS_ENABLED(CONFIG_PPA_API_DIRECTCONNECT)
	/* For directconnect destif, get destination SubifId={VapId, StationId} */
	if ((ifinfo->flags & NETIF_DIRECTCONNECT)) {
		PPA_NETIF *dp_netif = p_item->netif;

		/* Use phys interface <wlanX> for any logical interface on top of it. */
		if (ifinfo->flags & NETIF_PHY_IF_GOT)
			dp_netif = ppa_get_netif(ifinfo->phys_netif_name);

		dp_subif.port_id = ifinfo->phys_port;
		dp_subif.subif = ifinfo->subif_id;
		if ((dp_get_netif_subifid(dp_netif, NULL, NULL, mac, &dp_subif, 0))==DP_SUCCESS) {
			p_item->sub_ifid = dp_subif.subif;
		} else {
			ppa_netif_put(ifinfo);
			ppa_mem_cache_free(p_item, g_bridging_session_item_cache);
			return PPA_ENOTPOSSIBLE;
		}
	}
#endif
	if ((ifinfo->flags & NETIF_VLAN) || (ifinfo->flags & NETIF_PHY_ATM) || (ifinfo->flags & NETIF_DIRECTPATH)) {
		p_item->sub_ifid = ifinfo->subif_id;
	}
#endif
	ppa_netif_put(ifinfo);

	ppa_bridging_insert_session_item(p_item);

	dump_bridging_list_item(p_item, "ppa_bridging_add_session(after setting)");

	*pp_item = p_item;

	return PPA_SUCCESS;
}

int32_t ppa_bridging_flush_macs()
{
	struct br_mac_node *p_item;
	PPA_HLIST_NODE *tmp;
	uint32_t idx;

	for (idx = 0; idx < NUM_ENTITY(g_bridging_session_list_hash_table); idx++) {
		ppa_hlist_for_each_entry_safe(p_item, tmp, &g_bridging_session_list_hash_table[idx], br_hlist) {
			if (!(p_item->flags & SESSION_STATIC)) { /*bridge learned entry only delete*/
#if IS_ENABLED(CONFIG_SOC_GRX500)
				ppa_bridging_hw_del_mac(p_item);
#endif
				ppa_bridging_remove_mac(p_item);
#if IS_ENABLED(CONFIG_SOC_GRX500)
			} else {
				if (p_item->ref_count == 1) {
					ppa_bridging_hw_del_mac(p_item);
					ppa_bridging_remove_mac(p_item);
				} else {
					p_item->ref_count--;
					p_item->flags &= ~SESSION_LAN_ENTRY;
				}
#endif
			}
		}
	}

	return PPA_SUCCESS;
}

void ppa_bridging_remove_mac(struct br_mac_node *p_item)
{
	ppa_bridging_remove_session_item(p_item);

	ppa_bridging_free_uc_session_node(p_item);
}

void dump_bridging_list_item(struct br_mac_node *p_item, char *comment)
{
#if defined(DEBUG_DUMP_LIST_ITEM) && DEBUG_DUMP_LIST_ITEM

	if (!(g_ppa_dbg_enable & DBG_ENABLE_MASK_DUMP_BRIDGING_SESSION))
		return;

	if (comment)
		printk("dump_bridging_list_item - %s\n", comment);
	else
		printk("dump_bridging_list_item\n");
	printk("  next			 = %px\n", &p_item->br_hlist);
	printk("  mac[6]		   = %02x:%02x:%02x:%02x:%02x:%02x\n", p_item->mac[0], 
		p_item->mac[1], p_item->mac[2], p_item->mac[3], p_item->mac[4], p_item->mac[5]);
	printk("  netif			= %s\n", (p_item->netif ? p_item->netif->name : ""));
	printk("  timeout		  = %d\n",   p_item->timeout);
	printk("  last_hit_time	= %d\n",   p_item->last_hit_time);
	printk("  flags			= %08X\n", p_item->flags);
	printk("  bridging_entry   = %08X\n", p_item->bridging_entry);
#endif
}

int32_t ppa_bridging_session_start_iteration(uint32_t *ppos, struct br_mac_node **pp_item)
{
	PPA_HLIST_NODE *node = NULL;
	int idx;
	uint32_t l;

	l = *ppos + 1;

	ppa_br_get_htable_lock();

	for (idx = 0; l && idx < NUM_ENTITY(g_bridging_session_list_hash_table); idx++) {
		ppa_hlist_for_each(node, &g_bridging_session_list_hash_table[idx]) {
			if (!--l)
				break;
		}
	}

	if (l == 0 && node) {
		++*ppos;
		*pp_item = ppa_hlist_entry(node, struct br_mac_node, br_hlist);
		return PPA_SUCCESS;
	} else {
		*pp_item = NULL;
		return PPA_FAILURE;
	}
}
EXPORT_SYMBOL(ppa_bridging_session_start_iteration);

int32_t ppa_bridging_session_iterate_next(uint32_t *ppos, struct br_mac_node **pp_item)
{
	uint32_t idx;

	if (*pp_item == NULL)
		return PPA_FAILURE;

	if ((*pp_item)->br_hlist.next != NULL) {
		++*ppos;
		*pp_item = ppa_hlist_entry((*pp_item)->br_hlist.next, struct br_mac_node, br_hlist);
		return PPA_SUCCESS;
	} else {
		for (idx = PPA_BRIDGING_SESSION_LIST_HASH_VALUE((*pp_item)->mac) + 1;
			idx < NUM_ENTITY(g_bridging_session_list_hash_table);
			idx++)
			if (g_bridging_session_list_hash_table[idx].first != NULL) {
				++*ppos;
				*pp_item = ppa_hlist_entry(g_bridging_session_list_hash_table[idx].first,
					struct br_mac_node, br_hlist);
				return PPA_SUCCESS;
			}
		*pp_item = NULL;
		return PPA_FAILURE;
	}
}
EXPORT_SYMBOL(ppa_bridging_session_iterate_next);

void ppa_bridging_session_stop_iteration(void)
{
	ppa_br_release_htable_lock();
}
EXPORT_SYMBOL(ppa_bridging_session_stop_iteration);

/*
 *  bridging session hardware/firmware operation
 */

int32_t ppa_bridging_hw_add_mac(struct br_mac_node *p_item)
{
	PPA_BR_MAC_INFO br_mac={0};

	br_mac.port = p_item->dest_ifid;

	if ((p_item->flags & SESSION_DROP))
		br_mac.dest_list = 0;  //  no dest list, dropped
	else
		br_mac.dest_list = 1 << p_item->dest_ifid;

	ppa_memcpy(br_mac.mac, p_item->mac, PPA_ETH_ALEN);
	br_mac.f_src_mac_drop = p_item->flags & SESSION_SRC_MAC_DROP_EN;
	br_mac.dslwan_qid = p_item->dslwan_qid;
#if IS_ENABLED(CONFIG_SOC_GRX500)
	br_mac.age_timer = p_item->timeout;
	if (p_item->flags & SESSION_STATIC) {
		br_mac.static_entry = SESSION_STATIC; 
	}
	br_mac.fid = p_item->fid;
	br_mac.sub_ifid = 	p_item->sub_ifid;
#endif
	if (ppa_drv_add_bridging_entry(&br_mac, 0) == PPA_SUCCESS) {
		p_item->bridging_entry = br_mac.p_entry;
		p_item->flags |= SESSION_ADDED_IN_HW;
		return PPA_SUCCESS;
	}

	return PPA_FAILURE;
}

void ppa_bridging_hw_del_mac(struct br_mac_node *p_item)
{
	if ((p_item->flags & SESSION_ADDED_IN_HW)) {
		PPA_BR_MAC_INFO br_mac={0};
#if IS_ENABLED(CONFIG_SOC_GRX500)
		ppa_memcpy(br_mac.mac, p_item->mac, PPA_ETH_ALEN);
		br_mac.fid = p_item->fid;
#else
		br_mac.p_entry = p_item->bridging_entry;
#endif
		ppa_drv_del_bridging_entry(&br_mac, 0);
		p_item->bridging_entry = ~0;

		p_item->flags &= ~SESSION_ADDED_IN_HW;
	}
}

/*
 *  bridging polling timer
 */
void ppa_bridging_set_polling_timer(uint32_t polling_time)
{
	return;
}

/*
 *  special function
 */
void ppa_remove_sessions_on_netif (PPA_IFNAME *ifname)
{
	if (ifname) {
		ppa_session_delete_by_netif (ifname);
		ppa_remove_mc_groups_on_netif (ifname);
		ppa_remove_bridging_sessions_on_netif (ifname);
	}
}

#if IS_ENABLED(CONFIG_PPA_API_DIRECTCONNECT)
void ppa_remove_sessions_on_subif (PPA_DP_SUBIF *subif)
{
	ppa_session_delete_by_subif (subif);
	/* FIXME : multicast routing session can not be removed */
	ppa_remove_bridging_sessions_on_subif (subif);
}

void ppa_remove_sessions_on_macaddr(uint8_t *mac)
{
	ppa_session_delete_by_macaddr(mac);
	/* FIXME : multicast routing session can not be removed */
	ppa_remove_bridging_sessions_on_macaddr(mac);
}
#endif

/*
 * ####################################
 *		   Init/Cleanup API
 * ####################################
 */
#define DUMP_HASH_TBL_DEBUG 0 
#if DUMP_HASH_TBL_DEBUG
/* need outside lock */
static void dump_hash_table(char *htable_name, PPA_HLIST_HEAD *htable, uint32_t t_size)
{
	int idx;
	PPA_HLIST_NODE *node;

	printk("dump htable: %s\n", htable_name);
	
	for (idx = 0; idx < t_size; idx++) {
		printk("[%d]: first: 0x%x\n", idx, (uint32_t)htable[idx].first);
		ppa_hlist_for_each(node, &htable[idx]) {
			printk("node:0x%x\n", (uint32_t)node);
			if (node == htable[idx].first) {
				printk("WARNING: node has the same address as hash table!!!\n");
				break;
			}
		}
	}
}
#endif

#if IS_ENABLED(CONFIG_PPA_EXT_PKT_LEARNING)
extern uint64_t ppa_nfct_counter(PPA_SESSION *);
PPA_SESSION * ppa_get_skb_ct_info(PPA_BUF *, uint32_t *);
int ppa_find_sess_frm_tuple(PPA_TUPLE *, PPA_NETIF *, uint32_t *, uint32_t, uint32_t, struct uc_session_node **);
PPA_NETIF* ppa_logical_netif_from_superset(struct learn_superset *, uint32_t dir, PPA_NETIF *, struct netif_info**);

extern PPA_FILTER_STATS ppa_filter;
int32_t ppa_session_entry_check(struct uc_session_node* p_item)
{
	/* if the session is tcp established and connectiontrack exists*/
	if ((p_item->flags & SESSION_IS_TCP) && p_item->session) {
		if (ppa_is_tcp_established(p_item->session)) {
			/*connectiontrack counter greater than 10k(roughly 10 packets)*/
			if (ppa_nfct_counter(p_item->session) > (1024 * g_ppa_min_hits)) {
				return PPA_SUCCESS;
			} else {
				return PPA_FAILURE;
			}
		} else {
			return PPA_FAILURE;
		}
	} else {
		/* Just check whether the session has more than g_ppa_min_hits */
		if (!(ppa_session_entry_pass(p_item->num_adds, g_ppa_min_hits))) {
			return PPA_SUCCESS;
		}
	}
#if IS_ENABLED(CONFIG_LTQ_TOE_DRIVER) 
#if !IS_ENABLED(CONFIG_SOC_GRX500)
	/*check lro entry criteria for the local in sessions */
	if ((p_item->flag2 & SESSION_FLAG2_CPU_IN) && !ppa_bypass_lro(p_item->session)) {
		if (ppa_lro_entry_criteria(p_item, ppa_buf, flags) != PPA_SUCCESS) {
			/* if entry criteria is not met skip acceleration */
			return PPA_FAILURE;
		}
	}
#endif /*!IS_ENABLED(CONFIG_SOC_GRX500)*/
#endif /*IS_ENABLED(CONFIG_LTQ_TOE_DRIVER)*/
	return PPA_FAILURE;
}

static inline int32_t ppa_packet_filter(PPA_BUF *ppa_buf, PPA_SESSION *session,
	uint16_t l3_proto, uint16_t l4_proto, uint16_t sport, uint16_t dport, uint32_t flags)
{
	int32_t ret = PPA_SESSION_NOT_FILTED;

	/* ignore incoming broadcast*/
	if (ppa_is_pkt_broadcast(ppa_buf)) {
		ppa_filter.ppa_is_pkt_broadcast++;
		ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,"can not accelerate %s %d\n", __FUNCTION__, __LINE__);
		ret = PPA_SESSION_FILTED;
	}

	/* ignore loopback packet*/
	if (ppa_is_pkt_loopback(ppa_buf)) {
		ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,"ppa_is_pkt_loopback %s %d\n", __FUNCTION__, __LINE__);
		ppa_filter.ppa_is_pkt_loopback++;
		ret = PPA_SESSION_FILTED;
	}

	/* ignore un supported protocols */
	if (!(l4_proto == PPA_IPPROTO_TCP || l4_proto == PPA_IPPROTO_UDP || l4_proto == PPA_IPPROTO_ESP)) {
		ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,"can not accelerate %s %d\n", __FUNCTION__, __LINE__);
		ppa_filter.ppa_is_pkt_protocol_invalid++;
		ret = PPA_SESSION_FILTED;
	}

	/*
	 * Ignore special sessions with broadcast addresses and protocols with
	 * high probability of less packets being exchanged
	 * 1. windows netbios
	 * 2. DHCP
	 * 3. DNS
	 */
	if (sport == 137 || dport == 137
			|| sport == 138 || dport == 138
			|| sport == 68 || sport == 67
			|| dport == 68 || dport == 67
			|| sport == 53 || dport == 53
			|| sport == 1701 || dport == 1701) {
		ppa_filter.ppa_is_pkt_ignore_special_session++;
		ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,"can not accelerate %s %d\n", __FUNCTION__, __LINE__);
		ret = PPA_SESSION_FILTED;

	}

	return ret;
}
int32_t ppa_mc_session_entry_check(struct mc_session_node *p_item, struct lgm_mc_args *mc_args)
{
	/*TBD:  */
	return PPA_SUCCESS;
}

int32_t ppa_mc_add_pp_session(struct mc_session_node *p_mc_item, struct learn_superset *superset, struct pp_desc *ppdesc)
{
	int32_t ret = PPA_SUCCESS;
	struct lgm_mc_args mc_args = {0};

	mc_args.superset = superset;
	mc_args.hwhash.h1 = ppdesc->ud.hash_h1;
	mc_args.hwhash.h2 = ppdesc->ud.hash_h2;
	mc_args.hwhash.sig = ppdesc->ud.hash_sig;
	mc_args.ig_gpid = ppdesc->ud.rx_port;
	/*The below 2 parameters are valid only if the ingress port is MC special gpid*/
	mc_args.groupid	= ppdesc->ps & MC_GRP_MASK; /*BITS: 0-8 */
	mc_args.dst_idx = (ppdesc->ps & MC_DST_MASK) >> 9; /* BITS: 9-16 */

	/*Check if the session needs to be added to acceleration*/
	if (!ppa_mc_session_entry_check(p_mc_item, &mc_args)) {
		/*Pass the field vector and descriptor parameters*/
		p_mc_item->session_action = &mc_args;
		/*Update the existing HW session */
		if (ppa_hsel_update_wan_mc_group(p_mc_item, NULL, 0) != PPA_SUCCESS) {
			/*should not remove p_mc_item in PPA level, to avoid out-of-sync*/
			return PPA_SUCCESS;
		}
	}
	return ret;
}

int32_t ppa_add_pp_session(struct learn_superset *superset, bool drop)
{
	int32_t ret = PPA_SUCCESS;
	uint32_t flags = 0, cthash = 0;
	struct uc_session_node *p_item = NULL;
	struct mc_session_node *p_mc_item = NULL;
	struct netif_info *rx_ifinfo = NULL, *tx_ifinfo = NULL, *t_ifinfo = NULL;
	int f_is_ipoa_or_pppoa = 0;
	PPA_BUF *ppa_buf = NULL;
	PPA_SESSION *ct = NULL;
	int8_t strbuf[64];
	PPA_NETIF *rxif = NULL, *txif = NULL;
	PPA_TUPLE tuple = {0};
	uint16_t l3_proto = 0, l4_proto = 0;
	uint16_t srcport = 0, dstport = 0;
	PPA_SKBUF fake_skb = {0};
	uint32_t h1 = 0, h2 = 0, h3 = 0;
	uint8_t rx_port = 0;
#if defined(CONFIG_PPA_API_DIRECTCONNECT) && CONFIG_PPA_API_DIRECTCONNECT
	PPA_DP_SUBIF dp_subif = {0};
#endif
#if IS_ENABLED(CONFIG_SKB_EXTENSION)
	struct ppa_ct_cookie *ctc=NULL;
	int32_t ppa_ct_cookie_handle = -1;
	skb_cookie cookie = 0;
#endif 
	struct pp_desc *ppdesc = NULL;
	IP_ADDR_C mc_grp = {0}, src_ip = {0};

	/* extract the connection track pointer from skb*/
	ppa_buf = superset->skb;

	if (!ppa_buf)
		return PPA_FAILURE;
	
	/* Do not learn loopback packet */
	if (superset_is_loopback_pkt(superset, SUPRST_IN)) {
		return PPA_FAILURE;
	}

	/*================================================================================*/
	/* lookup the session db based on the hw hash */
	/* currently using the HW hash as additional parameter for resolving clash*/
	/*================================================================================*/
	/* Get the pp descriptor*/
	ppdesc = pp_pkt_desc_get(ppa_buf);

	if (ppdesc) {
		/* Fill in the Hash information */
		h1 = ppdesc->ud.hash_h1;
		h2 = ppdesc->ud.hash_h2;
		h3 = ppdesc->ud.hash_sig;
		/*Ingress GPID*/
		rx_port = ppdesc->ud.rx_port;
	}

	/*Get the l3 protocol of IG fV */
	l3_proto = superset_l3_proto(superset, SUPRST_IN);

	/* multicast data packets */
	if (superset_is_pkt_mc(superset)) {
		/*Lookup multicast session DB*/
		mc_grp.f_ipv6 = src_ip.f_ipv6 = (l3_proto == ETH_P_IPV6) ? 1 : 0;
		superset_src_ip((union nf_inet_addr *)&src_ip.ip, superset, SUPRST_IN);
		superset_dst_ip((union nf_inet_addr *)&mc_grp.ip, superset, SUPRST_IN);
		/*lock the mc_db*/
		ppa_mc_get_htable_lock();
		/*Do lookup */
		ret = __ppa_lookup_mc_group(&mc_grp, &src_ip, &p_mc_item);

		switch (ret) {
		case PPA_MC_SESSION_VIOLATION:
			/*Cannot add or update*/
			ppa_debug(DBG_ENABLE_MASK_DUMP_MC_GROUP, "IGMP violation, cannot be added or updated\n");
			ret = PPA_FAILURE;
			break;
		case PPA_SESSION_NOT_ADDED:
			/*MC data packets are coming but no Join received till now*/
			/*1.Check whether session is marked as drop session by the learnig driver*/
			/*2.If YES add a mc db node and drop entry in HW */
			if (drop) {
				/*TBD:: ret = ppa_mc_add_drop_entry(mc_grp, src_ip, superset, flags);*/
			}
			break;
		case PPA_SESSION_EXISTS:
			/*Add this mc session to pp hw*/
			ret = ppa_mc_add_pp_session(p_mc_item, superset, ppdesc);
			break;
		}
		/*Release lock */
		ppa_mc_release_htable_lock();

		goto __SKIP_SESSION_ACTION;
	}

#if IS_ENABLED(CONFIG_SKB_EXTENSION)
	/* in some routed scenarios the ct is reset by the time it reacehs the learning driver in egress*/
	ppa_ct_cookie_handle = skb_cookie_get_handle_by_name("ppa_ct_cookie");
	if (ppa_ct_cookie_handle >= 0) {
		cookie = skb_cookie_get(ppa_buf, ppa_ct_cookie_handle);
		if (cookie) {
			ctc = (struct ppa_ct_cookie *)cookie;
			ct = ctc->ct;
			flags = ctc->ctinfo;
			ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT, "%s %d ct_cookie = %px ct=%px flags=%d\n", __FUNCTION__, __LINE__, (void *)cookie, ct, flags);
		} else {
			/*ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,"Unable to retrieve ppa_ct_cookie!!!\n");*/
		}
	}
#else
	ct = ppa_get_skb_ct_info(ppa_buf, &flags);
#endif
	/**********************************************/
	/*TBD: Drop sessions are not handled currently*/
	if (drop) {
		ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT, "\n%s %dReturning from drop session\n", __FUNCTION__, __LINE__);
		goto __SKIP_SESSION_ACTION;
	}
	/**********************************************/

	ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,"\n\n%s %d ct = %px, hwh1=%x, hwh2=%x, hwh3=%x\n", __FUNCTION__, __LINE__, ct, h1, h2, h3);

	/*Get txif, l4proto, srcport, dstport from ingress fv*/
	txif 		= superset_netif (superset, SUPRST_EG);
	l4_proto	= superset_l4_proto(superset, SUPRST_IN);
	srcport		= superset_src_port(superset, SUPRST_IN);
	dstport		= superset_dst_port(superset, SUPRST_IN);

	ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT, "%s %d l3_proto=%x l4_proto=%x srcport=%d dstport=%d txif=%s\n",
		__FUNCTION__, __LINE__, l3_proto, l4_proto, srcport, dstport, txif->name);

	if (ppa_packet_filter(ppa_buf, ct, l3_proto, l4_proto, srcport, dstport, flags) == PPA_SESSION_FILTED) {
		return PPA_FAILURE;
	}

	if (ct) {
		ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,"%s %d ct = %px\n", __FUNCTION__, __LINE__, ct);
		ret = ppa_sess_find_by_tuple(ct, 
				flags & PPA_F_SESSION_REPLY_DIR,
				&cthash, h1, h2,
				&p_item);
	} else {
		ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,"%s %d ct is NULL \n", __FUNCTION__, __LINE__);
		tuple.src.l3num 	= (l3_proto == ETH_P_IPV6) ? AF_INET6 : AF_INET;
		tuple.dst.protonum 	= l4_proto;
		tuple.src.u.all		= srcport;
		tuple.src.u.all 	= cpu_to_be16p(&tuple.src.u.all);
		tuple.dst.u.all		= dstport;
		tuple.dst.u.all 	= cpu_to_be16p(&tuple.dst.u.all);
		superset_src_ip(&tuple.src.u3, superset, SUPRST_IN);
		superset_dst_ip(&tuple.dst.u3, superset, SUPRST_IN);

		ret = ppa_find_sess_frm_tuple(&tuple, txif, &cthash, h1, h2, &p_item);
	}

	if (PPA_SESSION_EXISTS != ret) {
	/* session is not found; we need to add a new session */

		/* skip sessions that needs to be forwarded to host for special handling(eg: ALG) */
		if (ct && ppa_check_is_special_session(ppa_buf, ct)) {
			ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,"can not accelerate: %px\n", p_item->session);
			goto __SKIP_SESSION_ACTION;
		}

		/* allocate the us_session_node */
		p_item = ppa_session_alloc_item();
		if (!p_item) {
			ret = PPA_ENOMEM;
			ppa_debug(DBG_ENABLE_MASK_ERR,"failed in memory allocation\n");
			goto __ADD_SESSION_DONE;
 		}
		dump_list_item(p_item, "ppa_alloc_session (after init)");

		/* fill the session parameters */
		p_item->session = ct;
		p_item->hash = cthash;

#if IS_ENABLED(CONFIG_X86_INTEL_LGM)
		/*ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,"HW hash h1=%x h2=%x h3=%x\n",h1,h2,h3);*/
		p_item->hwhash.h1 = h1;
		p_item->hwhash.h2 = h2;
		p_item->hwhash.sig = h3;
		p_item->ig_gpid = rx_port;
#endif
		if ((flags & PPA_F_SESSION_REPLY_DIR)) {
			p_item->flags	|= SESSION_IS_REPLY;
		}
 
		p_item->pkt.protocol	= l3_proto; 
		
		if (p_item->pkt.protocol == ETH_P_IPV6) {
			p_item->flags |= SESSION_IS_IPV6;
		}
		p_item->pkt.ip_proto	= l4_proto;

		if (p_item->pkt.ip_proto == PPA_IPPROTO_TCP) {
	  		p_item->flags |= SESSION_IS_TCP;
		}

		superset_src_ip((union nf_inet_addr *)&p_item->pkt.src_ip, superset, SUPRST_IN);
		superset_dst_ip((union nf_inet_addr *)&p_item->pkt.dst_ip, superset, SUPRST_IN);
		p_item->pkt.src_port	  = srcport;
		p_item->pkt.dst_port	  = dstport;
		
		/* get receive base interface */
		p_item->rx_if		= superset_netif (superset, SUPRST_IN);
		p_item->tx_if 		= txif;	

		if (!p_item->rx_if) {
			ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,"%s %d CPU out session\n", __FUNCTION__, __LINE__);
			p_item->flag2 |= SESSION_FLAG2_CPU_OUT;
		}
		if (!p_item->tx_if)  {
			ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,"%s %d CPU in session\n", __FUNCTION__, __LINE__);
			p_item->flag2 |= SESSION_FLAG2_CPU_IN;
		}

		p_item->timeout	   = ppa_get_default_session_timeout();
		p_item->last_hit_time = ppa_get_time_in_sec();
		p_item->host_bytes   += ppa_skb_len(ppa_buf) + PPA_ETH_HLEN + PPA_ETH_CRCLEN;
		p_item->num_adds++;

  		dump_list_item(p_item, "ppa_alloc_session (after Add)");
	
		ppa_atomic_set(&p_item->used, 2);

	/*ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,",p_item size	= %d\n", sizeof(struct uc_session_node));*/
	ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT," hlist		= %px\n", &p_item->hlist);
	ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT," session		= %px\n", p_item->session);
	ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT," ip_proto	= %08X\n", p_item->pkt.ip_proto);
	ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT," src_ip		= %s\n",   ppa_get_pkt_ip_string(p_item->pkt.src_ip, p_item->flags & SESSION_IS_IPV6, strbuf));
	ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT," src_port	= %d\n",   p_item->pkt.src_port);
#if IS_ENABLED(CONFIG_SOC_GRX500)
	ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT," src_mac[6]		= %s\n",   ppa_get_pkt_mac_string(p_item->s_mac, strbuf));
#endif
	ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT," dst_ip		= %s\n",   ppa_get_pkt_ip_string(p_item->pkt.dst_ip, p_item->flags & SESSION_IS_IPV6, strbuf));
	ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT," dst_port	= %d\n",   p_item->pkt.dst_port);
	ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT," rx_if		= %s\n",   p_item->rx_if->name);
	ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT," tx_if		= %s\n",   p_item->tx_if->name);
	ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT," timeout		= %d\n",   p_item->timeout);
	ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT," last_hit_time	= %d\n",   p_item->last_hit_time);
	ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT," num_adds	= %d\n",   p_item->num_adds);
	ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT," flags		= %08X\n", p_item->flags);
	
	ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,"%s %d session created skb=%px hash=%x\n", __FUNCTION__,__LINE__, ppa_buf, p_item->hash);

		ppa_session_list_lock();
		__ppa_session_insert_item(p_item);
		__ppa_session_put(p_item);
__ADD_SESSION_DONE:
  		ppa_session_list_unlock();
	} else {
	/* session exists update the required parameters */
		ppa_session_list_lock();
		p_item->host_bytes   += ppa_skb_len(ppa_buf) + PPA_ETH_HLEN + PPA_ETH_CRCLEN;
		p_item->num_adds++;
		ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,"%s %d session matched skb=%px hash=%x p_item->num_adds=%d\n", __FUNCTION__,__LINE__, ppa_buf, p_item->hash, p_item->num_adds);

		if (p_item->flags & SESSION_ADDED_IN_HW) {
		/*Already added session is coming to CPU; ie the session is not matching
		delete the session so that it can re-learn again */
			ppa_debug(DBG_ENABLE_MASK_DEBUG2_PRINT,"Session already programmed in HW %s %d\n", __FUNCTION__, __LINE__);
		} else if (p_item->flags & SESSION_ADDED_IN_SW) {
			ppa_debug(DBG_ENABLE_MASK_DEBUG2_PRINT,"session exists in SW acceleration already\n");	
		} else if (p_item->flags & SESSION_NOT_ACCEL_FOR_MGM) {
			/* if the counter is wrapping around reset the flag */
			if (p_item->num_adds > (1 << 16) - 2) {
				ppa_debug(DBG_ENABLE_MASK_DEBUG2_PRINT,"session exists for SESSION_NOT_ACCEL_FOR_MGM already\n");
				p_item->flags &= ~SESSION_NOT_ACCEL_FOR_MGM;
			}
		} else if (p_item->flags & SESSION_NOT_ACCELABLE) {
			ppa_debug(DBG_ENABLE_MASK_DEBUG2_PRINT,"session exists for SESSION_NOT_ACCELABLE already\n");	
		} else {

			if ((p_item->flags & SESSION_IS_TCP) && (p_item->session && !ppa_is_tcp_established(p_item->session))) {
				ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,"can not accelerate: %px\n", p_item->session);
				goto __UPDATE_SESSION_DONE;	
			}

			/* check for entry criteria */ 
			if (ppa_session_entry_check(p_item)) {
				ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,"%s %d g_ppa_min_hits=%d\n", __FUNCTION__, __LINE__, g_ppa_min_hits);
					if (p_item->num_adds+1 == g_ppa_min_hits)
						ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT, "p_item->num_adds(%d) < g_ppa_min_hits(%d)\n",
							p_item->num_adds, g_ppa_min_hits);
	  				SET_DBG_FLAG(p_item, SESSION_DBG_NOT_REACH_MIN_HITS);
			} else {

				ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,"%s %d update session\n", __FUNCTION__, __LINE__);
				/*if passed update session and add to HW*/
				CLR_DBG_FLAG(p_item, SESSION_DBG_NOT_REACH_MIN_HITS);

				p_item->pkt.ip_tos 	= superset_ip_tos(superset, SUPRST_IN);
				p_item->pkt.mark = ppa_get_skb_mark(ppa_buf);
#if IS_ENABLED(CONFIG_NETWORK_EXTMARK)
				p_item->pkt.extmark = ppa_get_skb_extmark(ppa_buf);
#endif
				ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,"%s %d update session\n", __FUNCTION__, __LINE__);
				p_item->pkt.priority = ppa_get_pkt_priority(ppa_buf);	   

				ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,"%s %d update session\n", __FUNCTION__, __LINE__);
				/* get receive base interface */
				p_item->rx_if		= superset_netif (superset, SUPRST_IN);
				p_item->tx_if 		= txif;	

				/* set session direction */
				if ((p_item->rx_if) && (ppa_netif_lookup(ppa_get_netif_name(p_item->rx_if),
						&rx_ifinfo) != PPA_SUCCESS)) {
					ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,
						"failed in getting info structure of rx_if (%s)\n", 
						ppa_get_netif_name(p_item->rx_if));
					SET_DBG_FLAG(p_item, SESSION_DBG_RX_IF_NOT_IN_IF_LIST);
					p_item->flags |= SESSION_NOT_ACCELABLE;
	  				goto __UPDATE_SESSION_DONE;
				}

				if ((p_item->tx_if) && (ppa_netif_lookup(ppa_get_netif_name(p_item->tx_if),
						&tx_ifinfo) != PPA_SUCCESS)) {
					ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,
						"failed in getting info structure of tx_if (%s)\n",
						ppa_get_netif_name(p_item->tx_if));
					SET_DBG_FLAG(p_item, SESSION_DBG_TX_IF_NOT_IN_IF_LIST);
					p_item->flags |= SESSION_NOT_ACCELABLE;
	  				goto __UPDATE_SESSION_DONE;
				}

				ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,"%s %d update session\n", __FUNCTION__, __LINE__);
				if (!superset_pppoe_info(superset, p_item, SUPRST_IN)) {
					if (superset_pppoe_info(superset, p_item, SUPRST_EG))
	  					SET_DBG_FLAG(p_item, SESSION_DBG_TX_PPPOE);
				} else {
	  				SET_DBG_FLAG(p_item, SESSION_DBG_RX_PPPOE);
				}

				ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,"%s %d update session\n", __FUNCTION__, __LINE__);
				if (!superset_6rd_info(superset, p_item, SUPRST_IN))
					superset_6rd_info(superset, p_item, SUPRST_EG);

				if (!superset_dslite_info(superset, p_item, SUPRST_IN))
					superset_dslite_info(superset, p_item, SUPRST_EG);

				if (!superset_gre_info(superset, p_item, SUPRST_IN))
					superset_gre_info(superset, p_item, SUPRST_EG);

				if (!superset_esp_info(superset, p_item, SUPRST_IN)) {
					if (superset_esp_info(superset, p_item, SUPRST_EG)) {
					/* TBD: Ipsec session information to be extracted */
					}
				}
				ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,"%s %d update session\n", __FUNCTION__, __LINE__);
#if defined(L2TP_CONFIG)
				/* extract l2tp information */
				if (!superset_l2tp_info(superset, p_item, SUPRST_IN)) {
					if (superset_l2tp_info(superset, p_item, SUPRST_EG))
						SET_DBG_FLAG(p_item, SESSION_DBG_TX_PPPOL2TP);
				} else {
					SET_DBG_FLAG(p_item, SESSION_DBG_RX_PPPOL2TP);
				}
#endif /*L2TP_CONFIG*/	
				/*extract all vlan related information*/
				superset_vlan_info(superset,p_item);
			
				ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,"%s %d update session\n", __FUNCTION__, __LINE__);
				/* if the base interface exists find the logical interface */
				if (p_item->rx_if) {
					/* receive logical interface is extracted form the skb_iif */
					if (rx_ifinfo->flags & NETIF_LAN_IF) {
						p_item->br_rx_if = ppa_dev_get_by_index(ppa_buf->skb_iif);
						ppa_put_netif (p_item->br_rx_if);
						p_item->flags |= SESSION_LAN_ENTRY;
					} else {
						p_item->flags |= SESSION_WAN_ENTRY;
						rxif = ppa_logical_netif_from_superset(superset, SUPRST_IN, p_item->rx_if, &t_ifinfo);
						if (rxif != p_item->rx_if) {
							ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,"%s %d update session rx_if=%s\n",
								__FUNCTION__, __LINE__, ppa_get_netif_name(rxif));
							p_item->rx_if = rxif;
							ppa_netif_put(rx_ifinfo);
							rx_ifinfo = t_ifinfo;
						}
					}
				}
				ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,"%s %d update session\n", __FUNCTION__, __LINE__);
				if (p_item->tx_if) {
					/* transmit logical interface needs to be found using the superset */
					if (tx_ifinfo->flags & NETIF_WAN_IF) {
						p_item->flags |= SESSION_LAN_ENTRY;
						txif = ppa_logical_netif_from_superset(superset, SUPRST_EG, p_item->tx_if, &t_ifinfo);
						if (txif != p_item->tx_if) {
							ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,"%s %d update session tx_if=%s\n",
								__FUNCTION__, __LINE__, ppa_get_netif_name(txif));
							p_item->tx_if = txif;
							ppa_netif_put(tx_ifinfo);
							tx_ifinfo = t_ifinfo;
						}
					} else {
						/*p_item->br_txif = ??*/
						p_item->flags |= SESSION_WAN_ENTRY;
					}
				}
/*
*  check whether physical port is determined or not
*/
				if (((tx_ifinfo && !(tx_ifinfo->flags & NETIF_PHYS_PORT_GOT)) ||
					(rx_ifinfo && !(rx_ifinfo->flags & NETIF_PHYS_PORT_GOT)))) {
					p_item->flags |= SESSION_NOT_VALID_PHY_PORT;
					ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,"tx no NETIF_PHYS_PORT_GOT\n");
				}

				if (tx_ifinfo && (tx_ifinfo->flags & (NETIF_LAN_IF | NETIF_PPPOE)) ==
					(NETIF_LAN_IF | NETIF_PPPOE)) { /*cannot accelerate*/
					ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,"Non-WAN Session cannot add ppp header\n");
					p_item->flags |= SESSION_NOT_ACCELABLE;
					ret = PPA_EAGAIN;
					goto __UPDATE_SESSION_DONE;
				}

				ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,"  rx_if			= %s\n",   p_item->rx_if->name);
				ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,"  tx_if			= %s\n",   p_item->tx_if->name);

				/* Get the dest ifid and subifid */
				if (tx_ifinfo) {
				ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,"%s %d update session\n", __FUNCTION__, __LINE__);
					p_item->dest_ifid = tx_ifinfo->phys_port;
#if IS_ENABLED(CONFIG_SOC_GRX500)
					if ((tx_ifinfo->flags & NETIF_VLAN) || (tx_ifinfo->flags & NETIF_PHY_ATM)
						|| (tx_ifinfo->flags & NETIF_DIRECTPATH)) {
						p_item->dest_subifid = tx_ifinfo->subif_id;
					}
#endif /*defined(CONFIG_SOC_GRX500) && CONFIG_SOC_GRX500*/
					if ((tx_ifinfo->flags & NETIF_PHY_ATM)) {
						fake_skb.priority = ppa_get_pkt_priority(ppa_buf);
						p_item->dslwan_qid = ppa_drv_get_netif_qid_with_pkt(&fake_skb, tx_ifinfo->vcc, 1);
						if (!p_item->dslwan_qid) {
							p_item->dslwan_qid = tx_ifinfo->dslwan_qid;
						}
						p_item->flags |= SESSION_VALID_DSLWAN_QID;

						if ((tx_ifinfo->flags & NETIF_EOA)) {
							SET_DBG_FLAG(p_item, SESSION_DBG_TX_BR2684_EOA);
						} else if ((tx_ifinfo->flags & NETIF_IPOA)) {
							p_item->flags |= SESSION_TX_ITF_IPOA;
							SET_DBG_FLAG(p_item, SESSION_TX_ITF_IPOA);
							f_is_ipoa_or_pppoa = 1;
						} else if ((tx_ifinfo->flags & NETIF_PPPOATM)) {
							p_item->flags |= SESSION_TX_ITF_PPPOA;
							SET_DBG_FLAG(p_item, SESSION_TX_ITF_PPPOA);
							f_is_ipoa_or_pppoa = 1;
						}
					}	
				} else {
#if IS_ENABLED(CONFIG_SOC_GRX500)
					/*add lro entry in lro engine*/
					if ((p_item->flag2 & SESSION_FLAG2_CPU_IN) && !ppa_bypass_lro(p_item->session)) {
						if ((ppa_add_lro_entry(p_item, 0)) == PPA_SUCCESS) {
							ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,"lro entry added\n");
						} else {
							ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,"lro entry add failed\n");
							p_item->flag2 |= SESSION_FLAG2_ADD_HW_FAIL;
							goto __UPDATE_SESSION_DONE;
						}
					}
#endif /*IS_ENABLED(CONFIG_SOC_GRX500)*/
				}

				/* Rx part handling */
  				if (rx_ifinfo) {
					ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,"%s %d update session\n", __FUNCTION__, __LINE__);
	  				if ((rx_ifinfo->flags & NETIF_PHY_ATM)) {
		  				if ((rx_ifinfo->flags & NETIF_IPOA)) {
			  				p_item->flags |= SESSION_TX_ITF_IPOA;
			  				SET_DBG_FLAG(p_item, SESSION_TX_ITF_IPOA);
		  				} else if ((rx_ifinfo->flags & NETIF_PPPOATM)) {
			  				p_item->flags |= SESSION_TX_ITF_PPPOA;
			  				SET_DBG_FLAG(p_item, SESSION_TX_ITF_PPPOA);
		  				}
	  				}
  				}
#if defined(CONFIG_SOC_GRX500) && CONFIG_SOC_GRX500
				if (rx_ifinfo) {
					p_item->src_ifid = rx_ifinfo->phys_port;
				}
#endif /*defined(CONFIG_SOC_GRX500) && CONFIG_SOC_GRX500*/
				/*Get the nat information */
				ppa_memset(&p_item->pkt.natip, 0, sizeof(p_item->pkt.natip));
				p_item->pkt.nat_port= 0;
				if (p_item->flags & SESSION_LAN_ENTRY) {
					ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,"%s %d update session\n", __FUNCTION__, __LINE__);
					/* check and extract snat info*/
					superset_src_ip((union nf_inet_addr *)&p_item->pkt.natip.natsrcip.ip, superset, SUPRST_EG);
					if (ppa_memcmp(&p_item->pkt.natip.natsrcip.ip, &p_item->pkt.src_ip, sizeof(PPA_IPADDR)) != 0) {
						p_item->flags |= SESSION_VALID_NAT_IP;	
					}
					p_item->pkt.nat_port = superset_src_port(superset, SUPRST_EG);
					if (p_item->pkt.nat_port != p_item->pkt.src_port) {
						p_item->flags |= SESSION_VALID_NAT_PORT | SESSION_VALID_NAT_SNAT;
					} 
				} else {
					if (ppa_get_pkt_ip_proto(ppa_buf) != PPA_IPPROTO_ESP) {
						/* check and extract dnat info */
						superset_dst_ip((union nf_inet_addr *)&p_item->pkt.natip.natsrcip.ip, superset, SUPRST_EG);
						if (ppa_memcmp(&p_item->pkt.natip.natsrcip.ip, &p_item->pkt.dst_ip, sizeof(PPA_IPADDR)) != 0) {
							p_item->flags |= SESSION_VALID_NAT_IP;
						}
						p_item->pkt.nat_port = superset_dst_port(superset, SUPRST_EG);
						if (p_item->pkt.nat_port != p_item->pkt.dst_port) {
							p_item->flags |= SESSION_VALID_NAT_PORT;
						}
						/* TBD: l2 nat handling in case of tunnels*/
					}
				}
				ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,"%s %d update session\n", __FUNCTION__, __LINE__);
				p_item->pkt.new_dscp = ppa_get_pkt_ip_tos(ppa_buf);
				if (p_item->pkt.new_dscp != p_item->pkt.ip_tos) {
					p_item->pkt.new_dscp >>= 2;
					p_item->flags |= SESSION_VALID_NEW_DSCP;
				}

				if (f_is_ipoa_or_pppoa)
					goto __UPDATE_SESSION_SKIP_MAC;  

#if IS_ENABLED(CONFIG_SOC_GRX500)
				superset_src_mac(p_item->s_mac, superset, SUPRST_IN);	
#endif /*IS_ENABLED(CONFIG_SOC_GRX500)*/
				/* egress src and dst mac address from the superset */
				superset_src_mac(p_item->pkt.src_mac, superset, SUPRST_EG);	
				superset_dst_mac(p_item->pkt.dst_mac, superset, SUPRST_EG);	

				ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,"%s %d update session\n", __FUNCTION__, __LINE__);
#if IS_ENABLED(CONFIG_PPA_API_DIRECTCONNECT)
				/* For directconnect destif, get destination SubifId={VapId, StationId} 
				corresponding to dst_mac */
				if (tx_ifinfo && (tx_ifinfo->flags & NETIF_DIRECTCONNECT)) {
					PPA_NETIF *dp_netif = tx_ifinfo->netif;

					/* Use phys interface <wlanX> for any logical interface on top of it. */
					if (tx_ifinfo->flags & NETIF_PHY_IF_GOT)
						dp_netif = ppa_get_netif(tx_ifinfo->phys_netif_name);

					dp_subif.port_id = tx_ifinfo->phys_port;
					dp_subif.subif = tx_ifinfo->subif_id;
					if (DP_SUCCESS == dp_get_netif_subifid(dp_netif,
						NULL, NULL, p_item->pkt.dst_mac, &dp_subif, 0)) {
						p_item->dest_subifid = dp_subif.subif;
					} else {
						ret = PPA_FAILURE;
	  						ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,"unable to find the subif info\n");
						goto __UPDATE_SESSION_DONE;
					}
				}
#endif /*defined(CONFIG_PPA_API_DIRECTCONNECT) && CONFIG_PPA_API_DIRECTCONNECT*/
				if (p_item->tx_if) {
					/* session mtu */
					p_item->mtu	=  ppa_get_mtu(p_item->tx_if);
					p_item->mtu	-= (superset_eg_header_len(superset) - PPA_ETH_HLEN);
				} else {
					p_item->mtu     =  ppa_get_mtu(p_item->rx_if);
				}
__UPDATE_SESSION_SKIP_MAC:
#if IS_ENABLED(CONFIG_X86_INTEL_LGM)
				ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,"%s %d update session\n", __FUNCTION__, __LINE__);
				p_item->session_meta = (void *)superset;
#endif /*IS_ENABLED(CONFIG_X86_INTEL_LGM)*/

				/*call hal selector to add session to hw */
				if ((ret = ppa_hsel_add_routing_session(p_item, ppa_buf, 0)) != PPA_SUCCESS) {
					p_item->flag2 |= SESSION_FLAG2_ADD_HW_FAIL;
					ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,"ppa_hw_add_session(p_item) fail\n");
					p_item->flags |= SESSION_NOT_ACCELABLE;
					ret = PPA_FAILURE;
					goto __UPDATE_SESSION_DONE;
				} else {
					p_item->flag2 &= ~SESSION_FLAG2_ADD_HW_FAIL;
					p_item->flags &= ~SESSION_NOT_ACCEL_FOR_MGM;
					ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,"%s %d HW session add success\n", __FUNCTION__, __LINE__);
				}
			}
__UPDATE_SESSION_DONE:
			ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,"%s %d update session\n", __FUNCTION__, __LINE__);
			ppa_netif_put(tx_ifinfo);
			ppa_netif_put(rx_ifinfo);
		}
		__ppa_session_put(p_item);
  		ppa_session_list_unlock();
	}
	
	//ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,"learning_done_event rxdev=%s txdev=%s skbdev=%s skb=%x\n", rxdev->name, txdev->name, superset->skb->dev->name, superset->skb);
	//ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,"size of learn_superset=%d, fv_pkt_info=%d, new_hdr=%d\n", sizeof(struct learn_superset), sizeof(struct fv_pkt_info), sizeof(struct new_hdr));
__SKIP_SESSION_ACTION:
	return ret;
} 


/**
 * learning done notification handler
 *
 * @param unused
 * @param event event id
 * @param superset superset pointer
 *
 * @return s32 0 for success
 */
static s32 learning_done_event(struct notifier_block *unused, unsigned long event, void *learned_superset)
{

		struct learn_superset *superset =
				(struct learn_superset *)learned_superset;

		switch (event) {
		case LEARNING_NOTIFY_DONE:		
		ppa_add_pp_session(superset, 0);
				break;
		case LEARNING_NOTIFY_DROP:
		ppa_add_pp_session(superset, 1);
				break;
		default:
		break;
		}

		return 0;
}
#endif /*CONFIG_PPA_EXT_PKT_LEARNING*/

int32_t ppa_api_session_manager_init(void)
{
	int i;
	//char rt_thread_name[]="ppa_rt_chk_hit_stat";
#if defined(PPA_INTF_MIB_TIMER) && defined(CONFIG_SOC_GRX500) && CONFIG_SOC_GRX500
	//char rt_thrd_intf_name[]="ppa_intf_stat";
#endif
	u32 flags = WQ_MEM_RECLAIM | WQ_UNBOUND;

	/* init hash list */
	for (i = 0; i < SESSION_LIST_MC_HASH_TABLE_SIZE; i++) {
		PPA_INIT_HLIST_HEAD(&g_mc_group_list_hash_table[i]);
	}

	for (i = 0; i < BRIDGING_SESSION_LIST_HASH_TABLE_SIZE; i++) {
		PPA_INIT_HLIST_HEAD(&g_bridging_session_list_hash_table[i]);
	}

	ppa_session_list_init();
#if defined(CONFIG_PPA_HANDLE_CONNTRACK_SESSIONS)
	ppa_session_mgmt_init();
#endif

	/*initialize timer workq*/
	g_timer_workq = alloc_workqueue("%s", flags, 2, "ppa_sessmgr_timer_wq");
	if (!g_timer_workq) {
		pr_err("Failed to create ppa_sessmgr_timer_wq work queue");
	}

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,15,0)
	/*  start timer*/
	ppa_timer_init(&g_hit_stat_timer, ppa_check_hit_stat);
	ppa_timer_add(&g_hit_stat_timer, g_hit_polling_time);
	ppa_timer_init(&g_mib_cnt_timer, ppa_mib_count);
	ppa_timer_add(&g_mib_cnt_timer, g_mib_polling_time);
#else
	ppa_hrt_init(&g_hit_stat_timer, ppa_check_hit_stat);
	ppa_hrt_start(&g_hit_stat_timer, g_hit_polling_time);
	ppa_hrt_init(&g_mib_cnt_timer, ppa_mib_count);
	ppa_hrt_start(&g_mib_cnt_timer, g_mib_polling_time);	
#endif

#if defined(PPA_INTF_MIB_TIMER) && IS_ENABLED(CONFIG_SOC_GRX500)
	/*  start timer*/
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,15,0)
	ppa_timer_init(&g_intf_mib_timer, ppa_check_intf_mib);
	ppa_timer_add(&g_intf_mib_timer, g_intf_mib_polling_time);
#else
	ppa_hrt_init(&g_intf_mib_timer, ppa_check_intf_mib);
	ppa_hrt_start(&g_intf_mib_timer, g_intf_mib_polling_time);
#endif /*LINUX_VERSION_CODE < KERNEL_VERSION(4,15,0)*/
	
#endif

#if IS_ENABLED(CONFIG_PPA_EXT_PKT_LEARNING)
	/* Register learning done notifications */
	register_learning_done_notifier(&learning_done_notifier);
#endif /*CONFIG_PPA_EXT_PKT_LEARNING*/

	return PPA_SUCCESS;
}

#if defined(CONFIG_PPA_MPE_IP97)
/* To delete ipsec sessions in ppa */
void ppa_ipsec_session_list_free(void)
{
	ppa_tunnel_entry *t_entry = NULL;
	uint32_t idx, tmpIdx = 0;
        PPA_IPSEC_INFO tmpIpsec[MAX_TUNNEL_ENTRIES];

	ppe_lock_get(&g_tunnel_table_lock);

	// get all inbound and outbound xfrm state structure address for all ipsec tunnels
	for (idx = 0;  idx < MAX_TUNNEL_ENTRIES; idx++) {
		t_entry = g_tunnel_table[idx];
		if (!t_entry) {
			continue;
		} else if (t_entry->tunnel_type == TUNNEL_TYPE_IPSEC) {
			tmpIpsec[tmpIdx].inbound = t_entry->tunnel_info.ipsec_hdr.inbound;
			tmpIpsec[tmpIdx].outbound = t_entry->tunnel_info.ipsec_hdr.outbound;
			tmpIdx++;
		}
	}
	ppe_lock_release(&g_tunnel_table_lock);

	for (idx = 0;  idx < tmpIdx; idx++) {
		ppa_session_ipsec_delete(tmpIpsec[idx].inbound);
		ppa_session_ipsec_delete(tmpIpsec[idx].outbound);
	}
}
#endif



void ppa_api_session_manager_exit(void)
{

#if IS_ENABLED(CONFIG_PPA_EXT_PKT_LEARNING)
	/* Unregister learning done notifications */
	unregister_learning_done_notifier(&learning_done_notifier);
#endif /*CONFIG_PPA_EXT_PKT_LEARNING*/

#if defined(PPA_INTF_MIB_TIMER) && IS_ENABLED(CONFIG_SOC_GRX500)
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,15,0)
	ppa_timer_del(&g_intf_mib_timer);
#else
	ppa_hrt_stop(&g_intf_mib_timer);
#endif /*LINUX_VERSION_CODE < KERNEL_VERSION(4,15,0)*/
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(4,15,0)
	ppa_timer_del(&g_hit_stat_timer);
	ppa_timer_del(&g_mib_cnt_timer);
#else
	ppa_hrt_stop(&g_hit_stat_timer);
	ppa_hrt_stop(&g_mib_cnt_timer);
#endif /*LINUX_VERSION_CODE < KERNEL_VERSION(4,15,0)*/

	/* delete the work queues */
	if (g_timer_workq)
		destroy_workqueue(g_timer_workq);

#if defined(CONFIG_PPA_HANDLE_CONNTRACK_SESSIONS)
	ppa_session_mgmt_exit();
#endif
	ppa_session_list_free();
#if defined(CONFIG_PPA_MPE_IP97)
	ppa_ipsec_session_list_free();
#endif
	ppa_free_mc_group_list();
	ppa_free_bridging_session_list();

	ppa_session_cache_shrink();
	ppa_kmem_cache_shrink(g_mc_group_item_cache);
	ppa_kmem_cache_shrink(g_bridging_session_item_cache);
}

int32_t ppa_api_session_manager_create(void)
{
	if (ppa_session_cache_create())
	{
		ppa_debug(DBG_ENABLE_MASK_ERR,"Failed in creating mem cache for routing session list.\n");
		goto PPA_API_SESSION_MANAGER_CREATE_FAIL;
	}

	if (ppa_mem_cache_create("mc_group_item", sizeof(struct mc_session_node), &g_mc_group_item_cache))
	{
		ppa_debug(DBG_ENABLE_MASK_ERR,"Failed in creating mem cache for multicast group list.\n");
		goto PPA_API_SESSION_MANAGER_CREATE_FAIL;
	}

	if (ppa_mem_cache_create("bridging_sess_item", sizeof(struct br_mac_node), &g_bridging_session_item_cache))
	{
		ppa_debug(DBG_ENABLE_MASK_ERR,"Failed in creating mem cache for bridging session list.\n");
		goto PPA_API_SESSION_MANAGER_CREATE_FAIL;
	}

#if defined(CONFIG_PPA_MPE_IP97) && CONFIG_PPA_MPE_IP97
	if (ppa_mem_cache_create("ipsec_group_item", sizeof(struct uc_session_node), &g_ipsec_group_item_cache))
	{
		ppa_debug(DBG_ENABLE_MASK_ERR,"Failed in creating mem cache for ipsec group list.\n");
		goto PPA_API_SESSION_MANAGER_CREATE_FAIL;
	}
#endif

	if (ppa_session_list_lock_init())
	{
		ppa_debug(DBG_ENABLE_MASK_ERR,"Failed in creating lock for routing session list.\n");
		goto PPA_API_SESSION_MANAGER_CREATE_FAIL;
	}

	if (ppa_lock_init(&g_mc_group_list_lock))
	{
		ppa_debug(DBG_ENABLE_MASK_ERR,"Failed in creating lock for multicast group list.\n");
		goto PPA_API_SESSION_MANAGER_CREATE_FAIL;
	}

	if (ppa_lock_init(&g_bridging_session_list_lock))
	{
		ppa_debug(DBG_ENABLE_MASK_ERR,"Failed in creating lock for bridging session list.\n");
		goto PPA_API_SESSION_MANAGER_CREATE_FAIL;
	}
	if (ppa_lock_init(&g_general_lock))
	{
		ppa_debug(DBG_ENABLE_MASK_ERR,"Failed in creating lock for general mib conter.\n");
		goto PPA_API_SESSION_MANAGER_CREATE_FAIL;
	}

	reset_local_mib();
	ppa_atomic_set(&g_hw_session_cnt,0);

	return PPA_SUCCESS;

PPA_API_SESSION_MANAGER_CREATE_FAIL:
	ppa_api_session_manager_destroy();
	return PPA_EIO;
}

void ppa_api_session_manager_destroy(void)
{
	ppa_session_cache_destroy();

	if (g_mc_group_item_cache)
	{
		ppa_mem_cache_destroy(g_mc_group_item_cache);
		g_mc_group_item_cache = NULL;
	}

	if (g_bridging_session_item_cache)
	{
		ppa_mem_cache_destroy(g_bridging_session_item_cache);
		g_bridging_session_item_cache = NULL;
	}

	ppa_session_list_lock_destroy();

	ppa_lock_destroy(&g_mc_group_list_lock);
	ppa_lock_destroy(&g_general_lock);

	ppa_lock_destroy(&g_bridging_session_list_lock);
}

uint32_t ppa_api_get_session_poll_timer(void)
{
	return g_hit_polling_time;
}
EXPORT_SYMBOL(ppa_api_get_session_poll_timer);

uint32_t ppa_api_set_test_qos_priority_via_tos(PPA_BUF *ppa_buf)
{
	uint32_t pri = ppa_get_pkt_ip_tos(ppa_buf) % 8;
	ppa_set_pkt_priority(ppa_buf, pri);
	return pri;
}

uint32_t ppa_api_set_test_qos_priority_via_mark(PPA_BUF *ppa_buf)
{
	uint32_t pri = ppa_get_skb_mark(ppa_buf) % 8;
	ppa_set_pkt_priority(ppa_buf, pri);
	return pri;
}

int ppa_get_hw_session_cnt(void)
{
	return ppa_atomic_read(&g_hw_session_cnt);
}

int32_t get_ppa_hash_count(PPA_CMD_COUNT_INFO *count_info)
{
	if (count_info)
	{
		count_info->count = SESSION_LIST_HASH_TABLE_SIZE;
	}

	return PPA_SUCCESS;
}
EXPORT_SYMBOL(get_ppa_hash_count);


uint32_t ppa_get_hit_polling_time(void)
{
  return g_hit_polling_time;
}
EXPORT_SYMBOL(ppa_get_hit_polling_time);

