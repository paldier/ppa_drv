/*******************************************************************************
 **
 ** FILE NAME    : ppa_stack_tnl_al.c
 ** PROJECT      : PPA
 ** MODULES      : PPA Stack Adaptation Layer
 **
 ** DATE         : 12 May 2014
 ** AUTHOR       : Mahipati Deshpande
 ** DESCRIPTION  : Stack Adaptation for tunneled interfaces and sessions.
 ** COPYRIGHT    :              Copyright (c) 2009
 **                          Lantiq Deutschland GmbH
 **                   Am Campeon 3; 85579 Neubiberg, Germany
 **
 **   For licensing information, see the file 'LICENSE' in the root folder of
 **   this software module.
 **
 ** HISTORY
 ** $Date        $Author                $Comment
 ** 12 May 2014   Mahipati Deshpande    Moved tunneled routines to this file
 **                                     Added support for GRE
 *******************************************************************************/
#include <linux/version.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 33)
#include <linux/autoconf.h>
#else
#include <generated/autoconf.h>
#endif

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/netdevice.h>
#include <linux/inetdevice.h>
#include <linux/if_vlan.h>
#include <linux/ip.h>
#include <linux/udp.h>
#include <linux/if_arp.h>
#include <net/ip.h>
#include <net/ipv6.h>
#include <net/addrconf.h>
#include <net/dst_cache.h>
#include <asm/time.h>
#include <../net/bridge/br_private.h>
#if LINUX_VERSION_CODE < KERNEL_VERSION(4,15,0)
#include <../net/8021q/vlan.h>
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,32)
#include <net/netfilter/nf_conntrack_helper.h>
#include <net/netfilter/nf_conntrack.h>
#endif
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,8,13)
#include <net/ipip.h>
#else
#include <net/ip_tunnels.h>
#endif
#include <linux/if_tunnel.h>
#include <net/ip6_tunnel.h>
#include <net/gre.h>

#include <net/ppa/ppa_api.h>
#include <net/ppa/ppa_stack_al.h>

#include "ppa_stack_tnl_al.h"
#include "ppa_api_misc.h"

#define PPA_PPP_MASK_LEN   	2
#define PPA_PPP_MASK    	3
#define PPA_PPPOE_ID       	0x00000001
#define PPA_PPPOA_ID    	0x00000002
#define PPA_PPPOL2TP_ID 	0x00000003

#if IS_ENABLED(CONFIG_PPPOL2TP)
enum {
	PPPOL2TP_GET_SESSION_ID = 1,
	PPPOL2TP_GET_TUNNEL_ID,
	PPPOL2TP_GET_BASEIF,
	PPPOL2TP_GET_ADDR,
	PPPOL2TP_GET_DMAC,
	PPPOL2TP_GET_SADDR,
PPPOL2TP_GET_DADDR,
};
#endif

static inline
int32_t ppa_get_mac_from_neigh(uint8_t* mac,struct neighbour *neigh,struct dst_entry *dst)
{
	int32_t ret=PPA_ENOTAVAIL;
	struct hh_cache *hh;
	unsigned int seq;

	ppa_neigh_hh_init(neigh);
	ppa_neigh_update_hhs(neigh);
	hh = &neigh->hh;

	do {
		seq = read_seqbegin(&hh->hh_lock);
		if ( hh->hh_len == ETH_HLEN ) {
			memcpy(mac, (uint8_t *)hh->hh_data + HH_DATA_ALIGN(hh->hh_len) - hh->hh_len,
					ETH_ALEN);
			ret = PPA_SUCCESS;
		} else
			ret = PPA_ENOTAVAIL;
	} while ( read_seqretry(&hh->hh_lock, seq) );

	return ret;
}

/*
 * Note: Caller must make sure dev is IPv4 tunnel
 */
	static
struct net_device* ppa_ipv4_tnl_phy(struct net_device *dev)
{
	struct iphdr *iph;
	struct rtable *rt = NULL;
	struct net_device *phydev = NULL;
	struct ip_tunnel *t;

	t =  netdev_priv(dev);

	iph = &t->parms.iph;

	if (iph->daddr) {
		struct flowi4 fl;
		memset(&fl,0,sizeof(fl));
		fl.flowi4_tos = RT_TOS(iph->tos);
		fl.flowi4_oif = t->parms.link;
		fl.flowi4_proto = t->parms.iph.protocol;
		fl.daddr = iph->daddr;
		fl.saddr = iph->saddr;
		rt=ppa_ip_route_output_key(dev_net(dev),&fl);
		if(IS_ERR(rt)) {
			return NULL;
		}
		phydev = rt->dst.dev;
		ip_rt_put(rt);
	} else {
		phydev = __ip_dev_find(&init_net, iph->saddr,false);
	}

	return phydev;
}

#include <net/arp.h>
/*
 * Returns PPA_SUCCESS if valid destination MAC is available
 * Note: Caller must make sure dev is IPv4 tunnel
 */
	static
int ppa_get_ipv4_tnl_dmac( uint8_t *mac,
		struct net_device *dev,
		struct sk_buff *skb)
{
	struct ip_tunnel *t;
	struct iphdr *iph;
	struct rtable *rt = NULL;
	int ret = PPA_ENOTAVAIL;
	struct neighbour* neigh;
	//u32 nexthop;
	__be32 pkey = 0x0;

	t = netdev_priv(dev);
	iph = &t->parms.iph;

	if (iph->daddr) {

		struct flowi4 fl;
		memset(&fl,0,sizeof(fl));
		fl.flowi4_tos = RT_TOS(iph->tos);
		fl.flowi4_oif = t->parms.link;
		fl.flowi4_proto = t->parms.iph.protocol;
		fl.daddr = iph->daddr;
		fl.saddr = iph->saddr;

		rt=ppa_ip_route_output_key(dev_net(dev), &fl);
		if(IS_ERR(rt))
			return ret;
	} else {
		return ret; //No daddr so cannot get dmac
	}

	/*
	 * Note: Since sbk's outer IP is not yer formed when packet reached here.
	 *
	 */
	//nexthop = (__force u32) rt_nexthop(rt, iph->daddr);
	/**To calculate MAC addr of nxt hop based on it is dst addr / gateway addr */
	if (rt->rt_gateway)
		pkey = (__be32 ) rt->rt_gateway;
	else
		pkey = (__be32)iph->daddr;
	neigh = __ipv4_neigh_lookup(rt->dst.dev, (__force u32)(pkey));
	//  neigh = __ipv4_neigh_lookup(rt->dst.dev, (__force u32)(iph->daddr));
	if(neigh) {
		ret = ppa_get_mac_from_neigh(mac,neigh,&rt->dst);
		neigh_release(neigh);
	}

	ip_rt_put(rt);
	return ret;
}

/*
 * Returns base dev of the given IPv6 tunneled devie
 * Note: Caller must make sure dev is IPv6 tunnel
 */
	static
struct net_device* ppa_ipv6_tnl_phy(struct net_device *dev)
{
	struct dst_entry *dst;
	struct net *net = dev_net(dev);
	struct net_device* phydev = NULL;
	struct ip6_tnl *t;
	struct flowi6 fl6;

	t = netdev_priv(dev);

	memcpy(&fl6, &t->fl.u.ip6, sizeof (fl6));
	fl6.flowi6_proto = t->parms.proto;
#if IS_ENABLED(CONFIG_IPV6)
	dst = ppa_ip6_route_output(net, NULL, &fl6);
#endif

	if(!(dst->error)) {
		dst = xfrm_lookup(net, dst, flowi6_to_flowi(&fl6), NULL, 0);
		if (IS_ERR(dst))
			return NULL;
	}

	if(dst && dst->dev != dev) {
		phydev=dst->dev;
	}

	dst_release(dst);

	return phydev;
}

/*
 * Returns PPA_SUCCESS if destination MAC is copied.
 * Note: Caller should make sure dev is IPv6 tunnel.
 */
	static
int ppa_get_ipv6_tnl_dmac(uint8_t *mac,
		struct net_device *dev,
		struct sk_buff *skb)
{
	struct ip6_tnl *t = netdev_priv(dev);
	struct dst_entry *dst = NULL;
	struct flowi6 fl6;
	struct net *net = dev_net(dev);
	int ret = PPA_ENOTAVAIL;

	memcpy(&fl6, &t->fl.u.ip6, sizeof (fl6));
	fl6.flowi6_proto = t->parms.proto;
#if IS_ENABLED(CONFIG_IPV6)
	dst = ppa_ip6_route_output(net, NULL, &fl6);
#endif
	if (dst->error) {
		goto no_dst;
	}
	dst = xfrm_lookup(net, dst, flowi6_to_flowi(&fl6), NULL, 0);
	if(IS_ERR(dst)) {
		return ret;
	}

	if(dst->dev == dev) {
		goto no_dst;
	}

	ret = ppa_get_dmac_from_dst_entry(mac,skb,dst);
no_dst:
	dst_release(dst);
	return ret;
}

	static
int ppa_get_ipv4_tnl_iph(struct iphdr* iph,
		struct net_device *dev,
		uint16_t dataLen)
{
	struct ip_tunnel *t;

	t = netdev_priv(dev);

	ppa_memset(iph,0,sizeof(*iph));
	ppa_memcpy(iph, &t->parms.iph,sizeof(*iph));
	iph->tot_len = sizeof(*iph) + dataLen;
	iph->check = ip_fast_csum(iph, iph->ihl);

	return PPA_SUCCESS;
}

#if IS_ENABLED(CONFIG_IPV6)
int ppa_get_ipv6_tnl_iph(struct ipv6hdr* ip6h,
		struct net_device *dev,
		uint16_t dataLen)
{
	struct ip6_tnl *t;

	t = netdev_priv(dev);

	ppa_memset(ip6h, 0, sizeof(*ip6h));
	ip6h->version = 6;
	ip6h->hop_limit = t->parms.hop_limit;
	ip6h->nexthdr = t->parms.proto;
	ipv6_addr_copy(&ip6h->saddr, &t->parms.laddr);
	ipv6_addr_copy(&ip6h->daddr, &t->parms.raddr);
	ip6h->payload_len = dataLen;

	return PPA_SUCCESS;
}

EXPORT_SYMBOL(ppa_get_ipv6_tnl_iph);
bool ppa_is_mape_mesh_session(uint32_t dest_ip,
		struct net_device *dev)
{
	struct ip6_tnl *t;
	struct __ip6_tnl_fmr *fmr;

	t = netdev_priv(dev);

	for (fmr = t->parms.fmrs; fmr; fmr = fmr->next) {
		unsigned mshift = 32 - fmr->ip4_prefix_len;
		if (ntohl(fmr->ip4_prefix.s_addr) >> mshift == dest_ip >> mshift)
			break;
	}

	if (fmr)
		return true;

	return false;
}

EXPORT_SYMBOL(ppa_is_mape_mesh_session);
#endif

#if IS_ENABLED(CONFIG_PPPOE)
enum{
        PPPOE_GET_ADDR = 1,
        PPPOE_GET_SESSION_ID,
        PPPOE_GET_ETH_ADDR,
};

static int32_t ppa_get_pppoe_info(struct net_device *dev, void *po, uint32_t pppoe_id, void *value)
{
        struct pppoe_addr pa;

        if(!po){
                printk("No pppoe sock \n");
                return -1;
        }

        if((pppox_sk(po)->chan.private != po)){
                return -1;
        }

        pa = pppox_sk(po)->pppoe_pa;

        if( (ppa_check_pppoe_addr_valid_fn!=NULL) && ppa_check_pppoe_addr_valid_fn(dev, &pa) < 0)
                return -1;

        switch(pppoe_id){
                case PPPOE_GET_ADDR:
                        *(struct pppoe_addr *)value = pa; break;
                case PPPOE_GET_SESSION_ID:
                        *(u_int16_t *)value = pa.sid; break;
                case PPPOE_GET_ETH_ADDR:
                        memcpy(value, pa.dev, sizeof(pa.dev));  break;
                default:
                        return -1;
        }

        return 0;

}
#endif

#if IS_ENABLED(CONFIG_PPPOE) || IS_ENABLED(CONFIG_PPPOL2TP) || IS_ENABLED(CONFIG_PPPOATM)
static int32_t ppa_ppp_get_info(struct net_device *ppp_dev, uint32_t ppp_info_id, void *value)
{
	int32_t ret = PPA_FAILURE;
	struct ppp_channel *chan = NULL;
	
	if (!ppa_ppp_get_chan_info_fn) {
		printk(KERN_INFO" PPP not initialized \n");
		return ret; 
	} else {
		if(((ret = ppa_ppp_get_chan_info_fn(ppp_dev, &chan))!=PPA_SUCCESS) || (chan == NULL) ||(chan->private == NULL)) {
			return ret;
		}
	}
	
	if((ppp_info_id & PPA_PPP_MASK) == PPA_PPPOE_ID){
#if IS_ENABLED(CONFIG_PPPOE)
        ret = ppa_get_pppoe_info(ppp_dev, chan->private, ppp_info_id >>PPA_PPP_MASK_LEN, value);
#endif
	} else if ((ppp_info_id & PPA_PPP_MASK) == PPA_PPPOL2TP_ID) {
#if IS_ENABLED(CONFIG_PPPOL2TP)
		if(ppa_get_pppol2tp_info_fn!=NULL) {
			if ((ppp_info_id >> PPA_PPP_MASK_LEN) == PPPOL2TP_GET_DMAC) {
				ret = ppa_get_pppol2tp_info_fn(ppp_dev, chan->private,
						ppp_info_id >> PPA_PPP_MASK_LEN, (uint8_t *)value);
			} else {
				ret = ppa_get_pppol2tp_info_fn(ppp_dev, chan->private,
                        ppp_info_id >> PPA_PPP_MASK_LEN, value);
			}
		}
		else {
			ret = PPA_FAILURE;
		}
#endif
    	}else{
#if IS_ENABLED(CONFIG_PPPOATM)
		if(ppa_get_pppoa_info_fn!=NULL)
			ret = ppa_get_pppoa_info_fn(ppp_dev, chan->private, ppp_info_id >>PPA_PPP_MASK_LEN, value);
		else
			ret = PPA_FAILURE;
#endif
    	}
	return ret;
}
#endif

#if IS_ENABLED(CONFIG_PPPOE)
int32_t ppa_pppoe_get_pppoe_addr(struct net_device *dev, struct pppoe_addr *pa)
{
        uint32_t id = (PPPOE_GET_ADDR << PPA_PPP_MASK_LEN) | PPA_PPPOE_ID;

        return ppa_ppp_get_info(dev, id, pa);
}

uint16_t ppa_pppoe_get_pppoe_session_id(struct net_device *dev)
{
        uint16_t seid = 0;
        uint32_t id = (PPPOE_GET_SESSION_ID << PPA_PPP_MASK_LEN) | PPA_PPPOE_ID;

        ppa_ppp_get_info(dev, id, &seid);

        return seid;
}

uint16_t ppa_get_pkt_pppoe_session_id(struct sk_buff *skb)
{
        if(!skb)
                return 0;

        return ppa_pppoe_get_pppoe_session_id(skb->dev);
}

int32_t ppa_pppoe_get_eth_netif(struct net_device *dev, char pppoe_eth_ifname[IFNAMSIZ])
{
        uint32_t id = (PPPOE_GET_ETH_ADDR << PPA_PPP_MASK_LEN) | PPA_PPPOE_ID;

        return ppa_ppp_get_info(dev, id, pppoe_eth_ifname);
}
#endif /*CONFIG_PPPOE*/
EXPORT_SYMBOL(ppa_pppoe_get_pppoe_session_id);
EXPORT_SYMBOL(ppa_get_pkt_pppoe_session_id);
EXPORT_SYMBOL(ppa_pppoe_get_pppoe_addr);
EXPORT_SYMBOL(ppa_pppoe_get_eth_netif);

#if IS_ENABLED(CONFIG_PPPOATM)
enum {
       PPA_PPPOA_GET_VCC = 1,
       PPA_PPPOA_CHECK_IFACE,
};

int32_t ppa_pppoa_get_vcc(struct net_device *dev, struct atm_vcc **patmvcc)
{
    uint32_t id = (PPA_PPPOA_GET_VCC << PPA_PPP_MASK_LEN) | PPA_PPPOA_ID;

        return ppa_ppp_get_info(dev, id, (void *) patmvcc);

}

int32_t ppa_if_is_pppoa(struct net_device *dev, char *ifname)
{
    uint32_t id = (PPA_PPPOA_CHECK_IFACE << PPA_PPP_MASK_LEN) | PPA_PPPOA_ID;

    if ( !dev )
    {
        dev = dev_get_by_name(&init_net,ifname);
        if ( dev )
            dev_put(dev);
        else{
            return 0;
        }
    }

    if(ppa_ppp_get_info(dev, id, &id) >= 0){
                return 1;
    }

        return 0;
}
#endif /*CONFIG_PPPOATM*/
EXPORT_SYMBOL(ppa_pppoa_get_vcc);
EXPORT_SYMBOL(ppa_if_is_pppoa);

#if IS_ENABLED(CONFIG_PPPOL2TP)
uint16_t ppa_pppol2tp_get_l2tp_session_id(struct net_device *dev)
{
        uint16_t seid = 0;
        uint32_t id = (PPPOL2TP_GET_SESSION_ID << PPA_PPP_MASK_LEN)
                | PPA_PPPOL2TP_ID;

        ppa_ppp_get_info(dev, id, &seid);

        return seid;
}

uint16_t ppa_pppol2tp_get_l2tp_tunnel_id(struct net_device *dev)
{
        uint16_t tuid = 0;
        uint32_t id = (PPPOL2TP_GET_TUNNEL_ID << PPA_PPP_MASK_LEN)
                | PPA_PPPOL2TP_ID;

        ppa_ppp_get_info(dev, id, &tuid);

        return tuid;
}

int32_t ppa_pppol2tp_get_base_netif(struct net_device *dev,
                                    char pppol2tp_eth_ifname[IFNAMSIZ])
{
        uint32_t id = (PPPOL2TP_GET_BASEIF << PPA_PPP_MASK_LEN)
                | PPA_PPPOL2TP_ID;

        return ppa_ppp_get_info(dev, id, pppol2tp_eth_ifname);
}

int32_t ppa_pppol2tp_get_l2tp_addr(struct net_device *dev,
                                   struct pppol2tp_addr *pa)
{
        uint32_t id = (PPPOL2TP_GET_ADDR << PPA_PPP_MASK_LEN)
                | PPA_PPPOL2TP_ID;
        return ppa_ppp_get_info(dev, id, pa);
}

int32_t ppa_pppol2tp_get_l2tp_dmac(struct net_device *dev, uint8_t *mac)
{
        uint32_t id = (PPPOL2TP_GET_DMAC << PPA_PPP_MASK_LEN)
                | PPA_PPPOL2TP_ID;
        return ppa_ppp_get_info(dev, id, mac);
}

int32_t ppa_pppol2tp_get_src_addr(struct net_device *dev, uint32_t *outer_srcip)
{
        uint32_t id = (PPPOL2TP_GET_SADDR << PPA_PPP_MASK_LEN)
                | PPA_PPPOL2TP_ID;
        return ppa_ppp_get_info(dev, id, outer_srcip);
}

int32_t ppa_pppol2tp_get_dst_addr(struct net_device *dev, uint32_t *outer_dstip)
{
        uint32_t id = (PPPOL2TP_GET_DADDR << PPA_PPP_MASK_LEN)
                | PPA_PPPOL2TP_ID;
        return ppa_ppp_get_info(dev, id, outer_dstip);
}
#endif /*CONFIG_PPPOL2TP*/
EXPORT_SYMBOL(ppa_pppol2tp_get_l2tp_session_id);
EXPORT_SYMBOL(ppa_pppol2tp_get_l2tp_tunnel_id);
EXPORT_SYMBOL(ppa_pppol2tp_get_base_netif);
EXPORT_SYMBOL(ppa_pppol2tp_get_dst_addr);
EXPORT_SYMBOL(ppa_pppol2tp_get_src_addr);
/*
 * Returns 1 if given netif is gre tunnel, else returns zero
 */
uint32_t ppa_is_gre_netif(struct net_device* dev)
{
	uint32_t t_flags = TUNNEL_SEQ;

	if (!dev)
		return PPA_NETIF_FAIL;

	/* IPv4 GRE/ GRE tap */
	if( dev->type == ARPHRD_IPGRE ||
			(ppa_is_ipv4_gretap_fn && ppa_is_ipv4_gretap_fn(dev)) ) {

		t_flags = ((struct ip_tunnel*)netdev_priv(dev))->parms.o_flags;
	} else if( dev->type == ARPHRD_IP6GRE ||
			(ppa_is_ipv6_gretap_fn && ppa_is_ipv6_gretap_fn(dev)) ) {

		/* IPv6 GRE/ GRE tap */
		t_flags = ((struct ip_tunnel*)netdev_priv(dev))->parms.o_flags;
	}

	/* GRE with sequence number is not supported */
	return (t_flags & TUNNEL_SEQ)?0:1;
}
EXPORT_SYMBOL(ppa_is_gre_netif);

#if LINUX_VERSION_CODE > KERNEL_VERSION(4,15,0)
#if IS_ENABLED(CONFIG_NET_IPGRE)
extern bool is_gretap_dev(const struct net_device *dev);
extern bool is_ip6gretap_dev(const struct net_device *dev);
static inline uint32_t is_ipv4_gretap(struct net_device* dev)
{
	return (uint32_t) is_gretap_dev(dev);
}

static inline uint32_t is_ipv6_gretap(struct net_device* dev)
{
	return (uint32_t) is_ip6gretap_dev(dev);
}

void init_gre_hooks()
{
	ppa_is_ipv4_gretap_fn = is_ipv4_gretap;
	ppa_is_ipv6_gretap_fn = is_ipv6_gretap;
}
EXPORT_SYMBOL(init_gre_hooks);

void uninit_gre_hooks()
{
	ppa_is_ipv4_gretap_fn = NULL;
	ppa_is_ipv6_gretap_fn = NULL;
}
EXPORT_SYMBOL(uninit_gre_hooks);
#endif/* IS_ENABLED(CONFIG_NET_IPGRE)*/
#endif /*LINUX_VERSION_CODE > KERNEL_VERSION(4,15,0)*/

uint32_t ppa_is_gre_netif_type(struct net_device* dev,
		uint8_t* isIPv4Gre,
		uint8_t* isGreTap)
{
	uint32_t t_flags = TUNNEL_SEQ;

	if (!dev)
		return PPA_NETIF_FAIL;

	/* IPv4 GRE/ GRE tap */
	if( dev->type == ARPHRD_IPGRE ||
			(ppa_is_ipv4_gretap_fn && ppa_is_ipv4_gretap_fn(dev)) ) {

		*isIPv4Gre = 1;
		*isGreTap = (dev->type == ARPHRD_IPGRE)?0:1;
		t_flags = ((struct ip_tunnel*)netdev_priv(dev))->parms.o_flags;
	} else if( dev->type == ARPHRD_IP6GRE ||
			(ppa_is_ipv6_gretap_fn && ppa_is_ipv6_gretap_fn(dev)) ) {

		/* IPv6 GRE/ GRE tap */
		*isIPv4Gre = 0;
		*isGreTap = (dev->type == ARPHRD_IP6GRE)?0:1;
		t_flags = ((struct ip_tunnel*)netdev_priv(dev))->parms.o_flags;
	}

	ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,"%s netdev=%s is_IPv4Gre=%d is_GreTap=%d", __FUNCTION__, dev->name, *isIPv4Gre, *isGreTap);
	return (t_flags & TUNNEL_SEQ)?0:1;
}
EXPORT_SYMBOL(ppa_is_gre_netif_type);

/*
 * Returns physical interface of the gre tunnel.
 * On failure returns NULL.
 */
struct net_device* ppa_get_gre_phyif(struct net_device* dev)
{
	if( dev->type == ARPHRD_IPGRE ||
			(ppa_is_ipv4_gretap_fn && ppa_is_ipv4_gretap_fn(dev)) ) {

		return ppa_ipv4_tnl_phy(dev);
	} else if(dev->type == ARPHRD_IP6GRE ||
			(ppa_is_ipv6_gretap_fn && ppa_is_ipv6_gretap_fn(dev)) ) {
		return ppa_ipv6_tnl_phy(dev);
	}
	return NULL;
}
EXPORT_SYMBOL(ppa_get_gre_phyif);

/*
 * On success returns destination MAC address.
 */
int32_t ppa_get_gre_dmac(uint8_t *mac,
		struct net_device* dev,
		struct sk_buff *skb)
{
	int ret = PPA_FAILURE;
	struct net_device* phydev;

	phydev = ppa_get_gre_phyif(dev);
	if(!phydev)
		goto mac_err;

	if( ppa_check_is_ppp_netif(phydev)) {
		/* Check if PPPoE interface */
		if ( ppa_check_is_pppoe_netif(phydev) )
			ret= ppa_pppoe_get_dst_mac(phydev, mac);
	} else {

		if( dev->type == ARPHRD_IPGRE ||
				(ppa_is_ipv4_gretap_fn && ppa_is_ipv4_gretap_fn(dev)) ) {

			ret = ppa_get_ipv4_tnl_dmac(mac, dev, skb);
		} else if(dev->type == ARPHRD_IP6GRE ||
				(ppa_is_ipv6_gretap_fn && ppa_is_ipv6_gretap_fn(dev)) ) {

			ret = ppa_get_ipv6_tnl_dmac(mac, dev, skb);
		}
	}

mac_err:
	return ret;
}
EXPORT_SYMBOL(ppa_get_gre_dmac);

int32_t ppa_get_gre_hdrlen(PPA_NETIF* dev, uint16_t *hdrlen)
{
	*hdrlen = 0;

	if (dev == NULL)
		return PPA_FAILURE;

	if( dev->type == ARPHRD_IPGRE ||
			(ppa_is_ipv4_gretap_fn && ppa_is_ipv4_gretap_fn(dev)) ) {
		struct ip_tunnel *t =  netdev_priv(dev);
		*hdrlen = t->hlen + sizeof(struct iphdr);
	} else if( dev->type == ARPHRD_IP6GRE ||
			(ppa_is_ipv6_gretap_fn && ppa_is_ipv6_gretap_fn(dev)) ) {

		struct ip6_tnl *t =  netdev_priv(dev);
		*hdrlen = t->hlen;
#if LINUX_VERSION_CODE > KERNEL_VERSION(4,6,7)
		*hdrlen += sizeof(struct ipv6hdr);
#endif
	} else {
		return PPA_FAILURE;
	}

	return PPA_SUCCESS;
}
EXPORT_SYMBOL(ppa_get_gre_hdrlen);


int32_t ppa_form_gre_hdr(PPA_NETIF* dev,
		uint8_t isIPv6,
		uint16_t dataLen,
		uint8_t *pHdr,
		uint16_t* len)
{
	int hlen;
	uint32_t t_flags;
	uint32_t t_key;
	uint16_t proto;
	uint8_t ipv4 = 0;
	int gre_hlen;
	uint16_t ip_hlen;
	struct gre_base_hdr* grehdr;

	if (dev == NULL)
		return PPA_FAILURE;

	if( dev->type == ARPHRD_IPGRE ||
			(ppa_is_ipv4_gretap_fn && ppa_is_ipv4_gretap_fn(dev)) ) {

		struct ip_tunnel *t =  netdev_priv(dev);
		ip_hlen = sizeof(struct iphdr);
		gre_hlen = t->hlen;
#if LINUX_VERSION_CODE > KERNEL_VERSION(4,6,7)
		t_flags = gre_tnl_flags_to_gre_flags(t->parms.o_flags);
#else
		t_flags = tnl_flags_to_gre_flags(t->parms.o_flags);
#endif
		t_key = t->parms.o_key;
		proto = (dev->type == ARPHRD_IPGRE)?ETH_P_IP:ETH_P_TEB;
		ipv4 = 1;
	} else if( dev->type == ARPHRD_IP6GRE ||
			(ppa_is_ipv6_gretap_fn && ppa_is_ipv6_gretap_fn(dev)) ) {

		struct ip6_tnl *t =  netdev_priv(dev);
		ip_hlen = sizeof(struct ipv6hdr);
		gre_hlen = t->hlen;
#if LINUX_VERSION_CODE > KERNEL_VERSION(4,6,7)
		t_flags = gre_tnl_flags_to_gre_flags(t->parms.o_flags);
#else
		gre_hlen -= ip_hlen;
		t_flags = t->parms.o_flags;
#endif
		t_key = t->parms.o_key;
		proto = (dev->type == ARPHRD_IP6GRE)?ETH_P_IPV6:ETH_P_TEB;
	} else {
		return PPA_FAILURE;
	}

	hlen = gre_hlen;

	if( *len < (hlen + ip_hlen) )
		return PPA_FAILURE;

	if( ipv4 ) {
		ppa_get_ipv4_tnl_iph((struct iphdr*)pHdr,dev,hlen+dataLen);
	} else {
#if IS_ENABLED(CONFIG_IPV6)
		ppa_get_ipv6_tnl_iph((struct ipv6hdr*)pHdr,dev,hlen+dataLen);
#endif
		((struct ipv6hdr*)pHdr)->nexthdr = IPPROTO_GRE;
	}
	pHdr += ip_hlen;
	hlen += ip_hlen;

	grehdr =  (struct gre_base_hdr*)pHdr;
	grehdr->flags = t_flags;

	if(proto != ETH_P_TEB) {
		if(isIPv6)
			proto = ETH_P_IPV6;
		else
			proto = ETH_P_IP;
	}

	grehdr->protocol = htons(proto);
	pHdr += sizeof(*grehdr);

	//Note: Sequence numbering is not supported.
	if(t_flags & GRE_CSUM) {
		*((uint32_t*)(pHdr)) = 0;
		pHdr += 4;
	}
	if(t_flags & GRE_KEY) {
		*((uint32_t*)(pHdr)) = t_key;
	}

	*len = hlen;
	return PPA_SUCCESS;
}
EXPORT_SYMBOL(ppa_form_gre_hdr);

/*
 * Returns PPA_SUCCESS if valid destination is available.
 *
 */
int32_t ppa_get_dmac_from_dst_entry( uint8_t* mac,
		PPA_BUF* skb,
		struct dst_entry *dst)
{
	struct neighbour *neigh;
	int ret = PPA_ENOTAVAIL;

	neigh = dst_neigh_lookup_skb(dst,skb);

	if(neigh) {
		ret = ppa_get_mac_from_neigh(mac,neigh,dst);
		neigh_release(neigh);
	}
	return ret;
}
EXPORT_SYMBOL(ppa_get_dmac_from_dst_entry);

/* 6RD tunnel routines */
#if IS_ENABLED(CONFIG_IPV6_SIT)

int ppa_get_6rd_dmac(struct net_device *dev, struct sk_buff *skb, uint8_t *mac,
		     uint32_t daddr)
{
	struct ip_tunnel *tunnel;
	struct iphdr *iph;
	struct rtable *rt = NULL;
	struct dst_entry *dst = NULL;
	struct hh_cache *hh;
	struct neighbour *neigh;
	struct flowi4 fl = {0};

	tunnel = netdev_priv(dev);
	iph = &tunnel->parms.iph;

	if (iph->daddr)
		daddr = iph->daddr;

	if (daddr) {
		fl.flowi4_tos = RT_TOS(iph->tos);
		fl.flowi4_oif = tunnel->parms.link;
		fl.flowi4_proto = IPPROTO_IPV6;
		fl.daddr = daddr;
		fl.saddr = iph->saddr;
	}

	rt = ip_route_output_key(dev_net(dev), &fl);
	if (rt == NULL)
		return -1;
	else
		dst = &rt->dst;

	neigh = dst_neigh_lookup_skb(dst, skb);
	if (neigh == NULL) {
		ip_rt_put(rt);
		return -1;
	}

	ppa_neigh_hh_init(neigh);
	ppa_neigh_update_hhs(neigh);

	hh = &neigh->hh;
	if (!hh) {
		if (neigh->ha[0] != 0
		    || neigh->ha[1] != 0
		    || neigh->ha[2] != 0
		    || neigh->ha[3] != 0
		    || neigh->ha[4] != 0
		    || neigh->ha[5] != 0)
			memcpy(mac, (uint8_t *)neigh->ha, ETH_ALEN);
		else
			goto MAC_ERROR;
	} else {
		unsigned int seq;
		do {
			seq = read_seqbegin(&hh->hh_lock);
			if (hh->hh_len != ETH_HLEN)
				goto MAC_ERROR;
			else
				memcpy(mac, (uint8_t *)hh->hh_data + HH_DATA_ALIGN(hh->hh_len) - hh->hh_len, ETH_ALEN);
		} while (read_seqretry(&hh->hh_lock, seq));
	}

	/* In case of Back to Back Setups as 6rd tun don't have mac */
	if (is_zero_ether_addr(mac)) {
		neigh_release(neigh);
		neigh = __ipv4_neigh_lookup(dst->dev, daddr);
		if (neigh == NULL) {
			ip_rt_put(rt);
			return -1;
		}

		if (is_valid_ether_addr(neigh->ha))
			memcpy(mac, (uint8_t *)neigh->ha, ETH_ALEN);
		else
			goto MAC_ERROR;
	}

	neigh_release(neigh);
	ip_rt_put(rt);
	return 0;

MAC_ERROR:
	neigh_release(neigh);
	ip_rt_put(rt);
	return -1;
}

struct net_device *ppa_get_6rd_phyif(struct net_device *dev)
{
	struct ip_tunnel *tunnel;
	struct iphdr *iph;
	struct rtable *rt = NULL;
	struct net_device *phydev = NULL;
	struct flowi4 fl;

	tunnel = netdev_priv(dev);
	iph = &tunnel->parms.iph;

	if (iph->daddr) {
		fl.flowi4_tos = RT_TOS(iph->tos);
		fl.flowi4_oif = tunnel->parms.link;
		fl.flowi4_proto = IPPROTO_IPV6;
		fl.daddr = iph->daddr;
		fl.saddr = iph->saddr;

		rt = ip_route_output_key(dev_net(dev), &fl);
		if (rt) {
			phydev = rt->dst.dev;
			ip_rt_put(rt);
		}
	} else {
		phydev = __ip_dev_find(&init_net, iph->saddr, false);
	}
	return phydev;
}

int32_t ppa_get_6rd_dst_mac(struct net_device *dev, PPA_BUF *ppa_buf,uint8_t *mac,uint32_t daddr)
{
	struct net_device *phy_dev;

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,8,10)
	buf = buf;
#endif

	phy_dev = ppa_get_6rd_phyif(dev);
	if(!phy_dev){
		ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,"cannot get infrastructural device\n");
		return PPA_ENOTAVAIL;
	}
	/* First need to check if PPP output interface */
	if( ppa_check_is_ppp_netif(phy_dev) )
	{
		/* Check if PPPoE interface */
		if ( ppa_check_is_pppoe_netif(phy_dev) )
		{
			return ppa_pppoe_get_dst_mac(phy_dev, mac);
		}
		return PPA_ENOTPOSSIBLE;
	} else {
		return ppa_get_6rd_dmac(dev, ppa_buf,mac,daddr);
	}

	return PPA_ENOTAVAIL;
}

	static inline
uint32_t try_6rd_ppa(struct in6_addr *v6dst, struct ip_tunnel *tunnel)
{
	uint32_t dst = 0;

#ifdef CONFIG_IPV6_SIT_6RD
	if(ipv6_prefix_equal(v6dst,
				&tunnel->ip6rd.prefix, tunnel->ip6rd.prefixlen)) {

		unsigned pbw0, pbi0;
		int pbi1;
		u32 d;

		pbw0 = tunnel->ip6rd.prefixlen >> 5;
		pbi0 = tunnel->ip6rd.prefixlen & 0x1f;

		d = (ntohl(v6dst->s6_addr32[pbw0]) << pbi0) >>
			tunnel->ip6rd.relay_prefixlen;

		pbi1 = pbi0 - tunnel->ip6rd.relay_prefixlen;
		if (pbi1 > 0)
			d |= ntohl(v6dst->s6_addr32[pbw0 + 1]) >>
				(32 - pbi1);

		dst = tunnel->ip6rd.relay_prefix | htonl(d);
	}
#else
	if (v6dst->s6_addr16[0] == htons(0x2002)) {
		/* 6to4 v6 addr has 16 bits prefix, 32 v4addr, 16 SLA, ... */
		memcpy(&dst, &v6dst->s6_addr16[1], 4);
	}
#endif
	return dst;
}

uint32_t ppa_get_6rdtunel_dst_ip(PPA_BUF *skb, PPA_NETIF *dev)
{
	struct ip_tunnel *tunnel = netdev_priv(dev);
	struct iphdr  *tiph = &tunnel->parms.iph;
	struct ipv6hdr *iph6 = ipv6_hdr(skb);
	__be32 dst = tiph->daddr;
	struct in6_addr *addr6;
	int addr_type;

	if (skb->protocol != htons(ETH_P_IPV6))
		goto tx_error;

	if (dev->priv_flags & IFF_ISATAP) {
		struct neighbour *neigh = NULL;
		bool do_tx_error = false;

		if (skb_dst(skb))
			//neigh = skb_dst(skb)->neighbour;
			neigh = dst_neigh_lookup(skb_dst(skb), &iph6->daddr);

		if (neigh == NULL) {
			net_dbg_ratelimited("sit: nexthop == NULL\n");
			goto tx_error;
		}

		addr6 = (struct in6_addr*)&neigh->primary_key;
		addr_type = ipv6_addr_type(addr6);

		if ((addr_type & IPV6_ADDR_UNICAST) &&
				ipv6_addr_is_isatap(addr6))
			dst = addr6->s6_addr32[3];
		else
			do_tx_error = true;

		neigh_release(neigh);
		if (do_tx_error)
			goto tx_error;
	}

	if (!dst)
		dst = try_6rd_ppa(&iph6->daddr, tunnel);

	if (!dst) {
		struct neighbour *neigh = NULL;

		if (skb_dst(skb))
			//neigh = skb_dst(skb)->neighbour;
			neigh = dst_neigh_lookup(skb_dst(skb), &iph6->daddr);

		if (neigh == NULL) {
			net_dbg_ratelimited("sit: nexthop == NULL\n");
			goto tx_error;
		}

		addr6 = (struct in6_addr*)&neigh->primary_key;
		addr_type = ipv6_addr_type(addr6);

		if (addr_type == IPV6_ADDR_ANY) {
			addr6 = &ipv6_hdr(skb)->daddr;
			addr_type = ipv6_addr_type(addr6);
		}


		if ((addr_type & IPV6_ADDR_COMPATv4) == 0) {
			neigh_release(neigh);
			goto tx_error;
		}

		dst = addr6->s6_addr32[3];
		neigh_release(neigh);
	}

tx_error:
	return dst;
}

#else

struct net_device *ppa_get_6rd_phyif(struct net_device *dev)
{
	return NULL;
}

uint32_t ppa_get_6rdtunel_dst_ip(PPA_BUF *skb, PPA_NETIF *dev)
{
	return 0;
}
#endif
EXPORT_SYMBOL(ppa_get_6rd_phyif);
EXPORT_SYMBOL(ppa_get_6rdtunel_dst_ip);

/* DS-Lite tunnel routines  */
#if IS_ENABLED(CONFIG_IPV6_TUNNEL)

struct net_device *ppa_get_ip4ip6_phyif(struct net_device *dev)
{
	struct ip6_tnl *t = netdev_priv(dev);
	struct dst_entry *dst = NULL;

	struct flowi6 fl6 = {0};
	int err;
	struct dst_entry *ndst = NULL;
	struct net *net = dev_net(dev);
	bool use_cache = (t->parms.flags & (IP6_TNL_F_USE_ORIG_TCLASS | IP6_TNL_F_USE_ORIG_FWMARK)) ? 0:1; 

	memcpy(&fl6, &t->fl.u.ip6, sizeof(fl6));
	fl6.flowi6_proto = IPPROTO_IPIP;

	if(use_cache) 
		dst = dst_cache_get(&t->dst_cache);
	if (dst != NULL) {
		if (dst->dev != dev)
			return dst->dev;
	} else {
		ndst = ip6_route_output(net, NULL, &fl6);

		if (ndst->error)
			goto tx_err_link_failure;
		ndst = xfrm_lookup(net, ndst, flowi6_to_flowi(&fl6), NULL, 0);
		if (IS_ERR(ndst)) {
			err = PTR_ERR(ndst);
			ndst = NULL;
			goto tx_err_link_failure;
		}
		dst = ndst;

		if (dst->dev != dev)
			return dst->dev;
	}
tx_err_link_failure:
	return NULL;
}

int ppa_get_ip4ip6_dmac(struct net_device *dev, struct sk_buff *skb,
			uint8_t *mac)
{
	struct ip6_tnl *t = netdev_priv(dev);
	struct dst_entry *dst = NULL;
	struct hh_cache *hh;
	bool use_cache = (t->parms.flags & (IP6_TNL_F_USE_ORIG_TCLASS | IP6_TNL_F_USE_ORIG_FWMARK)) ? 0:1; 

	struct flowi6 fl6 = {0};
	int err;
	struct dst_entry *ndst = NULL;
	struct net *net = dev_net(dev);
	struct neighbour *neigh;
	unsigned int seq;

	if (!mac)
		return -1;

	memcpy(&fl6, &t->fl.u.ip6, sizeof(fl6));
	fl6.flowi6_proto = IPPROTO_IPIP;

	if(use_cache) 
		dst = dst_cache_get(&t->dst_cache);
	if (dst == NULL || dst->dev == dev) {
		ndst = ip6_route_output(net, NULL, &fl6);
		if (ndst->error)
			return -1;
		ndst = xfrm_lookup(net, ndst, flowi6_to_flowi(&fl6), NULL, 0);
		if (IS_ERR(ndst)) {
			err = PTR_ERR(ndst);
			ndst = NULL;
			return -1;
		}
		dst = ndst;

		if (dst->dev == dev)
			return -1;
	}
	dst_hold(dst);

	neigh = dst_neigh_lookup(dst, &(t->parms.raddr));
	if (neigh == NULL)
		goto MAC_ERROR;

	if (dst->dev->header_ops->cache == NULL) {
		ppa_neigh_hh_init(neigh);
		ppa_neigh_update_hhs(neigh);
	}

	hh = &neigh->hh;
	if (!hh) {
		if (neigh && (neigh->ha[0] != 0
			      || neigh->ha[1] != 0
			      || neigh->ha[2] != 0
			      || neigh->ha[3] != 0
			      || neigh->ha[4] != 0
			      || neigh->ha[5] != 0)) {
			memcpy(mac, (uint8_t *)neigh->ha, ETH_ALEN);
		} else {
			neigh_release(neigh);
			goto MAC_ERROR;
		}
	} else {
		do {
			seq = read_seqbegin(&hh->hh_lock);
			if (hh->hh_len != ETH_HLEN) {
				neigh_release(neigh);
				goto MAC_ERROR;
			} else {
				memcpy(mac, (uint8_t *)hh->hh_data + HH_DATA_ALIGN(hh->hh_len) - hh->hh_len, ETH_ALEN);
			}
		} while (read_seqretry(&hh->hh_lock, seq));
	}
	neigh_release(neigh);
	dst_release(dst);
	return 0;

MAC_ERROR:
	dst_release(dst);
	return -1;
}

int32_t ppa_get_dslite_dst_mac(struct net_device *dev,PPA_BUF* buf, uint8_t *mac)
{
	struct net_device *phy_dev;

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,8,10)
	buf = buf;
#endif

	phy_dev = ppa_get_ip4ip6_phyif(dev);
	if(!phy_dev){
		ppa_debug(DBG_ENABLE_MASK_DEBUG_PRINT,"cannot get infrastructural device\n");
		return PPA_ENOTAVAIL;
	}

	/* First need to check if PPP output interface */
	if ( ppa_check_is_ppp_netif(phy_dev) ) {
		/* Check if PPPoE interface */
		if ( ppa_check_is_pppoe_netif(phy_dev) )
		{
			/* Determine PPPoE MAC address */
			return ppa_pppoe_get_dst_mac(phy_dev, mac);
		}
		return PPA_ENOTPOSSIBLE;
	} else {
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,8,10)
		return ppa_get_ip4ip6_dmac(dev, mac);
#else
		return  ppa_get_ip4ip6_dmac(dev,buf, mac);
#endif
	}

	return PPA_ENOTAVAIL;
}
#else

struct net_device *ppa_get_ip4ip6_phyif(struct net_device *dev)
{
	return NULL;
}
#endif
EXPORT_SYMBOL(ppa_get_ip4ip6_phyif);


void ppa_neigh_update_hhs(struct neighbour *neigh)
{
	struct hh_cache *hh;
	void (*update)(struct hh_cache*, const struct net_device*, const unsigned char *)
		= NULL;

	update = eth_header_cache_update;

	if (update) {
		hh = &neigh->hh;
		if(hh == NULL)
			return;

		if (hh->hh_len) {
			write_seqlock_bh(&hh->hh_lock);
			update(hh, neigh->dev, neigh->ha);
			write_sequnlock_bh(&hh->hh_lock);
		}
	}
}
EXPORT_SYMBOL(ppa_neigh_update_hhs);

void ppa_neigh_hh_init(struct neighbour *n)
{
	__be16 prot = n->tbl->protocol;

	struct hh_cache	*hh = &n->hh;

	write_lock_bh(&n->lock);

	/* Only one thread can come in here and initialize the
	 * hh_cache entry.
	 */
	if (!hh->hh_len)
		eth_header_cache(n, hh, prot);

	write_unlock_bh(&n->lock);
}
EXPORT_SYMBOL(ppa_neigh_hh_init);

uint32_t ppa_pkt_from_tunnelv6_netdev( PPA_NETIF *netdev, struct in6_addr saddr, struct in6_addr daddr)
{
	struct ip6_tnl *tunnel;

	tunnel = netdev_priv(netdev);
	if(tunnel) {
		if(!(ipv6_addr_equal(&saddr,&tunnel->parms.laddr) &&
			ipv6_addr_equal(&daddr,&tunnel->parms.raddr))) {
				return 1;
		}
	}

	return 0;
}
EXPORT_SYMBOL(ppa_pkt_from_tunnelv6_netdev);

uint32_t ppa_pkt_from_tunnelv4_netdev( PPA_NETIF *netdev, u32 saddr, u32 daddr)
{
	struct ip_tunnel *tunnel;

	tunnel = netdev_priv(netdev);
	if(tunnel) {
		if((saddr==tunnel->parms.iph.saddr) &&
			(daddr == tunnel->parms.iph.daddr)) {
			return 1;
		}
	}

	return 0;
}
EXPORT_SYMBOL(ppa_pkt_from_tunnelv4_netdev);

