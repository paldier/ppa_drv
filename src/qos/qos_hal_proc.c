/*****************************************************************************
 **
 ** FILE NAME    : qos_hal_dp.c
 ** PROJECT      : QOS HAL
 ** MODULES      : (QoS Engine )
 **
 ** DATE         : 29 May 2018
 ** AUTHOR       : Purnendu Ghosh
 ** DESCRIPTION  : QOS HAL Layer
 ** COPYRIGHT : Copyright (c) 2017 - 2018 Intel Corporation
 ** Copyright (c) 2010 - 2016 Lantiq Beteiligungs-GmbH & Co. KG
 **
 *****************************************************************************/

/*
 *  Common Header File
 */
#include <net/ppa/ppa_api.h>
#include <net/ppa/ppa_hal_api.h>
#include <net/datapath_api.h>
#include "qos_hal_debug.h"

#define NUM_ENTITY(x)                           (sizeof(x) / sizeof(*(x)))
extern int g_ppa_proc_dir_flag;
extern struct proc_dir_entry *g_ppa_proc_dir ;
extern int qos_hal_get_queue_info(struct net_device *netdev);
extern int qos_hal_get_queue_map(struct net_device *netdev);
extern int qos_hal_get_detailed_queue_map(uint32_t queue_index);
static int proc_read_queue_info_seq_open(struct inode *, struct file *);
static int proc_read_queue_info(struct seq_file *, void *);
static int proc_read_tc_map_seq_open(struct inode *, struct file *);
static int proc_read_tc_map(struct seq_file *, void *);
static int proc_read_queue_map_seq_open(struct inode *, struct file *);
static int proc_read_queue_map(struct seq_file *, void *);
static int proc_read_tc_group_seq_open(struct inode *, struct file *);
static int proc_read_tc_group(struct seq_file *, void *);

static int proc_read_dbg(struct seq_file *seq, void *v);
static ssize_t proc_write_queue_info(struct file *file, const char __user *buf, size_t count, loff_t *data);
static ssize_t proc_write_tc_map(struct file *file, const char __user *buf, size_t count, loff_t *data);
static ssize_t proc_write_queue_map(struct file *file, const char __user *buf, size_t count, loff_t *data);
static ssize_t proc_write_dbg(struct file *file, const char __user *buf, size_t count, loff_t *data);
static ssize_t proc_write_tc_group(struct file *file, const char __user *buf, size_t count, loff_t *data);
static struct proc_dir_entry *g_ppa_qos_hal_proc_dir = NULL;
static int g_ppa_qos_hal_proc_dir_flag = 0;

extern uint32_t high_prio_q_limit ;
extern uint32_t g_qos_dbg ;

static int strincmp(const char *p1, const char *p2, int n)
{
	int c1 = 0, c2;

	while ( n && *p1 && *p2 )
	{
		c1 = *p1 >= 'A' && *p1 <= 'Z' ? *p1 + 'a' - 'A' : *p1;
		c2 = *p2 >= 'A' && *p2 <= 'Z' ? *p2 + 'a' - 'A' : *p2;
		if ( (c1 -= c2) )
			return c1;
		p1++;
		p2++;
		n--;
	}

	return n ? *p1 - *p2 : c1;
}

int Atoi(char *str)
{
	int res = 0;  // Initialize result
	int sign = 1;  // Initialize sign as positive
	int i = 0;  // Initialize index of first digit

	// If number is negative, then update sign
	if (str[0] == '-')
	{
		sign = -1; 
		i++;  // Also update index of first digit
	}

	// Iterate through all digits and update the result
	for (; str[i] != '\0'; ++i)
		res = res*10 + str[i] - '0';

	// Return result with sign
	return sign*res;
}

int return_val( char *p, char *str)
{
        char *temp;
	char buf[30];
	snprintf(buf, sizeof(buf), "%s", p);
        if ((temp =strstr(buf, str)) != NULL ){
                while(*temp != ' ' && *temp ) {
                        temp++;
                }
                str = ++temp;
                while(*temp != ' ' && *temp ) {
                        temp++;
                }

                *temp = '\0';
        }
        return Atoi(str);

}

char * return_string( char *buf, char *str)
{
	char *temp;
        if ((temp =strstr(buf, str)) != NULL ){
                while(*temp != ' ' && *temp ) {
                        temp++;
                }
                str = ++temp;
                while(*temp != ' ' && *temp ) {
                        temp++;
                }

                *temp = '\0';
	}
	return str ;
}


static struct file_operations g_proc_file_queue_info_seq_fops = {
	.owner      = THIS_MODULE,
	.open       = proc_read_queue_info_seq_open,
	.read       = seq_read,
	.write	= proc_write_queue_info,
	.llseek     = seq_lseek,
	.release    = single_release,
};

static int proc_read_queue_info_seq_open(struct inode *inode, struct file *file)
{
    return single_open(file, proc_read_queue_info, NULL);
}

static int proc_read_queue_info(struct seq_file *seq, void *v)
{
        if (!capable(CAP_NET_ADMIN)) {
             printk ("Read Permission denied");
             return 0;
        }
	seq_printf(seq, "echo <interface> > /proc/ppa/qos_hal/queue_info\n");
        return 0;
}

static ssize_t proc_write_queue_info(struct file *file, const char __user *buf, size_t count, loff_t *data)
{
	int len;
	char str[64];
	char *p ;
	struct net_device *netdev ;
	uint32_t l_qos_dbg;

	if (!capable(CAP_NET_ADMIN)) {
		printk ("Write Permission denied");
		return 0;
	}
	len = min(count, (size_t)sizeof(str) - 1);
	len -= ppa_copy_from_user(str, buf, len);
	while ( len && str[len - 1] <= ' ' )
		len--;
	str[len] = 0;
	for ( p = str; *p && *p <= ' '; p++, len-- );
	if ( !*p )
		return count;

	if ( strstr(p, "help"))
        {
                printk("echo <interface>  > /proc/ppa/qos_hal/queue_info\n");
                return count;
        }

	if ( (netdev = dev_get_by_name(&init_net, p)) == NULL ) {
                printk("Invalid interface name\n");
                return PPA_FAILURE;
        }
	l_qos_dbg = g_qos_dbg;
	g_qos_dbg = QOS_DEBUG_TRACE; 
	qos_hal_get_queue_info(netdev);
	g_qos_dbg = l_qos_dbg;
	return count;
}

static int proc_read_dbg_seq_open(struct inode *inode, struct file *file)
{
    return single_open(file, proc_read_dbg, NULL);
}


static struct file_operations g_proc_file_dbg_seq_fops = {
    .owner      = THIS_MODULE,
    .open       = proc_read_dbg_seq_open,
    .read       = seq_read,
    .write	= proc_write_dbg,
    .llseek     = seq_lseek,
    .release    = single_release,
};

static int proc_read_dbg(struct seq_file *seq, void *v)
{
	int i;

        if (!capable(CAP_NET_ADMIN)) {
           printk ("Read Permission denied");
           return 0;
        }
	seq_printf(seq, "\t====================================================\n");
	seq_printf(seq,"\t|     CMD        |    Discription    |     Status    |\n");
	seq_printf(seq,"\t----------------------------------------------------\n");
	for(i=0;  i < NUM_ENTITY(dbg_enable_mask_str) -1; i++) {
		seq_printf(seq,"\t|%9s       |",dbg_enable_mask_str[i].cmd);
		seq_printf(seq,"%9s       |",dbg_enable_mask_str[i].des);
		seq_printf(seq,"%9s       |\n",(g_qos_dbg & dbg_enable_mask_str[i].flag)  ? "enabled" : "disabled");
		seq_printf(seq,"\t---------------------------------------------------- \n");
	}
	return 0;
}


static ssize_t proc_write_dbg(struct file *file, const char __user *buf, size_t count, loff_t *data)
{
	int len;
	char str[64];
	char *p;

	int f_enable = 0;
	int i;

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
	if ( strincmp(p, "enable", 6) == 0 )
	{
		p += 6 + 1;  //skip enable and one blank
		len -= 6 + 1;  //len maybe negative now if there is no other parameters
		f_enable = 1;
	}
	else if ( strincmp(p, "disable", 7) == 0 )
	{
		p += 7 + 1;  //skip disable and one blank
		len -= 7 + 1; //len maybe negative now if there is no other parameters
		f_enable = -1;
	}
	else if ( strincmp(p, "help", 4) == 0 || *p == '?' )
	{
		printk("echo <enable/disable> [");
		for ( i = 0; i < NUM_ENTITY(dbg_enable_mask_str); i++ ) printk("%s/", dbg_enable_mask_str[i].cmd );
		printk("] > /proc/ppa/qos_hal/dbg\n");
	}
	if ( f_enable )
	{
		if ( (len <= 0) || ( p[0] >= '0' && p[1] <= '9') )
		{
			if ( f_enable > 0 )
				g_qos_dbg |= QOS_ENABLE_ALL_DEBUG;
			else
				g_qos_dbg &= ~QOS_ENABLE_ALL_DEBUG;
		}
		else
		{
			do
			{
				for ( i = 0; i < NUM_ENTITY(dbg_enable_mask_str); i++ )
					if ( strincmp(p, dbg_enable_mask_str[i].cmd, strlen(dbg_enable_mask_str[i].cmd) ) == 0 )
					{
						if ( f_enable > 0 ) {
							int32_t j = 0;
							do {
								g_qos_dbg |= 1 << j ;
							} while(!(dbg_enable_mask_str[i].flag & 1 << j++) ); 
						} else {
							g_qos_dbg &= ~dbg_enable_mask_str[i].flag;
						}
						p += strlen(dbg_enable_mask_str[i].cmd) + 1; //skip one blank
						len -= strlen(dbg_enable_mask_str[i].cmd) + 1; //skip one blank. len maybe negative now if there is no other parameters

						break;
					}
			} while ( i < NUM_ENTITY(dbg_enable_mask_str) );
		}
	}
	QOS_HAL_DEBUG_MSG(QOS_DEBUG_ERR,"QOS HAL Err Debug is enable\n");
	QOS_HAL_DEBUG_MSG(QOS_DEBUG_HIGH,"QOS HAL HIGH Debug is enable\n");
	QOS_HAL_DEBUG_MSG(QOS_DEBUG_TRACE,"QOS HAL TRACE Debug is enable\n");

	return count;
}

static struct file_operations g_proc_file_tc_map_seq_fops = {
	.owner      = THIS_MODULE,
	.open       = proc_read_tc_map_seq_open,
	.read       = seq_read,
	.write	= proc_write_tc_map,
	.llseek     = seq_lseek,
	.release    = seq_release,
};

static int proc_read_tc_map_seq_open(struct inode *inode, struct file *file)
{
    return single_open(file, proc_read_tc_map, NULL);
}

static int proc_read_tc_map(struct seq_file *seq, void *v)
{
        if (!capable(CAP_NET_ADMIN)) {
             printk ("Read Permission denied");
             return 0;
        }
	seq_printf(seq,"echo <interface> > /proc/ppa/qos_hal/tc_map\n");
        return 0;
}

static ssize_t proc_write_tc_map(struct file *file, const char __user *buf, size_t count, loff_t *data)
{

	uint32_t len;
	char str[50];
	char *p ;
	struct net_device *netdev ;
	uint32_t l_qos_dbg;

        if (!capable(CAP_NET_ADMIN)) {
           printk ("Write Permission denied");
           return 0;
        }  
	len = min(count, (size_t)sizeof(str) - 1);
	len -= ppa_copy_from_user(str, buf, len);
	while ( len && str[len - 1] <= ' ' )
		len--;
	str[len] = 0;
	for ( p = str; *p && *p <= ' '; p++, len-- );
	if ( !*p )
		return count;

	if ( strstr(p, "help"))
        {
                printk("echo <interface> > /proc/ppa/qos_hal/tc_map\n");
                return count;

        } else {
		if ( (netdev = dev_get_by_name(&init_net, p)) == NULL ) {
        	        printk("Invalid interface name\n");
                	return PPA_FAILURE;
        	}
		
		l_qos_dbg = g_qos_dbg;
		g_qos_dbg = QOS_DEBUG_TRACE; 
		qos_hal_get_queue_map(netdev);
		g_qos_dbg = l_qos_dbg;
		return count;
	}

}

static struct file_operations g_proc_file_queue_map_seq_fops = {
	.owner      = THIS_MODULE,
	.open       = proc_read_queue_map_seq_open,
	.read       = seq_read,
	.write	= proc_write_queue_map,
	.llseek     = seq_lseek,
	.release    = seq_release,
};

static int proc_read_queue_map_seq_open(struct inode *inode, struct file *file)
{
    return single_open(file, proc_read_queue_map, NULL);
}

static int proc_read_queue_map(struct seq_file *seq, void *v)
{
        if (!capable(CAP_NET_ADMIN)) {
              printk ("Read Permission denied");
              return 0;
        }
	seq_printf(seq,"echo <QId> > /proc/ppa/qos_hal/queue_map\n");
        return 0;
}

static ssize_t proc_write_queue_map(struct file *file, const char __user *buf, size_t count, loff_t *data)
{

	uint32_t len;
	uint32_t l_qos_dbg;
	char str[50];
	char *p ;
	int32_t queue_index;

        if (!capable(CAP_NET_ADMIN)) {
             printk ("Write Permission denied");
             return 0;
        }
	len = min(count, (size_t)sizeof(str) - 1);
	len -= ppa_copy_from_user(str, buf, len);
	while ( len && str[len - 1] <= ' ' )
		len--;
	str[len] = 0;
	for ( p = str; *p && *p <= ' '; p++, len-- );
	if ( !*p )
		return count;

	if ( strstr(p, "help"))
        {
                printk("echo <QId> > /proc/ppa/qos_hal/queue_map\n");
                return count;

        } else {
		queue_index = Atoi(p);		
		l_qos_dbg = g_qos_dbg;
		g_qos_dbg = QOS_DEBUG_TRACE; 
		qos_hal_get_detailed_queue_map(queue_index);
		g_qos_dbg = l_qos_dbg;
		return count;
	}

}

static struct file_operations g_proc_file_tc_group_seq_fops = {
	.owner      = THIS_MODULE,
	.open       = proc_read_tc_group_seq_open,
	.read       = seq_read,
	.write	= proc_write_tc_group,
	.llseek     = seq_lseek,
	.release    = seq_release,
};

static int proc_read_tc_group_seq_open(struct inode *inode, struct file *file)
{
	return single_open(file, proc_read_tc_group, NULL);
}

static int proc_read_tc_group(struct seq_file *seq, void *v)
{
        if (!capable(CAP_NET_ADMIN)) {
             printk ("Read Permission denied");
             return 0;
        }
	seq_printf(seq,"echo <tc group split Idx> > /proc/ppa/qos_hal/tc_group\n");
        return 0;
}

static ssize_t proc_write_tc_group(struct file *file, const char __user *buf, size_t count, loff_t *data)
{
	uint32_t len;
	char str[50];
	char *p ;

	if (!capable(CAP_NET_ADMIN)) {
		printk ("Write Permission denied");
		return 0;
	}
	len = min(count, (size_t)sizeof(str) - 1);
	len -= ppa_copy_from_user(str, buf, len);
	while ( len && str[len - 1] <= ' ' )
		len--;
	str[len] = 0;
	for ( p = str; *p && *p <= ' '; p++, len-- );
	if ( !*p )
		return count;

	if ( strstr(p, "help")) {
		printk("echo <tc group split Idx> > /proc/ppa/qos_hal/tc_group\n");
		return count;

	} else {
		high_prio_q_limit = Atoi(p);
		printk( "high_prio_q_limit = %d\n", high_prio_q_limit);		
		return count;
	}

}

int qos_hal_proc_create(void)
{
	QOS_HAL_DEBUG_MSG(QOS_DEBUG_TRACE,"QOS HAL Create Proc entries \n");
	if (!g_ppa_proc_dir_flag) {
		g_ppa_proc_dir = proc_mkdir("ppa", NULL);
		g_ppa_proc_dir_flag = 1;
	}
	g_ppa_qos_hal_proc_dir = proc_mkdir("qos_hal", g_ppa_proc_dir);
	g_ppa_qos_hal_proc_dir_flag = 1 ;

	proc_create("queue_info", 0600,
			g_ppa_qos_hal_proc_dir,
			&g_proc_file_queue_info_seq_fops);
	proc_create("dbg", 0600,
			g_ppa_qos_hal_proc_dir,
			&g_proc_file_dbg_seq_fops);
	proc_create("tc_map", 0600,
			g_ppa_qos_hal_proc_dir,
			&g_proc_file_tc_map_seq_fops);
	proc_create("queue_map", 0600,
			g_ppa_qos_hal_proc_dir,
			&g_proc_file_queue_map_seq_fops);
	proc_create("tc_group", 0600,
			g_ppa_qos_hal_proc_dir,
			&g_proc_file_tc_group_seq_fops);

	return 0;

}

void qos_hal_proc_destroy(void)
{
	remove_proc_entry("queue_info",
			g_ppa_qos_hal_proc_dir);

	remove_proc_entry("dbg",
			g_ppa_qos_hal_proc_dir);

	remove_proc_entry("tc_map",
			g_ppa_qos_hal_proc_dir);

	remove_proc_entry("queue_map",
			g_ppa_qos_hal_proc_dir);

	remove_proc_entry("tc_group",
			g_ppa_qos_hal_proc_dir);

	if (g_ppa_qos_hal_proc_dir_flag) {
		remove_proc_entry("qos_hal",
				g_ppa_proc_dir);
		g_ppa_qos_hal_proc_dir = NULL;
		g_ppa_qos_hal_proc_dir_flag = 0;
	}

}
