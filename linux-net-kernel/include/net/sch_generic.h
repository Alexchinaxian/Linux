#ifndef __NET_SCHED_GENERIC_H
#define __NET_SCHED_GENERIC_H

#include <linux/netdevice.h>
#include <linux/types.h>
#include <linux/rcupdate.h>
#include <linux/module.h>
#include <linux/pkt_sched.h>
#include <linux/pkt_cls.h>
#include <net/gen_stats.h>
#include <net/rtnetlink.h>

struct Qdisc_ops;
struct qdisc_walker;
struct tcf_walker;
struct module;

// 流控速率控制表结构 （一）空闲资源流控算法
struct qdisc_rate_table { //所有的都添加到qdisc_rtab_list
	struct tc_ratespec rate;
	u32		data[256];//参考应用层tc_calc_rtable   //这里得到的就是2047个字节所消耗的空闲资源。
	struct qdisc_rate_table *next;
	int		refcnt;
};

//qdisc->state
enum qdisc_state_t {
	__QDISC_STATE_RUNNING,//在__qdisc_run中清除置位。 __QDISC_STATE_RUNNING标志用于保证一个流控对象不会同时被多个例程运行。
	__QDISC_STATE_SCHED,
	__QDISC_STATE_DEACTIVATED,
};

struct qdisc_size_table {
	struct list_head	list;
	struct tc_sizespec	szopts;
	int			refcnt;
	u16			data[];
};
/*
tc可以使用以下命令对QDisc、类和过滤器进行操作：
add，在一个节点里加入一个QDisc、类或者过滤器。添加时，需要传递一个祖先作为参数，传递参数时既可以使用ID也可以直接传递设备的根。如果要建立一个QDisc或者过滤器，可以使用句柄(handle)来命名；如果要建立一个类，可以使用类识别符(classid)来命名。
remove，删除有某个句柄(handle)指定的QDisc，根QDisc(root)也可以删除。被删除QDisc上的所有子类以及附属于各个类的过滤器都会被自动删除。
change，以替代的方式修改某些条目。除了句柄(handle)和祖先不能修改以外，change命令的语法和add命令相同。换句话说，change命令不能一定节点的位置。
replace，对一个现有节点进行近于原子操作的删除／添加。如果节点不存在，这个命令就会建立节点。
link，只适用于DQisc，替代一个现有的节点。
tc qdisc [ add | change | replace | link ] dev DEV [ parent qdisc-id | root ] [ handle qdisc-id ] qdisc [ qdisc specific parameters ]
tc class [ add | change | replace ] dev DEV parent qdisc-id [ classid class-id ] qdisc [ qdisc specific parameters ]
tc filter [ add | change | replace ] dev DEV [ parent qdisc-id | root ] protocol protocol prio priority filtertype [ filtertype specific parameters ] flowid flow-id
tc [-s | -d ] qdisc show [ dev DEV ]
tc [-s | -d ] class show dev DEV tc filter show dev DEV

tc qdisc show dev eth0
tc class show dev eth0
*/
//tc qdisc add dev eth0 parent 22:4 handle 33中的22:4中的4实际上对应的是Qdisc私有数据部分分类信息中的3,parent 22:x中的x是从1开始排，但是对应到分类数组中具体的类的时候，是从0开始排，所以要减1，例如prio参考prio_get
//前言linux内核中提供了流量控制的相关处理功能，相关代码在net/sched目录下；而应用层上的控制是通过iproute2软件包中的tc来实现，tc和sched的关系就好象iptables和netfilter的关系一样，一个是用户层接口，一个是具体实现，关于tc的使用方法可详将Linux Advanced Routing HOWTO，本文主要分析内核中的具体实现。
//该结构中文称呼为:流控对象(队列规定)
//Qdisc开辟空间qdisc_alloc后面跟的是priv_size数据，见pfifo_qdisc_ops prio_qdisc_ops tbf_qdisc_ops sfq_qdisc_ops ingress_qdisc_ops(入口流控对象 ) 等中的priv_size， 图形化参考TC流量控制实现分析（初步） 
/*
队列规程分为无类队列规程和有类对了规程，有类的队列规程可以创建多个子队列规程(可以是分类的也可以是无类的队列规程)，如果只创建一个无类队列规程就相当于一个叶子规程
，SKB直接入队到该队列规程的skb队列中。如果是创建一个分类的队列规程，则第一个创建的队列规程就是跟，下面可以包括多个子队列规程，但所以分类队列规程必须有对应
的叶子无类队列规程，因为分类队列规程里面是没有skb队列的。
当一个SKB到分类队列规程的跟的时候，该选择走那条子队列规程入队呢? 这就是过滤器的作用，过滤器可以通过IP MASK等信息来确定走那个子队列规程分支。如果没有设置
过滤器，则一般根据skb->priority来确定走那个分支。
tc qdisc add dev eth0 root handle 1: htb 创建跟队列规程 (在创建跟分类规程的时候，一般默认是会有自队列规程的，例如pfifo无类规程)
tc class add dev eth0 parent 1: classid 1:2 htb xxxx  在1:队列规程下面的第1:2分支上，用htb创建一个子有类队列规程htb。并且在xxx中指定htb的参数信息
tc class add dev eth0 parent 1: classid 1:1 htb xxxx  在1:队列规程下面的第1:1分支上，用htb创建一个子有类队列规程htb。并且在xxx中指定htb的参数信息
tc filter add dev eth0 protocol ip parent 1: prio 2 u32 match ip dst 4.3.2.1/32 flowid 1:2 如果收到的是ip地址为4.3.2.1的SKB包，则走子队列规程1:2入队，而不是走1:1分子入队
*/ //最好的源码理解参考<<linux内核中流量控制>>
struct Qdisc { /* 参考 TC流量控制实现分析（初步）*/ //prio_sched_data中的queues指向该Qdisc              #注意命令中的ID(parent 1:2 xxx flowid 3:3)参数都被理解为16进制的数
//qdisc_alloc分配中在struct Qdisc结构后面的私有数据为pfifo_qdisc_ops prio_qdisc_ops tbf_qdisc_ops sfq_qdisc_ops ingress_qdisc_ops中的priv_size部分
    //enqueue和dequeue的赋值见qdisc_alloc
	int 			(*enqueue)(struct sk_buff *skb, struct Qdisc *dev); /* 入队接口 */
	struct sk_buff *	(*dequeue)(struct Qdisc *dev);  /* 出对接口 */
	unsigned		flags; //排队规则标志，取值为下面这几种宏定义  TCQ_F_THROTTLED
#define TCQ_F_BUILTIN		1 //表示排队规则是空的排队规则，在删除释放时不需要做过多的资源释放
#define TCQ_F_THROTTLED		2 //标识排队规则正处于由于限制而延时出队的状态中 
#define TCQ_F_INGRESS		4 //表示排队规则为输入排队规则
#define TCQ_F_CAN_BYPASS	8
#define TCQ_F_MQROOT		16
#define TCQ_F_WARN_NONWC	(1 << 16)// 作为已经打印了警告信息的标志
    /*
    由于排队规则的内存需要32字节对齐，而通过动态分配得到的内存起始地址不一定是32字节
    对齐，因此需要通过填充将队列规则对齐到32字节处。
    */
	int			padded;

	/*pfifo_qdisc_ops tbf_qdisc_ops sfq_qdisc_ops这几个都为出口，ingress_qdisc_ops为入口 */
	struct Qdisc_ops	*ops;//prio队列规则ops为pfifo_qdisc_ops，其他还有prio_qdisc_ops tbf_qdisc_ops sfq_qdisc_ops ingress_qdisc_ops(入口流控对象 ) 等， 
	struct qdisc_size_table	*stab;
	struct list_head	list;//连接到所配置的网络设备上

	/*排队规则实例的标识分为主编号部分和副编号部分，其中主编号部分由用户分配，范围从
	0X0001到0X7FFFF，如果用户指定主编号为0，那么内核讲在0X8000到0XFFFF之间分配一个主编号
	标识在单个网络设备是唯一的，但在多个网络设备之间可以由重复*/
	u32			handle; //本Qdisc的句柄，tc qdisc add dev eth0 root handle 22中的22
	u32			parent;//父队列规则的句柄值  tc qdisc add dev eth0 parent 22:4 handle 33 中handle为33 parent为22
	atomic_t		refcnt;//引用计数
	struct gnet_stats_rate_est	rate_est;//队列当前的速率，包括以字节和报文数为单位两种

    /*用于实现更复杂的流量控制机制，很少排队规则会实现此接口。当一个外部队列向内部队列
    传递报文时，可能出现报文必须被丢弃的情况，如当没有可用缓冲区时。如果排队规则实现了该回调
    函数，那么这时就可以被内部排队规则调用*/
	int			(*reshape_fail)(struct sk_buff *skb,
					struct Qdisc *q);

	void			*u32_node;//指向tc_u_common，见u32_init  指向的是指定队列规程的第一个u32过滤器

	/* This field is deprecated, but it is still used by CBQ
	 * and it will live until better solution will be invented.
	 */
	struct Qdisc		*__parent;
	struct netdev_queue	*dev_queue;
	struct Qdisc		*next_sched;

	struct sk_buff		*gso_skb;
	/*
	 * For performance sake on SMP, we put highly modified fields at the end
	 */
	unsigned long		state;
	struct sk_buff_head	q; //SKB就是添加到该队列中的  pfifo是入队的时候直接加入该skb链表，所以是典型的先进先出
	struct gnet_stats_basic_packed bstats;//记录入队报文总字节数和入队报文总数
	struct gnet_stats_queue	qstats;//记录队列相关统计数据
	struct rcu_head     rcu_head;//通过本字节在没有对象再使用该排队规则时释放该排队规则
};

/*
//分类的队列规定，例如prio cbq htb，这些队列规则Qdisc都会对应一个类接口，如果是无类的队列规定，则没有该类操作接口
//prio对应prio_class_ops htb对应htb_class_ops cbq对应cbq_class_ops等等

//分类队列规程Qdisc ops中的Qdisc_class_ops主要是在创建子Qdisc的时候，按照parent 22:4中的22:4对父Qdisc进行分类，从而通过22:4作为参数，
//选出该子Qdisc应该加到那个分类Qdisc后面。可以参考prio_qdisc_ops中的prio_get和prio_graft，就很好明白了
*/ //创建子队列规则或者class的时候，该结构的作用就是通过parent 22:8中的8从prio_get(以prio分类队列规程为例)选出的prize_size私有数据部分数组中的那一个具体信息，
struct Qdisc_class_ops { //主要在qdisc_graft执行下面的相关函数       可以参考prio_qdisc_ops，以prio为例        tc_ctl_tclass
	/* Child qdisc manipulation */
	struct netdev_queue *	(*select_queue)(struct Qdisc *, struct tcmsg *);

	//函数qdisc_graft中调用
	int			(*graft)(struct Qdisc *, unsigned long cl,
					struct Qdisc *, struct Qdisc **);?//用于将一个队列规则Qdisc绑定到一个类，并返回先前绑定到这个类的队列规则
    //获取当前绑定到所在类的队列规则
	struct Qdisc *		(*leaf)(struct Qdisc *, unsigned long cl);

	//用于相应队列长度变化
	void			(*qlen_notify)(struct Qdisc *, unsigned long);

	/* Class manipulation routines */
    //根据给点的类描述符从排队规则中查找对应的类，并引用该类，该类的引用计数增。
    //表示使用队列规程里面的第几个分类信息，一个分类队列规程里面都会有好几个分类信息，通过classid从其中选一个，例如prio分类规程通过prio_get获取分类频道中的第几个频道
    //根据该函数来确定使用该Qdisc的那个类，判断条件为tc qdisc add dev eth0 parent 22:4 handle 33中的22:4,以prio分类队列规程为例，见prio_get
	unsigned long		(*get)(struct Qdisc *, u32 classid); //通过qdisc_graft调用
    //递减指定类的引用计数，如果引用计数为0，则删除释放此类。
	void			(*put)(struct Qdisc *, unsigned long); //函数qdisc_graft中调用
    //用于变更指定类的参数，如果该类不存在则新建之。
	int			(*change)(struct Qdisc *, u32, u32,
					struct nlattr **, unsigned long *);
    //用于删除并释放指定的类。首先会递减该类的引用计数，如果引用计数递减后为0，删除释放之。
	int			(*delete)(struct Qdisc *, unsigned long);
    //遍历一个排队规则的所有类，取回实现了回调函数类的配置数据及统计信息
	void			(*walk)(struct Qdisc *, struct qdisc_walker * arg);

	/* Filter manipulation */
	//获取绑定到该类的过滤器所在链表的首节点
	struct tcf_proto **	(*tcf_chain)(struct Qdisc *, unsigned long);

    //在一个过滤器正准备绑定到指定的类之前被调用，通过类标识符获取类，首先递增引用计数，然后是一些其他的检查
	unsigned long		(*bind_tcf)(struct Qdisc *, unsigned long,
					u32 classid); //见tcf_bind_filter

    //在过滤器完成绑定到指定的类后被调用，递减类引用计数
    void			(*unbind_tcf)(struct Qdisc *, unsigned long);

	/* rtnetlink specific */
	int			(*dump)(struct Qdisc *, unsigned long,
					struct sk_buff *skb, struct tcmsg*);
	int			(*dump_stats)(struct Qdisc *, unsigned long,
					struct gnet_dump *);
};

//所有的Qdisc_ops结构通过register_qdisc添加到qdisc_base链表中
//Qdisc中的ops指向这里              /*pfifo_fast_ops pfifo_qdisc_ops tbf_qdisc_ops sfq_qdisc_ops prio_class_ops这几个都为出口，ingress_qdisc_ops为入口 */
struct Qdisc_ops { //prio队列规则ops为pfifo_qdisc_ops，其他还有tbf_qdisc_ops sfq_qdisc_ops等， 
	struct Qdisc_ops	*next;//指向下一个Qdisc_ops
    //所有规则提供的类操作接口。
	const struct Qdisc_class_ops	*cl_ops; //无类的队列pfifo bfifo规则没有class子类ops，
	char			id[IFNAMSIZ]; //排队规则名
	//附属在排队规则上的私有信息块大小，该信息块通常与排队规则一起分配内存，紧跟在排队
	//规则后面，可用qdisc_priv获取， 
	int			priv_size; //本类对象私有数据大小 Qdisc_alloc开辟Qdisc空间的时候会多开辟priv_size空间

//enqueue返回值NET_XMIT_SUCCESS等
	int 			(*enqueue)(struct sk_buff *, struct Qdisc *); //调用地方qdisc_enqueue   //dev_xmit_queue一直下去调用
//将先前出队的报文重新排入到队列中的函数。不同于enqueue的是，重新入队的报文需要被放置在她
//出队前在排队规则队列中所处的位置上。该接口通常用于报文要发送出去而有dequeue出队后，因某个不可预见的原因最终未能发送的情况。
	struct sk_buff *	(*dequeue)(struct Qdisc *);//dequeue_skb中调用
	struct sk_buff *	(*peek)(struct Qdisc *);

	//从队列移除并丢弃一个报文的函数
	unsigned int		(*drop)(struct Qdisc *);

    //在qdisc_create中调用
	int			(*init)(struct Qdisc *, struct nlattr *arg); //对象初始化函数  //分类的队列规则在初始化的时候会默认指向noop_qdisc，例如prio_qdisc_ops中的init
	void			(*reset)(struct Qdisc *); //复位为初始状态，删除定时器 释放空间等
	void			(*destroy)(struct Qdisc *);
	int			(*change)(struct Qdisc *, struct nlattr *arg); //更高Qdisc参数
	void			(*attach)(struct Qdisc *);


	int			(*dump)(struct Qdisc *, struct sk_buff *);
    //用于输出排队规则的配置参数和统计数据的函数。
   	int			(*dump_stats)(struct Qdisc *, struct gnet_dump *);

	struct module		*owner;
};

//通过解析SKB中的内容来匹配过滤器tc filter，匹配结果存到该结构中。也就是直接获取该过滤器所在class的(tc add class的时候创建的class树节点)htb_class
struct tcf_result {
	unsigned long	class; //这个实际上是一个指针地址，指向的是tc filter add xxxx flowid 22:4对应的htb_class结构，见tcf_bind_filter
	u32		classid;//见u32_set_parms，该值为//tc filter add dev eth0 protocol ip parent 22: prio 2 u32 match ip dst 4.3.2.1/32 flowid 22:4中的flowid，表示该过滤器属于那个队列规程树节点
};

//tcf_proto中的ops，所有的tcf_proto_ops通过tcf_proto_base连接在一起，见register_tcf_proto_ops
//主要有cls_u32_ops cls_basic_ops  cls_cgroup_ops  cls_flow_ops cls_route4_ops RSVP_OPS
struct tcf_proto_ops {
	struct tcf_proto_ops	*next; //用来将已注册过滤器连接到tcf_proto_base链表上的指针
	char			kind[IFNAMSIZ];//过滤器类名 

	int			(*classify)(struct sk_buff*, struct tcf_proto*,
					struct tcf_result *); //分类函数，结果保存在tcf_result中，返回值有TC_POLICE_OK等
	int			(*init)(struct tcf_proto*); //tc_ctl_tclass中调用

    //释放并删除过滤器函数
	void			(*destroy)(struct tcf_proto*);

    //讲一个过滤器元素的句柄映射到一个内部过滤器标识符，实际上是过滤器实例指针，并将其返回
	unsigned long		(*get)(struct tcf_proto*, u32 handle); //获取对应的过滤器
    //释放对get得到的过滤器的引用
	void			(*put)(struct tcf_proto*, unsigned long);
	//用于配置一个新过滤器或是变更一个已存在的过滤器配置。
	int			(*change)(struct tcf_proto*, unsigned long,
					u32 handle, struct nlattr **,
					unsigned long *);
	int			(*delete)(struct tcf_proto*, unsigned long);
    //遍历所有的元素并且调用回调函数取得配置数据和统计数据
	void			(*walk)(struct tcf_proto*, struct tcf_walker *arg);

	/* rtnetlink specific */  //用于输出所有的元素并且调用回调函数取得配置数据和统计数据
	int			(*dump)(struct tcf_proto*, unsigned long,
					struct sk_buff *skb, struct tcmsg*);

	struct module		*owner;
};
/* 优先级队列规定的band为16个,参考TC流量控制实现分析(初步)-图3  建立”prio”类型的根流控对象_2 */   //详细理解也可以参考<<LINUX高级路由和流量控制>>
//tc filter add dev eth0 protocol ip parent 22: prio 2 u32 match ip dst 4.3.2.1/32 flowid 22:4
/*现在数据包的入队流程如下：
1.      根对象的过滤器链非空，遍历根对象的过滤器链，遇到第一个匹配的过滤器就返回，并根据返回的结果选择子类。
2.      每个过滤器都调用相应的分类函数，并根据过滤器的私有数据来匹配数据包。
*/
//tc filter u32过滤器的结构    过滤器创建在tc_ctl_tfilter中，并在该函数中初始化
struct tcf_proto { //该结构是加入到prio_sched_data中的filter_list链表中  每调用一次tc filter add就会创建一个tcf_proto结构，调用多个tc filter add的时候就创建多个tcf_proto结构，通过next连接
	/* Fast access part */ //tcf一般表示tcf_proto过滤器的简写
	struct tcf_proto	*next;
	void			*root; //如果为u32类型，指向过滤器跟tc_u_hnode， 见u32_init，该过滤器下面的所有tc_u_common节点都添加到该tc_u_hnode跟上
	int			(*classify)(struct sk_buff*, struct tcf_proto*,
					struct tcf_result *); //分类函数，结果保存在tcf_result中。通过SKB中的内容，来匹配这个过滤器，结果返回给tcf_result，见tc_classify_compat
	__be16			protocol; //协议号，//tc filter add dev eth0 protocol ip中protocol ip对应的是数字ETH_P_IP

	/* All the rest */
	u32			prio; //根据这个优先级加入到prio_sched_data中的filter_list链表中。tc filter add dev eth0 protocol ip parent 22: prio 2为2
	u32			classid; //指定父Qdisc中的子类位置=22:4
	struct Qdisc		*q; //父Qdisc,就是该过滤器所处的队列规则节点的上级父Qdisc
	void			*data; //如果最后创建的u32类型过滤器节点tc_u_common，见u32_init
	struct tcf_proto_ops	*ops; //cls_u32_ops //主要有cls_u32_ops cls_basic_ops  cls_cgroup_ops  cls_flow_ops cls_route4_ops RSVP_OPS
};

struct qdisc_skb_cb {
	unsigned int		pkt_len;//见qdisc_enqueue_root，当入队的时候，该值为SKB->len
	char			data[];
};

static inline int qdisc_qlen(struct Qdisc *q)
{
	return q->q.qlen;
}

static inline struct qdisc_skb_cb *qdisc_skb_cb(struct sk_buff *skb)
{
	return (struct qdisc_skb_cb *)skb->cb;
}

static inline spinlock_t *qdisc_lock(struct Qdisc *qdisc)
{
	return &qdisc->q.lock;
}

static inline struct Qdisc *qdisc_root(struct Qdisc *qdisc)
{
	return qdisc->dev_queue->qdisc;
}

static inline struct Qdisc *qdisc_root_sleeping(struct Qdisc *qdisc)
{
	return qdisc->dev_queue->qdisc_sleeping;
}

/* The qdisc root lock is a mechanism by which to top level
 * of a qdisc tree can be locked from any qdisc node in the
 * forest.  This allows changing the configuration of some
 * aspect of the qdisc tree while blocking out asynchronous
 * qdisc access in the packet processing paths.
 *
 * It is only legal to do this when the root will not change
 * on us.  Otherwise we'll potentially lock the wrong qdisc
 * root.  This is enforced by holding the RTNL semaphore, which
 * all users of this lock accessor must do.
 */
static inline spinlock_t *qdisc_root_lock(struct Qdisc *qdisc)
{
	struct Qdisc *root = qdisc_root(qdisc);

	ASSERT_RTNL();
	return qdisc_lock(root);
}

static inline spinlock_t *qdisc_root_sleeping_lock(struct Qdisc *qdisc)
{
	struct Qdisc *root = qdisc_root_sleeping(qdisc);

	ASSERT_RTNL();
	return qdisc_lock(root);
}

static inline struct net_device *qdisc_dev(struct Qdisc *qdisc)
{
	return qdisc->dev_queue->dev;
}

static inline void sch_tree_lock(struct Qdisc *q)
{
	spin_lock_bh(qdisc_root_sleeping_lock(q));
}

static inline void sch_tree_unlock(struct Qdisc *q)
{
	spin_unlock_bh(qdisc_root_sleeping_lock(q));
}

#define tcf_tree_lock(tp)	sch_tree_lock((tp)->q)
#define tcf_tree_unlock(tp)	sch_tree_unlock((tp)->q)

extern struct Qdisc noop_qdisc;
extern struct Qdisc_ops noop_qdisc_ops;
extern struct Qdisc_ops pfifo_fast_ops;
extern struct Qdisc_ops mq_qdisc_ops;

//该结构为htb_class -> common
struct Qdisc_class_common {//存放在Qdisc_class_hash中, 变量class在qdisc_class_find
	u32			classid;// 类别ID值, 高16位用于区分不同的HTB流控, 低16位为区分同一HTB流控中的不同类别
	struct hlist_node	hnode; //通过这个hnode最终把htb_class加入到htb_sched->clhash中，见htb_change_class -> qdisc_class_hash_insert
};

//该结构为htb私有数据htb_sched中的clhash，用来存储所有tc class add创建的htb_class
struct Qdisc_class_hash { //hash过程见qdisc_class_hash_grow
	struct hlist_head	*hash;//该链表中存放的是Qdisc_class_common,该hash表空间在qdisc_class_hash_init创建     qdisc_class_find
	unsigned int		hashsize; //默认初始值见qdisc_class_hash_init。如果hash节点数hashelems超过设置的hashsize的0.75，则从新hash，hashsize扩大到之前hashsize两倍，见qdisc_class_hash_grow
	unsigned int		hashmask;  //qdisc_class_hash_init
	unsigned int		hashelems; //实际的hash class节点数 //hashelems和hashsize关系见qdisc_class_hash_grow
};

static inline unsigned int qdisc_class_hash(u32 id, u32 mask)
{
	id ^= id >> 8;
	id ^= id >> 4;
	return id & mask;
}

//查找
static inline struct Qdisc_class_common *
qdisc_class_find(struct Qdisc_class_hash *hash, u32 id)
{
	struct Qdisc_class_common *cl;
	struct hlist_node *n;
	unsigned int h;

	h = qdisc_class_hash(id, hash->hashmask);
	hlist_for_each_entry(cl, n, &hash->hash[h], hnode) {// 根据句柄计算哈希值, 然后遍历该哈希链表
		if (cl->classid == id)
			return cl;
	}
	return NULL;
}

extern int qdisc_class_hash_init(struct Qdisc_class_hash *);
extern void qdisc_class_hash_insert(struct Qdisc_class_hash *, struct Qdisc_class_common *);
extern void qdisc_class_hash_remove(struct Qdisc_class_hash *, struct Qdisc_class_common *);
extern void qdisc_class_hash_grow(struct Qdisc *, struct Qdisc_class_hash *);
extern void qdisc_class_hash_destroy(struct Qdisc_class_hash *);

extern void dev_init_scheduler(struct net_device *dev);
extern void dev_shutdown(struct net_device *dev);
extern void dev_activate(struct net_device *dev);
extern void dev_deactivate(struct net_device *dev);
extern struct Qdisc *dev_graft_qdisc(struct netdev_queue *dev_queue,
				     struct Qdisc *qdisc);
extern void qdisc_reset(struct Qdisc *qdisc);
extern void qdisc_destroy(struct Qdisc *qdisc);
extern void qdisc_tree_decrease_qlen(struct Qdisc *qdisc, unsigned int n);
extern struct Qdisc *qdisc_alloc(struct netdev_queue *dev_queue,
				 struct Qdisc_ops *ops);
extern struct Qdisc *qdisc_create_dflt(struct net_device *dev,
				       struct netdev_queue *dev_queue,
				       struct Qdisc_ops *ops, u32 parentid);
extern void qdisc_calculate_pkt_len(struct sk_buff *skb,
				   struct qdisc_size_table *stab);
extern void tcf_destroy(struct tcf_proto *tp);
extern void tcf_destroy_chain(struct tcf_proto **fl);

/* Reset all TX qdiscs greater then index of a device.  */
static inline void qdisc_reset_all_tx_gt(struct net_device *dev, unsigned int i)
{
	struct Qdisc *qdisc;

	for (; i < dev->num_tx_queues; i++) {
		qdisc = netdev_get_tx_queue(dev, i)->qdisc;
		if (qdisc) {
			spin_lock_bh(qdisc_lock(qdisc));
			qdisc_reset(qdisc);
			spin_unlock_bh(qdisc_lock(qdisc));
		}
	}
}

static inline void qdisc_reset_all_tx(struct net_device *dev)
{
	qdisc_reset_all_tx_gt(dev, 0);
}

/* Are all TX queues of the device empty?  */
static inline bool qdisc_all_tx_empty(const struct net_device *dev)
{
	unsigned int i;
	for (i = 0; i < dev->num_tx_queues; i++) {
		struct netdev_queue *txq = netdev_get_tx_queue(dev, i);
		const struct Qdisc *q = txq->qdisc;

		if (q->q.qlen)
			return false;
	}
	return true;
}

/* Are any of the TX qdiscs changing?  */
static inline bool qdisc_tx_changing(struct net_device *dev)
{
	unsigned int i;
	for (i = 0; i < dev->num_tx_queues; i++) {
		struct netdev_queue *txq = netdev_get_tx_queue(dev, i);
		if (txq->qdisc != txq->qdisc_sleeping)
			return true;
	}
	return false;
}

/* Is the device using the noop qdisc on all queues?  */
static inline bool qdisc_tx_is_noop(const struct net_device *dev)
{
	unsigned int i;
	for (i = 0; i < dev->num_tx_queues; i++) {
		struct netdev_queue *txq = netdev_get_tx_queue(dev, i);
		if (txq->qdisc != &noop_qdisc)
			return false;
	}
	return true;
}

static inline unsigned int qdisc_pkt_len(struct sk_buff *skb)
{
	return qdisc_skb_cb(skb)->pkt_len;
}

/* additional qdisc xmit flags (NET_XMIT_MASK in linux/netdevice.h) */
enum net_xmit_qdisc_t {
	__NET_XMIT_STOLEN = 0x00010000,
	__NET_XMIT_BYPASS = 0x00020000,
};

#ifdef CONFIG_NET_CLS_ACT
#define net_xmit_drop_count(e)	((e) & __NET_XMIT_STOLEN ? 0 : 1)
#else
#define net_xmit_drop_count(e)	(1)
#endif


static inline int qdisc_enqueue(struct sk_buff *skb, struct Qdisc *sch)
{
#ifdef CONFIG_NET_SCHED
	if (sch->stab)
		qdisc_calculate_pkt_len(skb, sch->stab);
#endif
	return sch->enqueue(skb, sch);///*prio_qdisc_ops pfifo_qdisc_ops tbf_qdisc_ops sfq_qdisc_ops这几个都为出口，ingress_qdisc_ops为入口 */
}

//ingress通过ing_filter入队
static inline int qdisc_enqueue_root(struct sk_buff *skb, struct Qdisc *sch) //sch dev设备的qdisc
{
	qdisc_skb_cb(skb)->pkt_len = skb->len;
	return qdisc_enqueue(skb, sch) & NET_XMIT_MASK;
}

static inline void __qdisc_update_bstats(struct Qdisc *sch, unsigned int len)
{
	sch->bstats.bytes += len;
	sch->bstats.packets++;
}

static inline int __qdisc_enqueue_tail(struct sk_buff *skb, struct Qdisc *sch,
				       struct sk_buff_head *list)
{
	__skb_queue_tail(list, skb);
	sch->qstats.backlog += qdisc_pkt_len(skb);
	__qdisc_update_bstats(sch, qdisc_pkt_len(skb));

	return NET_XMIT_SUCCESS;
}

static inline int qdisc_enqueue_tail(struct sk_buff *skb, struct Qdisc *sch)
{
	return __qdisc_enqueue_tail(skb, sch, &sch->q);
}

static inline struct sk_buff *__qdisc_dequeue_head(struct Qdisc *sch,
						   struct sk_buff_head *list)
{
	struct sk_buff *skb = __skb_dequeue(list);

	if (likely(skb != NULL))
		sch->qstats.backlog -= qdisc_pkt_len(skb);

	return skb;
}

//__qdisc_run -> qdisc_restart -> dequeue_skb -> prio_dequeue(这里面有个递归调用过程) -> qdisc_dequeue_head
static inline struct sk_buff *qdisc_dequeue_head(struct Qdisc *sch)
{
	return __qdisc_dequeue_head(sch, &sch->q);
}

static inline unsigned int __qdisc_queue_drop_head(struct Qdisc *sch,
					      struct sk_buff_head *list)
{
	struct sk_buff *skb = __qdisc_dequeue_head(sch, list);

	if (likely(skb != NULL)) {
		unsigned int len = qdisc_pkt_len(skb);
		kfree_skb(skb);
		return len;
	}

	return 0;
}

static inline unsigned int qdisc_queue_drop_head(struct Qdisc *sch)
{
	return __qdisc_queue_drop_head(sch, &sch->q);
}

static inline struct sk_buff *__qdisc_dequeue_tail(struct Qdisc *sch,
						   struct sk_buff_head *list)
{
	struct sk_buff *skb = __skb_dequeue_tail(list);

	if (likely(skb != NULL))
		sch->qstats.backlog -= qdisc_pkt_len(skb);

	return skb;
}

static inline struct sk_buff *qdisc_dequeue_tail(struct Qdisc *sch)
{
	return __qdisc_dequeue_tail(sch, &sch->q);
}

static inline struct sk_buff *qdisc_peek_head(struct Qdisc *sch)
{
	return skb_peek(&sch->q);
}

/* generic pseudo peek method for non-work-conserving qdisc */
static inline struct sk_buff *qdisc_peek_dequeued(struct Qdisc *sch)
{
	/* we can reuse ->gso_skb because peek isn't called for root qdiscs */
	if (!sch->gso_skb) {
		sch->gso_skb = sch->dequeue(sch);
		if (sch->gso_skb)
			/* it's still part of the queue */
			sch->q.qlen++;
	}

	return sch->gso_skb;
}

/* use instead of qdisc->dequeue() for all qdiscs queried with ->peek() */
static inline struct sk_buff *qdisc_dequeue_peeked(struct Qdisc *sch)
{
	struct sk_buff *skb = sch->gso_skb;

	if (skb) {
		sch->gso_skb = NULL;
		sch->q.qlen--;
	} else {
		skb = sch->dequeue(sch);
	}

	return skb;
}

static inline void __qdisc_reset_queue(struct Qdisc *sch,
				       struct sk_buff_head *list)
{
	/*
	 * We do not know the backlog in bytes of this list, it
	 * is up to the caller to correct it
	 */
	__skb_queue_purge(list);
}

static inline void qdisc_reset_queue(struct Qdisc *sch)
{
	__qdisc_reset_queue(sch, &sch->q);
	sch->qstats.backlog = 0;
}

static inline unsigned int __qdisc_queue_drop(struct Qdisc *sch,
					      struct sk_buff_head *list)
{
	struct sk_buff *skb = __qdisc_dequeue_tail(sch, list);

	if (likely(skb != NULL)) {
		unsigned int len = qdisc_pkt_len(skb);
		kfree_skb(skb);
		return len;
	}

	return 0;
}

//丢弃qdisc排队规程skb队列上的数据
static inline unsigned int qdisc_queue_drop(struct Qdisc *sch)
{
	return __qdisc_queue_drop(sch, &sch->q);
}

static inline int qdisc_drop(struct sk_buff *skb, struct Qdisc *sch)
{
	kfree_skb(skb);
	sch->qstats.drops++;

	return NET_XMIT_DROP;
}

static inline int qdisc_reshape_fail(struct sk_buff *skb, struct Qdisc *sch)
{
	sch->qstats.drops++;

#ifdef CONFIG_NET_CLS_ACT
	if (sch->reshape_fail == NULL || sch->reshape_fail(skb, sch))
		goto drop;

	return NET_XMIT_SUCCESS;

drop:
#endif
	kfree_skb(skb);
	return NET_XMIT_DROP;
}

/*
瀚海书香 说

那个大虾给讲解一下结构
qdisc_rate_table{
  struct tc_ratespec rate;
  u32 data[256];
  struct qdisc_rate_table *next;
  int refcnt;
}
最近在看流控源码，看到tbf时，总是看不懂这个地方。
还有就是
    qdisc_l2t()是怎么算的啊。怎么就是看不懂呢？
希望懂的大虾给讲解一下，小弟先谢谢了
emmoblin 说

呵呵，这个我看了相当一段时间才看明白。

这个结构主要是用来在内核计算令牌时用的。
我能理解，不过我有点说不明白。
内核的最小调度单位是一个tick。所以内核要把世界时间转化为内核的tick时间。
你在好好体会一下，就相当于是一个汇率，世界时间的100ms，转换到内核tick时间是要成一个系数的。



（一）空闲资源流控算法
算法概述：单位时间内产生的空闲资源一定，每发送一个字节都要消耗相应大小的空闲资源，当空闲资源不足时停止发送数据包，设定的流速越大，
发送一个字节所消耗的空闲资源就越小，通过设置发送一个字节所消耗的空闲资源来进行流速控制。

基本概念:

1. 空闲资源：发送一个数据包都必须消耗空闲资源，如果某个对象的空闲资源为0，将无法发送数据包，只要空闲资源足够多就可以发送数据包。
(TC用户空间规则定每秒产生的空闲资源是TIME_UNITS_PER_SEC       1000000，而TC内核根据空闲时间来计算空闲资源。)

2.空闲时间：假设对象最近一次发送数据包的时刻是T1，系统当前的时刻是T2，则空闲时间tk = T1 – T2。

2. 流速rate：每秒允许发送的的字节个数。
3. 空闲资源积累量：以空闲时间为参数根据一定的算法得到的值（比如可以将空闲时间乘上一个正数），但是要保证空闲时间越大，对应的空闲资源的积累量必定要越大。
4. 空闲资源剩余量：最近一次发送数据包以后，空闲资源的剩余量。
5. 当前可用空闲资源：以空闲资源的剩余量和空闲资源的积累量为参数根据一定的算法得到的值（比如可以 = 1/6空闲资源的剩余量 + (1 – 1/6)空闲资源的积累），
但是要保证当前可用空闲资源都是空闲资源剩余量和空闲资源积累量的递增函数。

为了更好的理解空闲资源流控算法，需要引入流速概念的第二种描述，也就是，使用空闲资源来描述流速的概念。
 

6.流速kc(用空闲资源描述)：假设每秒产生的空闲资源是TIME_UNITS_PER_SEC，流速rate(每秒允许发送的数据量是rate个字节)，则发送一个字节的流量需要消耗的
空闲资源是kc = TIME_UNITS_PER_SEC/rate
这里的kc就是新引入的流速描述方法。流速rate越大，kc就越小。

如果要发送size字节的数据包需要消耗size*(TIME_UNITS_PER_SEC/rate)的空闲资源。

只要空闲资源足够多，就可以发送数据包，每发送一个数据包，空闲资源减去相应的消耗量。

只要空闲时间一直累积，空闲资源将会变得很大，这时就失去了调控流速的意义，所以引入最大空闲资源，以使空闲资源不会太大。

调控流速的过程：
假设只要空闲资源非零，就试图发送一个长度是L的数据包，流速是kc。
1.      初始时刻空闲资源和空闲时间都为0，显然不允许发送数据包。
2.      休眠一段时间，空闲时间大于0，计算空闲资源累积量，并计算当前可用空闲资源tu。
3.      计算L长度的数据包需要消耗kc*L的空闲资源，如果tu > a*L，发送数据包，否则再休眠一段时间。
4.      发送数据包后减少空闲资源：tu = tu – a*L，如果tu > 0，重复3的过程，直到再次休眠。
5.      最理想的状态是：总是成立ts = a*L。

基本上时可以达到调控的目的，但是结果是不准确的，相同的算法，相同的参数，在不同的网络环境（主要是硬件的配置不同）中流控的结果肯定不同。
但是可以根据具体的网络环境，来选择适当的参数来提高算法的准确度。
可以调整的参数有两类：1. 算法参数，2. 配置参数。
可调整算法参数有：1. 空闲时间和空闲资源的换算参数 2. 每秒可产生的空闲资源TIME_UNITS_PER_SEC。

*/
/* Length to Time (L2T) lookup in a qdisc_rate_table, to determine how
   long it will take to send a packet given its size.
 
 */ // 将长度转换为令牌数 参考<（一）空闲资源流控算法>  参考应用层tc_calc_rtable   
static inline u32 qdisc_l2t(struct qdisc_rate_table* rtab, unsigned int pktlen) //表示发送ptklen长度需要消耗多少空闲资源时间
{
	int slot = pktlen + rtab->rate.cell_align + rtab->rate.overhead;// 根据大小计算合适的槽位
	if (slot < 0)
		slot = 0;
	slot >>= rtab->rate.cell_log;
	if (slot > 255)// 如果超过了255, 限制为255
		return (rtab->data[255]*(slot >> 8) + rtab->data[slot & 0xFF]);
	return rtab->data[slot];//默认情况下//这里得到的就是2047个字节所消耗的空闲资源。
}

#ifdef CONFIG_NET_CLS_ACT
static inline struct sk_buff *skb_act_clone(struct sk_buff *skb, gfp_t gfp_mask)
{
	struct sk_buff *n = skb_clone(skb, gfp_mask);

	if (n) {
		n->tc_verd = SET_TC_VERD(n->tc_verd, 0);
		n->tc_verd = CLR_TC_OK2MUNGE(n->tc_verd);
		n->tc_verd = CLR_TC_MUNGED(n->tc_verd);
	}
	return n;
}
#endif

#endif
