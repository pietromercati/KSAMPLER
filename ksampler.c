/* -------   KSAMPLER MODULE FOR ANDROID/LINUX   ----------------------------
 * 
 * Author: Pietro Mercati
 * email : pimercat@eng.ucsd.edu
 * 
 * Parts of this code are obtained from:
 *
 * http://stackoverflow.com/questions/3247373/how-to-measure-program-execution-time-in-arm-cortex-a8-processor
 *
 *
 * If using this code for research purposes, include 
 * references to the following publications
 * 
 * 1) P.Mercati, A. Bartolini, F. Paterna, T. Rosing and L. Benini; A Linux-governor based 
 *    Dynamic Reliability Manager for android mobile devices. DATE 2014.
 * 2) P.Mercati, A. Bartolini, F. Paterna, L. Benini and T. Rosing; An On-line Reliability 
 *    Emulation Framework. EUC 2014
 * 
	This file is part of KSAMPLER.
        KSAMPLER is free software: you can redistribute it and/or modify
        it under the terms of the GNU General Public License as published by
        the Free Software Foundation, either version 3 of the License, or
        (at your option) any later version.
        KSAMPLER is distributed in the hope that it will be useful,
        but WITHOUT ANY WARRANTY; without even the implied warranty of
        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
        GNU General Public License for more details.
        You should have received a copy of the GNU General Public License
        along with KSAMPLER.  If not, see <http://www.gnu.org/licenses/>.
*/


#include <linux/cpumask.h>
#include <linux/init.h>
#include <linux/module.h>  /* Needed by all modules */
#include <linux/kernel.h>  /* Needed for KERN_ALERT */
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/jiffies.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/string.h>
#include <linux/time.h>
#include <linux/cpumask.h>
#include <linux/cpu.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/slab.h>
#include <linux/cpumask.h>
#include <linux/cdev.h>
#include <linux/device.h>  //for class_create

MODULE_LICENSE("Dual BSD/GPL");
MODULE_INFO(vermagic, "3.10.9-04004-g45fc3d7-dirty SMP preempt mod_unload ARMv7 p2v8 ");

#define DEBUG

#define MY_INFO(str,arg...) printk(KERN_ALERT str, ## arg);

#define POWER_SENSOR_COUNT 4
#define MAX_CPUS 8
#define SLEEP_TIME_MS 100

extern unsigned int power_core_monitor_single[POWER_SENSOR_COUNT];

struct my_perf_data_struct {
	//jiffies
	unsigned int jif1;
	unsigned int jif2;
	unsigned int jifd;
	//cycles
	unsigned int cycle1;
	unsigned int cycle2;
       	unsigned int cycled;
	//instructions
	unsigned int ins1;
	unsigned int ins2;
	unsigned int insd;
	//cache misses
	unsigned int cm1;
	unsigned int cm2;
	unsigned int cmd;
	//branch misses
	unsigned int bm1;
	unsigned int bm2;
	unsigned int bmd;
	//power A15, A7, GPU, mem
	unsigned int power_sensor[POWER_SENSOR_COUNT];
};

DEFINE_PER_CPU(struct my_perf_data_struct, my_perf_data);
DEFINE_PER_CPU(struct my_perf_data_struct*, my_perf_data_ptr);
DEFINE_PER_CPU(int, prova);

struct task_struct *task[8];
int data;
int ret;

static inline void program_perf_counter_control_register (int32_t do_reset, int32_t enable_divider){

	// in general enable all counters (including cycle counter)
	int32_t value = 1;

	// peform reset:
	if (do_reset)
	{
	value |= 2;     // reset all counters to zero.
	value |= 4;     // reset cycle counter to zero.
	}

	if (enable_divider)
	value |= 8;     // enable "by 64" divider for CCNT.
	value |= 16;

	// program the performance-counter control-register:
	asm volatile ("MCR p15, 0, %0, c9, c12, 0\t\n" :: "r"(value));
}

static inline void enable_all_counters(void){
	// enable all counters:
	asm volatile ("MCR p15, 0, %0, c9, c12, 1\t\n" :: "r"(0x8000000f)); // 0x8000000f
}

static inline void clear_overflows(void){
	// clear overflows:
	asm volatile ("MCR p15, 0, %0, c9, c12, 3\t\n" :: "r"(0x8000000f));
}

static inline void select_register(uint32_t reg){
    asm volatile ("MCR p15, 0, %0, c9, c12, 5" ::"r" (reg));
}

static inline void select_event(uint32_t event){
    asm volatile ("MCR p15, 0, %0, c9, c13, 1" ::"r"(event));
}

static inline unsigned int get_cyclecount (void){
	unsigned int value;
	// Read CCNT Register
	asm volatile ("MRC p15, 0, %0, c9, c13, 0\t\n": "=r"(value));
	return value;
}

static inline unsigned int get_instructions(void){
	unsigned int value;
	select_register(1);
    	asm volatile ("MRC p15, 0, %0, c9, c13, 2\t\n": "=r"(value));
	return value;
}

static inline unsigned int get_cachemisses(void){
	unsigned int value;
	select_register(2);
    	asm volatile ("MRC p15, 0, %0, c9, c13, 2\t\n": "=r"(value));
	return value;
}

static inline unsigned int get_branchmisses(void){
	unsigned int value;
	select_register(3);
    	asm volatile ("MRC p15, 0, %0, c9, c13, 2\t\n": "=r"(value));
	return value;
}

static inline void enable_usermode(void){
	/* enable user-mode access to the performance counter*/
	asm ("mcr p15, 0, %0, c9, c14, 0\n\t" :: "r" (0x00000001));
}

static inline void disable_counter_overflow(void){
	/* disable counter overflow interrupts (just in case)*/
	asm ("MRC p15, 0, %0, C9, C14, 2\n\t" :: "r"(0x8000000f));
}


static inline void reset_PMCR(void){
	/* disable counter overflow interrupts (just in case)*/
	asm ("mcr p15, 0, %0, c9, c12, 0\n\t" :: "r"(0x00000017));
}

static inline void count_enable_set_register(void){
	/* disable counter overflow interrupts (just in case)*/
	asm ("mcr p15, 0, %0, c9, c12, 1\n\t" :: "r"(0x8000000f));
}

static inline void overflow_flag_status_register(void){
	/* disable counter overflow interrupts (just in case)*/
	asm ("mcr p15, 0, %0, c9, c12, 3\n\t" :: "r"(0x8000000f));
}

void inline init_perfcounters(void){
	enable_usermode();
	reset_PMCR();
	count_enable_set_register();
	overflow_flag_status_register();
	program_perf_counter_control_register(1,0);
	enable_all_counters();
	clear_overflows();
	disable_counter_overflow();
}


int thread_function(void *data){
	
	int i;
	unsigned int cpu = smp_processor_id();

	//for correct timing
	static struct timeval tm1;
	struct timeval tm2;
	unsigned long long t;



	struct my_perf_data_struct* my_perf_data_local;

	// initialize perf counters
	init_perfcounters();

	//select register and events
	select_register(1);
	select_event(0x08); // instructions
	select_register(2);
	select_event(0x03); // cache misses 
	select_register(3);
	select_register(0x10);// branch misses 

	schedule();
	
	#ifdef DEBUG
	printk(KERN_ALERT "kthread_should_stop = %d\n", (int)kthread_should_stop());
	#endif 

	while(!kthread_should_stop()){
	
		// get difference 
		// -- jiffies

		// time execution start
		do_gettimeofday(&tm1);

		my_perf_data_local = &get_cpu_var(my_perf_data);
		
		my_perf_data_local->jif2 = jiffies;
		my_perf_data_local->jifd = my_perf_data_local->jif2 - my_perf_data_local->jif1; 

		my_perf_data_local->cycle2 = get_cyclecount();
		my_perf_data_local->cycled = my_perf_data_local->cycle2 - my_perf_data_local->cycle1; 

		my_perf_data_local->ins2 = get_instructions();
		my_perf_data_local->insd = my_perf_data_local->ins2 - my_perf_data_local->ins1; 
		
		my_perf_data_local->cm2 = get_cachemisses();
		my_perf_data_local->cmd = my_perf_data_local->cm2 - my_perf_data_local->cm1; 

		my_perf_data_local->bm2 = get_branchmisses();
		my_perf_data_local->bmd = my_perf_data_local->bm2 - my_perf_data_local->bm1; 		

		// update value
		my_perf_data_local->jif1 = my_perf_data_local->jif2;
		my_perf_data_local->cycle1 = my_perf_data_local->cycle2;
		my_perf_data_local->ins1 = my_perf_data_local->ins2;
		my_perf_data_local->cm1 = my_perf_data_local->cm2;
		my_perf_data_local->bm1 = my_perf_data_local->bm2;

		// get power 
		for(i=0; i<POWER_SENSOR_COUNT; i++){
			my_perf_data_local->power_sensor[i] = power_core_monitor_single[i];
		}

		put_cpu_var(my_perf_data);


#ifdef DEBUG
		printk(KERN_ALERT "cpu=%d\tjifd=%u\tcycled=%u\tinsd=%u\tcmd=%u\tbmd=%u\tPA15=%u\tPA7=%u\tPGPU=%u\tPMEM=%u\n", 	
											cpu,
											my_perf_data_local->jifd, 
											my_perf_data_local->cycled,
											my_perf_data_local->insd,
											my_perf_data_local->cmd,
											my_perf_data_local->bmd,
											my_perf_data_local->power_sensor[0],
											my_perf_data_local->power_sensor[1],
											my_perf_data_local->power_sensor[2],
											my_perf_data_local->power_sensor[3]
				      );

#endif //DEBUG	
		//take execution time
		do_gettimeofday(&tm2);

		t = 1000 * (tm2.tv_sec - tm1.tv_sec) + (tm2.tv_usec - tm1.tv_usec) / 1000;
		//printk(KERN_ALERT "execution time = %llu ms\n", t);

		msleep(SLEEP_TIME_MS-t);
	}
	return 0;
}

static int my_init(void){
	int cpu;
	int i;

	MY_INFO("spawn threads \n");
	// spawn one kernel thread per each core
	
	my_perf_data_ptr = alloc_percpu(my_perf_data);

	get_online_cpus();
	for_each_online_cpu(cpu){	
		task[cpu] = kthread_create(thread_function, (void *)data, "ksampler_thread");
		kthread_bind(task[cpu], cpu);
		printk(KERN_ALERT "active cpu = %d\n", cpu);
	}
	put_online_cpus();


	get_online_cpus();
	for_each_online_cpu(cpu){	
		wake_up_process(task[cpu]);
	}
	put_online_cpus();

        return 0;
}

static void my_exit(void){
	int cpu=0;

	MY_INFO("EXITING KSAMPLER MODULE ...\n");

	get_online_cpus();
	for_each_online_cpu(cpu){	
		kthread_stop(task[cpu]);	
	}
	put_online_cpus();

}

module_init(my_init);
module_exit(my_exit);
