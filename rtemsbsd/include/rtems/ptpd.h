#ifndef RTEMS_PTPD_H_
#define RTEMS_PTPD_H_

#include <rtems.h>
#include <rtems/rtems/status.h>

typedef struct rtems_ptpd_config {
	rtems_task_priority priority;
	int argc;
	char **argv;
	void (*prepare)(const struct rtems_ptpd_config *config,
	    int argc, char **argv);
	void (*destroy)(const struct rtems_ptpd_config *config,
	    int exit_code);
} rtems_ptpd_config;

int ptpd_main(int argc, char **argv);
rtems_status_code rtems_ptpd_start(const rtems_ptpd_config *config);

#endif /* RTEMS_PTPD_H_ */
