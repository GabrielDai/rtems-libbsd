#include <assert.h>
#include <rtems/ptpd.h>
#include <machine/rtems-bsd-program.h>
#include <rtems/shell.h>
#include <rtems/console.h>
#include <rtems/rfs/rtems-rfs-mutex.h>
#include <rtems/score/timecounter.h>
#include <machine/rtems-bsd-commands.h>


rtems_recursive_mutex ptpd_mutex =
    RTEMS_RECURSIVE_MUTEX_INITIALIZER("ptpd");

static bool ptpd_initialized;

static void
ptpd_task(rtems_task_argument arg)
{
	const char *default_argv[] = { "ptpd", NULL };
	const rtems_ptpd_config *config;
	int argc;
	char **argv;
	int exit_code;

	config = (const rtems_ptpd_config *)arg;

	if (config != NULL) {
		argc = config->argc;
		argv = config->argv;
	} else {
		argc = RTEMS_BSD_ARGC(default_argv);
		argv = default_argv;
	}

	exit_code = rtems_bsd_program_call_main("ptpd", ptpd_main, argc, argv);

	if (config != NULL && config->destroy != NULL) {
		(*config->destroy)(config, exit_code);
	}

	rtems_task_delete(RTEMS_SELF);
}

rtems_status_code
rtems_ptpd_start(const rtems_ptpd_config *config)
{
	static const char name[] = "PTPD";
	rtems_status_code sc;

	rtems_recursive_mutex_lock(&ptpd_mutex);

	if (!ptpd_initialized) {
		rtems_task_priority priority;
		rtems_id id;

		if (config != NULL && config->priority != 0) {
			priority = config->priority;
		} else {
			priority = rtems_bsd_get_task_priority(name);
		}

		sc = rtems_task_create(rtems_build_name(name[0], name[1], name[2], name[3]), priority,
		    rtems_bsd_get_task_stack_size(name), RTEMS_DEFAULT_MODES,
		    RTEMS_FLOATING_POINT, &id);
		if (sc == RTEMS_SUCCESSFUL) {
			ptpd_initialized = true;

			sc = rtems_task_start(id, ptpd_task,
			    (rtems_task_argument) config);
			assert(sc == RTEMS_SUCCESSFUL);
		}
	} else {
		sc = RTEMS_INCORRECT_STATE;
	}

	rtems_recursive_mutex_unlock(&ptpd_mutex);
	return sc;
}
