#include <stdlib.h>
#include <rtems/shell.h>
#include <rtems/ptpd.h>

typedef struct {
	rtems_ptpd_config config;
	char *argv[RTEMS_ZERO_LENGTH_ARRAY];
} ptpd_command_config;

static void
ptpd_command_destroy_config(const rtems_ptpd_config *config, int exit_code)
{
	char **argv;

	(void)exit_code;

	argv = config->argv;
	while (*argv != NULL) {
		free(*argv);
		++argv;
	}

	free(RTEMS_DECONST(rtems_ptpd_config *, config));
}

int
rtems_bsd_command_ptpd(int argc, char **argv)
{
	ptpd_command_config *config;
	rtems_status_code sc;
	int i;

	config = calloc(1, sizeof(*config) + (argc + 1) * sizeof(char *));
	if (config == NULL) {
		fprintf(stdout, "ptpd error: not enough memory\n");
		return 1;
	}

	for (i = 0; i < argc; ++i) {
		config->argv[i] = strdup(argv[i]);
		if (config->argv[i] == NULL) {
			ptpd_command_destroy_config(&config->config, 0);
			fprintf(stdout, "ptpd error: not enough memory\n");
			return 1;
		}
	}

	config->config.argc = argc;
	config->config.argv = &config->argv[0];
	config->config.destroy = ptpd_command_destroy_config;

	sc = rtems_ptpd_start(&config->config);
	if (sc != RTEMS_SUCCESSFUL) {
		ptpd_command_destroy_config(&config->config, 0);
		fprintf(stdout, "ptpd start failed: %s\n", rtems_status_text(sc));
	}

	return 0;
}

rtems_shell_cmd_t rtems_shell_PTPD_Command = {
  .name = "ptpd",
  .usage = "ptpd [args]",
  .topic = "net",
  .command = rtems_bsd_command_ptpd
};
