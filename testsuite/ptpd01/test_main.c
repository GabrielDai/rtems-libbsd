/**
 * @file
 *
 * @brief It runs a PTPd instance configured as slave only.
 *
 * PTPd logs are printed out.
 * For changing the configuration, update ptpd_conf.
 */

/*
 * Copyright (c) 2023 Gabriel Moyano, German Aerospace Center (DLR)
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <assert.h>
#include <stdio.h>
#include <net/if.h>

#include <rtems.h>
#include <machine/rtems-bsd-commands.h>

#define TEST_NAME "LIBBSD PTPD 1"
#define TEST_STATE_USER_INPUT 1

static const char* ptpd_if = "ptpengine:interface = ";

static const char* ptpd_conf = \
    "\n" \
    "ptpengine:preset = slaveonly\n" \
    "ptpengine:ip_mode = multicast\n" \
    "ptpengine:transport = ipv4\n" \
    "ptpengine:use_libpcap = N\n" \
    "ptpengine:delay_mechanism = E2E\n" \
    "ptpengine:domain = 0\n" \
    "ptpengine:slave_only = N\n" \
    "ptpengine:inbound_latency = 0\n" \
    "ptpengine:outbound_latency = 0\n" \
    "ptpengine:always_respect_utc_offset = N\n" \
    "ptpengine:log_announce_interval = 1\n" \
    "ptpengine:log_sync_interval = 0\n" \
    "ptpengine:log_delayreq_interval_initial = 0\n" \
    "ptpengine:log_delayreq_interval = 0\n" \
    "ptpengine:log_peer_delayreq_interval = 1\n" \
    "ptpengine:foreignrecord_capacity = 5\n" \
    "ptpengine:ptp_allan_variance = 28768\n" \
    "ptpengine:ptp_clock_accuracy = ACC_UNKNOWN\n" \
    "ptpengine:utc_offset = 0\n" \
    "ptpengine:utc_offset_valid = N\n" \
    "ptpengine:time_traceable = N\n" \
    "ptpengine:frequency_traceable = N\n" \
    "ptpengine:ptp_timescale = ARB\n" \
    "ptpengine:ptp_timesource = INTERNAL_OSCILLATOR\n" \
    "ptpengine:clock_class = 255\n" \
    "ptpengine:priority1 = 128\n" \
    "ptpengine:priority2 = 128\n" \
    "ptpengine:unicast_address = \n" \
    "ptpengine:igmp_refresh = Y\n" \
    "ptpengine:multicast_ttl = 64\n" \
    "ptpengine:ip_dscp = 0\n" \
    "ptpengine:panic_mode = N\n" \
    "ptpengine:panic_mode_duration = 2\n" \
    "ptpengine:ntp_failover = N\n" \
    "ptpengine:ntp_failover_timeout = 60\n" \
    "ptpengine:prefer_ntp = N\n" \
    "ptpengine:panic_mode_ntp = N\n" \
    "clock:no_adjust = N\n" \
    "clock:no_reset = N\n" \
    "clock:drift_handling = preserve\n" \
    "clock:drift_file = /etc/ptpd2_kernelclock.drift\n" \
    "servo:delayfilter_stiffness = 6\n" \
    "servo:kp = 0.1\n" \
    "servo:ki = 0.001\n" \
    "servo:max_delay = 0\n" \
    "servo:max_delay = 0\n" \
    "servo:max_offset = 0\n" \
    "global:use_syslog = N\n" \
    "global:lock_file = \n" \
    "global:auto_lockfile = N\n" \
    "global:ignore_lock = Y\n" \
    "global:quality_file = \n" \
    "global:quality_file_max_size = 0\n" \
    "global:quality_file_max_files = 0\n" \
    "global:quality_file_truncate = N\n" \
    "global:log_status = Y\n" \
    "global:status_update_interval = 1\n" \
    "global:log_file_max_size = 0\n" \
    "global:log_file_max_files = 0\n" \
    "global:log_file_truncate = N\n" \
    "global:log_level = LOG_ALL\n" \
    "global:statistics_log_interval = 0\n" \
    "global:statistics_file_max_size = 0\n" \
    "global:statistics_file_truncate = N\n" \
    "global:statistics_file_max_files = 0\n" \
    "global:dump_packets = N\n" \
    "global:verbose_foreground = N\n" \
    "global:foreground = N\n" \
    "global:log_statistics = Y\n" \
    "global:cpuaffinity_cpucore = -1\n" \
    "global:statistics_update_interval = 5\n" \
    "ntpengine:enabled = N\n" \
    "ntpengine:control_enabled = N\n" \
    "ntpengine:check_interval = 15\n" \
    "ntpengine:key_id = 0\n" \
    "ntpengine:key = \n" \
    "\n";


static void
create_config_file(char *ifname)
{

    FILE* fd;
    fd = fopen("/etc/ptpd_rtems.conf", "w");
    fwrite(ptpd_if, strlen(ptpd_if), 1, fd);
    fwrite(ifname, strlen(ifname), 1, fd);
    fwrite(ptpd_conf, strlen(ptpd_conf), 1, fd);
    fclose(fd);
}

static void
test_main(void)
{

	rtems_status_code sc;
	int exit_code;
	char *ifname;
	char ifnamebuf[IF_NAMESIZE];

	ifname = if_indextoname(1, &ifnamebuf[0]);
	assert(ifname != NULL);
    create_config_file(ifname);

	/* Give some time to obtain an IP */
	sc = rtems_task_wake_after(2000);
	assert(sc == RTEMS_SUCCESSFUL);

	char *ptpd[] = {
		"ptpd",
        "-c",
        "/etc/ptpd_rtems.conf",
        NULL
    };
	exit_code = rtems_bsd_command_ptpd(RTEMS_BSD_ARGC(ptpd), ptpd);
	assert(exit_code == 0);
	rtems_task_exit();
}

#define DEFAULT_NETWORK_DHCPCD_ENABLE
#define DEFAULT_NETWORK_SHELL

#include <rtems/bsd/test/default-network-init.h>
