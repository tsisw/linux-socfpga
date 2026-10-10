// SPDX-License-Identifier: GPL-2.0-only
/*
 * Minimal static init for the QEMU ACPI enumeration check: mount sysfs,
 * report the ACPI tables present and every platform device the SkyLP SSDT
 * produced (name, modalias, firmware path, bound driver), then power off.
 *
 * Build: aarch64-linux-gnu-gcc -static -Os -o init enum-init.c
 */
#include <dirent.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/mount.h>
#include <sys/reboot.h>
#include <sys/stat.h>
#include <unistd.h>

static void cat(const char *path, const char *label)
{
	char buf[512];
	int fd = open(path, O_RDONLY), n;

	if (fd < 0)
		return;
	n = read(fd, buf, sizeof(buf) - 1);
	close(fd);
	if (n <= 0)
		return;
	buf[n] = 0;
	if (buf[n - 1] == '\n')
		buf[n - 1] = 0;
	printf("    %s: %s\n", label, buf);
}

static void link_tail(const char *path, const char *label)
{
	char buf[512];
	ssize_t n = readlink(path, buf, sizeof(buf) - 1);
	const char *p;

	if (n < 0)
		return;
	buf[n] = 0;
	p = strrchr(buf, '/');
	printf("    %s: %s\n", label, p ? p + 1 : buf);
}

int main(void)
{
	struct dirent *de;
	DIR *d;

	mount("sysfs", "/sys", "sysfs", 0, NULL);
	mount("proc", "/proc", "proc", 0, NULL);

	printf("ENUM: acpi tables:");
	d = opendir("/sys/firmware/acpi/tables");
	if (d) {
		while ((de = readdir(d)))
			if (de->d_name[0] != '.')
				printf(" %s", de->d_name);
		closedir(d);
	}
	printf("\n");

	d = opendir("/sys/bus/platform/devices");
	if (!d) {
		printf("ENUM: no platform bus\n");
		goto off;
	}
	while ((de = readdir(d))) {
		char p[600];

		if (strncmp(de->d_name, "PRP0001", 7) && strncmp(de->d_name, "TSI0", 4))
			continue;
		printf("ENUM: device %s\n", de->d_name);
		snprintf(p, sizeof(p), "/sys/bus/platform/devices/%s/modalias", de->d_name);
		cat(p, "modalias");
		snprintf(p, sizeof(p), "/sys/bus/platform/devices/%s/firmware_node/path",
			 de->d_name);
		cat(p, "acpi path");
		snprintf(p, sizeof(p), "/sys/bus/platform/devices/%s/driver", de->d_name);
		link_tail(p, "driver");
	}
	closedir(d);
off:
	printf("ENUM: done\n");
	sync();
	reboot(RB_POWER_OFF);
	return 0;
}
