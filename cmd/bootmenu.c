// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2011-2013 Pali Rohár <pali.rohar@gmail.com>
 */

#include <common.h>
#include <command.h>
#include <ansi.h>
#include <menu.h>
#include <watchdog.h>
#include <malloc.h>
#include <linux/string.h>

/* maximum bootmenu entries */
#define MAX_COUNT	99

/* maximal size of bootmenu env
 *  9 = strlen("bootmenu_")
 *  2 = strlen(MAX_COUNT)
 *  1 = NULL term
 */
#define MAX_ENV_SIZE	(9 + 2 + 1)

struct bootmenu_entry {
	unsigned short int num;		/* unique number 0 .. MAX_COUNT */
	char key[3];			/* key identifier of number */
	char *title;			/* title of entry */
	char *command;			/* hush command of entry */
	struct bootmenu_data *menu;	/* this bootmenu */
	struct bootmenu_entry *next;	/* next menu entry (num+1) */
};

struct bootmenu_data {
	int delay;			/* delay for autoboot */
	int active;			/* active menu entry */
	int count;			/* total count of menu entries */
	struct bootmenu_entry *first;	/* first menu entry */
};

enum bootmenu_key {
	KEY_NONE = 0,
	KEY_UP,
	KEY_DOWN,
	KEY_SELECT,
};

static struct bootmenu_data *bootmenu = NULL;

static char *bootmenu_getoption(unsigned short int n)
{
	char name[MAX_ENV_SIZE];

	if (n > MAX_COUNT)
		return NULL;

	sprintf(name, "bootmenu_%d", n);
	return env_get(name);
}

static void bootmenu_print_entry(void *data)
{
	struct bootmenu_entry *entry = data;
	int reverse = (entry->menu->active == entry->num);

	/*
	 * Move cursor to line where the entry will be drown (entry->num)
	 * First 3 lines contain bootmenu header + 1 empty line
	 */
	printf(ANSI_CURSOR_POSITION, entry->num + 4, 1);

	puts("     ");

	if (reverse)
		puts(ANSI_COLOR_REVERSE);

	puts(entry->title);

	if (reverse)
		puts(ANSI_COLOR_RESET);
}

static void bootmenu_autoboot_loop(struct bootmenu_data *menu,
				enum bootmenu_key *key, int *esc)
{
	int i, c;

	if (menu->delay > 0) {
		printf(ANSI_CURSOR_POSITION, menu->count + 5, 1);
		printf("  Hit any key to stop autoboot: %2d ", menu->delay);
	}

	while (menu->delay > 0) {
		for (i = 0; i < 100; ++i) {
			if (!tstc()) {
				WATCHDOG_RESET();
				mdelay(10);
				continue;
			}

			menu->delay = -1;
			c = getc();

			switch (c) {
			case '\e':
				*esc = 1;
				*key = KEY_NONE;
				break;
			case '\r':
				*key = KEY_SELECT;
				break;
			default:
				*key = KEY_NONE;
				break;
			}

			break;
		}

		if (menu->delay < 0)
			break;

		--menu->delay;
		printf("\b\b\b%2d ", menu->delay);
	}

	printf(ANSI_CURSOR_POSITION, menu->count + 5, 1);
	puts(ANSI_CLEAR_LINE);

	if (menu->delay == 0)
		*key = KEY_SELECT;
}

static void bootmenu_loop(struct bootmenu_data *menu,
		enum bootmenu_key *key, int *esc)
{
	int c;

	while (!tstc()) {
		WATCHDOG_RESET();
		mdelay(10);
	}

	c = getc();

	switch (*esc) {
	case 0:
		/* First char of ANSI escape sequence '\e' */
		if (c == '\e') {
			*esc = 1;
			*key = KEY_NONE;
		}
		break;
	case 1:
		/* Second char of ANSI '[' */
		if (c == '[') {
			*esc = 2;
			*key = KEY_NONE;
		} else {
			*esc = 0;
		}
		break;
	case 2:
	case 3:
		/* Third char of ANSI (number '1') - optional */
		if (*esc == 2 && c == '1') {
			*esc = 3;
			*key = KEY_NONE;
			break;
		}

		*esc = 0;

		/* ANSI 'A' - key up was pressed */
		if (c == 'A')
			*key = KEY_UP;
		/* ANSI 'B' - key down was pressed */
		else if (c == 'B')
			*key = KEY_DOWN;
		/* other key was pressed */
		else
			*key = KEY_NONE;

		break;
	}

	/* enter key was pressed */
	if (c == '\r')
		*key = KEY_SELECT;
}

static char *bootmenu_choice_entry(void *data)
{
	struct bootmenu_data *menu = data;
	struct bootmenu_entry *iter;
	enum bootmenu_key key = KEY_NONE;
	int esc = 0;
	int i;

	while (1) {
		if (menu->delay >= 0) {
			/* Autoboot was not stopped */
			bootmenu_autoboot_loop(menu, &key, &esc);
		} else {
			/* Some key was pressed, so autoboot was stopped */
			bootmenu_loop(menu, &key, &esc);
		}

		switch (key) {
		case KEY_UP:
			if (menu->active > 0)
				--menu->active;
			/* no menu key selected, regenerate menu */
			return NULL;
		case KEY_DOWN:
			if (menu->active < menu->count - 1)
				++menu->active;
			/* no menu key selected, regenerate menu */
			return NULL;
		case KEY_SELECT:
			iter = menu->first;
			for (i = 0; i < menu->active; ++i)
				iter = iter->next;
			return iter->key;
		default:
			break;
		}
	}

	/* never happens */
	debug("bootmenu: this should not happen");
	return NULL;
}

static void bootmenu_destroy(struct bootmenu_data **menu)
{
	struct bootmenu_entry *iter;
	struct bootmenu_entry *next;

	if (!*menu)
		return;

	iter = (*menu)->first;
	while (iter) {
		next = iter->next;
		free(iter->title);
		free(iter->command);
		free(iter);
		iter = next;
	}
	free(*menu);
	*menu = NULL;
}

struct bootmenu_entry *bootmenu_make_entry(struct bootmenu_data *menu,
	const char *title, int titlelen, const char *command,
	int commandlen)
{
	int i;
	struct bootmenu_entry *entry;
	struct bootmenu_entry *iter;

	if (menu->count > MAX_COUNT)
		return 0;

	if (!(entry = malloc(sizeof(struct bootmenu_entry))))
		return 0;
	memset(entry, 0, sizeof(struct bootmenu_entry));

	if (title != NULL) {
		if (!titlelen) titlelen = strlen(title);
		if (!(entry->title = malloc(titlelen + 5))) {
			free(entry);
			return 0;
		}
		i = sprintf(entry->title, "%d. ", menu->count+1);
		if (titlelen > 0) memcpy(&entry->title[i], title, titlelen);
		entry->title[titlelen+i] = 0;
	}

	if (command != NULL) {
		if (!commandlen) commandlen = strlen(command);
		if (!(entry->command = malloc(commandlen+1))) {
			if (entry->title) free(entry->title);
			free(entry);
			return 0;
		}
		if (commandlen > 0) memcpy(entry->command, command, commandlen);
		entry->command[commandlen] = 0;
	}

	entry->menu = menu;
	entry->num = menu->count++;
	sprintf(entry->key, "%d", entry->num);
	if ((iter = menu->first)) {
		while (iter->next) iter = iter->next;
		iter->next = entry;
	} else {
		menu->first = entry;
	}

	return entry;
}

#ifdef CONFIG_HAVE_BLOCK_DEVICE
#include <blk.h>

static enum if_type bootmenu_if_types[] = {
#ifdef CONFIG_IDE
	IF_TYPE_IDE,
#endif
#ifdef CONFIG_SCSI
	IF_TYPE_SCSI,
#endif
#ifdef CONFIG_CMD_USB
	IF_TYPE_USB,
#endif
#ifdef CONFIG_CMD_MMC
	IF_TYPE_MMC,
#endif
#ifdef CONFIG_SATA
	IF_TYPE_SATA,
#endif
#ifdef CONFIG_SANDBOX
	IF_TYPE_HOST,
#endif
	IF_TYPE_UNKNOWN, /* term of list */
};

#define BOOTMENU_BLKDEV_CMD_NAME "bootcmd_blkdev"

#define BOOTMENU_BLKDEV_CMD_DEFAULT_BODY \
	"if ${devtype} dev ${devnum}; then " \
		"run scan_dev_for_boot_part; " \
	"fi"

#define BOOTMENU_BLKDEV_ENTRY_CMD(devtype, devnum) \
	"setenv devtype " #devtype "; " \
	"setenv devnum " #devnum "; " \
	"run " BOOTMENU_BLKDEV_CMD_NAME

#endif

static struct bootmenu_data *bootmenu_create(int delay)
{
	struct bootmenu_data *menu;

	int len;
	char *option, *sep, *targets, buffer[0x100];
	struct bootmenu_entry *entry;
#ifdef CONFIG_HAVE_BLOCK_DEVICE
	struct blk_desc *desc;
	const char *if_typename;
	enum if_type *type;
	int i, devnum;
#endif
	menu = malloc(sizeof(struct bootmenu_data));
	if (!menu)
		return NULL;
	memset(menu, 0, sizeof(struct bootmenu_data));
	menu->delay = delay;

	while ((option = bootmenu_getoption(menu->count)) && menu->count < MAX_COUNT) {
		sep = strchr(option, '=');
		if (!sep) {
			printf("Invalid bootmenu entry: %s\n", option);
			break;
		}

		entry = bootmenu_make_entry(menu, option, sep-option, sep+1, 0);
		if (!entry)
			goto cleanup;
	}
	
#ifdef CONFIG_HAVE_BLOCK_DEVICE
	if (!env_get(BOOTMENU_BLKDEV_CMD_NAME))
		env_set(BOOTMENU_BLKDEV_CMD_NAME, BOOTMENU_BLKDEV_CMD_DEFAULT_BODY);

	for (type = bootmenu_if_types; menu->count < MAX_COUNT && *type != IF_TYPE_UNKNOWN; ++type) {
		for (devnum = 0;menu->count < MAX_COUNT &&
			(desc = blk_get_devnum_by_type(*type, devnum)); ++devnum) {
			if (desc->type == DEV_TYPE_UNKNOWN)
				continue;
			if (desc->if_type == IF_TYPE_UNKNOWN) /* wtf?!?! */
				continue;
			if_typename = blk_get_if_type_name(desc->if_type);
			len = sprintf(buffer, "%s:%d %s", if_typename, desc->devnum, desc->product);
			sprintf(&buffer[len+1], BOOTMENU_BLKDEV_ENTRY_CMD(%s, %d), if_typename, desc->devnum);

			/* to upper */
			i = 0;
			while (buffer[i] > 96 && buffer[i] < 123 && i < len)
				buffer[i++] -= 32;

			entry = bootmenu_make_entry(menu, buffer, len, &buffer[len+1], 0);
			if (!entry)
				goto cleanup;
		}
	}

#endif
	memcpy(&buffer[0], "run bootcmd_", 12);
	targets = env_get("bootmenu_targets");
	for (;targets!=0&&menu->count<MAX_COUNT;targets=(sep?&targets[len+1]:0)) {
		sep = strchr(targets, ' ');
		if ((len=sep?(sep-targets):strlen(targets)) && len < 0x100) {
			memcpy(&buffer[12], targets, len);
			buffer[12+len] = 0;
			/*if (env_get(&buffer[4])) {*/
			entry = bootmenu_make_entry(menu, &buffer[8], len+4, &buffer[0], 12+len);
			if (!entry)
				goto cleanup;

			sep = strchr(entry->title, '.');
			memcpy(sep+2, "Run ", 4);
			/* to upper */
			sep += 6;
			while (*sep > 96 && *sep < 123)
				*sep++ -= 32;
		}
	}

	/* Add U-Boot console entry at the end */
	if (menu->count < MAX_COUNT) {
		entry = bootmenu_make_entry(menu, "U-Boot console", 14, 0, 0);
		if (!entry)
			goto cleanup;
	}

	return menu;

cleanup:
	bootmenu_destroy(&menu);
	return NULL;
}

static void bootmenu_show(int delay)
{
	int init = 0;
	char *title = NULL;
	char *command = NULL;
	struct menu *menu = NULL;
	struct bootmenu_entry *iter;
	char *sep, strdef[3] = {'0', 0, 0};
	int def = 0;

	if (!bootmenu) {
		bootmenu = bootmenu_create(delay);
		if (!bootmenu)
			return;

		if ((sep = env_get("bootmenu_default")) && *sep != 0 &&
			(def = simple_strtol(sep, NULL, 10))) {
			bootmenu->active = (def<0)
				? ((def%=-bootmenu->count)?def+bootmenu->count:0)
				: ((def%=bootmenu->count)?def:bootmenu->count)-1;
			sprintf(strdef, "%d", bootmenu->active);
		}
	} else {
		bootmenu->delay = delay;
	}

	/* If delay is 0 do not create menu, just run active entry or bootcmd */
	if (delay == 0) {
		if (bootmenu->count <= 1 && (command = env_get("bootcmd"))) {
			title = strdup("bootcmd");
		} else {
			for (iter = bootmenu->first; iter; iter = iter->next) {
				if (bootmenu->active == iter->num) {
					title = strdup(iter->title);
					command = strdup(iter->command);
				}
			}
		}
		goto cleanup;
	}

	menu = menu_create(NULL, bootmenu->delay, 1, bootmenu_print_entry,
			   bootmenu_choice_entry, bootmenu);
	if (!menu) {
		bootmenu_destroy(&bootmenu);
		return;
	}

	for (iter = bootmenu->first; iter; iter = iter->next) {
		if (!menu_item_add(menu, iter->key, iter))
			goto cleanup;
	}

	/* Default menu entry is always first */
	menu_default_set(menu, strdef);

	puts(ANSI_CURSOR_HIDE);
	puts(ANSI_CLEAR_CONSOLE);
	printf(ANSI_CURSOR_POSITION, 1, 1);

	init = 1;

	if (menu_get_choice(menu, (void**)&iter)) {
		title = strdup(iter->title);
		command = strdup(iter->command);
	}

cleanup:
	if (menu) menu_destroy(menu);
	bootmenu_destroy(&bootmenu);

	if (init) {
		puts(ANSI_CURSOR_SHOW);
		puts(ANSI_CLEAR_CONSOLE);
		printf(ANSI_CURSOR_POSITION, 1, 1);
	}

	if (title) {
		debug("Starting entry '%s'\n", title);
		free(title);
	}

	if (command) {
		run_command(command, 0);
		free(command);
	}

#ifdef CONFIG_POSTBOOTMENU
	run_command(CONFIG_POSTBOOTMENU, 0);
#endif
}

void menu_display_statusline(struct menu *m)
{
	struct bootmenu_entry *entry;
	struct bootmenu_data *menu;

	if (menu_default_choice(m, (void *)&entry) < 0)
		return;

	menu = entry->menu;

	printf(ANSI_CURSOR_POSITION, 1, 1);
	puts(ANSI_CLEAR_LINE);
	printf(ANSI_CURSOR_POSITION, 2, 1);
	puts("  *** U-Boot Boot Menu ***");
	puts(ANSI_CLEAR_LINE_TO_END);
	printf(ANSI_CURSOR_POSITION, 3, 1);
	puts(ANSI_CLEAR_LINE);

	/* First 3 lines are bootmenu header + 2 empty lines between entries */
	printf(ANSI_CURSOR_POSITION, menu->count + 5, 1);
	puts(ANSI_CLEAR_LINE);
	printf(ANSI_CURSOR_POSITION, menu->count + 6, 1);
	puts("  Press UP/DOWN to move, ENTER to select");
	puts(ANSI_CLEAR_LINE_TO_END);
	printf(ANSI_CURSOR_POSITION, menu->count + 7, 1);
	puts(ANSI_CLEAR_LINE);
}

#ifdef CONFIG_MENU_SHOW
int menu_show(int bootdelay)
{
#ifdef CONFIG_MENUKEY
	const char *stopkey;
	char menukey = CONFIG_MENUKEY;

	if ((stopkey = env_get("bootstopkey")) && stopkey[0] != '\0')
		menukey = stopkey[0];

	if (menukey != -1 && tstc())
		if (menukey == getc())
			bootdelay = -1;
#endif
	bootmenu_show(bootdelay);
	return -1; /* -1 - abort boot and run monitor code */
}
#endif

int do_bootmenu(cmd_tbl_t *cmdtp, int flag, int argc, char *const argv[])
{
	char *delay_str = NULL;
	int delay = 10;

#if defined(CONFIG_BOOTDELAY) && (CONFIG_BOOTDELAY >= 0)
	delay = CONFIG_BOOTDELAY;
#endif

	if (argc >= 2)
		delay_str = argv[1];

	if (!delay_str)
		delay_str = env_get("bootmenu_delay");

	if (delay_str)
		delay = (int)simple_strtol(delay_str, NULL, 10);

	bootmenu_show(delay);
	return 0;
}

U_BOOT_CMD(
	bootmenu, 2, 1, do_bootmenu,
	"ANSI terminal bootmenu",
	"[delay]\n"
	"    - show ANSI terminal bootmenu with autoboot delay"
);

int do_bootmenu_add(cmd_tbl_t *cmdtp, int flag, int argc, char *const argv[])
{
	if (argc < 3)
		return -1;

	if (bootmenu == NULL) {
		bootmenu = malloc(sizeof(struct bootmenu_data));
		if (!bootmenu)
			return -1;
		memset(bootmenu, 0, sizeof(struct bootmenu_data));
	} else if (bootmenu->count >= MAX_COUNT) {
		return -1;
	}

	if (!bootmenu_make_entry(bootmenu, argv[1], 0, argv[2], 0))
		return -1;

	return 0;
}

U_BOOT_CMD(
	bootmenu_add, 3, 1, do_bootmenu_add,
	"add menu to bootmenu",
	"text\n"
	"    - Text to be shown in the menu\n"
	"command\n"
	"    - Run command when choosing"
);

int do_bootmenu_clean(cmd_tbl_t *cmdtp, int flag, int argc, char *const argv[])
{
	if (bootmenu)
		bootmenu_destroy(&bootmenu);
	return 0;
}

U_BOOT_CMD(
	bootmenu_clean, 1, 0, do_bootmenu_clean,
	"Delete all added menu's",
	"\n"
);
