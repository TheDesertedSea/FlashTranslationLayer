/**
 * Fuse is used to first read in a file into the flash simulator and then
 * read/write from that flashsimulator to fullfil user's requirements
 *
 * Note: This file currently assumes workload of only 1 file. Flashsimulator
 * has no concept of filesystem.
 */

#define FUSE_USE_VERSION 26

#include <ctype.h>
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <fuse.h>
#include <getopt.h>
#include <limits.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/xattr.h>
#include <time.h>
#include <unistd.h>
#include <stdarg.h>

#include "746FlashSim.h"
#include "common.h"

#define MAX_PATH_LEN 4096

#define UNUSED __attribute__((unused))

/* Is debugging enabled? Default - Disabled*/
int is_debug_enabled;

/* Absolute path to mount point */
char *abs_mount_path;


/* Absolute path to refernce point */
char *abs_ref_path;


/* Absolute data file path */
char *abs_fname;

/* Just the filename */
char *rel_fname;

/* Log file */
FILE *log_fp;

/* Interface to flash simulator */
FlashSimTest *sim;

/* File size */
size_t fsize;


int dprintf(const char *fmt, ...)  __attribute__ ((format (printf, 1, 2)));

int dprintf(const char *fmt, ...)
{
    va_list args;

    if (is_debug_enabled == 0)
    	return 0;

    va_start(args, fmt);
    int rc = vprintf(fmt, args);
    va_end(args);
    return rc;
}

/*
 * Returns relative path to the mount point
 * Input is the relative path in respect to the mount point
 */
void get_rel_path(const char *path, char *rel_path)
{
	/* Just replace initial '/' with './' */
	rel_path[0] = '.';
	strcpy(&rel_path[1], path);
}

static void *myFuse_init(struct fuse_conn_info *conn UNUSED)
{
	int ret;

	dprintf("In init\n");

	ret = chdir(abs_ref_path);
	if (ret < 0) {

		dprintf("Failed to chdir to ref dir %s\n", abs_ref_path);
		exit(-1);
	}
	return NULL;
}

static void myFuse_destroy(void *private_data UNUSED)
{
	dprintf("Destroyed\n");
	deinit_flashsim();
}


static int myFuse_setxattr(const char *path, const char *name,
                     const char *value, size_t size, int flags)
{
	int ret;
	char rel_path[MAX_PATH_LEN];
	get_rel_path(path, rel_path);

	ret = setxattr(rel_path, name, value, size, flags);
	if (ret < 0) {
		dprintf("setxattr failed: Ret %d, Error %s\n", ret,
				strerror(errno));
		return -errno;
	}
	return ret;
}

int myFuse_getxattr(const char *path, const char *name, char *value,
			size_t size)
{
	int ret;
	char rel_path[MAX_PATH_LEN];
	get_rel_path(path, rel_path);

	ret = getxattr(rel_path, name, value, size);
	if (ret < 0) {
		dprintf("getxattr failed: Ret %d, Error %s\n", ret,
				strerror(errno));
		return -errno;
	}
	return ret;

}

int myFuse_getattr(const char *path, struct stat *buf)
{
	int ret;
	char rel_path[MAX_PATH_LEN];
	get_rel_path(path, rel_path);

	dprintf("In getattr %s\n", rel_path);

	ret = stat(rel_path, buf);
	if (ret < 0) {
		dprintf("getattr failed: Ret %d, Error %s\n", ret,
				strerror(errno));
		return -errno;
	}

	if (strcmp(path, rel_fname) == 0)
		buf->st_size = fsize;

	return ret;

}

int myFuse_mkdir(const char *path, mode_t mode)
{
	int ret;
	char rel_path[MAX_PATH_LEN];
	get_rel_path(path, rel_path);

	ret = mkdir(rel_path, mode);
	if (ret < 0) {
		dprintf("mkdir failed: Ret %d, Error %s\n", ret,
				strerror(errno));
		return -errno;
	}
	return ret;

}

int myFuse_rmdir(const char *path)
{
	int ret;
	char rel_path[MAX_PATH_LEN];
	get_rel_path(path, rel_path);

	ret = rmdir(rel_path);
	if (ret < 0) {
		dprintf("rmdir failed: Ret %d, Error %s\n", ret,
				strerror(errno));
		return -errno;
	}

	return ret;

}

int myFuse_open(const char *path, struct fuse_file_info *fi)
{
	int ret;
	char rel_path[MAX_PATH_LEN];
	get_rel_path(path, rel_path);

	dprintf("In open %s\n", rel_path);

	ret = open(rel_path, fi->flags);
	if (ret < 0) {
		dprintf("open failed: Ret %d, Error %s\n", ret,
				strerror(errno));
		return -errno;
	}

	dprintf("FD is %d\n", ret);

	fi->fh = ret;

	return 0;

}

int myFuse_read(const char *path, char *buf, size_t size, off_t offset,
			struct fuse_file_info *fi)
{
	int ret;
	class datastore_page_t page;
	int page_size = sizeof(page.buf);

	dprintf("In read %s, Comparining with %s\n", path, rel_fname);

	/* Small Files */
	if (strcmp(path, rel_fname) != 0) {
		ret = pread(fi->fh, buf, size, offset);
		if (ret < 0) {
			dprintf("read failed: Ret %d, Error %s\n", ret,
				strerror(errno));
			return -errno;
		}

		return ret;

	} else {

		dprintf("Reading from sim: offset %zu, size %zu\n",
			offset, size);

		int start_page_num = offset / page_size;
		int end_page_num = (offset + size - 1) / page_size;

		int page_num;
		int buf_idx = 0;

		for (page_num = start_page_num; page_num <= end_page_num;
			page_num++) {

			int start_offset, end_offset;
			int start_idx = 0;
			int end_idx = page_size - 1;

			ret = sim->Read(log_fp, page_num, &page);
			if (ret != 1) {

				fprintf(stderr, "Fail to read input file\n");
				exit(-1);
			}

			start_offset = page_num * page_size;
			end_offset = (page_num + 1) * page_size - 1;

			if (offset > start_offset)
				start_idx = offset - start_offset;

			if (offset + (int)size - 1 < end_offset)
				end_idx = offset + size - 1 - start_offset;

			memcpy(&buf[buf_idx], &page.buf[start_idx],
				end_idx - start_idx + 1);

			buf_idx += end_idx - start_idx + 1;
		}

		return buf_idx;
	}
}

int myFuse_write(const char *path, const char *buf, size_t size, off_t offset,
	     struct fuse_file_info *fi)
{
	int ret;
	class datastore_page_t page;
	int page_size = sizeof(page.buf);

	dprintf("In write %s, size %zu, offset %zu\n", path, size, offset);

	if (strcmp(path, rel_fname) != 0) {
		ret = pwrite(fi->fh, buf, size, offset);
		if (ret < 0) {
			dprintf("write failed: Ret %d, Error %s\n", ret,
				strerror(errno));
			return -errno;
		}

		return ret;

	} else {

		int start_page_num = offset / page_size;
		int end_page_num = (offset + size - 1) / page_size;

		int page_num;
		int buf_idx = 0;

		for (page_num = start_page_num; page_num <= end_page_num;
			page_num++) {

			int start_offset, end_offset;
			int start_idx = 0;
			int end_idx = page_size - 1;

			start_offset = page_num * page_size;
			end_offset = (page_num + 1) * page_size - 1;

			if (offset > start_offset)
				start_idx = offset - start_offset;

			if (offset + (int)size - 1 < end_offset)
				end_idx = offset + size - 1 - start_offset;


			if ((start_idx != 0) || (end_idx != page_size -1)) {

				ret = sim->Read(log_fp, page_num, &page);
				if (ret != 1) {
					/* Might not be in the memory */
					memset(page.buf, 0, sizeof(page.buf));
				}
			}

			memcpy(&page.buf[start_idx], &buf[buf_idx],
				end_idx - start_idx + 1);

			ret = sim->Write(log_fp, page_num, page);
			if (ret != 1) {
				fprintf(stderr, "Fail to write to input file\n");
				exit(-1);
			}

			buf_idx += end_idx - start_idx + 1;
		}

		if (fsize < offset + size)
			fsize = offset + size;

		return buf_idx;
	}

	return 0;

}
/* This function is called on last close of file */
int myFuse_release(const char *path UNUSED, struct fuse_file_info *fi)
{
	int ret = close(fi->fh);
	if (ret < 0) {
		dprintf("close failed: Ret %d, Error %s\n", ret,
				strerror(errno));
		return -errno;
	}

	return 0;
}

/*
 * This is called on each close of file - Such as when a child process
 * closes a file it inherented
 */
int myFuse_flush(const char *path UNUSED, struct fuse_file_info *fi UNUSED)
{
	return 0;
}

int myFuse_opendir(const char *path, struct fuse_file_info *fi)
{
	int ret = 0;
	DIR *dir;
	char rel_path[MAX_PATH_LEN];
	get_rel_path(path, rel_path);

	dir = opendir(rel_path);
	if (dir == NULL) {
		dprintf("opendir failed: Ret %d, Error %s\n", ret,
				strerror(errno));
		return -errno;
	}

	fi->fh = (uint64_t)dir;

	return ret;

}

int myFuse_readdir(const char *path UNUSED, void *buf, fuse_fill_dir_t filler,
			off_t offset UNUSED, struct fuse_file_info *fi)
{
	DIR *dir;
	struct dirent *de;
	int ret = 0;

	dir = (DIR *)fi->fh;

	while (1) {

		de = readdir(dir);

		if (de == NULL)
			break;


		if (filler(buf, de->d_name, NULL, 0) != 0) {
	    		dprintf("Readdir filler buffer full");
			ret = -ENOMEM;
			goto exit;
		}

	};

exit:
	if (ret < 0)
		dprintf("releasedir failed: Ret %d, Error %s\n", ret,
				strerror(ret));

	return ret;
}

int myFuse_releasedir(const char *path UNUSED, struct fuse_file_info *fi)
{
	DIR *dir;
	int ret;

	dir = (DIR *)fi->fh;

	ret = closedir(dir);

	if (ret < 0) {
		dprintf("releasedir failed: Ret %d, Error %s\n", ret,
				strerror(errno));

		return -errno;
	}

	return ret;

}

/* As ftruncate has not been implemented, this function is called instead */
int myFuse_ftruncate(const char *path UNUSED, off_t newsize,
			struct fuse_file_info *fi)
{
	int ret = 0;
	char rel_path[MAX_PATH_LEN];
	get_rel_path(path, rel_path);

	dprintf("In ftruncate %s\n", path);


	ret = ftruncate(fi->fh, newsize);
	if (ret < 0) {

		dprintf("ftruncate failed: Ret %d, Error %s\n", ret,
				strerror(errno));

		return -errno;

	}

	if (strcmp(path, rel_fname) == 0)
		fsize = newsize;

	return ret;

}

int myFuse_truncate(const char *path, off_t newsize)
{
	int ret = 0;
	char rel_path[MAX_PATH_LEN];
	get_rel_path(path, rel_path);

	dprintf("In truncate %s\n", rel_path);


	ret = truncate(rel_path, newsize);
	if (ret < 0) {

		dprintf("truncate failed: Ret %d, Error %s\n", ret,
				strerror(errno));

		return -errno;

	}

	if (strcmp(path, rel_fname) == 0)
		fsize = newsize;


	return ret;
}


int myFuse_unlink(const char *path)
{
	int ret = 0;
	char rel_path[MAX_PATH_LEN];
	get_rel_path(path, rel_path);

	dprintf("In unlink %s\n", rel_path);

	ret = unlink(rel_path);
	if (ret < 0) {

		dprintf("unlink failed: Ret %d, Error %s\n", ret,
				strerror(errno));

		return -errno;
	}

	if (strcmp(path, rel_fname) == 0)
		fsize = 0;

	return ret;
}

int myFuse_chmod(const char *path, mode_t mode)
{
	int ret = 0;
	char rel_path[MAX_PATH_LEN];
	get_rel_path(path, rel_path);

	ret = chmod(rel_path, mode);
	if (ret < 0) {

		dprintf("chmod failed: Ret %d, Error %s\n", ret,
				strerror(errno));

		return -errno;
	}

	return ret;

}

int myFuse_utimens(const char *path, const struct timespec tv[2])
{
	int ret = 0;
	char rel_path[MAX_PATH_LEN];
	get_rel_path(path, rel_path);

	/* Path name is relative */
	ret = utimensat(AT_FDCWD, rel_path, tv, 0);
	if (ret < 0) {

		dprintf("utimens failed: Ret %d, Error %s\n", ret,
				strerror(errno));

		return -errno;
	}

	return ret;

}

int myFuse_access(const char *path, int mode)
{
	int ret = 0;
	char rel_path[MAX_PATH_LEN];
	get_rel_path(path, rel_path);

	dprintf("In access %s\n", rel_path);

	ret = access(rel_path, mode);
	if (ret < 0) {

		dprintf("access failed: Ret %d, Error %s\n", ret,
				strerror(errno));

		return -errno;

	}
	return ret;

}

/* Even when open() is called with O_CREAT flag, FUSE calls mknod */
int myFuse_mknod(const char *path, mode_t mode, dev_t dev)
{
	int ret = 0;
	char rel_path[MAX_PATH_LEN];
	get_rel_path(path, rel_path);

	dprintf("In mknod %s\n", rel_path);

	ret = mknod(rel_path, mode, dev);
	if (ret < 0) {
		dprintf("mknod failed: Ret %d, Error %s\n", ret,
				strerror(errno));

		return -errno;
	}

	return ret;
}

/*
 * Functions supported by myFuse
 */
static
struct fuse_operations myFuse_operations;

/**
 * @brief Initializes the flash simulator
 * @param conf_file Configuration file for the simulator
 * @param fname Data file on which the configuratoin file works on
 * @return 0 on success, < 0 on error
 */
int initialize_flashsim(char *conf_file, char *fname, char *log_fname)
{
	class datastore_page_t page;
	int page_count = 0;
	int ret;
	FILE *fp;
	struct stat stats;

	init_flashsim();
	sim = new FlashSimTest(conf_file);

	fp = fopen(fname, "r");
	if (fp == NULL) {
		fprintf(stderr, "Couldn't open input file %s\n", fname);
		exit(-1);
	}

	ret = stat(fname, &stats);
	if (ret < 0) {
		fprintf(stderr, "Couldn't fetch input file's stat %s\n", fname);
		exit(-1);
	}

	fsize = stats.st_size;

	log_fp = fopen(log_fname, "w+");
	if (log_fp == NULL) {
		fprintf(stderr, "Couldn't open log file %s\n", log_fname);
		exit(-1);
	}

	/* We are not closing this file, so lets not buffer it */
	setbuf(log_fp, NULL);

	/* Read whole file into the simulator */
	while (1) {

		size_t rbytes;

		rbytes = fread(page.buf, 1, sizeof(page.buf), fp);

		if (rbytes != sizeof(page.buf))
			memset(&page.buf[rbytes], 0, sizeof(page.buf) - rbytes);

		ret = sim->Write(log_fp, page_count, page);
		if (ret != 1) {
			fprintf(stderr, "Couldn't read in the input file\n");
			exit(-1);
		}

		page_count++;

		if (rbytes == 0)
			break;
	};

	fclose(fp);

	return 0;
}

/*
 * Call like ./myFuse -c [abs conf file] -f [filename abs path] -m [mount point]
 * -s [ref point] -l [log file] -d [debug level]
 *
 * Run fuse in single threaded mode for ease of development
 * XXX: Does this slow down the application?
 */
int main(int argc, char *argv[]) {

	char c;
	int ret;

	/* Absolute conf file path */
	char *abs_conf_file;
	char *fuse_argv[5];
	char *log_file = NULL;

	while ((c = getopt (argc, argv, "f:m:c:s:l:d:")) != -1) {

		switch (c) {

		case 'f':
			abs_fname = optarg;

			rel_fname = abs_fname + strlen(abs_fname);

			/* rel_fname contains just the file name, with a '/' */
			while (*rel_fname != '/')
				rel_fname--;

	        	break;
		case 'm':
			abs_mount_path = optarg;
			break;
		case 's':
			abs_ref_path = optarg;
			break;
		case 'c':
			abs_conf_file = optarg;
			break;
		case 'l':
			log_file = optarg;
			break;

		case 'd':
			if (atoi(optarg) != 0)
				is_debug_enabled = 1;
			break;
		default:
			printf("Unknown argument %c\n", c);
			exit(-1);
			break;
		}
	}

	if (abs_conf_file == NULL || abs_fname == NULL ||
		abs_mount_path == NULL || abs_ref_path == NULL ||
		log_file == NULL) {

		fprintf(stderr, "Invalid arguments\n");
		fprintf(stderr, "Usage: myFuse -c <abs conf file path>"
				" -f <abs ref file path>"
				" -m <abs mount dir path >"
				" -s <abs ref dir path >"
				" -l <abs log file path>"
				" -d <0-No Debug, 1-Debug Logs on stdout>\n");
		exit(-1);
	}

	dprintf("Conf File %s, File name %s, Mount point %s, Ref point %s,"
		"Log file %s\n",abs_conf_file, abs_fname, abs_mount_path,
			abs_ref_path, log_file);

	ret = initialize_flashsim(abs_conf_file, abs_fname, log_file);
	if (ret < 0)
		return ret;

	myFuse_operations.getattr	= myFuse_getattr;
	myFuse_operations.mknod		= myFuse_mknod;
	myFuse_operations.mkdir		= myFuse_mkdir;
	myFuse_operations.unlink	= myFuse_unlink;
	myFuse_operations.rmdir		= myFuse_rmdir;
	myFuse_operations.chmod		= myFuse_chmod;
	myFuse_operations.truncate	= myFuse_truncate;
	myFuse_operations.open		= myFuse_open;
	myFuse_operations.read		= myFuse_read;
	myFuse_operations.write		= myFuse_write;
	myFuse_operations.flush		= myFuse_flush;
	myFuse_operations.release	= myFuse_release;
	myFuse_operations.setxattr	= myFuse_setxattr;
	myFuse_operations.getxattr	= myFuse_getxattr;
	myFuse_operations.opendir	= myFuse_opendir;
	myFuse_operations.readdir	= myFuse_readdir;
	myFuse_operations.init          = myFuse_init;
	myFuse_operations.destroy       = myFuse_destroy;
	myFuse_operations.access	= myFuse_access;
	myFuse_operations.ftruncate	= myFuse_ftruncate;
	myFuse_operations.utimens	= myFuse_utimens;


	fuse_argv[0] = argv[0];
	fuse_argv[1] = "-s";
	fuse_argv[2] = "-f";
	fuse_argv[3] = abs_mount_path;
	fuse_argv[4] = NULL;

	return fuse_main(4, fuse_argv, &myFuse_operations, NULL);
}
