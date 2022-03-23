/* Sample IIO sensors test code.
 *
 * Copyright (c) 2021 STMicroelectronics
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is primarily intended as an example application.
 * Reads the current buffer setup from sysfs and starts a short capture
 * from the specified device, pretty printing the result after appropriate
 * conversion.
 */

#include <unistd.h>
#include <stdlib.h>
#include <dirent.h>
#include <fcntl.h>
#include <stdio.h>
#include <errno.h>
#include <sys/stat.h>
#include <time.h>
#include <linux/types.h>
#include <string.h>
#include <poll.h>
#include <endian.h>
#include <getopt.h>
#include <inttypes.h>
#include <math.h>
#include <pthread.h>
#include <time.h>

#include "iio_utils.h"
#include "lib/motion_fx.h"
#include "lib/timer_lib.h"

#define IIO_M_S_2_TO_G(ms2)     (((ms2) / 9.80665))
#define IIO_RAD_TO_DEGREE(rad)  (((rad) * 57.2958))
#define IIO_mG_TO_uT_50(mG)     (((mG) * 0.002))

#define MAX_NUM_THRED		10

/* motion FX configuration */
#define DEFAULT_ATIME       (0.889f)
#define DEFAULT_MTIME       (2.667f)
#define DEFAULT_FRTIME      (0.667f)
#define MFX_STR_LENG        35

/* timer configuration */
#define CLOCKID             CLOCK_REALTIME
#define SIG                 SIGRTMIN

/**
 * sensor type
 */
enum {
    SENSOR_TYPE_ACCELEROMETER = 1,
    SENSOR_TYPE_MAGNETIC_FIELD = 2,
    SENSOR_TYPE_ORIENTATION = 3,
    SENSOR_TYPE_GYROSCOPE = 4,
    SENSOR_TYPE_PRESSURE = 6,
    SENSOR_TYPE_TEMPERATURE = 7,
};

/**
 * enum autochan - state for the automatic channel enabling mechanism
 */
enum autochan {
	AUTOCHANNELS_DISABLED,
	AUTOCHANNELS_ENABLED,
	AUTOCHANNELS_ACTIVE,
};

/**
 * struct test_thread_t - thread env
 */
typedef struct {
	unsigned long num_loops;
	unsigned long timedelay;
	unsigned long buf_len;
	enum autochan autochannels;
	char *device_name;
} test_thread_t;

static char lib_version[MFX_STR_LENG];
static MFX_knobs_t iKnobs;
static MFXState_t mfxState_pt = NULL;
static MFX_output_t data_out;
static MFX_input_t data_in;
static MFX_MagCal_input_t mag_data_in;
static MFX_MagCal_output_t mdata_out;
static int enable_motion_fx;
static pthread_mutex_t lock;
static int show_sensor_data;
static int acc_gyro_mag;
static float dT;
static int select_motionfx_algo = MFX_ENGINE_6X;

/**
 * init_motionFX() - initialize motionFX library
 **/
int init_motionFX(int algo_6x_9x, int mag_sampletime)
{
    size_t stateSize;
    
    stateSize = MotionFX_GetStateSize();
    mfxState_pt = (MFXState_t*)malloc(stateSize);
    if (!mfxState_pt) {
        printf("unable to allocate memory\n");

        return -ENOMEM;
    }
    
    /* Sensor Fusion API initialization function */
    MotionFX_initialize(mfxState_pt);
    MotionFX_GetLibVersion(lib_version);
    printf("MotionFX lib version: %s\n", lib_version);
    
    MotionFX_getKnobs(mfxState_pt, &iKnobs);

    iKnobs.ATime = DEFAULT_ATIME;
    iKnobs.MTime = DEFAULT_MTIME;
    iKnobs.FrTime = DEFAULT_FRTIME;

    /* enable gyro calibration mode 1 */
    iKnobs.LMode = 1;
    iKnobs.gbias_acc_th_sc = 0.005148491;
    iKnobs.gbias_gyro_th_sc = 0.01948056503;
    iKnobs.modx = 1;

    if (algo_6x_9x == MFX_ENGINE_9X) {
        MotionFX_MagCal_init(mag_sampletime, 1);
        iKnobs.gbias_mag_th_sc = 0.00000046;
        iKnobs.output_type = MFX_ENGINE_OUTPUT_ENU;
    }

    /* Modify knobs settings & set the knobs */
    MotionFX_setKnobs(mfxState_pt, &iKnobs);
    
    if (algo_6x_9x == MFX_ENGINE_9X) {
        /* Enable 9-axis sensor fusion */
        MotionFX_enable_9X(mfxState_pt, MFX_ENGINE_ENABLE);
    } else {
        /* Enable 6-axis sensor fusion */
        MotionFX_enable_6X(mfxState_pt, MFX_ENGINE_ENABLE);
    }

    return 0;
}

/**
 * motionfx_algo() - run motionFX library
 **/
void motionfx_algo(size_t timer_id, void *user_data)
{
    pthread_mutex_lock(&lock);
    if (select_motionfx_algo == MFX_ENGINE_9X) {
        MotionFX_MagCal_run(&mag_data_in);
        MotionFX_MagCal_getParams(&mdata_out);
        if (mdata_out.cal_quality > 0) {
            data_in.mag[0] = mag_data_in.mag[0] - mdata_out.hi_bias[0];
            data_in.mag[1] = mag_data_in.mag[1] - mdata_out.hi_bias[1];
            data_in.mag[2] = mag_data_in.mag[2] - mdata_out.hi_bias[2];
        } else {
            data_in.mag[0] = mag_data_in.mag[0];
            data_in.mag[1] = mag_data_in.mag[1];
            data_in.mag[2] = mag_data_in.mag[2];
        }
    }

    MotionFX_propagate(mfxState_pt, &data_out, &data_in, &dT);
    MotionFX_update(mfxState_pt, &data_out, &data_in, &dT, NULL);
    pthread_mutex_unlock(&lock);
}

/**
 * motionfx_show() - show motionFX output
 **/
void motionfx_show(size_t timer_id, void *user_data)
{
    float gbias[3];

    pthread_mutex_lock(&lock);

    /* run sensor fusion algorithm */
    MotionFX_getGbias(mfxState_pt, gbias);
    if (select_motionfx_algo == MFX_ENGINE_6X) {
        printf("q(%05f,%05f,%05f,%05f), gb(%05f,%05f,%05f), a(%05f,%05f,%05f), g(%05f,%05f,%05f)\n",
                data_out.quaternion[0], 
                data_out.quaternion[1],
                data_out.quaternion[2],
                data_out.quaternion[3],
                gbias[0], gbias[1], gbias[2],
                data_in.acc[0], data_in.acc[1], data_in.acc[2],
                data_in.gyro[0], data_in.gyro[1], data_in.gyro[2]);
    } else {
        printf("q(%05f,%05f,%05f,%05f), gb(%05f,%05f,%05f), mc(%05f %05f %05f), a(%05f,%05f,%05f), g(%05f,%05f,%05f), mt(%d), m(%05f,%05f,%05f)\n",
                data_out.quaternion[0], 
                data_out.quaternion[1],
                data_out.quaternion[2],
                data_out.quaternion[3],
                gbias[0], gbias[1], gbias[2],
                mdata_out.hi_bias[0], mdata_out.hi_bias[1], mdata_out.hi_bias[2],
                data_in.acc[0], data_in.acc[1], data_in.acc[2],
                data_in.gyro[0], data_in.gyro[1], data_in.gyro[2],
                mag_data_in.time_stamp,
                data_in.mag[0], data_in.mag[1], data_in.mag[2]);
    }
    pthread_mutex_unlock(&lock);
}

/**
 * size_from_channelarray() - calculate the storage size of a scan
 * @channels:		the channel info array
 * @num_channels:	number of channels
 *
 * Has the side effect of filling the channels[i].location values used
 * in processing the buffer output.
 **/
static int size_from_channelarray(struct iio_channel_info *channels, int num_channels)
{
	int bytes = 0;
	int i = 0;

	while (i < num_channels) {
		if (bytes % channels[i].bytes == 0)
			channels[i].location = bytes;
		else
			channels[i].location = bytes - bytes % channels[i].bytes
					       + channels[i].bytes;

		bytes = channels[i].location + channels[i].bytes;
		i++;
	}

	return bytes;
}

static void print1byte(uint8_t input, struct iio_channel_info *info)
{
	/*
	 * Shift before conversion to avoid sign extension
	 * of left aligned data
	 */
	input >>= info->shift;
	input &= info->mask;
	if (info->is_signed) {
		int8_t val = (int8_t)(input << (8 - info->bits_used)) >>
			     (8 - info->bits_used);
		if (show_sensor_data)
            printf("%05f ", ((float)val + info->offset) * info->scale);
	} else {
		if (show_sensor_data)
            printf("%05f ", ((float)input + info->offset) * info->scale);
	}
}

static void print2byte(uint16_t input, struct iio_channel_info *info, float *fval)
{
	/* First swap if incorrect endian */
	if (info->be)
		input = be16toh(input);
	else
		input = le16toh(input);

	/*
	 * Shift before conversion to avoid sign extension
	 * of left aligned data
	 */
	input >>= info->shift;
	input &= info->mask;
	if (info->is_signed) {
		int16_t val = (int16_t)(input << (16 - info->bits_used)) >>
			      (16 - info->bits_used);

		if (show_sensor_data)
            printf("%05f ", ((float)val + info->offset) * info->scale);

		*fval = ((float)val + info->offset) * info->scale;
	} else {
        if (show_sensor_data)
            printf("%05f ", ((float)input + info->offset) * info->scale);

		*fval = ((float)input + info->offset) * info->scale;
	}
}

static void print4byte(uint32_t input, struct iio_channel_info *info)
{
	/* First swap if incorrect endian */
	if (info->be)
		input = be32toh(input);
	else
		input = le32toh(input);

	/*
	 * Shift before conversion to avoid sign extension
	 * of left aligned data
	 */
	input >>= info->shift;
	input &= info->mask;
	if (info->is_signed) {
		int32_t val = (int32_t)(input << (32 - info->bits_used)) >>
			      (32 - info->bits_used);

		if (show_sensor_data)
            printf("%05f ", ((float)val + info->offset) * info->scale);
	} else {
		if (show_sensor_data)
            printf("%05f ", ((float)input + info->offset) * info->scale);
	}
}

static void print8byte(uint64_t input, struct iio_channel_info *info)
{
	/* First swap if incorrect endian */
	if (info->be)
		input = be64toh(input);
	else
		input = le64toh(input);

	/*
	 * Shift before conversion to avoid sign extension
	 * of left aligned data
	 */
	input >>= info->shift;
	input &= info->mask;
	if (info->is_signed) {
		int64_t val = (int64_t)(input << (64 - info->bits_used)) >>
			      (64 - info->bits_used);
		/* special case for timestamp */
		if (info->scale == 1.0f && info->offset == 0.0f) {
			if (show_sensor_data)
                printf("%" PRId64 " ", val);
		} else {
			if (show_sensor_data)
                printf("%05f ",
			       ((float)val + info->offset) * info->scale);
		}
	} else {
		if (show_sensor_data)
            printf("%05f ", ((float)input + info->offset) * info->scale);
	}
}

/**
 * process_scan() - print out the values in SI units
 * @data:		    pointer to the start of the scan
 * @channels:	    information about the channels.
 *	                Note: size_from_channelarray must have been
 *			        called first to fill the location offsets.
 * @num_channels:	number of channels
 * @type:           sensor type
 **/
static void process_scan(char *data, struct iio_channel_info *channels,
                         int num_channels, int type)
{
	int k;
	float val[3];
	struct timespec start;
    unsigned long long seconds;
	unsigned long long ns;

	clock_gettime(CLOCK_MONOTONIC, &start);

	seconds = start.tv_sec;
	ns = start.tv_nsec;

	for (k = 0; k < num_channels; k++) {
		switch (channels[k].bytes) {
			/* only a few cases implemented so far */
		case 1:
			print1byte(*(uint8_t *)(data + channels[k].location),
				   &channels[k]);
			break;
		case 2:
			print2byte(*(uint16_t *)(data + channels[k].location),
				   &channels[k], &val[k]);
            if (type == SENSOR_TYPE_ACCELEROMETER) {
                data_in.acc[k] = IIO_M_S_2_TO_G(val[k]);
                acc_gyro_mag |= SENSOR_TYPE_ACCELEROMETER;
            }
            else if (type == SENSOR_TYPE_GYROSCOPE) {
                data_in.gyro[k] = IIO_RAD_TO_DEGREE(val[k]);
                acc_gyro_mag |= SENSOR_TYPE_GYROSCOPE;
            }
            else if (type == SENSOR_TYPE_MAGNETIC_FIELD) {
                mag_data_in.mag[k] = IIO_mG_TO_uT_50(val[k]);
                mag_data_in.time_stamp = seconds * 1000 + ns / 1000000;
                acc_gyro_mag |= SENSOR_TYPE_MAGNETIC_FIELD;
            }
			break;
		case 4:
			print4byte(*(uint32_t *)(data + channels[k].location),
				   &channels[k]);
			break;
		case 8:
			print8byte(*(uint64_t *)(data + channels[k].location),
				   &channels[k]);
			break;
		default:
			break;
		}

    }

	if (show_sensor_data)
        printf("\n");
}

static int enable_disable_all_channels(char *dev_dir_name, int enable)
{
	const struct dirent *ent;
	char scanelemdir[256];
	DIR *dp;
	int ret;

	snprintf(scanelemdir, sizeof(scanelemdir),
		 FORMAT_SCAN_ELEMENTS_DIR, dev_dir_name);
	scanelemdir[sizeof(scanelemdir)-1] = '\0';

	dp = opendir(scanelemdir);
	if (!dp) {
		fprintf(stderr, "Enabling/disabling channels: can't open %s\n",
			scanelemdir);
		return -EIO;
	}

	ret = -ENOENT;
	while (ent = readdir(dp), ent) {
		if (iioutils_check_suffix(ent->d_name, "_en")) {
			printf("%sabling: %s\n",
			       enable ? "En" : "Dis",
			       ent->d_name);
			ret = write_sysfs_int(ent->d_name, scanelemdir,
					      enable);
			if (ret < 0)
				fprintf(stderr, "Failed to enable/disable %s\n",
					ent->d_name);
		}
	}

	if (closedir(dp) == -1) {
		perror("Enabling/disabling channels: "
		       "Failed to close directory");
		return -errno;
	}
	return 0;
}

static int get_type_from_name(char *device_name)
{
    char *tmp;

    tmp = strstr(device_name, "_accel");
    if (tmp)
        return SENSOR_TYPE_ACCELEROMETER;
    tmp = strstr(device_name, "_gyro");
    if (tmp)
        return SENSOR_TYPE_GYROSCOPE;
    tmp = strstr(device_name, "_magn");
    if (tmp)
        return SENSOR_TYPE_MAGNETIC_FIELD;

    return -1;
}

static void *test_loop(void *tenv)
{
	test_thread_t *test_thread = (test_thread_t *)tenv;
	struct iio_channel_info *channels;
	char *dev_dir_name, *buf_dir_name;
	enum autochan autochannels;
	int ret, i, toread, fp;
	char *buffer_access;
	ssize_t read_size;
	int num_channels;
	unsigned long j;
	int scan_size;
	int dev_num;
	char *data;
    int type;

	autochannels = test_thread->autochannels;

	/* Find the device requested */
	dev_num = find_type_by_name(test_thread->device_name, "iio:device");
	if (dev_num < 0) {
		fprintf(stderr, "Failed to find the %s\n", test_thread->device_name);

		return NULL;
	}

    type = get_type_from_name(test_thread->device_name);

	printf("iio device number being used is %d type %d\n", dev_num, type);
	ret = asprintf(&dev_dir_name, "%siio:device%d", iio_dir, dev_num);
	if (ret < 0)
		return NULL;

	/*
	 * Parse the files in scan_elements to identify what channels are
	 * present
	 */
	ret = build_channel_array(dev_dir_name, &channels, &num_channels);
	if (ret) {
		fprintf(stderr, "Problem reading scan element information\n"
			"diag %s\n", dev_dir_name);
		goto error_disable_channels;
	}

	if (num_channels && autochannels == AUTOCHANNELS_ENABLED) {
		fprintf(stderr, "Auto-channels selected but some channels "
			"are already activated in sysfs\n");
		fprintf(stderr, "Proceeding without activating any channels\n");
	}

	if (!num_channels && autochannels == AUTOCHANNELS_ENABLED) {
		fprintf(stderr,
			"No channels are enabled, enabling all channels\n");

		ret = enable_disable_all_channels(dev_dir_name, 1);
		if (ret) {
			fprintf(stderr, "Failed to enable all channels\n");
			goto error_disable_channels;
		}

		/* This flags that we need to disable the channels again */
		autochannels = AUTOCHANNELS_ACTIVE;

		ret = build_channel_array(dev_dir_name, &channels,
					  &num_channels);
		if (ret) {
			fprintf(stderr, "Problem reading scan element "
				"information\n"
				"diag %s\n", dev_dir_name);
			goto error_disable_channels;
		}
		if (!num_channels) {
			fprintf(stderr, "Still no channels after "
				"auto-enabling, giving up\n");
			goto error_disable_channels;
		}
	}

	if (!num_channels && autochannels == AUTOCHANNELS_DISABLED) {
		fprintf(stderr,
			"No channels are enabled, we have nothing to scan.\n");
		fprintf(stderr, "Enable channels manually in "
			FORMAT_SCAN_ELEMENTS_DIR
			"/*_en or pass -a to autoenable channels and "
			"try again.\n", dev_dir_name);
		ret = -ENOENT;
		goto error_disable_channels;
	}

	/*
	 * Construct the directory name for the associated buffer.
	 * As we know that the lis3l02dq has only one buffer this may
	 * be built rather than found.
	 */
	ret = asprintf(&buf_dir_name,
		       "%siio:device%d/buffer", iio_dir, dev_num);
	if (ret < 0) {
		ret = -ENOMEM;
		goto error_free_channels;
	}

	/* Setup ring buffer parameters */
	ret = write_sysfs_int("length", buf_dir_name, test_thread->buf_len);
	if (ret < 0)
		goto error_free_buf_dir_name;

	/* Enable the buffer */
	ret = write_sysfs_int("enable", buf_dir_name, 1);
	if (ret < 0) {
		fprintf(stderr,
			"Failed to enable buffer: %s\n", strerror(-ret));
		goto error_free_buf_dir_name;
	}

	scan_size = size_from_channelarray(channels, num_channels);
	data = malloc(scan_size * test_thread->buf_len);
	if (!data) {
		ret = -ENOMEM;
		goto error_free_buf_dir_name;
	}

	ret = asprintf(&buffer_access, "/dev/iio:device%d", dev_num);
	if (ret < 0) {
		ret = -ENOMEM;
		goto error_free_data;
	}

	/* Attempt to open non blocking the access dev */
	fp = open(buffer_access, O_RDONLY | O_NONBLOCK);
	if (fp == -1) { /* TODO: If it isn't there make the node */
		ret = -errno;
		fprintf(stderr, "Failed to open %s\n", buffer_access);
		goto error_free_buffer_access;
	}

	for (j = 0; j < test_thread->num_loops; j++) {
		struct pollfd pfd = {
			.fd = fp,
			.events = POLLIN,
		};

		ret = poll(&pfd, 1, -1);
		if (ret < 0) {
			ret = -errno;
			goto error_close_buffer_access;
		} else if (ret == 0) {
			continue;
		}

		toread = test_thread->buf_len;

		read_size = read(fp, data, toread * scan_size);
		if (read_size < 0) {
			if (errno == EAGAIN) {
				fprintf(stderr, "nothing available\n");
				continue;
			} else {
				break;
			}
		}

        pthread_mutex_lock(&lock);
        if (show_sensor_data)
            printf("Sensor %s ", test_thread->device_name);
		for (i = 0; i < read_size / scan_size; i++) {
			process_scan(data + scan_size * i, channels,
                         num_channels, type);

        }
        pthread_mutex_unlock(&lock);
	}

	/* Stop the buffer */
	ret = write_sysfs_int("enable", buf_dir_name, 0);
	if (ret < 0)
		goto error_close_buffer_access;

error_close_buffer_access:
	if (close(fp) == -1)
		perror("Failed to close buffer");

error_free_buffer_access:
	free(buffer_access);

error_free_data:
	free(data);

error_free_buf_dir_name:
	free(buf_dir_name);

error_free_channels:
	for (i = num_channels - 1; i >= 0; i--) {
		free(channels[i].name);
		free(channels[i].generic_name);
	}
	free(channels);

error_disable_channels:
	if (autochannels == AUTOCHANNELS_ACTIVE) {
		ret = enable_disable_all_channels(dev_dir_name, 0);
		if (ret)
			fprintf(stderr, "Failed to disable all channels\n");
	}

	if (dev_dir_name)
		free(dev_dir_name);

	return NULL;
}

static void print_usage(void)
{
	fprintf(stderr, "Usage: iio_test_sensors [options] <iio_device_list>\n"
		"Capture, convert and output data from IIO device buffer\n"
		"  -a\t\tAuto-activate all available channels\n"
		"  -c <n>\tDo n conversions\n"
		"  -l <n>\tSet buffer length to n samples\n"
		"  -x <dT>\tTest MotionFX with delta time dT in ms\n"
		"  -m <ms>\tMag sample time in ms (default 10)\n"
		"  -x <dT>\tTest MotionFX with delta time dT in ms\n"
		"  -o <ms>\tShow MotionFX output with rate (default 1000 ms)\n"
		"  -g <ms>\tSelect Motion FX algo (0 - 6x, 1 = 9x, default 0)\n"
		"  -s\t\tShow sensor data output\n"
		"  -w <n>\tSet delay between reads in us (event-less mode)\n");
}

int main(int argc, char **argv)
{
	unsigned long num_loops = 2;
	unsigned long timedelay = 1000000;
	unsigned long buf_len = 128;
    int run_motionfx = 0;
    int run_motionfx_show = 1000;
	test_thread_t test_thread[MAX_NUM_THRED];
	enum autochan autochannels = AUTOCHANNELS_DISABLED;
	pthread_t tid[MAX_NUM_THRED];
    size_t timer_motionfx = -1, timer_showmotion = -1;
    int mag_sample_time = 10;

	char *device_name[MAX_NUM_THRED] = { NULL };
	int c, ret, index, devnum = 0;
	char *dummy;

	while ((c = getopt(argc, argv, "ac:l:w:sx:o:g:m:")) != -1) {
		switch (c) {
		case 'a':
			autochannels = AUTOCHANNELS_ENABLED;
			break;
		case 'c':
			errno = 0;
			num_loops = strtoul(optarg, &dummy, 10);
			if (errno)
				return -errno;

			break;
		case 'l':
			errno = 0;
			buf_len = strtoul(optarg, &dummy, 10);
			if (errno)
				return -errno;

			break;
		case 'w':
			errno = 0;
			timedelay = strtoul(optarg, &dummy, 10);
			if (errno)
				return -errno;
			break;
        case 'x':
            run_motionfx = atoi(optarg);
            break;
        case 'o':
            run_motionfx_show = atoi(optarg);
            break;
        case 'm':
            mag_sample_time = atoi(optarg);
            break;
        case 'g':
            select_motionfx_algo = atoi(optarg);
            break;
        case 's':
            show_sensor_data = 1;
            break;
		case '?':
			print_usage();
			return -1;
		}
	}

	for (index = optind; index < argc; index++) {
		device_name[devnum] = argv[index];
		devnum++;
	}

	if (!devnum) {
		fprintf(stderr, "Device name not set\n");
		print_usage();

		return -1;
	}

    if (pthread_mutex_init(&lock, NULL) != 0) {
        fprintf(stderr, "pthread_mutex_init fails\n");

        return 1;
    }

	for (index = 0; index < devnum; index++) {
		printf ("running test on device %s\n", device_name[index]);
		test_thread[index].num_loops = num_loops;
		test_thread[index].buf_len = buf_len;
		test_thread[index].timedelay = timedelay;
		test_thread[index].autochannels = autochannels;
		test_thread[index].device_name = device_name[index];

		pthread_create(&tid[index], NULL, test_loop, (void *)&test_thread[index]);
	}
    
    if (run_motionfx) {
        dT = (float)((float)run_motionfx / 1000.0f);

        printf("Init motionFX with algo %s\n",
                select_motionfx_algo == MFX_ENGINE_6X ? "6X" : "9X");
        ret = init_motionFX(select_motionfx_algo, mag_sample_time);
        if (ret) {
            fprintf(stderr, "init_motionFX fails\n");
            ret = -1;

            goto exit_test;
        }

        enable_motion_fx = 1;
        timer_initialize();
        timer_motionfx = start_timer(run_motionfx, CLOCK_REALTIME,
                                     motionfx_algo, TIMER_PERIODIC,
                                     (void *)&dT);
        timer_showmotion = start_timer(run_motionfx_show, CLOCK_REALTIME,
                                     motionfx_show, TIMER_PERIODIC,
                                     NULL);
    }

	pthread_exit(NULL);

exit_test:
    stop_timer(timer_motionfx);
    stop_timer(timer_showmotion);
    timer_finalize();

	return ret;
}

