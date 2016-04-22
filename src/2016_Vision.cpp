/**************************************************************************
 *   framegrabber Version 0.1                                              *
 *   Copyright (C) 2013 by Matthew Witherwax (lemoneer)                    *
 *   lemoneer@outlook.com                                                  *
 *   blog.lemoneerlabs.com                                                 *
 *                                                                         *
 *   based on V4L2 Specification, Appws_endix B: Video Capture Example        *
 *   (http://linuxtv.org/downloads/v4l-dvb-apis/capture-example.html)      *
 *   and work by Matthew Witherwax on v4l2grab                             *
 *   (https://github.com/twam/v4l2grab)                                    *
 ***************************************************************************
 ***************************************************************************
 BSD LICENSE

 Copyright (c) 2013, Matthew Witherwax
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in
 the documentation and/or other materials provided with the
 distribution.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ***************************************************************************/

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <assert.h>
#include <getopt.h>             /* getopt_long() */
#include <fcntl.h>              /* low-level i/o */
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <time.h>

#include <linux/videodev2.h>
#include <opencv2/opencv.hpp>
//#include "opencv2/core/core_c.h"
//#include "opencv2/highgui/highgui.hpp"

#include "ntcore.h"
#include "tables/ITableListener.h"
#include "networktables/NetworkTable.h"


using namespace cv;
using namespace std;

#define CLEAR(x) memset(&(x), 0, sizeof(x))

//clock_t ws_begin, ws_end;
//double time_spent;
RNG rng(12345);

std::shared_ptr<NetworkTable> table;

enum io_method {
	IO_METHOD_READ, IO_METHOD_MMAP, IO_METHOD_USERPTR,
};

struct buffer {
	void *start;
	size_t length;
};

static char *dev_name = "/dev/video0";
static enum io_method io = IO_METHOD_MMAP;
static int fd = -1;
struct buffer *buffers;
static unsigned int n_buffers;
static int out_buf;
static int frame_count = 1;
static int set_format;
static unsigned int width = 640;
static unsigned int height = 480;
static unsigned int fps = 30;
static unsigned int timeout = 1;
static unsigned int timeouts_max = 1;
static char *out_name = "capture.jpg";

static bool NODASHBOARD = false;
static int NUM_AVERAGES = 5;
static int BRIGHTNESS = 30;
static int CONTRAST = 5;
static int SATURATION = 200;
static int H_MIN = 60;
static int H_MAX = 90;
static int S_MIN = 45;
static int S_MAX = 256;
static int V_MIN = 0;
static int V_MAX = 256;
static int TIME_BETWEEN_CAPTURES = 10;

int m_H_MIN = 60;
int m_S_MIN = 45;
int m_V_MIN = 0;

int m_H_MAX = 90;
int m_S_MAX = 255;
int m_V_MAX = 255;

/* Allowed formats: V4L2_PIX_FMT_YUYV, V4L2_PIX_FMT_MJPEG, V4L2_PIX_FMT_H264
 *  The default will not be used unless the width and/or height is specified
 *  but the user does not specify a pixel format */
static unsigned int pixel_format = V4L2_PIX_FMT_MJPEG;

/* Signal Handling
 * Clean up on Ctrl-C as opposed to leaving
 * the device in an inconsistent state*/
static int s_interrupted = 0;
static void s_signal_handler(int signal_value) {
	s_interrupted = 1;
}

static void s_catch_signals(void) {
	struct sigaction action;
	action.sa_handler = s_signal_handler;
	action.sa_flags = 0;
	sigemptyset(&action.sa_mask);
	sigaction(SIGINT, &action, NULL);
	sigaction(SIGTERM, &action, NULL);
}

static void errno_exit(const char *s) {
	fprintf(stderr, "%s error %d, %s\n", s, errno, strerror(errno));
	exit(EXIT_FAILURE);
}

static int xioctl(int fh, int request, void *arg) {
	int r;

	do {
		r = ioctl(fh, request, arg);
	} while (-1 == r && EINTR == errno);

	return r;
}

static int countP = 0;
static int cc = 0;

static void toHSV(Mat src, Mat dst) {
	cvtColor(src, dst, COLOR_BGR2HSV);
}

static void ws_process(Mat img) {

	// Make an empty matrix with our image for hsv
	Mat hsvMat(img.size(), img.type());

	toHSV(img, hsvMat); // put the currentImg into hsv format

	double find_rectangles_time ;
	double draw_largest_time ;

	Mat threshMat; // = new Mat(); // Will be the image matrix for the binary
	// values (black and white image)

	Mat lower;
	Mat upper;


	inRange(hsvMat, Scalar(m_H_MIN, m_S_MIN, m_V_MIN), Scalar(m_H_MAX, m_S_MAX, m_V_MAX), threshMat);

	// Mat threshold_output;
	vector < Vec4i > hierarchy;
	vector < vector<Point> > contours;

	findContours(threshMat, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	int targetBottom = 0;
	int targetCenter = 0;

      if (contours.size() > 0)
      {
         Rect target = boundingRect(contours[0]);
         double c_width = target.width; // get width of the contour
         int biggestContourIndex = 0;

         for (int i = 1; i < contours.size() - 1; i++)
         {
            // Iterate through all the contours
            Rect newRect = boundingRect(contours[i]);

            double new_c_width = newRect.width;

            if (new_c_width > c_width)
            {
               // find the contour with the biggest width (Probably the target)
               target = newRect;
               biggestContourIndex = i;
               c_width = new_c_width;
            }
         }

      	targetBottom = target.y;
        targetCenter = target.x + (target.width / 2);


		// Just Draw Blue for now
//         drawContours(img, contours, -1, Scalar(255, 0, 0), 4);
//         rectangle(img, Point(target.x, target.y), Point(target.x + target.width, target.y + target.height), Scalar(0, 255, 0), 2);
      }

		if (countP % fps == 0)
		{
//			cout << "---------------------------------------------------------" << endl;
			// Write image to file on disk
			char fn[100];
			snprintf(fn, sizeof fn, "images/image%02d.jpg", countP);
			//imwrite(fn, img);
		}

         hsvMat.release();
	threshMat.release();
	
	table->PutNumber("Target Bottom", targetBottom);
	table->PutNumber("Target Center", targetCenter);
//      ((RemoteAnalogOutput) Core.getOutputManager().getOutput(BBBOutputs.TARGET_BOTTOM.getName())).setValue(targetBottom);
//      ((RemoteAnalogOutput) Core.getOutputManager().getOutput(BBBOutputs.TARGET_CENTER.getName())).setValue(targetCenter);
//      ((RemoteAnalogOutput) Core.getOutputManager().getOutput(BBBOutputs.VISION_ANGLE.getName())).setValue(currentAverageAngle);
//      ((RemoteAnalogOutput) Core.getOutputManager().getOutput(BBBOutputs.VISION_DISTANCE.getName())).setValue(currentAverageDistance);
//   }

	return;
}

static void process_image(const void *p, int size) {
	if (out_buf) {
		countP += 1;
		cc += 1;
		Mat mat;
		Mat img;

		mat = Mat(height, width, CV_8UC3, (void*) p);

		// decode the image
		img = imdecode(mat, 1);

		// Process the image to get the values we need
		ws_process(img);

		// release the image
		//release(img);
	}
}

static int read_frame(void) {
	struct v4l2_buffer buf;
	unsigned int i;

	switch (io) {
	case IO_METHOD_READ:
		if (-1 == read(fd, buffers[0].start, buffers[0].length)) {
			switch (errno) {
			case EAGAIN:
				return 0;

			case EIO:
				/* Could ignore EIO, see spec. */

				/* fall through */

			default:
				errno_exit("read");
			}
		}

		process_image(buffers[0].start, buffers[0].length);
		break;

	case IO_METHOD_MMAP:
		CLEAR(buf);

		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;

		if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf)) {
			switch (errno) {
			case EAGAIN:
				return 0;

			case EIO:
				/* Could ignore EIO, see spec. */

				/* fall through */

			default:
				errno_exit("VIDIOC_DQBUF");
			}
		}

		assert(buf.index < n_buffers);

		process_image(buffers[buf.index].start, buf.bytesused);

		if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
			errno_exit("VIDIOC_QBUF");
		break;

	case IO_METHOD_USERPTR:
		CLEAR(buf);

		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_USERPTR;

		if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf)) {
			switch (errno) {
			case EAGAIN:
				return 0;

			case EIO:
				/* Could ignore EIO, see spec. */

				/* fall through */

			default:
				errno_exit("VIDIOC_DQBUF");
			}
		}

		for (i = 0; i < n_buffers; ++i)
			if (buf.m.userptr == (unsigned long) buffers[i].start
					&& buf.length == buffers[i].length)
				break;

		assert(i < n_buffers);

		process_image((void *) buf.m.userptr, buf.bytesused);

		if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
			errno_exit("VIDIOC_QBUF");
		break;
	}

	return 1;
}


static void grab_frames(void) {
	//clock_t ws_begin, ws_end;
//	double time_spent;

	unsigned int count;
	unsigned int timeout_count;

	count = frame_count;
	timeout_count = timeouts_max;

	//ws_begin = clock();

//	cout << "Convert" << '\t' <<  "threshold" << '\t' << "contours" << '\t' << "find rectangles" << '\t' << "draw" << ws_endl;

	while (true) {
		for (;;) {

//			if (countP % fps == 0)
//			{
//				//ws_end = clock();
//				//time_spent = (double) (ws_end - ws_begin) / CLOCKS_PER_SEC;
//				//cout << time_spent << "s" << ws_endl;
//				//ws_begin = clock();
//			}
			if (s_interrupted) {
				fprintf(stderr, "\nInterrupt received - aborting capture\n");
				return;
			}

			fd_set fds;
			struct timeval tv;
			int r;

			FD_ZERO(&fds);
			FD_SET(fd, &fds);

			/* Timeout. */
			tv.tv_sec = timeout;
			tv.tv_usec = 0;

			r = select(fd + 1, &fds, NULL, NULL, &tv);

			if (-1 == r) {
				if (EINTR == errno)
					continue;
				errno_exit("select");
			}

			if (0 == r) {
				if (timeout_count > 0) {
					timeout_count--;
				} else {
					fprintf(stderr, "select timeout\n");
					exit(EXIT_FAILURE);
				}
			}

			if (read_frame())
				break;
			/* EAGAIN - continue select loop. */
		}
	}
//	ws_end = clock();
//	time_spent = (double) (ws_end - ws_begin) / CLOCKS_PER_SEC;
//	fprintf(stderr, "Captured %i frames and Processed %i in %f seconds\n",
//			frame_count, cc, time_spent);
}

static void mainloop(void) {
	grab_frames();
}

static void stop_capturing(void) {
	enum v4l2_buf_type type;

	switch (io) {
	case IO_METHOD_READ:
		/* Nothing to do. */
		break;

	case IO_METHOD_MMAP:
	case IO_METHOD_USERPTR:
		type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		if (-1 == xioctl(fd, VIDIOC_STREAMOFF, &type))
			errno_exit("VIDIOC_STREAMOFF");
		break;
	}
}

static void start_capturing(void) {
	unsigned int i;
	enum v4l2_buf_type type;

	switch (io) {
	case IO_METHOD_READ:
		/* Nothing to do. */
		break;

	case IO_METHOD_MMAP:
		for (i = 0; i < n_buffers; ++i) {
			struct v4l2_buffer buf;

			CLEAR(buf);
			buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			buf.memory = V4L2_MEMORY_MMAP;
			buf.index = i;

			if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
				errno_exit("VIDIOC_QBUF");
		}
		type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		if (-1 == xioctl(fd, VIDIOC_STREAMON, &type))
			errno_exit("VIDIOC_STREAMON");
		break;

	case IO_METHOD_USERPTR:
		for (i = 0; i < n_buffers; ++i) {
			struct v4l2_buffer buf;

			CLEAR(buf);
			buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
			buf.memory = V4L2_MEMORY_USERPTR;
			buf.index = i;
			buf.m.userptr = (unsigned long) buffers[i].start;
			buf.length = buffers[i].length;

			if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
				errno_exit("VIDIOC_QBUF");
		}
		type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		if (-1 == xioctl(fd, VIDIOC_STREAMON, &type))
			errno_exit("VIDIOC_STREAMON");
		break;
	}
}

static void uninit_device(void) {
	unsigned int i;

	switch (io) {
	case IO_METHOD_READ:
		free(buffers[0].start);
		break;

	case IO_METHOD_MMAP:
		for (i = 0; i < n_buffers; ++i)
			if (-1 == munmap(buffers[i].start, buffers[i].length))
				errno_exit("munmap");
		break;

	case IO_METHOD_USERPTR:
		for (i = 0; i < n_buffers; ++i)
			free(buffers[i].start);
		break;
	}

	free(buffers);
}

static void init_read(unsigned int buffer_size) {
	buffers = calloc(1, sizeof(*buffers));

	if (!buffers) {
		fprintf(stderr, "Out of memory\n");
		exit(EXIT_FAILURE);
	}

	buffers[0].length = buffer_size;
	buffers[0].start = malloc(buffer_size);

	if (!buffers[0].start) {
		fprintf(stderr, "Out of memory\n");
		exit(EXIT_FAILURE);
	}
}

static void init_mmap(void) {
	struct v4l2_requestbuffers req;

	CLEAR(req);

	req.count = 4;
	req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req.memory = V4L2_MEMORY_MMAP;

	if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req)) {
		if (EINVAL == errno) {
			fprintf(stderr, "%s does not support "
					"memory mapping\n", dev_name);
			exit(EXIT_FAILURE);
		} else {
			errno_exit("VIDIOC_REQBUFS");
		}
	}

	if (req.count < 2) {
		fprintf(stderr, "Insufficient buffer memory on %s\n", dev_name);
		exit(EXIT_FAILURE);
	}

	buffers = calloc(req.count, sizeof(*buffers));

	if (!buffers) {
		fprintf(stderr, "Out of memory\n");
		exit(EXIT_FAILURE);
	}

	for (n_buffers = 0; n_buffers < req.count; ++n_buffers) {
		struct v4l2_buffer buf;

		CLEAR(buf);

		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;
		buf.index = n_buffers;

		if (-1 == xioctl(fd, VIDIOC_QUERYBUF, &buf))
			errno_exit("VIDIOC_QUERYBUF");

		buffers[n_buffers].length = buf.length;
		buffers[n_buffers].start = mmap(NULL /* start anywhere */, buf.length,
				PROT_READ | PROT_WRITE /* required */,
				MAP_SHARED /* recommws_ended */, fd, buf.m.offset);

		if (MAP_FAILED == buffers[n_buffers].start)
			errno_exit("mmap");
	}
}

static void init_userp(unsigned int buffer_size) {
	struct v4l2_requestbuffers req;

	CLEAR(req);

	req.count = 4;
	req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req.memory = V4L2_MEMORY_USERPTR;

	if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req)) {
		if (EINVAL == errno) {
			fprintf(stderr, "%s does not support "
					"user pointer i/o\n", dev_name);
			exit(EXIT_FAILURE);
		} else {
			errno_exit("VIDIOC_REQBUFS");
		}
	}

	buffers = calloc(4, sizeof(*buffers));

	if (!buffers) {
		fprintf(stderr, "Out of memory\n");
		exit(EXIT_FAILURE);
	}

	for (n_buffers = 0; n_buffers < 4; ++n_buffers) {
		buffers[n_buffers].length = buffer_size;
		buffers[n_buffers].start = malloc(buffer_size);

		if (!buffers[n_buffers].start) {
			fprintf(stderr, "Out of memory\n");
			exit(EXIT_FAILURE);
		}
	}
}

static void init_device(void) {
	struct v4l2_capability cap;
	struct v4l2_cropcap cropcap;
	struct v4l2_crop crop;
	struct v4l2_format fmt;
	struct v4l2_streamparm frameint;
	unsigned int min;

	if (-1 == xioctl(fd, VIDIOC_QUERYCAP, &cap)) {
		if (EINVAL == errno) {
			fprintf(stderr, "%s is no V4L2 device\n", dev_name);
			exit(EXIT_FAILURE);
		} else {
			errno_exit("VIDIOC_QUERYCAP");
		}
	}

	if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
		fprintf(stderr, "%s is no video capture device\n", dev_name);
		exit(EXIT_FAILURE);
	}

	switch (io) {
	case IO_METHOD_READ:
		if (!(cap.capabilities & V4L2_CAP_READWRITE)) {
			fprintf(stderr, "%s does not support read i/o\n", dev_name);
			exit(EXIT_FAILURE);
		}
		break;

	case IO_METHOD_MMAP:
	case IO_METHOD_USERPTR:
		if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
			fprintf(stderr, "%s does not support streaming i/o\n", dev_name);
			exit(EXIT_FAILURE);
		}
		break;
	}

	/* Select video input, video standard and tune here. */

	CLEAR(cropcap);

	cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	if (0 == xioctl(fd, VIDIOC_CROPCAP, &cropcap)) {
		crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		crop.c = cropcap.defrect; /* reset to default */

		if (-1 == xioctl(fd, VIDIOC_S_CROP, &crop)) {
			switch (errno) {
			case EINVAL:
				/* Cropping not supported. */
				break;
			default:
				/* Errors ignored. */
				break;
			}
		}
	} else {
		/* Errors ignored. */
	}

	CLEAR(fmt);

	fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (set_format) {
		fmt.fmt.pix.width = width;
		fmt.fmt.pix.height = height;
		fmt.fmt.pix.pixelformat = pixel_format;
		fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;

		if (-1 == xioctl(fd, VIDIOC_S_FMT, &fmt))
			errno_exit("VIDIOC_S_FMT");

		if (fmt.fmt.pix.pixelformat != pixel_format) {
			fprintf(stderr,
					"Libv4l didn't accept pixel format. Can't proceed.\n");
			exit(EXIT_FAILURE);
		}

		/* Note VIDIOC_S_FMT may change width and height. */
	} else {
		/* Preserve original settings as set by v4l2-ctl for example */
		if (-1 == xioctl(fd, VIDIOC_G_FMT, &fmt))
			errno_exit("VIDIOC_G_FMT");
	}

	CLEAR(frameint);

	/* Attempt to set the frame interval. */
	frameint.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	frameint.parm.capture.timeperframe.numerator = 1;
	frameint.parm.capture.timeperframe.denominator = fps;
	if (-1 == xioctl(fd, VIDIOC_S_PARM, &frameint))
		fprintf(stderr, "Unable to set frame interval.\n");

	/* Buggy driver paranoia. */
	min = fmt.fmt.pix.width * 2;
	if (fmt.fmt.pix.bytesperline < min)
		fmt.fmt.pix.bytesperline = min;
	min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
	if (fmt.fmt.pix.sizeimage < min)
		fmt.fmt.pix.sizeimage = min;

	switch (io) {
	case IO_METHOD_READ:
		init_read(fmt.fmt.pix.sizeimage);
		break;

	case IO_METHOD_MMAP:
		init_mmap();
		break;

	case IO_METHOD_USERPTR:
		init_userp(fmt.fmt.pix.sizeimage);
		break;
	}
}

static void close_device(void) {
	if (-1 == close(fd))
		errno_exit("close");

	fd = -1;
}

static void open_device(void) {
	struct stat st;

	if (-1 == stat(dev_name, &st)) {
		fprintf(stderr, "Cannot identify '%s': %d, %s\n", dev_name, errno,
				strerror(errno));
		exit(EXIT_FAILURE);
	}

	if (!S_ISCHR(st.st_mode)) {
		fprintf(stderr, "%s is no device\n", dev_name);
		exit(EXIT_FAILURE);
	}

	fd = open(dev_name, O_RDWR /* required */| O_NONBLOCK, 0);

	if (-1 == fd) {
		fprintf(stderr, "Cannot open '%s': %d, %s\n", dev_name, errno,
				strerror(errno));
		exit(EXIT_FAILURE);
	}
}

static void usage(FILE *fp, int argc, char **argv) {
	fprintf(fp, "Usage: %s [options]\n\n"
			"Version 1.0\n"
			"Options:\n"
			"-d | --device name   Video device name [%s]\n"
			"-h | --help          Print this message\n"
			"-m | --mmap          Use memory mapped buffers [default]\n"
			"-r | --read          Use read() calls\n"
			"-u | --userp         Use application allocated buffers\n"
			"-W | --width         Set image width\n"
			"-H | --height        Set image height\n"
			"-I | --interval      Set frame interval (fps) [%i]\n"
			"-f | --format        Set pixel format [YUYV | MJPG | H264]\n"
			"-t | --timeout       Set capture timeout in seconds [%i]\n"
			"-T | --timeouts-max  Set the maximum number of timeouts [%i]\n"
			"-o | --output        Outputs stream to stdout\n"
			"-c | --count         Number of frames to grab [%i]\n"
			"", argv[0], dev_name, fps, timeout, timeouts_max, frame_count);
}

static const char short_options[] = "d:hmruW:H:I:f:t:T:oc:";

static const struct option long_options[] = { { "device", required_argument,
NULL, 'd' }, { "help", no_argument, NULL, 'h' }, { "mmap", no_argument,
NULL, 'm' }, { "read", no_argument, NULL, 'r' }, { "userp", no_argument,
NULL, 'u' }, { "width", required_argument, NULL, 'W' }, { "height",
		required_argument, NULL, 'H' }, { "interval", required_argument, NULL,
		'I' }, { "format", required_argument, NULL, 'f' }, { "timeout",
		required_argument, NULL, 't' }, { "timeouts-max", required_argument,
NULL, 'T' }, { "output", no_argument, NULL, 'o' }, { "count", required_argument,
		NULL, 'c' }, { 0, 0, 0, 0 } };

int main(int argc, char **argv) {
	s_catch_signals();
	for (;;) {
		int idx;
		int c;

		c = getopt_long(argc, argv, short_options, long_options, &idx);

		if (-1 == c)
			break;

		switch (c) {
		case 0: /* getopt_long() flag */
			break;

		case 'd':
			dev_name = optarg;
			break;

		case 'h':
			usage(stdout, argc, argv);
			exit(EXIT_SUCCESS);

		case 'm':
			io = IO_METHOD_MMAP;
			break;

		case 'r':
			io = IO_METHOD_READ;
			break;

		case 'u':
			io = IO_METHOD_USERPTR;
			break;

		case 'W':
// set width
			width = atoi(optarg);
			set_format++;
			break;

		case 'H':
// set height
			height = atoi(optarg);
			set_format++;
			break;

		case 'I':
// set fps
			fps = atoi(optarg);
			break;

		case 'f':
// set pixel format
			if (strcmp(optarg, "YUYV") == 0 || strcmp(optarg, "yuyv") == 0) {
				pixel_format = V4L2_PIX_FMT_YUYV;
				set_format++;
			} else if (strcmp(optarg, "MJPG") == 0
					|| strcmp(optarg, "mjpg") == 0) {
				pixel_format = V4L2_PIX_FMT_MJPEG;
				set_format++;
			} else if (strcmp(optarg, "H264") == 0
					|| strcmp(optarg, "h264") == 0) {
				pixel_format = V4L2_PIX_FMT_H264;
				set_format++;
			}
			break;

		case 't':
// set timeout
			timeout = atoi(optarg);
			break;

		case 'T':
// set max timeout
			timeouts_max = atoi(optarg);
			break;

		case 'o':
			out_buf++;
			break;

		case 'c':
			errno = 0;
			frame_count = strtol(optarg, NULL, 0);
			if (errno)
				errno_exit(optarg);
			break;

		default:
			usage(stderr, argc, argv);
			exit(EXIT_FAILURE);
		}
	}
	//clock_t ws_begin, ws_end;
	//double time_spent;

	//ws_begin = clock();
	open_device();
	init_device();
	start_capturing();
	//ws_end = clock();
	//time_spent = (double) (ws_end - ws_begin) / CLOCKS_PER_SEC;

	// Do network tables things
	// Set client
	// Set IP address
	
	NetworkTable::SetClientMode();
	NetworkTable::SetTeam(111);
	NetworkTable::SetIPAddress("10.1.11.2");
	table = NetworkTable::GetTable("remoteIO");

	if (!NODASHBOARD)
	{
		m_H_MIN = (int) table.getNumber("Minimum Hue", H_MIN);
		m_H_MAX = (int) table.getNumber("Maximum Hue", H_MAX);
		m_S_MIN = (int) table.getNumber("Minimum Saturation", S_MIN);
		m_S_MAX = (int) table.getNumber("Maximum Saturation", S_MAX);
		m_V_MIN = (int) table.getNumber("Minimum Value", V_MIN);
		m_V_MAX = (int) table.getNumber("Maximum Value", V_MAX);
	}

	//fprintf(stderr, "Startup took %f seconds\n", time_spent);

	mainloop();

	//ws_begin = clock();
	stop_capturing();
	uninit_device();
	close_device();
	//ws_end = clock();
	//time_spent = (double) (ws_end - ws_begin) / CLOCKS_PER_SEC;
	//fprintf(stderr, "Shutdown took %f seconds\n", time_spent);
	//fprintf(stderr, "\n");
	return 0;
}
