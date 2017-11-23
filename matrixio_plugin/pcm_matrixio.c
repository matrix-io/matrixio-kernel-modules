#include <alsa/asoundlib.h>
#include <alsa/pcm_external.h>
#include <alsa/pcm_plugin.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#define ARRAY_SIZE(ary) (sizeof(ary) / sizeof(ary[0]))

#define MATRIXIO_MICARRAY_BUFFER_SIZE (128 * 2 * 8)

struct matrixio_t {
	snd_pcm_ioplug_t io;
	snd_pcm_t *pcm;
	snd_pcm_hw_params_t *hw_params;
	int pcm_fd;
	char buf[MATRIXIO_MICARRAY_BUFFER_SIZE];
	unsigned int ptr;
	unsigned int channel;
};
/* set up the fixed parameters of pcm PCM hw_parmas */
static int matrixio_slave_hw_params_half(struct matrixio_t *capture,
					 unsigned int rate,
					 snd_pcm_format_t format)
{
	int err;

	unsigned int buffer_time = 0;
	unsigned int period_time = 0;
	if ((err = snd_pcm_hw_params_malloc(&capture->hw_params)) < 0)
		return err;

	if ((err = snd_pcm_hw_params_any(capture->pcm, capture->hw_params)) <
	    0) {
		SNDERR("Cannot get pcm hw_params");
		goto out;
	}
	if ((err = snd_pcm_hw_params_set_access(
		 capture->pcm, capture->hw_params,
		 SND_PCM_ACCESS_RW_INTERLEAVED)) < 0) {
		SNDERR("Cannot set pcm access RW_INTERLEAVED");
		goto out;
	}
	if ((err = snd_pcm_hw_params_set_channels(capture->pcm,
						  capture->hw_params, 1)) < 0) {
		SNDERR("Cannot set pcm channels 1");
		goto out;
	}
	if ((err = snd_pcm_hw_params_set_format(
		 capture->pcm, capture->hw_params, format)) < 0) {
		SNDERR("Cannot set pcm format");
		goto out;
	}
	if ((err = snd_pcm_hw_params_set_rate(capture->pcm, capture->hw_params,
					      rate, 0)) < 0) {
		SNDERR("Cannot set pcm rate %d", rate);
		goto out;
	}

	err = snd_pcm_hw_params_get_buffer_time_max(capture->hw_params,
						    &buffer_time, 0);
	if (buffer_time > 80000)
		buffer_time = 80000;
	period_time = buffer_time / 4;

	err = snd_pcm_hw_params_set_period_time_near(
	    capture->pcm, capture->hw_params, &period_time, 0);
	if (err < 0) {
		SNDERR("Unable to set_period_time_near");
		goto out;
	}
	err = snd_pcm_hw_params_set_buffer_time_near(
	    capture->pcm, capture->hw_params, &buffer_time, 0);
	if (err < 0) {
		SNDERR("Unable to set_buffer_time_near");
		goto out;
	}

	return 0;

out:
	free(capture->hw_params);
	capture->hw_params = NULL;
	return err;
}

static int matrixio_start(snd_pcm_ioplug_t *io)
{
	struct matrixio_t *capture = io->private_data;
	if (!capture->pcm) {
		SNDERR("pcm is lost\n");
	}

	capture->pcm_fd = open("/dev/matrixio_pcm", O_RDONLY);
	capture->ptr = 0;
	return 0;
}

static int matrixio_stop(snd_pcm_ioplug_t *io)
{
	// struct matrixio_t *capture = io->private_data;
	// close(capture->pcm_fd);

	return 0;
}

static snd_pcm_sframes_t matrixio_pointer(snd_pcm_ioplug_t *io)
{

	struct matrixio_t *capture = io->private_data;
	assert(capture);

	capture->ptr += 128;
	return capture->ptr;
}

static snd_pcm_sframes_t matrixio_transfer(snd_pcm_ioplug_t *io,
					   const snd_pcm_channel_area_t *areas,
					   snd_pcm_uframes_t offset,
					   snd_pcm_uframes_t size)
{
	struct matrixio_t *capture = io->private_data;
	int chn;
	uint16_t *samples[io->channels];
	uint16_t *buf16 = (unsigned short *)capture->buf;
	/*
			printf("%s:\n", __func__);
			printf(" channel = %d \n", capture->channel);
			printf(" rate = %d \n", io->rate);
	*/
	printf(" %x size = %d  areas[0].first%8=%d  areas[0].step/8=%d "
	       "offset=%d \n",
	       areas[0].addr, size, areas[0].first % 8, areas[0].step / 8,
	       offset);

	printf(" period_size = %d \n", io->period_size);
	printf(" buffer_size = %d \n", io->buffer_size);

	/*			printf(" offset = %d \n", offset);
		*/
	read(capture->pcm_fd, buf16, MATRIXIO_MICARRAY_BUFFER_SIZE);

	for (chn = 0; chn < io->channels; chn++) {
		samples[chn] = (unsigned short *)(areas[chn].addr + offset);
		/*	+
				areas[chn].first / 8 +
				areas[chn].step / 8 * offset;*/
		//	samples[chn] += offset;
	}
	int c = 1; // size / 128;
	for (int j = 0; j < c * 128; j++)
		for (int chn = 0; chn < io->channels; chn++) {
			samples[chn][j] = buf16[j * 8 + chn];
		}

	return size;
}

/*
 * poll-related callbacks - just pass to pcm PCM
 */
static int matrixio_poll_descriptors_count(snd_pcm_ioplug_t *io) { return 1; }

static int matrixio_poll_descriptors(snd_pcm_ioplug_t *io, struct pollfd *pfd,
				     unsigned int space)
{
	struct matrixio_t *capture = io->private_data;

	if (space < 1)
		return 0;

	pfd[0].fd = capture->pcm_fd;
	pfd[0].events = POLLIN;
	pfd[0].revents = 0;

	return 1;
}

static int matrixio_poll_revents(snd_pcm_ioplug_t *io, struct pollfd *pfd,
				 unsigned int nfds, unsigned short *revents)
{
	*revents = pfd[0].revents;

	return 0;
}

/*
 * close callback
 */
static int matrixio_close(snd_pcm_ioplug_t *io)
{
	struct matrixio_t *capture = io->private_data;

	if (capture->pcm)
		snd_pcm_close(capture->pcm);

	return 0;
}

static int setSoftwareParams(struct matrixio_t *capture)
{
	snd_pcm_sw_params_t *softwareParams;
	int err;

	snd_pcm_uframes_t bufferSize = 0;
	snd_pcm_uframes_t periodSize = 0;
	snd_pcm_uframes_t startThreshold, stopThreshold;
	snd_pcm_sw_params_alloca(&softwareParams);

	// Get the current software parameters
	err = snd_pcm_sw_params_current(capture->pcm, softwareParams);
	if (err < 0) {
		SNDERR("Unable to get software parameters: %s",
		       snd_strerror(err));
		goto done;
	}

	// Configure ALSA to start the transfer when the buffer is almost full.
	snd_pcm_get_params(capture->pcm, &bufferSize, &periodSize);

	startThreshold = 1;
	stopThreshold = bufferSize;

	err = snd_pcm_sw_params_set_start_threshold(
	    capture->pcm, softwareParams, startThreshold);
	if (err < 0) {
		SNDERR("Unable to set start threshold to %lu frames: %s",
		       startThreshold, snd_strerror(err));
		goto done;
	}

	err = snd_pcm_sw_params_set_stop_threshold(capture->pcm, softwareParams,
						   stopThreshold);
	if (err < 0) {
		SNDERR("Unable to set stop threshold to %lu frames: %s",
		       stopThreshold, snd_strerror(err));
		goto done;
	}
	// Allow the transfer to start when at least periodSize samples can be
	// processed.
	err = snd_pcm_sw_params_set_avail_min(capture->pcm, softwareParams,
					      periodSize);
	if (err < 0) {
		SNDERR("Unable to configure available minimum to %lu: %s",
		       periodSize, snd_strerror(err));
		goto done;
	}

	// Commit the software parameters back to the device.
	err = snd_pcm_sw_params(capture->pcm, softwareParams);
	if (err < 0)
		SNDERR("Unable to configure software parameters: %s",
		       snd_strerror(err));

	return 0;
done:
	snd_pcm_sw_params_free(softwareParams);

	return err;
}

static int matrixio_hw_params(snd_pcm_ioplug_t *io, snd_pcm_hw_params_t *params)
{
	struct matrixio_t *capture = io->private_data;
	snd_pcm_uframes_t period_size;
	snd_pcm_uframes_t buffer_size;
	int err;
	if (!capture->hw_params) {
		err = matrixio_slave_hw_params_half(capture, io->rate,
						    io->format);
		if (err < 0) {
			SNDERR("matrixio_slave_hw_params_half error\n");
			return err;
		}
	}
	period_size = io->period_size;
	if ((err = snd_pcm_hw_params_set_period_size_near(
		 capture->pcm, capture->hw_params, &period_size, NULL)) < 0) {
		SNDERR("Cannot set pcm period size %ld", period_size);
		return err;
	}
	buffer_size = io->buffer_size;
	if ((err = snd_pcm_hw_params_set_buffer_size_near(
		 capture->pcm, capture->hw_params, &buffer_size)) < 0) {
		SNDERR("Cannot set pcm buffer size %ld", buffer_size);
		return err;
	}
	if ((err = snd_pcm_hw_params(capture->pcm, capture->hw_params)) < 0) {
		SNDERR("Cannot set pcm hw_params");
		return err;
	}
	setSoftwareParams(capture);
	return 0;
}

static int matrixio_hw_free(snd_pcm_ioplug_t *io)
{
	struct matrixio_t *capture = io->private_data;
	free(capture->hw_params);
	capture->hw_params = NULL;

	return snd_pcm_hw_free(capture->pcm);
}

static int matrixio_prepare(snd_pcm_ioplug_t *io)
{
	struct matrixio_t *capture = io->private_data;
	capture->pcm_fd = -1;
	capture->ptr = 0;
	return snd_pcm_prepare(capture->pcm);
}

static snd_pcm_ioplug_callback_t matrixio_ops = {
    .start = matrixio_start,
    .stop = matrixio_stop,
    .pointer = matrixio_pointer,
    .transfer = matrixio_transfer,
    .poll_descriptors_count = matrixio_poll_descriptors_count,
    .poll_descriptors = matrixio_poll_descriptors,
    .poll_revents = matrixio_poll_revents,
    .close = matrixio_close,
    .hw_params = matrixio_hw_params,
    .hw_free = matrixio_hw_free,
    .prepare = matrixio_prepare,
};

static int matrixio_set_hw_constraint(struct matrixio_t *capture)
{
	static unsigned int accesses[] = {SND_PCM_ACCESS_RW_INTERLEAVED};
	unsigned int formats[] = {SND_PCM_FORMAT_S16};

	unsigned int rates[] = {8000, 16000, 32000, 44100, 48000};
	int err;

	err = snd_pcm_ioplug_set_param_list(&capture->io,
					    SND_PCM_IOPLUG_HW_ACCESS,
					    ARRAY_SIZE(accesses), accesses);
	if (err < 0) {
		SNDERR("ioplug cannot set matrixio hw access");
		return err;
	}

	if ((err = snd_pcm_ioplug_set_param_list(
		 &capture->io, SND_PCM_IOPLUG_HW_FORMAT, ARRAY_SIZE(formats),
		 formats)) < 0 ||
	    (err = snd_pcm_ioplug_set_param_minmax(
		 &capture->io, SND_PCM_IOPLUG_HW_CHANNELS, 1, 4)) < 0 ||
	    (err = snd_pcm_ioplug_set_param_list(
		 &capture->io, SND_PCM_IOPLUG_HW_RATE, ARRAY_SIZE(rates),
		 rates)) < 0) {
		SNDERR("ioplug cannot set matrixio format channel rate!");
		return err;
	}
	err = snd_pcm_ioplug_set_param_minmax(
	    &capture->io, SND_PCM_IOPLUG_HW_BUFFER_BYTES, 1, 4 * 1024 * 1024);
	if (err < 0) {
		SNDERR("ioplug cannot set matrixio hw buffer bytes");
		return err;
	}

	err = snd_pcm_ioplug_set_param_minmax(
	    &capture->io, SND_PCM_IOPLUG_HW_PERIOD_BYTES, 128, 2 * 1024 * 1024);
	if (err < 0) {
		SNDERR("ioplug cannot set matrixio hw period bytes");
		return err;
	}

	err = snd_pcm_ioplug_set_param_minmax(
	    &capture->io, SND_PCM_IOPLUG_HW_PERIODS, 3, 1024);
	if (err < 0) {
		SNDERR("ioplug cannot set matrixio hw periods");
		return err;
	}
	return 0;
}

/*
 * Main entry point
 */
SND_PCM_PLUGIN_DEFINE_FUNC(matrixio)
{
	snd_config_iterator_t i, next;
	int err;
	const char *pcm_string = NULL;
	struct matrixio_t *capture;
	int channel;
	int channels;
	if (stream != SND_PCM_STREAM_CAPTURE) {
		SNDERR("matrixio is only for capture");
		return -EINVAL;
	}

	snd_config_for_each(i, next, conf)
	{
		snd_config_t *n = snd_config_iterator_entry(i);
		const char *id;
		if (snd_config_get_id(n, &id) < 0)
			continue;
		if (strcmp(id, "comment") == 0 || strcmp(id, "type") == 0 ||
		    strcmp(id, "hint") == 0)
			continue;

		if (strcmp(id, "slavepcm") == 0) {
			if (snd_config_get_string(n, &pcm_string) < 0) {
				SNDERR("matrixio slavepcm must be a string");
				return -EINVAL;
			}
			continue;
		}

		if (strcmp(id, "channel") == 0) {
			long val;
			if (snd_config_get_integer(n, &val) < 0) {
				SNDERR("Invalid type for %s", id);
				return -EINVAL;
			}
			channel = val;
			continue;
		}
		if (strcmp(id, "channels") == 0) {
			long val;
			if (snd_config_get_integer(n, &val) < 0) {
				SNDERR("Invalid type for %s", id);
				return -EINVAL;
			}
			channels = val;
			if (channels != 1) {
				SNDERR("channels must be 1");
				return -EINVAL;
			}
			continue;
		}
	}

	capture = calloc(1, sizeof(*capture));
	if (!capture) {
		SNDERR("cannot allocate");
		return -ENOMEM;
	}
	err = snd_pcm_open(&capture->pcm, pcm_string, stream, mode);
	if (err < 0)
		goto error;

	// SND_PCM_NONBLOCK
	capture->io.version = SND_PCM_IOPLUG_VERSION;
	capture->io.name = "MATRIXIO decode Plugin";
	capture->io.mmap_rw = 0;
	capture->io.callback = &matrixio_ops;
	capture->io.private_data = capture;
	capture->channel = channel;

	err = snd_pcm_ioplug_create(&capture->io, name, stream, mode);
	if (err < 0)
		goto error;

	if ((err = matrixio_set_hw_constraint(capture)) < 0) {
		snd_pcm_ioplug_delete(&capture->io);
		return err;
	}
	*pcmp = capture->io.pcm;
	return 0;

error:
	if (capture->pcm)
		snd_pcm_close(capture->pcm);
	free(capture);
	return err;
}

SND_PCM_PLUGIN_SYMBOL(matrixio);
