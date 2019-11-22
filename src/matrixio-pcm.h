#ifndef __MATRIXIO_PCM_H__
#define __MATRIXIO_PCM_H__

#include "matrixio-core.h"

#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <sound/pcm.h>

struct playback_params {
	uint32_t rate;
	uint32_t period;
	uint32_t bit_time;
};

/* Defines for capture parameters */
#define MATRIXIO_CHANNELS_MAX 8
#define MATRIXIO_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 | \
		SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 | \
		SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_96000)
#define MATRIXIO_FORMATS SNDRV_PCM_FMTBIT_S16_LE
/* 512 is 2 ** (ADDR_WIDTH_BUFFER - CHANNELS_WIDTH - 1) from mic_array.v
 * The FPGA has a 1024 samples x 8 channels buffer divided into two fragments.
 * There is an interrupt after each fragment.  */
#define MATRIXIO_PERIOD_FRAMES (512u)
#define MATRIXIO_PERIOD_BYTES_PER_CH (MATRIXIO_PERIOD_FRAMES * sizeof(uint16_t))
/* Enough for at least 32 periods, ~170 ms at max sample rate and channels */
#define MATRIXIO_BUFFER_MAX (1u << 18)
#define MATRIXIO_FIR_TAP_SIZE (128u * sizeof(uint16_t))
/* This could be 2, but some software (pyaudio) uses the smallest buffer it can
 * get and provides no way to ask for a larger one.  So we make the smallest
 * buffer enough to allow for reasonable latency. */
#define MATRIXIO_MIN_PERIODS 16

/* For playback */
struct matrixio_substream {
	struct matrixio *mio;
	unsigned irq;
	struct mutex lock;
	struct snd_pcm_substream *substream;
	struct playback_params *playback_params;
	snd_pcm_uframes_t position;
};

struct matrixio_mic_substream {
	struct matrixio *mio;
	unsigned irq;
	struct workqueue_struct *wq;
	struct work_struct work;
	struct snd_pcm_substream *substream;

	spinlock_t worker_lock; /* Use in atomic trigger callback, can't be mutex */
	atomic_t position;	/* Position in DMA buffer in frames */
	uint16_t *frag_buffer;	/* One interrupt worth of data's bounce buffer */
	/* bit 0 - capture on
	 * bit 1 - period SPI xfer pending */
	unsigned long flags;
};

/* Managing races:
 *
 * The irq handler only needs the matriox_substream itself, and the flag, work
 * and wq members.  Do not delete these without first freeing the irq.	Use the
 * atomic flag methods with the flags.	The capture on bit is NOT enough to
 * prevent a race vs the irq handler; it is only meant to reduce the performance
 * impact of the "always on" irq design of the matrix io pcm.
 *
 * The workqueue will need various pcm data and one should not modify the
 * position field while the worker might be running.  To do this, do not modify
 * position while the pcm substream is active without holding the worker_lock.
 * Also insure the worker is no longer running when the pcm substream stops.  It
 * is ok to read the position without holding worker_lock, as nothing which
 * writes to position is permitted to set it to an incorrect value at any time.
 */
#endif
