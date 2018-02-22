#ifndef __MATRIXIO_PCM_H__
#define __MATRIXIO_PCM_H__

#include "matrixio-core.h"

#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <sound/pcm.h>

#define MATRIXIO_CHANNELS_MAX 8

#define MATRIXIO_RATES                                                         \
	(SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 | SNDRV_PCM_RATE_16000 |   \
	 SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |  \
	 SNDRV_PCM_RATE_48000)

#define MATRIXIO_FORMATS SNDRV_PCM_FMTBIT_S16_LE


/*
#define MATRIXIO_FORMATS                                                       \
	(SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE |                   \
	 SNDRV_PCM_FMTBIT_S32_LE)
*/
#define MATRIXIO_MICARRAY_BUFFER_SIZE (256 * 1 /*MATRIXIO_CHANNELS_MAX*/ * 2)

#define MATRIXIO_FIFO_SIZE (MATRIXIO_MICARRAY_BUFFER_SIZE * 32)

struct matrixio_substream {
	struct matrixio *mio;
	int irq;
	struct mutex lock;
	struct snd_pcm_substream *capture_substream;
	struct workqueue_struct *wq;
	struct work_struct work;

	struct snd_card *card;
	struct snd_pcm *pcm;

	snd_pcm_uframes_t position;
	int channels;
};

#endif
