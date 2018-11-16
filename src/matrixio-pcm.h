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

struct matrixio_substream {
	struct matrixio *mio;
	int irq;
	struct mutex lock;
	struct snd_pcm_substream *substream;
	struct workqueue_struct *wq;
	struct work_struct work;

	struct snd_card *card;
	struct snd_pcm *pcm;
	struct playback_params *playback_params;

	snd_pcm_uframes_t position; /* position in buffer in bytes*/
	int channels;
};

#endif
