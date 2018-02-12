#ifndef __MATRIXIO_PCM_H__
#define __MATRIXIO_PCM_H__

#include "matrixio-core.h"

#include <linux/kfifo.h>
#include <linux/workqueue.h>
#include <sound/pcm.h>

struct matrixio_substream {
	struct matrixio *mio;
	int irq;
	void __iomem *iobase;
	spinlock_t lock;
	struct snd_pcm_substream *capture_substream;
	struct snd_pcm_substream *playback_substream;
	struct workqueue_struct *wq;
	struct work_struct work;
	int force_end_work;

	struct snd_card *card;
	struct snd_pcm *pcm;

	struct kfifo_rec_ptr_2 capture_fifo;

	int stamp;
};

#endif
