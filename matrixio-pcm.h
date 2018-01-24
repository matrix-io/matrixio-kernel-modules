#ifndef __MATRIXIO_PCM_H__
#define __MATRIXIO_PCM_H__

#include "matrixio-core.h"

#include <linux/workqueue.h>
#include <sound/pcm.h>

struct matrixio_substream {
	struct matrixio *mio;
	int irq;
	spinlock_t lock;
	struct snd_pcm_substream __rcu *rx_substream;
	struct workqueue_struct *wq;
	struct work_struct work;
	int force_end_work;
	int stamp;
};

#endif
