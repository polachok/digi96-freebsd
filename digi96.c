#ifdef HAVE_KERNEL_OPTION_HEADERS
#include "opt_snd.h"
#endif

#include <dev/sound/pcm/sound.h>

#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>
#include <sys/sysctl.h>
#include <sys/endian.h>

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/module.h>

#include "mixer_if.h"
#include "digi96.h"

#define MAX_AUDIO_CHANNEL	2

static const struct {
	uint16_t vendor;
	uint16_t devid;
	char *desc;
} digi96_hw[] = {
	{ RME_VENDOR_ID, RME_DIGI96, 		"RME Digi96"     },
	{ RME_VENDOR_ID, RME_DIGI96_8, 		"RME Digi96/8"   },
	{ RME_VENDOR_ID, RME_DIGI96_PRO, 	"RME Digi96/8 PRO" },
	{ RME_VENDOR_ID, RME_DIGI96_PAD, 	"RME Digi96/8 PAD" },
};

struct digi96_info;

struct digi96_chinfo {
	struct digi96_info	*parent;
	struct pcm_channel	*channel;
	struct snd_dbuf		*buffer;
	u_int32_t		fmt, spd, phys_buf, bps;
	int			dir;
};

struct digi96_info {
	device_t dev;
	struct mtx *lock;

	int have_adat;
	int have_analog;
	int mode;
#define MD_SPDIF	0
#define MD_AES		1
#define MD_ADAT		2

	struct resource *reg, *irq;
	int regtype, regid, irqid;
	void *ih;
	unsigned int	bufsz;

	struct digi96_chinfo chan[MAX_AUDIO_CHANNEL];
	bus_space_tag_t st;
	bus_space_handle_t sh;
	u_int32_t ctrl[2]; /* control registers */
};

/* channel interface */
static u_int32_t digi96_fmt[] = {
	SND_FORMAT(AFMT_AC3, 1, 0),
	SND_FORMAT(AFMT_S16_LE, 1, 0),
	SND_FORMAT(AFMT_S32_LE, 1, 0),
	0
};

static struct pcmchan_caps digi96_caps = {4000, 48000, digi96_fmt, 0};

static int digi96_probe(device_t);
static int digi96_attach(device_t);
static int digi96_detach(device_t);

void digi96_uninit(struct digi96_info *);

static void
digi96_write(struct digi96_info *sc, int reg, u_int32_t data)
{
	if (reg == DIGI96_CTRL1)
		sc->ctrl[0] = data;
	else if (reg == DIGI96_CTRL2)
		sc->ctrl[1] = data;
	bus_space_write_4(sc->st, sc->sh, reg, data);
}

static int digi96_probe(device_t dev)
{
	int i;
	uint16_t devid, vendor;

	vendor = pci_get_vendor(dev);
	devid = pci_get_device(dev);
	for (i = 0; i < sizeof(digi96_hw) / sizeof(digi96_hw[0]); i++) {
		if (vendor == digi96_hw[i].vendor &&
		    devid == digi96_hw[i].devid) {
			device_set_desc(dev, digi96_hw[i].desc);
			return (BUS_PROBE_DEFAULT);
		}
	}

	return (ENXIO);
}

static void digi96_intr(void *p) {
	struct digi96_info *sc = p;
	struct digi96_chinfo *ch;
	u_int32_t playstat, recstat;
	int i;

	playstat = bus_space_read_4(sc->st, sc->sh, DIGI96_GET_PLAYPOS);
	recstat = bus_space_read_4(sc->st, sc->sh, DIGI96_GET_RECPOS);

	for (i = 0; i < MAX_AUDIO_CHANNEL; i++) {
		ch = &sc->chan[i];

		if (playstat & POS_PLAYIRQ) {
			digi96_write(sc, DIGI96_PLAYACK, 0);
		}
		if (playstat & POS_RECIRQ) {
			digi96_write(sc, DIGI96_RECACK, 0);
		}
	}
}

static void *
digi96_chan_init(kobj_t obj, void *devinfo,
	     struct snd_dbuf *b, struct pcm_channel *c, int dir)
{
	struct digi96_info *sc = devinfo;
	struct digi96_chinfo *ch = (dir == PCMDIR_PLAY) ? &sc->chan[0] : &sc->chan[1];

	ch->parent = sc;
	ch->channel = c;
	/* wtf? ch->bps = 1; */
	ch->fmt = SND_FORMAT(AFMT_AC3 | AFMT_S16_LE | AFMT_S32_LE, 1, 0);
	ch->spd = DSP_DEFAULT_SPEED;
	ch->buffer = b;
	if (dir == PCMDIR_PLAY)
		sndbuf_setup(ch->buffer, rman_get_virtual(sc->reg), DIGI96_BUFSZ);
	else
		sndbuf_setup(ch->buffer, (u_int8_t *)rman_get_virtual(sc->reg)+DIGI96_BUFSZ, DIGI96_BUFSZ);
	device_printf(sc->dev, "%s buf %p\n", (dir == PCMDIR_PLAY)?
			      "play" : "rec", sndbuf_getbuf(ch->buffer));
#if 0
	if (sndbuf_alloc(ch->buffer, sc->parent_dmat, 0, sc->bufsz) != 0) {
		printf("cmichan_init failed\n");
		return NULL;
	}
#endif
	ch->dir = dir;
	return ch;
}

static int
digi96_chan_setformat(kobj_t obj, void *data, u_int32_t format)
{
	struct digi96_chinfo *ch = data;

	ch->fmt = format;
	return 0;
}

static struct pcmchan_caps *
digi96_chan_getcaps(kobj_t obj, void *data)
{
	return &digi96_caps;
}

static kobj_method_t digi96_chan_methods[] = {
    	KOBJMETHOD(channel_init,		digi96_chan_init),
    	KOBJMETHOD(channel_setformat,		digi96_chan_setformat),
    	KOBJMETHOD(channel_getcaps,		digi96_chan_getcaps),
#if 0
    	KOBJMETHOD(channel_setspeed,		digi96_chan_setspeed),
    	KOBJMETHOD(channel_setblocksize,	digi96_chan_setblocksize),
    	KOBJMETHOD(channel_trigger,		digi96_chan_trigger),
    	KOBJMETHOD(channel_getptr,		digi96_chan_getptr),
#endif
	KOBJMETHOD_END
};
CHANNEL_DECLARE(digi96_chan);

static int
digi96_mixer_init(struct snd_mixer *m)
{
	return (0);
}

static int
digi96_mixer_set(struct snd_mixer *m, unsigned dev, unsigned left, unsigned right)
{
	return (0);
}

static u_int32_t
digi96_mixer_setrecsrc(struct snd_mixer *m, u_int32_t src)
{
	return (0);
}

static kobj_method_t digi96_mixer_methods[] = {
	KOBJMETHOD(mixer_init, digi96_mixer_init),
	KOBJMETHOD(mixer_set, digi96_mixer_set),
	KOBJMETHOD(mixer_setrecsrc, digi96_mixer_setrecsrc),
	KOBJMETHOD_END
};
MIXER_DECLARE(digi96_mixer);

static int digi96_attach(device_t dev)
{
	struct digi96_info *sc;
	uint16_t devid, revid;
	char status[SND_STATUSLEN];

	sc = malloc(sizeof(*sc), M_DEVBUF, M_WAITOK | M_ZERO);
	sc->lock = snd_mtxcreate(device_get_nameunit(dev), "snd_digi96 softc");
	sc->dev = dev;

	pci_enable_busmaster(dev);
	pci_enable_io(dev, SYS_RES_MEMORY);

	sc->regid = PCIR_BAR(0);
	sc->regtype = SYS_RES_MEMORY;
	sc->reg = bus_alloc_resource_any(dev, sc->regtype,
	    &sc->regid, RF_ACTIVE);
	if (!sc->reg) {
		device_printf(dev, "unable to allocate register space\n");
		goto bad;
	}
	sc->st = rman_get_bustag(sc->reg);
	sc->sh = rman_get_bushandle(sc->reg);

	sc->irqid = 0;
	sc->irq = bus_alloc_resource_any(dev, SYS_RES_IRQ, &sc->irqid,
	    RF_ACTIVE | RF_SHAREABLE);
	if (!sc->irq || snd_setup_intr(dev, sc->irq, INTR_MPSAFE,
	    digi96_intr, sc, &sc->ih)) {
		device_printf(dev, "unable to map interrupt\n");
		goto bad;
	}
	sc->bufsz = pcm_getbuffersize(dev, DIGI96_BUFSZ, DIGI96_BUFSZ, DIGI96_BUFSZ);

	sc->have_adat = 0;
	sc->have_analog = 0;
	sc->mode = MD_SPDIF;

	devid = pci_get_device(dev);
	revid = pci_get_revid(dev);
	switch (devid) {
		case RME_DIGI96:
			break;
		case RME_DIGI96_8:
		case RME_DIGI96_PRO:
			sc->have_adat = 1;
			break;
		case RME_DIGI96_PAD:
			sc->have_adat = 1;
			sc->have_analog = 1;
			break;
	}
	/* set defaults */
	digi96_write(sc, DIGI96_CTRL2, 0);
	/* reset DAC */
	digi96_write(sc, DIGI96_CTRL1, CTRL1_ISEL | CTRL1_FREQ48 | CTRL1_PD);
	digi96_write(sc, DIGI96_CTRL1, CTRL1_ISEL | CTRL1_FREQ48 | CTRL1_MASTER |
		       	CTRL1_SEL);
	digi96_write(sc, DIGI96_CTRL2, CTRL2_DAC_EN); /* soft unmute DAC */

	if (mixer_init(dev, &digi96_mixer_class, sc))
		goto bad;
	if (pcm_register(dev, sc, 1, 0))
		goto bad;
	pcm_addchan(dev, PCMDIR_PLAY, &digi96_chan_class, sc);
	snprintf(status, SND_STATUSLEN, "at io 0x%lx irq %ld %s",
		 rman_get_start(sc->reg), rman_get_start(sc->irq), PCM_KLDSTRING(snd_digi96));
	pcm_setstatus(dev, status);
	return (0);
bad:
	digi96_uninit(sc);
	return (ENXIO);

}

void digi96_uninit(struct digi96_info *sc)
{
	digi96_write(sc, DIGI96_CTRL2, 0); /* soft mute */
	if (sc->ih) {
		bus_teardown_intr(sc->dev, sc->irq, sc->ih);
		sc->ih = NULL;
	}
	if (sc->reg) {
		bus_release_resource(sc->dev, sc->regtype, sc->regid, sc->reg);
		sc->reg = NULL;
	}
	if (sc->irq) {
		bus_release_resource(sc->dev, SYS_RES_IRQ, sc->irqid, sc->irq);
		sc->irq = NULL;
	}
	if (sc->lock) {
		snd_mtxfree(sc->lock);
		sc->lock = NULL;
	}
	free(sc, M_DEVBUF);
}

static int digi96_detach(device_t dev) {
	struct digi96_info *sc;
	int r;

	r = pcm_unregister(dev);
	if (r)
		return r;
	sc = pcm_getdevinfo(dev);
	digi96_uninit(sc);
	return (0);
}

static device_method_t digi96_methods[] = {
	/* Methods from the device interface */
	DEVMETHOD(device_probe,         digi96_probe),
	DEVMETHOD(device_attach,        digi96_attach),
	DEVMETHOD(device_detach,        digi96_detach),

	/* Terminate method list */
	{ 0, 0 }
};

static driver_t digi96_driver = {
	"pcm",
	digi96_methods,
	PCM_SOFTC_SIZE,
};

DRIVER_MODULE(snd_digi96, pci, digi96_driver, pcm_devclass, NULL, NULL);
MODULE_DEPEND(snd_digi96, sound, SOUND_MINVER, SOUND_PREFVER, SOUND_MAXVER);
MODULE_VERSION(snd_digi96, 0);
