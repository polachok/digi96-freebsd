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
#include <sys/ktr.h>
#include <sys/types.h>
#include <sys/systm.h>

#include "mixer_if.h"
#include "digi96.h"

#define KTR_DIGI KTR_DEV

#define MAX_AUDIO_CHANNEL	2

#define	DIGI96_HAS_ANALOG_OUT(digi96_info) ((digi96_info)->devid == RME_DIGI96_PRO || \
				     (digi96_info)->devid == RME_DIGI96_PAD)
#define	DIGI96_DAC_IS_1852(digi96_info) (DIGI96_HAS_ANALOG_OUT(digi96_info) && (digi96_info)->revision >= 4)
#define	DIGI96_DAC_IS_1855(digi96_info) (((digi96_info)->devid == RME_DIGI96_PAD && (digi96_info)->revision < 4) || \
			          ((digi96_info)->devid == RME_DIGI96_PRO && (digi96_info)->revision == 2))
#define	DIGI96_185X_MAX_OUT(digi96_info) ((1 << (DIGI96_DAC_IS_1852(digi96_info) ? DIGI96_AD1852_VOL_BITS : DIGI96_AD1855_VOL_BITS)) - 1)

#define DIGI96_CLOCKMODE_SLAVE 		0
#define DIGI96_CLOCKMODE_MASTER 	1
#define DIGI96_CLOCKMODE_WORDCLOCK 	2

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
	u_int32_t		fmt, spd, bps;
	u_int32_t 		blksz;
	u_int32_t 		pos;
	void 			*ioaddr;
	int			dir;
	int 			spdidx; /* speed index in speed_table array */
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
	int doublespeed;

	struct resource *reg, *irq;
	int regtype, regid, irqid;
	void *ih;
	unsigned int	bufsz;

	struct digi96_chinfo chan[MAX_AUDIO_CHANNEL];
	bus_space_tag_t st;
	bus_space_handle_t sh;
	u_int32_t ctrl[2]; /* control registers */
	/* volume */
	uint16_t vol[2];
	uint16_t attenuation;
	/* pci info */
	uint16_t devid;
	uint16_t revision;
	int clockmode;
};

/* channel interface */
static u_int32_t digi96_fmt[] = {
	SND_FORMAT(AFMT_AC3, 2, 0),
	SND_FORMAT(AFMT_S16_LE, 2, 0),
	SND_FORMAT(AFMT_S32_LE, 2, 0),
	0
};

static struct pcmchan_caps digi96_caps = { 32000, 96000, digi96_fmt, 0 };

#define digi96_lock(_sc) snd_mtxlock((_sc)->lock)
#define digi96_unlock(_sc) snd_mtxunlock((_sc)->lock)

static int digi96_probe(device_t);
static int digi96_attach(device_t);
static int digi96_detach(device_t);

void digi96_uninit(struct digi96_info *);

static void
digi96_write(struct digi96_info *sc, int reg, u_int32_t data)
{
	bus_space_write_4(sc->st, sc->sh, reg, data);
}

static u_int32_t 
digi96_read(struct digi96_info *sc, int reg)
{
	return bus_space_read_4(sc->st, sc->sh, reg);
}

static void
digi96_ctrl1(struct digi96_info *sc, u_int32_t data)
{
	sc->ctrl[0] = data;
	digi96_write(sc, DIGI96_CTRL1, data);
}

static void
digi96_ctrl2(struct digi96_info *sc, u_int32_t data)
{
	sc->ctrl[1] = data;
	digi96_write(sc, DIGI96_CTRL2, data);
}

static int
digi96_probe(device_t dev)
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

static void
digi96_intr(void *p) {
	struct digi96_info *sc = p;
	struct digi96_chinfo *ch;
	u_int32_t playstat, recstat;

	playstat = digi96_read(sc, DIGI96_GET_PLAYPOS);
	recstat = digi96_read(sc, DIGI96_GET_RECPOS);

	if (playstat & POS_PLAYIRQ) {
		ch = &sc->chan[0];
#if 0
		device_printf(sc->dev, "playstat = 0x%x, PLAY_POS = 0x%x\n",
				playstat,
				(digi96_read(sc, DIGI96_GET_PLAYPOS) & POS_ADDR));
		//device_printf(sc->dev, "Play interrupt\n");
#endif
		digi96_write(sc, DIGI96_PLAYACK, 0);
		chn_intr(ch->channel);
	}
	if (playstat & POS_RECIRQ) {
		ch = &sc->chan[1];
		//device_printf(sc->dev, "Rec interrupt\n");
		digi96_write(sc, DIGI96_RECACK, 0);
		chn_intr(ch->channel);
	}
}

static void *
digi96_chan_init(kobj_t obj, void *devinfo,
	     struct snd_dbuf *b, struct pcm_channel *c, int dir)
{
	struct digi96_info *sc = devinfo;
	struct digi96_chinfo *ch = (dir == PCMDIR_PLAY) ? &sc->chan[0] : &sc->chan[1];
	void *buf = rman_get_virtual(sc->reg);

	ch->parent = sc;
	ch->channel = c;
	/* wtf? ch->bps = 1; */
	ch->fmt = SND_FORMAT(AFMT_S16_LE, 2, 0);
	ch->spd = 48000;
	ch->spdidx = 3;
	ch->buffer = b;
	ch->pos = 0;
	ch->ioaddr = (dir == PCMDIR_PLAY) ? buf : (u_int8_t*)buf + DIGI96_BUFSZ;
	sndbuf_setup(ch->buffer, ch->ioaddr, sc->bufsz);
	device_printf(sc->dev, "%s buf %p alignment %d\n", (dir == PCMDIR_PLAY)?
			      "play" : "rec", sndbuf_getbuf(ch->buffer),
				sndbuf_getalign(ch->buffer));
	ch->dir = dir;
	return ch;
}

static int
digi96_chan_setformat(kobj_t obj, void *data, u_int32_t format)
{
	struct digi96_chinfo *ch = data;
	struct digi96_info *sc = ch->parent;

	//device_printf(sc->dev, "%s\n", __func__);
	ch->fmt = format;
	if (format & AFMT_AC3) {
		//device_printf(sc->dev, "Setting format AC3\n");
		digi96_ctrl1(sc, sc->ctrl[0] | CTRL1_AC3 | CTRL1_PD);
	} else if (format & AFMT_S16_LE) {
		//device_printf(sc->dev, "Setting format S16LE\n");
		digi96_ctrl1(sc, sc->ctrl[0] & ~CTRL1_MODE24_PLAY);
		digi96_ctrl1(sc, sc->ctrl[0] & ~(CTRL1_AC3 | CTRL1_PD));
	} else if (format & AFMT_S32_LE) {
		//device_printf(sc->dev, "Setting format S32LE\n");
		digi96_ctrl1(sc, sc->ctrl[0] | CTRL1_MODE24_PLAY);
		digi96_ctrl1(sc, sc->ctrl[0] & ~(CTRL1_AC3 | CTRL1_PD));
	}
	return 0;
}

static u_int32_t
digi96_chan_setspeed(kobj_t obj, void *data, u_int32_t speed) 
{
	struct digi96_chinfo *ch = data;
	struct digi96_info *sc = ch->parent;
	u_int32_t cmd = sc->ctrl[0];
	int ds;

	ds = sc->ctrl[0] & CTRL1_DS;
	switch (speed) {
		case 32000:
			cmd &= ~CTRL1_DS;
			cmd = (cmd | CTRL1_FREQ32) & ~CTRL1_FREQ44;
			break;
		case 44100:
			cmd &= ~CTRL1_DS;
			cmd = (cmd | CTRL1_FREQ44) & ~CTRL1_FREQ32;
			break;
		case 48000:
			cmd &= ~CTRL1_DS;
			cmd = (cmd | CTRL1_FREQ48);
			break;
		case 64000:
			cmd |= CTRL1_DS;
			cmd = (cmd | CTRL1_FREQ32) & ~CTRL1_FREQ44;
			break;
		case 88200:
			cmd |= CTRL1_DS;
			cmd = (cmd | CTRL1_FREQ44) & ~CTRL1_FREQ32;
			break;
		case 96000:
			cmd |= CTRL1_DS;
			cmd = (cmd | CTRL1_FREQ48);
			break;
	}
	if (!!ds != !!(cmd & CTRL1_DS))
		digi96_ctrl1(sc, cmd | CTRL1_PD); /* reset the DAC */
	digi96_ctrl1(sc, cmd);
	ch->spd = speed;
	//device_printf(sc->dev, "%s %d\n", __func__, ch->spd);
	return ch->spd;
}

static u_int32_t 
digi96_chan_setblocksize(kobj_t obj, void *data, u_int32_t blocksize)
{
	struct digi96_chinfo *ch = data;
	struct digi96_info *sc = ch->parent;
	u_int32_t cmd = sc->ctrl[0];

	if (blocksize < 8192) {
		ch->blksz = 2048;
		cmd |= CTRL1_ISEL;
	} else {
		ch->blksz = 8192;
		cmd &= ~CTRL1_ISEL;
	}

	digi96_ctrl1(sc, cmd);
	//device_printf(sc->dev, "%s requested: %d got: %d\n", __func__, blocksize, ch->blksz);
	sndbuf_resize(ch->buffer, sc->bufsz / ch->blksz, ch->blksz);
	return ch->blksz;
}

static struct pcmchan_caps *
digi96_chan_getcaps(kobj_t obj, void *data)
{
	return &digi96_caps;
}

static u_int32_t
digi96_chan_getptr(kobj_t obj, void *data)
{
	struct digi96_chinfo *ch = data;
	struct digi96_info *sc = ch->parent;
	u_int32_t ret;
	digi96_lock(sc);
	ret = digi96_read(sc, DIGI96_GET_PLAYPOS) & POS_ADDR;
#if 0
	//device_printf(sc->dev, "channel pointer 0x%x\n", ret);
#endif
	digi96_unlock(sc);
	return ret;
}

void
digi96_prepare_output(struct digi96_chinfo *ch);
void
digi96_prepare_output(struct digi96_chinfo *ch)
{
	struct digi96_info *sc = ch->parent;
	//u_int32_t cmd = sc->ctrl[0];
	int speed;

	if (ch->dir != PCMDIR_PLAY) {
		//device_printf(sc->dev, "attempt to use play chan for rec\n");
		return;
	}
	digi96_write(sc, DIGI96_RESET_PLAY, 0);
#if 0
	cmd |= CTRL1_MASTER;
	digi96_ctrl1(sc, cmd);
#endif
	//digi96_ctrl1(sc, sc->ctrl[0] | CTRL1_ISEL);
	//device_printf(sc->dev, "Initial PLAY POS = 0x%x\n", 
			//(digi96_read(sc, DIGI96_GET_PLAYPOS) & POS_ADDR) >> sc->frlog);

	if ((sc->ctrl[0] & CTRL1_FREQ44) && (sc->ctrl[0] & CTRL1_FREQ32)) /* 48kHz */
		speed = 48000;
	else if (sc->ctrl[0] & CTRL1_FREQ32)
		speed = 32000;
	else if (sc->ctrl[0] & CTRL1_FREQ44)
		speed = 44100;
	else 
		speed = 0;
	//device_printf(sc->dev, "Playing at speed %d\n", (sc->ctrl[0] & CTRL1_DS) ? speed << 1 : speed);
	//device_printf(sc->dev, "ADAT Mode: %s\n", (sc->ctrl[0] & CTRL1_ADAT) ? "yes" : "no");
	//device_printf(sc->dev, "Play format %s\n", ch->fmt & AFMT_S16_LE ? "S16LE" : (ch->fmt & AFMT_S32_LE ? "S32LE" : "Unknown"));
}

static void
digi96_prepare_input(struct digi96_chinfo *ch) {
	struct digi96_info *sc = ch->parent;
	//u_int32_t cmd = sc->ctrl[0];

	if (ch->dir != PCMDIR_REC) {
		return;
	}
	digi96_write(sc, DIGI96_RESET_REC, 0);
}

static int
digi96_chan_trigger(kobj_t obj, void *data, int go)
{
	struct digi96_chinfo *ch = data;
	struct digi96_info *sc = ch->parent;
	u_int32_t cmd = sc->ctrl[0];
	
	if (!PCMTRIG_COMMON(go))
		return 0;

	digi96_lock(sc);
	switch (go) {
		case PCMTRIG_START:
			device_printf(sc->dev, "buffer alignment %d\n", sndbuf_getalign(ch->buffer));
			//device_printf(sc->dev, "Starting chan\n");
			if (ch->dir == PCMDIR_PLAY) {
				digi96_prepare_output(ch);
				cmd |= CTRL1_STARTPLAY;
			} else if (ch->dir == PCMDIR_REC) {
				digi96_prepare_input(ch);
				cmd |= CTRL1_STARTREC;
			}
			digi96_ctrl1(sc, cmd);
			//device_printf(sc->dev, "Chan started\n");
			CTR2(KTR_DIGI, "Chan started, %dKhz sampling rate, %dBit resolution\n", ch->spd, ch->fmt & AFMT_S16LE ? 16 : 24);
			break;
		case PCMTRIG_STOP:
		case PCMTRIG_ABORT:
		default:
			if (ch->dir == PCMDIR_PLAY)
				cmd &= ~CTRL1_STARTPLAY;
			else if (ch->dir == PCMDIR_REC)
				cmd &= ~CTRL1_STARTREC;
			digi96_ctrl1(sc, cmd);
			//device_printf(sc->dev, "Stop chan\n");
			break;
	}
	digi96_unlock(sc);
	return (0);
}

static kobj_method_t digi96_chan_methods[] = {
    	KOBJMETHOD(channel_init,		digi96_chan_init),
    	KOBJMETHOD(channel_setformat,		digi96_chan_setformat),
    	KOBJMETHOD(channel_getcaps,		digi96_chan_getcaps),
    	KOBJMETHOD(channel_setspeed,		digi96_chan_setspeed),
    	KOBJMETHOD(channel_setblocksize,	digi96_chan_setblocksize),
    	KOBJMETHOD(channel_getptr,		digi96_chan_getptr),
    	KOBJMETHOD(channel_trigger,		digi96_chan_trigger),
	KOBJMETHOD_END
};
CHANNEL_DECLARE(digi96_chan);

/*
 * XXX: GPLv2
 * The CDATA, CCLK and CLATCH bits can be used to write to the SPI interface
 * of the AD1852 or AD1852 D/A converter on the board.  CDATA must be set up
 * on the falling edge of CCLK and be stable on the rising edge.  The rising
 * edge of CLATCH after the last data bit clocks in the whole data word.
 * A fast processor could probably drive the SPI interface faster than the
 * DAC can handle (3MHz for the 1855, unknown for the 1852).  The DELAY(1)
 * limits the data rate to 500KHz and only causes a delay of 33 microsecs.
 *
 * NOTE: increased delay from 1 to 10, since there where problems setting
 * the volume.
 */
static void
digi96_write_SPI(struct digi96_info *sc, uint16_t val)
{
	int i;
	uint32_t reg = sc->ctrl[1];

	for (i = 0; i < 16; i++) {
		if (val & 0x8000) {
			reg |= CTRL2_CDATA;
		} else {
			reg &= ~CTRL2_CDATA;
		}
		reg &= ~(CTRL2_CCLK | CTRL2_CLATCH);
		digi96_ctrl2(sc, reg);
		DELAY(10);
		reg |= CTRL2_CCLK;
		digi96_ctrl2(sc, reg);
		DELAY(10);
		val <<= 1;
	}
	reg &= ~(CTRL2_CCLK | CTRL2_CDATA);
	reg |= CTRL2_CLATCH;
	digi96_ctrl2(sc, reg);
	DELAY(10);
	reg &= ~CTRL2_CLATCH;
	digi96_ctrl2(sc, reg);
}

/* XXX: GPLv2 */
static void
digi96_apply_volume(struct digi96_info *sc)
{
	if (DIGI96_DAC_IS_1852(sc)) {
		digi96_write_SPI(sc, (sc->vol[0] << 2));
		digi96_write_SPI(sc, (sc->vol[1] << 2) | 0x2);
	} else if (DIGI96_DAC_IS_1855(sc)) {
		digi96_write_SPI(sc, (sc->vol[0] & 0x3ff));
		digi96_write_SPI(sc, (sc->vol[0] & 0x3ff) | 0x400);
	}
}

static void
digi96_apply_attenuation(struct digi96_info *sc)
{
	uint32_t reg = sc->ctrl[0] & ~CTRL1_GAIN;

	reg |= ((sc->attenuation << 2) & CTRL1_GAIN);
	digi96_ctrl1(sc, reg);
}

static int
digi96_mixer_init(struct snd_mixer *m)
{
	struct digi96_info *sc = mix_getdevinfo(m);

	//device_printf(sc->dev, "Initializing mixer..\n");
	mix_setdevs(m, SOUND_MASK_VOLUME | SOUND_MASK_OGAIN);

	sc->vol[0] = sc->vol[1] = 0;
	sc->attenuation = 0;
	digi96_apply_attenuation(sc);
	if (DIGI96_HAS_ANALOG_OUT(sc))
		digi96_apply_volume(sc);
	return (0);
}

static int
digi96_mixer_set(struct snd_mixer *m, unsigned dev, unsigned left, unsigned right)
{
	struct digi96_info *sc = mix_getdevinfo(m);
	uint16_t maxvol;

	maxvol = DIGI96_185X_MAX_OUT(sc);

	switch (dev) {
		case SOUND_MIXER_VOLUME:
			if (left < maxvol && left != sc->vol[0]) {
				sc->vol[0] = maxvol * left / 100;
				digi96_apply_volume(sc);
			}
			if (right < maxvol && right != sc->vol[1]) {
				sc->vol[1] = maxvol * right / 100;
				digi96_apply_volume(sc);
			}
			break;
		case SOUND_MIXER_OGAIN:
			if ((left = right) == 100)
				left--;
			sc->attenuation = 3 - (left / 25);
			digi96_apply_attenuation(sc);
			break;
	}
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

static void
digi96_setclockmode(struct digi96_info *sc, int mode) {
	uint32_t reg[2];

	reg[0] = sc->ctrl[0];
	reg[1] = sc->ctrl[1];
	switch (mode) {
		case DIGI96_CLOCKMODE_SLAVE: /* AutoSync */
			reg[0] &= ~CTRL1_MASTER;
			reg[1] &= ~CTRL2_WSEL;
			break;
		case DIGI96_CLOCKMODE_MASTER: /* Internal */
			reg[0] |= CTRL1_MASTER;
			reg[1] &= ~CTRL2_WSEL;
			break;
		case DIGI96_CLOCKMODE_WORDCLOCK: /* Word clock */
			reg[0] |= CTRL1_MASTER;
			reg[1] |= CTRL2_WSEL;
			break;
	}
	digi96_ctrl1(sc, reg[0]);
	digi96_ctrl2(sc, reg[1]);
	sc->clockmode = mode;
}

static int
sysctl_digi96_clockmode(SYSCTL_HANDLER_ARGS) {
	struct digi96_info *sc;
	device_t dev;
	int val, err;

	dev = oidp->oid_arg1;
	sc = pcm_getdevinfo(dev);
	if (sc == NULL)
		return EINVAL;
	val = sc->clockmode;
	err = sysctl_handle_int(oidp, &val, 0, req);
	if (err || req->newptr == NULL)
		return (err);
	if (val < 0 || val > 2)
		return (EINVAL);
	digi96_setclockmode(sc, val);
	return err;
}

static int
digi96_attach(device_t dev)
{
	struct digi96_info *sc;
	uint16_t devid, revid;
	char status[SND_STATUSLEN];

	sc = malloc(sizeof(*sc), M_DEVBUF, M_WAITOK | M_ZERO);
	sc->lock = snd_mtxcreate(device_get_nameunit(dev), "snd_digi96 softc");
	sc->dev = dev;
	sc->devid = pci_get_device(dev);
	sc->revision = pci_read_config(dev, 8, 1);

	pci_enable_busmaster(dev);
	pci_enable_io(dev, SYS_RES_MEMORY);

	sc->regid = PCIR_BAR(0);
	sc->regtype = SYS_RES_MEMORY;
	sc->reg = bus_alloc_resource_any(dev, sc->regtype,
	    &sc->regid, RF_ACTIVE);
	if (!sc->reg) {
		//device_printf(dev, "unable to allocate register space\n");
		goto bad;
	}
	sc->st = rman_get_bustag(sc->reg);
	sc->sh = rman_get_bushandle(sc->reg);

	sc->irqid = 0;
	sc->irq = bus_alloc_resource_any(dev, SYS_RES_IRQ, &sc->irqid,
	    RF_ACTIVE | RF_SHAREABLE);
	if (!sc->irq || snd_setup_intr(dev, sc->irq, INTR_MPSAFE,
	    digi96_intr, sc, &sc->ih)) {
		//device_printf(dev, "unable to map interrupt\n");
		goto bad;
	}
	sc->bufsz = pcm_getbuffersize(dev, 2048, DIGI96_BUFSZ, DIGI96_BUFSZ);

	sc->have_adat = 0;
	sc->have_analog = 0;
	sc->mode = MD_SPDIF;
	sc->doublespeed = 0;

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
	digi96_ctrl2(sc, 0);
	/* reset DAC */
	digi96_ctrl1(sc, CTRL1_ISEL | CTRL1_FREQ48 | CTRL1_PD);
	digi96_ctrl1(sc, CTRL1_ISEL | CTRL1_FREQ48 | CTRL1_MASTER | CTRL1_SEL);
	digi96_ctrl2(sc, CTRL2_DAC_EN); /* soft unmute DAC */

	digi96_setclockmode(sc, DIGI96_CLOCKMODE_MASTER);
	if (mixer_init(dev, &digi96_mixer_class, sc))
		goto bad;
	if (pcm_register(dev, sc, 1, 0))
		goto bad;
	pcm_addchan(dev, PCMDIR_PLAY, &digi96_chan_class, sc);
	pcm_addchan(dev, PCMDIR_REC, &digi96_chan_class, sc);
	SYSCTL_ADD_PROC(device_get_sysctl_ctx(sc->dev),
			SYSCTL_CHILDREN(device_get_sysctl_tree(sc->dev)), OID_AUTO,
			"clockmode", CTLTYPE_INT | CTLFLAG_RW, sc->dev,
		       	sizeof(sc->dev), sysctl_digi96_clockmode, "I",
			"Set clock mode: 0 - slave, 1 - master, 2 - word clock");
	snprintf(status, SND_STATUSLEN, "at io 0x%lx irq %ld %s",
		 rman_get_start(sc->reg), rman_get_start(sc->irq),
		 PCM_KLDSTRING(snd_digi96));
	pcm_setstatus(dev, status);
	CTR0(KTR_DIGI, "DIGI96: Hello KTR world\n");
	return (0);
bad:
	digi96_uninit(sc);
	return (ENXIO);

}

void 
digi96_uninit(struct digi96_info *sc)
{
	digi96_ctrl2(sc, 0); /* soft mute */
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

static int
digi96_detach(device_t dev) {
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
