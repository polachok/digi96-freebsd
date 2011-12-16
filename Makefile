# $FreeBSD: src/sys/modules/sound/driver/atiixp/Makefile,v 1.1.18.1 2011/09/23 00:51:37 kensmith Exp $

#.PATH: ${.CURDIR}/../../../../dev/sound/pci
.PATH: /usr/src/sys/dev/sound/pci

KMOD=	snd_digi96
SRCS=	device_if.h bus_if.h pci_if.h channel_if.h mixer_if.h
SRCS+=	digi96.c

.include <bsd.kmod.mk>
