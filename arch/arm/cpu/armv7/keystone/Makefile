#
# Copyright (C) 2012 Texas Instruments
#
# See file CREDITS for list of people who contributed to this
# project.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
#

include $(TOPDIR)/config.mk

LIB	= $(obj)lib$(SOC).o

COBJS	+= aemif.o init.o psc.o clock.o cmd_clock.o cppi_dma.o
ifdef CONFIG_SOC_TCI6614
COBJS	+= timer.o
endif
ifdef CONFIG_SOC_TCI6638
COBJS	+= cmd_mon.o
COBJS	+= cmd_fmtimg.o
COBJS	+= msmc.o
endif
ifdef CONFIG_SPL_BUILD
COBJS	+= spl.o
endif

SOBJS	+= lowlevel_init.o

SRCS	:= $(START:.o=.S) $(SOBJS:.o=.S) $(COBJS:.o=.c)

OBJS	:= $(addprefix $(obj),$(COBJS) $(SOBJS))
START	:= $(addprefix $(obj),$(START))

all:	$(obj).depend $(LIB)

$(LIB):	$(OBJS)
	$(call cmd_link_o_target, $(OBJS))

#########################################################################

# defines $(obj).depend target
include $(SRCTREE)/rules.mk

sinclude $(obj).depend

#########################################################################
