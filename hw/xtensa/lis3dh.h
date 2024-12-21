/*
 * (C)2024 Jannik Vogel
 * Stub for implementing LIS3DH emulation (based on twl92230.c for the i2c skeleton)
 *
 * Copyright (C) 2008 Nokia Corporation
 * Written by Andrzej Zaborowski <andrew@openedhand.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 or
 * (at your option) version 3 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#include "qemu/osdep.h"
#include "qemu/timer.h"
#include "hw/i2c/i2c.h"
#include "hw/irq.h"
#include "migration/qemu-file-types.h"
#include "sysemu/sysemu.h"
#include "sysemu/rtc.h"
#include "qemu/bcd.h"
#include "qemu/module.h"
#include "qom/object.h"

#define VERBOSE 1

#define TYPE_LIS3DH "lis3dh"
OBJECT_DECLARE_SIMPLE_TYPE(Lis3dhState, LIS3DH)

struct Lis3dhState {
    I2CSlave parent_obj;

    int firstbyte;
    uint8_t reg;

#if 0
    uint8_t vcore[5];
    uint8_t dcdc[3];
    uint8_t ldo[8];
    uint8_t sleep[2];
    uint8_t osc;
    uint8_t detect;
    uint16_t mask;
    uint16_t status;
    uint8_t dir;
    uint8_t inputs;
    uint8_t outputs;
    uint8_t bbsms;
    uint8_t pull[4];
    uint8_t mmc_ctrl[3];
    uint8_t mmc_debounce;
    struct {
        uint8_t ctrl;
        uint16_t comp;
        QEMUTimer *hz_tm;
        int64_t next;
        struct tm tm;
        struct tm new;
        struct tm alm;
        int64_t sec_offset;
        int64_t alm_sec;
        int next_comp;
    } rtc;
    uint16_t rtc_next_vmstate;
    qemu_irq out[4];
    uint8_t pwrbtn_state;
#endif
};

static inline void lis3dh_update(Lis3dhState *s)
{
    //qemu_set_irq(s->out[3], s->status & ~s->mask);
}

static void lis3dh_reset(I2CSlave *i2c)
{
    Lis3dhState *s = LIS3DH(i2c);

#if 0
    s->reg = 0x00;

    s->vcore[0] = 0x0c; /* XXX: X-loader needs 0x8c? check!  */
    s->vcore[1] = 0x05;
    s->vcore[2] = 0x02;
    s->vcore[3] = 0x0c;
    s->vcore[4] = 0x03;
    s->dcdc[0] = 0x33;  /* Depends on wiring */
    s->dcdc[1] = 0x03;
    s->dcdc[2] = 0x00;
    s->ldo[0] = 0x95;
    s->ldo[1] = 0x7e;
    s->ldo[2] = 0x00;
    s->ldo[3] = 0x00;   /* Depends on wiring */
    s->ldo[4] = 0x03;   /* Depends on wiring */
    s->ldo[5] = 0x00;
    s->ldo[6] = 0x00;
    s->ldo[7] = 0x00;
    s->sleep[0] = 0x00;
    s->sleep[1] = 0x00;
    s->osc = 0x01;
    s->detect = 0x09;
    s->mask = 0x0fff;
    s->status = 0;
    s->dir = 0x07;
    s->outputs = 0x00;
    s->bbsms = 0x00;
    s->pull[0] = 0x00;
    s->pull[1] = 0x00;
    s->pull[2] = 0x00;
    s->pull[3] = 0x00;
    s->mmc_ctrl[0] = 0x03;
    s->mmc_ctrl[1] = 0xc0;
    s->mmc_ctrl[2] = 0x00;
    s->mmc_debounce = 0x05;

    if (s->rtc.ctrl & 1)
        lis3dh_rtc_stop(s);
    s->rtc.ctrl = 0x00;
    s->rtc.comp = 0x0000;
    s->rtc.next = 1000;
    s->rtc.sec_offset = 0;
    s->rtc.next_comp = 1800;
    s->rtc.alm_sec = 1800;
    s->rtc.alm.tm_sec = 0x00;
    s->rtc.alm.tm_min = 0x00;
    s->rtc.alm.tm_hour = 0x00;
    s->rtc.alm.tm_mday = 0x01;
    s->rtc.alm.tm_mon = 0x00;
    s->rtc.alm.tm_year = 2004;
#endif

    lis3dh_update(s);
}

#if 0
static void lis3dh_gpio_set(void *opaque, int line, int level)
{
    Lis3dhState *s = (Lis3dhState *) opaque;

    if (line < 3) {
        /* No interrupt generated */
        s->inputs &= ~(1 << line);
        s->inputs |= level << line;
        return;
    }

    if (!s->pwrbtn_state && level) {
        s->status |= 1 << 11;               /* PSHBTN */
        lis3dh_update(s);
    }
    s->pwrbtn_state = level;
}
#endif

#define LIS3DH_REG 0xAA

static uint8_t lis3dh_read(void *opaque, uint8_t addr)
{
    //Lis3dhState *s = (Lis3dhState *) opaque;

    switch (addr) {
    case LIS3DH_REG:
        return 0x0;

    default:
#ifdef VERBOSE
        printf("%s: unknown register %02x\n", __func__, addr);
#endif
        break;
    }
    return 0;
}

static void lis3dh_write(void *opaque, uint8_t addr, uint8_t value)
{
    //Lis3dhState *s = (Lis3dhState *) opaque;

    switch (addr) {
    case LIS3DH_REG:
        //s->vcore[0] = (value & 0xe) | MIN(value & 0x1f, 0x12);
        break;

    default:
#ifdef VERBOSE
        printf("%s: unknown register %02x\n", __func__, addr);
#endif
        break;
    }
}

static int lis3dh_event(I2CSlave *i2c, enum i2c_event event)
{
    Lis3dhState *s = LIS3DH(i2c);

    printf("event!\n");
    if (event == I2C_START_SEND)
        s->firstbyte = 1;

    return 0;
}

static int lis3dh_tx(I2CSlave *i2c, uint8_t data)
{
    Lis3dhState *s = LIS3DH(i2c);

    /* Interpret register address byte */
    if (s->firstbyte) {
        s->reg = data;
        s->firstbyte = 0;
    } else
        lis3dh_write(s, s->reg ++, data);

    return 0;
}

static uint8_t lis3dh_rx(I2CSlave *i2c)
{
    Lis3dhState *s = LIS3DH(i2c);

    return lis3dh_read(s, s->reg ++);
}


static void lis3dh_realize(DeviceState *dev, Error **errp)
{
//    Lis3dhState *s = LIS3DH(dev);

#if 0
    s->rtc.hz_tm = timer_new_ms(rtc_clock, lis3dh_rtc_hz, s);
    /* Three output pins plus one interrupt pin.  */
    qdev_init_gpio_out(dev, s->out, 4);

    /* Three input pins plus one power-button pin.  */
    qdev_init_gpio_in(dev, lis3dh_gpio_set, 4);
#endif

    lis3dh_reset(I2C_SLAVE(dev));
}

static void lis3dh_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    I2CSlaveClass *sc = I2C_SLAVE_CLASS(klass);

    dc->realize = lis3dh_realize;
    sc->event = lis3dh_event;
    sc->recv = lis3dh_rx;
    sc->send = lis3dh_tx;
}

static const TypeInfo lis3dh_info = {
    .name          = TYPE_LIS3DH,
    .parent        = TYPE_I2C_SLAVE,
    .instance_size = sizeof(Lis3dhState),
    .class_init    = lis3dh_class_init,
};

static void lis3dh_register_types(void)
{
    type_register_static(&lis3dh_info);
}

type_init(lis3dh_register_types)
