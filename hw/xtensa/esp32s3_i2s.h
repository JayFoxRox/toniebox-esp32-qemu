// (C)2024 Jannik Vogel

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/registerfields.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"

#pragma once

#include "hw/sysbus.h"
#include "hw/hw.h"
#include "hw/registerfields.h"

#define TYPE_ESP32S3_I2S "esp32s3.i2s"
#define esp32s3_i2s(obj)           OBJECT_CHECK(Esp32S3I2SState, (obj), TYPE_ESP32S3_I2S)
#define ESP32S3_I2S_GET_CLASS(obj) OBJECT_GET_CLASS(Esp32S3I2SClass, obj, TYPE_ESP32S3_I2S)
#define ESP32S3_I2S_CLASS(klass)   OBJECT_CLASS_CHECK(Esp32S3I2SClass, klass, TYPE_ESP32S3_I2S)


typedef struct Esp32S3I2SState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    qemu_irq irq;
} Esp32S3I2SState;

typedef struct Esp32S3I2SClass {
    SysBusDeviceClass parent_class;
} Esp32S3I2SClass;

static uint64_t esp32s3_i2s_read(void *opaque, hwaddr addr, unsigned int size)
{
    //Esp32S3I2SState *s = esp32s3_i2s(opaque);
    uint64_t r = 0;

    printf("\t\ti2s read addr=0x%lx size=%u  => 0x%lx\n", addr, size, r);
    return r;
}

static void esp32s3_i2s_write(void *opaque, hwaddr addr,
                       uint64_t value, unsigned int size)
{
  printf("\t\ti2s write addr=0x%lx size=%u <= 0x%lx\n", addr, size, value);
}

static const MemoryRegionOps i2s_ops = {
    .read =  esp32s3_i2s_read,
    .write = esp32s3_i2s_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp32s3_i2s_reset(DeviceState *dev)
{
}

static void esp32s3_i2s_realize(DeviceState *dev, Error **errp)
{
}

static void esp32s3_i2s_init(Object *obj)
{
    Esp32S3I2SState *s = esp32s3_i2s(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &i2s_ops, s,
                          TYPE_ESP32S3_I2S, 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
}

static Property esp32s3_i2s_properties[] = {
    DEFINE_PROP_END_OF_LIST(),
};

static void esp32s3_i2s_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = esp32s3_i2s_reset;
    dc->realize = esp32s3_i2s_realize;
    device_class_set_props(dc, esp32s3_i2s_properties);
}

static const TypeInfo esp32s3_i2s_info = {
    .name = TYPE_ESP32S3_I2S,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32S3I2SState),
    .instance_init = esp32s3_i2s_init,
    .class_init = esp32s3_i2s_class_init,
    .class_size = sizeof(Esp32S3I2SClass),
};

static void esp32s3_i2s_register_types(void)
{
    type_register_static(&esp32s3_i2s_info);
}

type_init(esp32s3_i2s_register_types)
