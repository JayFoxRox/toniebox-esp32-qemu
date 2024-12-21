/*
 * ESP32 GPIO emulation
 *
 * Copyright (c) 2019 Espressif Systems (Shanghai) Co. Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 or
 * (at your option) any later version.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "hw/hw.h"
#include "hw/sysbus.h"
#include "hw/registerfields.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "hw/gpio/esp32_gpio.h"



static uint64_t esp32_gpio_read(void *opaque, hwaddr addr, unsigned int size)
{
    Esp32GpioState *s = ESP32_GPIO(opaque);
    uint64_t r = 0;
    switch (addr) {
    case A_GPIO_STRAP:
        r = s->strap_mode;
        break;

    default:
        break;
    }

    printf("\t\tgpio read addr=0x%lx size=%u  => 0x%lx\n", addr, size, r);
    return r;
}

static void esp32_gpio_write(void *opaque, hwaddr addr,
                       uint64_t value, unsigned int size)
{
  printf("\t\tgpio write addr=0x%lx size=%u <= 0x%lx\n", addr, size, value);


#if 0
GPIO_OUT_REG GPIO0 ~ 31 output register 0x0004 R/W
GPIO_OUT_W1TS_REG GPIO0 ~ 31 output bit set register 0x0008 WO
GPIO_OUT_W1TC_REG GPIO0 ~ 31 output bit clear register 0x000C WO

GPIO_OUT1_REG GPIO32 ~ 48 output register 0x0010 R/W
GPIO_OUT1_W1TS_REG GPIO32 ~ 48 output bit set register 0x0014 WO
GPIO_OUT1_W1TC_REG GPIO32 ~ 48 output bit clear register 0x0018 WO


GPIO_ENABLE_REG GPIO0 ~ 31 output enable register 0x0020 R/W
GPIO_ENABLE_W1TS_REG GPIO0 ~ 31 output enable bit set register 0x0024 WO
GPIO_ENABLE_W1TC_REG GPIO0 ~ 31 output enable bit clear register 0x0028 WO

GPIO_ENABLE1_REG GPIO32 ~ 48 output enable register 0x002C R/W
GPIO_ENABLE1_W1TS_REG GPIO32 ~ 48 output enable bit set register 0x0030 WO
GPIO_ENABLE1_W1TC_REG GPIO32 ~ 48 output enable bit clear register 0x0034 
#endif

  if ((addr >= 0x74) && (addr <= 0x0134)) {
    int gpio = (addr - 0x74) / 4;
    printf("\t\t\tGPIO%d configuration\n", gpio);
  } else if ((addr >= 0x554) && (addr <= 0x0614)) {
    int gpio = (addr - 0x554) / 4;
    printf("\t\t\tGPIO%d peripheral output selection\n", gpio);
  } else {
    // nop
  }

}

static const MemoryRegionOps uart_ops = {
    .read =  esp32_gpio_read,
    .write = esp32_gpio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void esp32_gpio_reset(DeviceState *dev)
{
}

static void esp32_gpio_realize(DeviceState *dev, Error **errp)
{
}

static void esp32_gpio_init(Object *obj)
{
    Esp32GpioState *s = ESP32_GPIO(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    /* Set the default value for the strap_mode property */
    object_property_set_int(obj, "strap_mode", ESP32_STRAP_MODE_FLASH_BOOT, &error_fatal);

    memory_region_init_io(&s->iomem, obj, &uart_ops, s,
                          TYPE_ESP32_GPIO, 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq);
}

static Property esp32_gpio_properties[] = {
    /* The strap_mode needs to be explicitly set in the instance init, thus, set
     * the default value to 0. */
    DEFINE_PROP_UINT32("strap_mode", Esp32GpioState, strap_mode, 0),
    DEFINE_PROP_END_OF_LIST(),
};

static void esp32_gpio_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = esp32_gpio_reset;
    dc->realize = esp32_gpio_realize;
    device_class_set_props(dc, esp32_gpio_properties);
}

static const TypeInfo esp32_gpio_info = {
    .name = TYPE_ESP32_GPIO,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32GpioState),
    .instance_init = esp32_gpio_init,
    .class_init = esp32_gpio_class_init,
    .class_size = sizeof(Esp32GpioClass),
};

static void esp32_gpio_register_types(void)
{
    type_register_static(&esp32_gpio_info);
}

type_init(esp32_gpio_register_types)
