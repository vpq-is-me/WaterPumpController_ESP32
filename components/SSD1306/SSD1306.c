/*
 * SPDX-FileCopyrightText: 2015-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "driver/i2c.h"
#include "ssd1306.h"
#include "string.h" // for memset

#define SSD1306_WRITE_CMD           (0x00)
#define SSD1306_WRITE_DAT           (0x40)

typedef struct {
    i2c_port_t bus;
    uint16_t dev_addr;
    uint8_t s_chDisplayBuffer[128][SSD1306_HEIGHT/8];
} ssd1306_dev_t;

static uint32_t _pow(uint8_t m, uint8_t n)
{
    uint32_t result = 1;
    while (n--) {
        result *= m;
    }
    return result;
}

static esp_err_t ssd1306_write_data(ssd1306_handle_t dev, const uint8_t *const data, const uint16_t data_len)
{
    ssd1306_dev_t *device = (ssd1306_dev_t *) dev;
    esp_err_t ret;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ret = i2c_master_start(cmd);
    assert(ESP_OK == ret);
    ret = i2c_master_write_byte(cmd, device->dev_addr | I2C_MASTER_WRITE, true);
    assert(ESP_OK == ret);
    ret = i2c_master_write_byte(cmd, SSD1306_WRITE_DAT, true);
    assert(ESP_OK == ret);
    ret = i2c_master_write(cmd, data, data_len, true);
    assert(ESP_OK == ret);
    ret = i2c_master_stop(cmd);
    assert(ESP_OK == ret);
    ret = i2c_master_cmd_begin(device->bus, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

static esp_err_t ssd1306_write_cmd(ssd1306_handle_t dev, const uint8_t *const data, const uint16_t data_len)
{
    ssd1306_dev_t *device = (ssd1306_dev_t *) dev;
    esp_err_t ret;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    ret = i2c_master_start(cmd);
    assert(ESP_OK == ret);
    ret = i2c_master_write_byte(cmd, device->dev_addr | I2C_MASTER_WRITE, true);
    assert(ESP_OK == ret);
    ret = i2c_master_write_byte(cmd, SSD1306_WRITE_CMD, true);
    assert(ESP_OK == ret);
    ret = i2c_master_write(cmd, data, data_len, true);
    assert(ESP_OK == ret);
    ret = i2c_master_stop(cmd);
    assert(ESP_OK == ret);
    ret = i2c_master_cmd_begin(device->bus, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}

static inline esp_err_t ssd1306_write_cmd_byte(ssd1306_handle_t dev, const uint8_t cmd)
{
    return ssd1306_write_cmd(dev, &cmd, 1);
}

void ssd1306_fill_rectangle(ssd1306_handle_t dev, uint8_t chXpos1,
                            uint8_t chYpos1, uint8_t chXpos2, uint8_t chYpos2, uint8_t chDot)
{
    uint8_t chXpos, chYpos;

    for (chXpos = chXpos1; chXpos <= chXpos2; chXpos++) {
        for (chYpos = chYpos1; chYpos <= chYpos2; chYpos++) {
            ssd1306_fill_point(dev, chXpos, chYpos, chDot);
        }
    }
}

void ssd1306_draw_num(ssd1306_handle_t dev, uint8_t chXpos, uint8_t chYpos,
                      uint32_t chNum, uint8_t chLen, uint8_t chSize)
{
    uint8_t i;
    uint8_t chTemp, chShow = 0;

    for (i = 0; i < chLen; i++) {
        chTemp = (chNum / _pow(10, chLen - i - 1)) % 10;
        if (chShow == 0 && i < (chLen - 1)) {
            if (chTemp == 0) {
                ssd1306_draw_char(dev, chXpos + (chSize / 2) * i, chYpos,
                                  ' ', chSize, 1);
                continue;
            } else {
                chShow = 1;
            }
        }
        ssd1306_draw_char(dev, chXpos + (chSize / 2) * i, chYpos,
                          chTemp + '0', chSize, 1);
    }
}

void ssd1306_draw_char(ssd1306_handle_t dev, uint8_t chXpos, uint8_t chYpos,
                       uint8_t chChr, uint8_t chSize, uint8_t chMode)
{
    uint8_t i, j;
    uint8_t chTemp, chYpos0 = chYpos;

    chChr = chChr - ' ';
    for (i = 0; i < chSize; i++) {
        if (chSize == 12) {
            if (chMode) {
                chTemp = c_chFont1206[chChr][i];
            } else {
                chTemp = ~c_chFont1206[chChr][i];
            }
        } else {
            if (chMode) {
                chTemp = c_chFont1608[chChr][i];
            } else {
                chTemp = ~c_chFont1608[chChr][i];
            }
        }

        for (j = 0; j < 8; j++) {
            if (chTemp & 0x80) {
                ssd1306_fill_point(dev, chXpos, chYpos, 1);
            } else {
                ssd1306_fill_point(dev, chXpos, chYpos, 0);
            }
            chTemp <<= 1;
            chYpos++;

            if ((chYpos - chYpos0) == chSize) {
                chYpos = chYpos0;
                chXpos++;
                break;
            }
        }
    }
}

void ssd1306_draw_string(ssd1306_handle_t dev, uint8_t chXpos, uint8_t chYpos,
                         const uint8_t *pchString, uint8_t chSize, uint8_t chMode)
{
    while (*pchString != '\0') {
        if (chXpos > (SSD1306_WIDTH - chSize / 2)) {
            chXpos = 0;
            chYpos += chSize;
            if (chYpos > (SSD1306_HEIGHT - chSize)) {
                chYpos = chXpos = 0;
                ssd1306_clear_screen(dev, 0x00);
            }
        }
        ssd1306_draw_char(dev, chXpos, chYpos, *pchString, chSize, chMode);
        chXpos += chSize / 2;
        pchString++;
    }
}

void ssd1306_fill_point(ssd1306_handle_t dev, uint8_t chXpos, uint8_t chYpos, uint8_t chPoint)
{
    ssd1306_dev_t *device = (ssd1306_dev_t *) dev;
    uint8_t chPos, chTemp = 0;

    if (chXpos > 127 || chYpos > (SSD1306_HEIGHT-1)) {
        return;
    }
    chPos = chYpos / 8;
    chTemp = 1 << (chYpos % 8);

    if (chPoint) {
        device->s_chDisplayBuffer[chXpos][chPos] |= chTemp;
    } else {
        device->s_chDisplayBuffer[chXpos][chPos] &= ~chTemp;
    }
}

void ssd1306_draw_1612char(ssd1306_handle_t dev, uint8_t chXpos, uint8_t chYpos, uint8_t chChar)
{
    uint8_t i, j;
    uint8_t chTemp = 0, chYpos0 = chYpos, chMode = 0;
    uint8_t max_i=chChar=='.'?14:24;
    for (i = 0; i < max_i; i++) {
        chTemp = chChar=='.'? c_chColon1604[i] : c_chFont1612[chChar - 0x30][i];
        for (j = 0; j < 8; j++) {
            chMode = chTemp & 0x80 ? 1 : 0;
            ssd1306_fill_point(dev, chXpos, chYpos, chMode);
            chTemp <<= 1;
            chYpos++;
            if ((chYpos - chYpos0) == 16) {
                chYpos = chYpos0;
                chXpos++;
                break;
            }
        }
    }
}

void ssd1306_draw_3216char(ssd1306_handle_t dev, uint8_t chXpos, uint8_t chYpos, uint8_t chChar)
{
    uint8_t i, j;
    uint8_t chTemp = 0, chYpos0 = chYpos, chMode = 0;
    int max_i=chChar=='.' ? 40 : 64;
    for (i = 0; i < max_i; i++) {
        chTemp = chChar=='.' ? c_chColon3210[i] : c_chFont3216[chChar - 0x30][i];
        for (j = 0; j < 8; j++) {
            chMode = chTemp & 0x80 ? 1 : 0;
            ssd1306_fill_point(dev, chXpos, chYpos, chMode);
            chTemp <<= 1;
            chYpos++;
            if ((chYpos - chYpos0) == 32) {
                chYpos = chYpos0;
                chXpos++;
                break;
            }
        }
    }
}

void ssd1306_draw_bitmap(ssd1306_handle_t dev, uint8_t chXpos, uint8_t chYpos,
                         const uint8_t *pchBmp, uint8_t chWidth, uint8_t chHeight)
{
    uint16_t i, j, byteWidth = (chWidth + 7) / 8;

    for (j = 0; j < chHeight; j++) {
        for (i = 0; i < chWidth; i++) {
            if (*(pchBmp + j * byteWidth + i / 8) & (128 >> (i & 7))) {
                ssd1306_fill_point(dev, chXpos + i, chYpos + j, 1);
            }
        }
    }
}


esp_err_t ssd1306_init(ssd1306_handle_t dev)
{
    esp_err_t ret;

    ssd1306_write_cmd_byte(dev, 0xAE); //--turn off oled panel
    ssd1306_write_cmd_byte(dev, 0x40); //--set start line address  Set Mapping RAM Display Start Line (0x00~0x3F)
    const uint8_t contr[2] = {0x81, 0x7f};
    ssd1306_write_cmd(dev, contr,2); //--set contrast control register
    ssd1306_write_cmd_byte(dev, 0xA1); //--Set SEG/Column Mapping
    ssd1306_write_cmd_byte(dev, 0xC8); //Set COM/Row Scan Direction
    ssd1306_write_cmd_byte(dev, 0xA6); //--set normal display
    const uint8_t mux_ratio[2] = {0xA8, (SSD1306_HEIGHT-1)};
    ssd1306_write_cmd(dev, mux_ratio,2); //--set multiplex ratio(1 to 64)
    const uint8_t clock[2] = {0xd5, 0x81};
    ssd1306_write_cmd(dev, clock,2); //--set display clock divide ratio/oscillator frequency
    const uint8_t precharge[2] = {0xD9, 0x22};
    ssd1306_write_cmd(dev, precharge,2); //--set pre-charge period
    const uint8_t com_pins[2] = {0xDA, 0x00};
    ssd1306_write_cmd(dev, com_pins,2); //--set com pins hardware configuration
    const uint8_t com_level[2] = {0xDB, 0x20};
    ssd1306_write_cmd(dev, com_level,2); //--set vcomh
    ssd1306_write_cmd_byte(dev, 0x40); //Set display start line
    const uint8_t chg_pump[2] = {0x8d, 0x14};
    ssd1306_write_cmd(dev, chg_pump,2); //--set Charge Pump enable/disable !
    ssd1306_write_cmd_byte(dev, 0xA4); // Disable Entire Display On (0xa4/0xa5)
    ssd1306_write_cmd_byte(dev, 0xA6); // Disable Inverse Display On (0xa6/a7)

    const uint8_t cmd[2] = {0x20, 1}; //-- set vertical adressing mode
    ssd1306_write_cmd(dev, cmd, sizeof(cmd));

    uint8_t cmd2[3] = {0x21, 0, 127};
    ssd1306_write_cmd(dev, cmd2, sizeof(cmd2)); //--set column address to zero
    cmd2[0] = 0x22;
    cmd2[2] = SSD1306_HEIGHT/8-1;
    ssd1306_write_cmd(dev, cmd2, sizeof(cmd2)); //--set row address to zero

    ret = ssd1306_write_cmd_byte(dev, 0xAF); //--turn on oled panel

    ssd1306_clear_screen(dev, 0x00);
    return ret;
}

ssd1306_handle_t ssd1306_create(i2c_port_t bus, uint16_t dev_addr)
{
    ssd1306_dev_t *dev = (ssd1306_dev_t *) calloc(1, sizeof(ssd1306_dev_t));
    dev->bus = bus;
    dev->dev_addr = dev_addr << 1;
    ssd1306_init((ssd1306_handle_t) dev);
    return (ssd1306_handle_t) dev;
}

void ssd1306_delete(ssd1306_handle_t dev)
{
    ssd1306_dev_t *device = (ssd1306_dev_t *) dev;
    free(device);
}

esp_err_t ssd1306_refresh_gram(ssd1306_handle_t dev)
{
    ssd1306_dev_t *device = (ssd1306_dev_t *) dev;
    return ssd1306_write_data(dev, &device->s_chDisplayBuffer[0][0], sizeof(device->s_chDisplayBuffer));
}

void ssd1306_clear_screen(ssd1306_handle_t dev, uint8_t chFill)
{
    ssd1306_dev_t *device = (ssd1306_dev_t *) dev;
    memset(device->s_chDisplayBuffer, chFill, sizeof(device->s_chDisplayBuffer));
}

