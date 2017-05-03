#ifndef __CHIPS_PLATFORM_H__
#define __CHIPS_PLATFORM_H__

int chips_parse_dts(struct chips_data* chips_data);
void chips_release_gpio(struct chips_data* chips_data);
int chips_hw_reset(struct chips_data *chips_data, unsigned int delay_ms);
int chips_get_irqno(struct chips_data *chips_data);
int chips_set_spi_mode(struct chips_data *chips_data);
int chips_pwr_up(struct chips_data *chips_dev);
int chips_set_reset_gpio(struct chips_data *chips_data, unsigned int level);

#endif 