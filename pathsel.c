/*
 * pathsel.c
 *
 *  Created on: Dec 27, 2021
 *      Author: kosa
 */

#include "pathsel.h"
#include "hwr.h"
#include "gpio.h"
#include "timer.h"


void pathsel_init(void)
{
	gpio_set(&VCO_EN_PORT, VCO_EN_PIN, 1); //enable
	gpio_setup(&VCO_EN_PORT, VCO_EN_PIN, true, true, true); // output,push-pull,fast

	gpio_set(&PA_V_EN_PORT, PA_V_EN_PIN, 0); //disable
	gpio_setup(&PA_V_EN_PORT, PA_V_EN_PIN, true, true, true); // output,push-pull,fast

	gpio_set(&PA_BIAS_EN_PORT, PA_BIAS_EN_PIN, 0); //disable
	gpio_setup(&PA_BIAS_EN_PORT, PA_BIAS_EN_PIN, true, true, true); // output,push-pull,fast

	gpio_set(&TX_EN_PORT, TX_EN_PIN, 0); //disable
	gpio_setup(&TX_EN_PORT, TX_EN_PIN, true, true, true); // output,push-pull,fast

	gpio_set(&RX_EN_PORT, RX_EN_PIN, 1); //enable
	gpio_setup(&RX_EN_PORT, RX_EN_PIN, true, true, true); // output,push-pull,fast

	gpio_set(&RX_LNA_EN_PORT, RX_LNA_EN_PIN, 1); //enable
	gpio_setup(&RX_LNA_EN_PORT, RX_LNA_EN_PIN, true, true, true); // output,push-pull,fast

	gpio_set(&M0_PORT, M0_PIN, 0);
	gpio_setup(&M0_PORT, M0_PIN, true, true, true); // output,push-pull,fast

}


void pathsel_rx(void)
{
	gpio_set(&PA_BIAS_EN_PORT, PA_BIAS_EN_PIN, 0); //disable
	gpio_set(&PA_V_EN_PORT, PA_V_EN_PIN, 0); //disable

	Timer dcdcoff_dly;
	timer_init(&dcdcoff_dly,20);
	while(!timer_async(&dcdcoff_dly));

	gpio_set(&TX_EN_PORT, TX_EN_PIN, 0); //disable
	gpio_set(&RX_EN_PORT, RX_EN_PIN, 1); //enable

	gpio_set(&M0_PORT, M0_PIN, 0);

	gpio_set(&RX_LNA_EN_PORT, RX_LNA_EN_PIN, 1); //enable

}

void pathsel_tx(void)
{
	gpio_set(&RX_LNA_EN_PORT, RX_LNA_EN_PIN, 0); //disable

	gpio_set(&TX_EN_PORT, TX_EN_PIN, 1); //enable
	gpio_set(&RX_EN_PORT, RX_EN_PIN, 0); //disable

	gpio_set(&M0_PORT, M0_PIN, 1);

	gpio_set(&PA_V_EN_PORT, PA_V_EN_PIN, 1); //enable

	Timer dcdcon_dly;
	timer_init(&dcdcon_dly,50);
	while(!timer_async(&dcdcon_dly));

	gpio_set(&PA_BIAS_EN_PORT, PA_BIAS_EN_PIN, 1); //enable

}
