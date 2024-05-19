// The configurations for the example codes are based on the circuit
// "Figure 77. 3-Wire RTD Measurement" shown in the
// [ADS1220 data sheet](https://www.ti.com/jp/lit/ds/symlink/ads1220.pdf).
//
// If you want to use your own circuit and configurations,
// I recommend to read the following data sheet and application note first.
//
// - [ADS1220 Data Sheet](https://www.ti.com/jp/lit/ds/symlink/ads1220.pdf)
// - [TI Application Node: A Basic Guide to RTD Measurements](https://www.ti.com/lit/an/sbaa275a/sbaa275a.pdf)

#include <ADS1220.hpp>

ads1220::ADS1220 ads;
static constexpr uint8_t PIN_CS = SS;
static constexpr uint8_t PIN_DRDY = 2;

void setup()
{
    Serial.begin(115200);
    while (!Serial);
    delay(5000);
    Serial.println("Start ADS1220");

    SPI.begin();
    SPISettings spi_settings(1000000, MSBFIRST, SPI_MODE1);
    ads.begin(SPI, PIN_CS, PIN_DRDY, spi_settings);

    using namespace ads1220::config;
    ads.write_config_pga_bypass(pga_bypass::ENABLE);
    ads.write_config_pga_gain(gain::GAIN_1);
    ads.write_config_input_multiplexer(mux::AIN1_AIN0);
    ads.write_config_burn_out_current_sources(burn_out_current_sources::OFF);
    ads.write_config_temperature_sensor(temperature_sensor::ENABLE);
    ads.write_config_conversion_mode(conversion_mode::SINGLE_SHOT);
    ads.write_config_operating_mode(mode::NORMAL);
    ads.write_config_data_rate_normal(data_rate::NORMAL_600_SPS);
    ads.write_config_idac_current(idac::IDAC_OFF);
    ads.write_config_low_side_power_switch(low_side_power_switch::ALWAYS_OPEN);
    ads.write_config_fir_filter(fir_50_60::FIR_OFF);
    ads.write_config_vref_selection(vref::INTERNAL_2_048_V);
    ads.write_config_drdy_mode(drdy_mode::DRDY_ONLY);
    ads.write_config_idac2_routing(i2mux::DISABLE);
    ads.write_config_idac1_routing(i1mux::DISABLE);

    ads.print_config_registers();

    ads.begin_internal_temperature_sensor_mode();
}

void loop()
{
    if (ads.is_internal_temperature_sensor_mode()) {
        float temperature = ads.read_internal_temperature();
        Serial.print("internal temperature sensor: ");
        Serial.println(temperature, 2);
        delay(1000);
    }

    // Write anything from the serial console to toggle continuous conversion
    if (Serial.available() > 0) {
        // Discard read data
        while (Serial.available()) {
            Serial.read();
        }
        // Toggle internal temperature sensor mode
        if (ads.is_internal_temperature_sensor_mode()) {
            ads.end_internal_temperature_sensor_mode();
        } else {
            ads.begin_internal_temperature_sensor_mode();
        }
    }
}
