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
static constexpr float R_REF_OHMS = 2400;

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
    ads.write_config_temperature_sensor(temperature_sensor::DISABLE);
    ads.write_config_conversion_mode(conversion_mode::SINGLE_SHOT);
    ads.write_config_operating_mode(mode::NORMAL);
    ads.write_config_data_rate_normal(data_rate::NORMAL_600_SPS);
    ads.write_config_idac_current(idac::IDAC_250_UA);
    ads.write_config_low_side_power_switch(low_side_power_switch::ALWAYS_OPEN);
    ads.write_config_fir_filter(fir_50_60::FIR_OFF);
    ads.write_config_vref_selection(vref::EXTERNAL_REFP0_REFN0);
    ads.write_config_drdy_mode(drdy_mode::DRDY_ONLY);
    ads.write_config_idac2_routing(i2mux::AIN3);
    ads.write_config_idac1_routing(i1mux::AIN2);

    ads.print_config_registers();
}

void loop()
{
    int32_t data = ads.read();
    Serial.print("Read data: 0x");
    Serial.println(data, HEX);

    float gain = ads.get_config_pga_gain_value();
    float r_rtd = ads1220::get_three_wired_rtd_ohms_low_side_ref_from_read_data(R_REF_OHMS, gain, data);
    Serial.print("R_RTD: ");
    Serial.print(r_rtd);
    Serial.println(" [ohms]");

    float temperature = ads1220::get_temperature_from_rtd_ohms_with_linear_approx(r_rtd);
    Serial.print("Approx temperature: ");
    Serial.print(temperature);
    Serial.println(" [deg C]\n");

    delay(1000);
}
