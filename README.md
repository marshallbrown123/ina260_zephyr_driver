# ina260_zephyr_driver
TI INA260 current &amp; voltage sensor with integrated shunt driver for Zephyr

Driver based of the ina23x files but the register names changed. 


add something like this to your overlay

      mppt_solar_ina260: ina260@40 {
        status = "okay";
        compatible = "ti,ina260";
        reg = <0x40>;
        config = <INA260_CONFIG(
          INA260_OPER_MODE_SHUNT_BUS_VOLTAGE_CONT,
          INA260_CONV_TIME_8244,
          INA260_CONV_TIME_8244,
          INA260_AVG_MODE_1024)>;
      };
    
   
   and something like this to your application (main.c or whatever)
    #include "../opito_ina260_i2c/ina260.h"
    #include "../opito_ina260_i2c/ina26x_common.h"


    #define MPPT_SOLAR_INA260 DT_NODELABEL(mppt_solar_ina260)
    #if DT_NODE_HAS_STATUS(MPPT_SOLAR_INA260,okay)
      const struct device *const mppt_solar_ina260 = DEVICE_DT_GET(MPPT_SOLAR_INA260);
    #else
      #warning "mppt_solar_ina260 device is disabled."
    #endif




    void initMPPTSolarINA60(void){

      struct sensor_value v_bus, power, current;
      int rc;

      if (!device_is_ready(mppt_solar_ina260)) {
        LOG_ERR("MPPT Solar %s is not ready.", mppt_solar_ina260->name);
        return;
      }


        rc = sensor_sample_fetch(mppt_solar_ina260);
        if (rc) {
          LOG_ERR("INA219 Could not fetch sensor data.");
          return;
        }

        sensor_channel_get(mppt_solar_ina260, SENSOR_CHAN_VOLTAGE, &v_bus);
        sensor_channel_get(mppt_solar_ina260, SENSOR_CHAN_POWER, &power);
        sensor_channel_get(mppt_solar_ina260, SENSOR_CHAN_CURRENT, &current);

        LOG_INF("MPPT Solar: %f [V] -- "
          "Power: %f [W] -- "
          "Current: %f [A]",
               sensor_value_to_double(&v_bus),
               sensor_value_to_double(&power),
               sensor_value_to_double(&current));

    }
    
    
    void initfunction(void){
    	initMPPTSolarINA60();
    }
