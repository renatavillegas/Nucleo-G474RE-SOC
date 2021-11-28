/*
 * INA219.c
 *
 *  Created on: 28 set 2020
 *      Author: nico
 */

#include "INA219.h"
//#include "stm32f1xx_hal_i2c.h"


/*!
 *  @brief  begin I2C and set up the hardware
 */
void INA219_init(I2C_HandleTypeDef hi2c, int16_t DevAddress) {
  // Set chip to large range config values to start

	INA219_setCalibration_32V_2A(hi2c, DevAddress);
}

/*!
 *  @brief  Gets the raw bus voltage (16-bit signed integer, so +-32767)
 *  @return the raw bus voltage reading
 */
int16_t INA219_getBusVoltage_raw(I2C_HandleTypeDef hi2c, int16_t DevAddress) {

	uint8_t value[2];
	uint16_t tmp;

	HAL_I2C_Mem_Read(&hi2c, DevAddress, INA219_REG_BUSVOLTAGE, 1, value, 2, 50);

    // Shift to the right 3 to drop CNVR and OVF and multiply by LSB

	tmp= (int16_t) (((value[0]<<8) | value[1])>>3);

    return tmp;
}

/*!
 *  @brief  Gets the raw shunt voltage (16-bit signed integer, so +-32767)
 *  @return the raw shunt voltage reading
 */
int16_t INA219_getShuntVoltage_raw(I2C_HandleTypeDef hi2c, int16_t DevAddress) {

	uint8_t value[2];

	HAL_I2C_Mem_Read(&hi2c, DevAddress, INA219_REG_SHUNTVOLTAGE, 1, value, 2, 50);


	return (int16_t)(((value[0])<<8) | value[1]);
}

/*!
 *  @brief  Gets the raw current value (16-bit signed integer, so +-32767)
 *  @return the raw current reading
 */
int16_t INA219_getCurrent_raw(I2C_HandleTypeDef hi2c, int16_t DevAddress) {

  uint8_t value[2];

  // Sometimes a sharp load will reset the INA219, which will
  // reset the cal register, meaning CURRENT and POWER will
  // not be available ... avoid this by always setting a cal
  // value even if it's an unfortunate extra step
//  Adafruit_BusIO_Register calibration_reg =
//      Adafruit_BusIO_Register(i2c_dev, INA219_REG_CALIBRATION, 2, MSBFIRST);
//  calibration_reg.write(ina219_calValue, 2);

  // Now we can safely read the CURRENT register!
  INA219_setCalibration_32V_2A(hi2c, DevAddress);
  HAL_I2C_Mem_Read(&hi2c, DevAddress, INA219_REG_CURRENT, 1, value, 2, 50);

  uint16_t res= (uint16_t) (((value[0])<<8) | value[1]);

  return res;
}

/*!
 *  @brief  Gets the raw power value (16-bit signed integer, so +-32767)
 *  @return raw power reading
 */
int16_t INA219_getPower_raw(I2C_HandleTypeDef hi2c, int16_t DevAddress) {

  uint8_t value[2];

  // Sometimes a sharp load will reset the INA219, which will
  // reset the cal register, meaning CURRENT and POWER will
  // not be available ... avoid this by always setting a cal
  // value even if it's an unfortunate extra step
//  Adafruit_BusIO_Register calibration_reg =
//      Adafruit_BusIO_Register(i2c_dev, INA219_REG_CALIBRATION, 2, MSBFIRST);
//  calibration_reg.write(ina219_calValue, 2);

  // Now we can safely read the POWER register!
  INA219_setCalibration_32V_2A(hi2c, DevAddress);
  HAL_I2C_Mem_Read(&hi2c, DevAddress, INA219_REG_POWER, 1, value, 2, 50);

  uint16_t res= (uint16_t) (((value[0])<<8) | value[1]);

  return res;
}

/*!
 *  @brief  Gets the shunt voltage in mV (so +-327mV)
 *  @return the shunt voltage converted to millivolts
 */
float INA219_getShuntVoltage_mV(I2C_HandleTypeDef hi2c, int16_t DevAddress) {
  int16_t value;
  value = INA219_getShuntVoltage_raw(hi2c, DevAddress);
  return value * 0.01;
}

/*!
 *  @brief  Gets the shunt voltage in volts
 *  @return the bus voltage converted to volts
 */
float INA219_getBusVoltage_V(I2C_HandleTypeDef hi2c, int16_t DevAddress) {
  int16_t value = INA219_getBusVoltage_raw(hi2c, DevAddress);
  return value * 0.004;
}

/*!
 *  @brief  Gets the current value in mA, taking into account the
 *          config settings and current LSB
 *  @return the current reading convereted to milliamps
 */
float INA219_getCurrent_mA(I2C_HandleTypeDef hi2c, int16_t DevAddress) {

	float valueDec = INA219_getCurrent_raw(hi2c, DevAddress);

	valueDec /= ina219_currentDivider_mA;

	return valueDec;
}

/*!
 *  @brief  Gets the power value in mW, taking into account the
 *          config settings and current LSB
 *  @return power reading converted to milliwatts
 */
float INA219_getPower_mW(I2C_HandleTypeDef hi2c, int16_t DevAddress) {

	float valueDec = INA219_getPower_raw(hi2c, DevAddress);

	valueDec *= ina219_powerMultiplier_mW;

	return valueDec;
}

/*!
 *  @brief  Configures to INA219 to be able to measure up to 32V and 2A
 *          of current.  Each unit of current corresponds to 100uA, and
 *          each unit of power corresponds to 2mW. Counter overflow
 *          occurs at 3.2A.
 *  @note   These calculations assume a 0.1 ohm resistor is present
 */
void INA219_setCalibration_32V_2A(I2C_HandleTypeDef hi2c, int16_t DevAddress) {
  // By default we use a pretty huge range for the input voltage,
  // which probably isn't the most appropriate choice for system
  // that don't use a lot of power.  But all of the calculations
  // are shown below if you want to change the settings.  You will
  // also need to change any relevant register settings, such as
  // setting the VBUS_MAX to 16V instead of 32V, etc.

  // VBUS_MAX = 32V             (Assumes 32V, can also be set to 16V)
  // VSHUNT_MAX = 0.32          (Assumes Gain 8, 320mV, can also be 0.16, 0.08, 0.04)
  // RSHUNT = 0.1               (Resistor value in ohms)

  // 1. Determine max possible current
  // MaxPossible_I = VSHUNT_MAX / RSHUNT
  // MaxPossible_I = 3.2A

  // 2. Determine max expected current
  // MaxExpected_I = 2.0A

  // 3. Calculate possible range of LSBs (Min = 15-bit, Max = 12-bit)
  // MinimumLSB = MaxExpected_I/32767
  // MinimumLSB = 0.000061              (61uA per bit)
  // MaximumLSB = MaxExpected_I/4096
  // MaximumLSB = 0,000488              (488uA per bit)

  // 4. Choose an LSB between the min and max values
  //    (Preferrably a roundish number close to MinLSB)
  // CurrentLSB = 0.0001 (100uA per bit)

  // 5. Compute the calibration register
  // Cal = trunc (0.04096 / (Current_LSB * RSHUNT))
  // Cal = 4096 (0x1000)

  ina219_calValue = 4096;
  int8_t tmp[2];
  tmp[0]=ina219_calValue>>8;
  tmp[1]=(ina219_calValue&0x00FF);

  // 6. Calculate the power LSB
  // PowerLSB = 20 * CurrentLSB
  // PowerLSB = 0.002 (2mW per bit)

  // 7. Compute the maximum current and shunt voltage values before overflow
  //
  // Max_Current = Current_LSB * 32767
  // Max_Current = 3.2767A before overflow
  //
  // If Max_Current > Max_Possible_I then
  //    Max_Current_Before_Overflow = MaxPossible_I
  // Else
  //    Max_Current_Before_Overflow = Max_Current
  // End If
  //
  // Max_ShuntVoltage = Max_Current_Before_Overflow * RSHUNT
  // Max_ShuntVoltage = 0.32V
  //
  // If Max_ShuntVoltage >= VSHUNT_MAX
  //    Max_ShuntVoltage_Before_Overflow = VSHUNT_MAX
  // Else
  //    Max_ShuntVoltage_Before_Overflow = Max_ShuntVoltage
  // End If

  // 8. Compute the Maximum Power
  // MaximumPower = Max_Current_Before_Overflow * VBUS_MAX
  // MaximumPower = 3.2 * 32V
  // MaximumPower = 102.4W

  // Set multipliers to convert raw current/power values
  ina219_currentDivider_mA = 10; // Current LSB = 100uA per bit (1000/100 = 10)
  ina219_powerMultiplier_mW = 2; // Power LSB = 1mW per bit (2/1)

  // Set Calibration register to 'Cal' calculated above
  HAL_I2C_Mem_Write(&hi2c, DevAddress, INA219_REG_CALIBRATION, 1, (uint8_t *)tmp, 2, 50);

  // Set Config register to take into account the settings above
  uint16_t config = INA219_CONFIG_BVOLTAGERANGE_32V |
                    INA219_CONFIG_GAIN_8_320MV | INA219_CONFIG_BADCRES_12BIT |
                    INA219_CONFIG_SADCRES_12BIT_1S_532US |
                    INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;

  int8_t tmp_config[2];
  tmp_config[0]=config>>8;
  tmp_config[1]=(config&0x00FF);

  HAL_I2C_Mem_Write(&hi2c, DevAddress, INA219_REG_CONFIG, 1, (uint8_t *)tmp_config, 2, 50);

}

/*!
 *  @brief  Set power save mode according to parameters
 *  @param  on
 *          boolean value
 */
//void Adafruit_INA219::powerSave(bool on) {
//  Adafruit_BusIO_Register config_reg =
//      Adafruit_BusIO_Register(i2c_dev, INA219_REG_CONFIG, 2, MSBFIRST);
//
//  Adafruit_BusIO_RegisterBits mode_bits =
//      Adafruit_BusIO_RegisterBits(&config_reg, 3, 0);
//  if (on) {
//    mode_bits.write(INA219_CONFIG_MODE_POWERDOWN);
//  } else {
//    mode_bits.write(INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS);
//  }
//}

/*!
 *  @brief  Configures to INA219 to be able to measure up to 32V and 1A
 *          of current.  Each unit of current corresponds to 40uA, and each
 *          unit of power corresponds to 800uW. Counter overflow occurs at
 *          1.3A.
 *  @note   These calculations assume a 0.1 ohm resistor is present
 */
void INA219_setCalibration_32V_1A(I2C_HandleTypeDef hi2c, int16_t DevAddress) {
  // By default we use a pretty huge range for the input voltage,
  // which probably isn't the most appropriate choice for system
  // that don't use a lot of power.  But all of the calculations
  // are shown below if you want to change the settings.  You will
  // also need to change any relevant register settings, such as
  // setting the VBUS_MAX to 16V instead of 32V, etc.

  // VBUS_MAX = 32V		(Assumes 32V, can also be set to 16V)
  // VSHUNT_MAX = 0.32	(Assumes Gain 8, 320mV, can also be 0.16, 0.08, 0.04)
  // RSHUNT = 0.1			(Resistor value in ohms)

  // 1. Determine max possible current
  // MaxPossible_I = VSHUNT_MAX / RSHUNT
  // MaxPossible_I = 3.2A

  // 2. Determine max expected current
  // MaxExpected_I = 1.0A

  // 3. Calculate possible range of LSBs (Min = 15-bit, Max = 12-bit)
  // MinimumLSB = MaxExpected_I/32767
  // MinimumLSB = 0.0000305             (30.5uA per bit)
  // MaximumLSB = MaxExpected_I/4096
  // MaximumLSB = 0.000244              (244uA per bit)

  // 4. Choose an LSB between the min and max values
  //    (Preferrably a roundish number close to MinLSB)
  // CurrentLSB = 0.0000400 (40uA per bit)

  // 5. Compute the calibration register
  // Cal = trunc (0.04096 / (Current_LSB * RSHUNT))
  // Cal = 10240 (0x2800)

  ina219_calValue = 10240;
  int8_t tmp[2];
  tmp[0]=ina219_calValue>>8;
  tmp[1]=(ina219_calValue&0x00FF);

  // 6. Calculate the power LSB
  // PowerLSB = 20 * CurrentLSB
  // PowerLSB = 0.0008 (800uW per bit)

  // 7. Compute the maximum current and shunt voltage values before overflow
  //
  // Max_Current = Current_LSB * 32767
  // Max_Current = 1.31068A before overflow
  //
  // If Max_Current > Max_Possible_I then
  //    Max_Current_Before_Overflow = MaxPossible_I
  // Else
  //    Max_Current_Before_Overflow = Max_Current
  // End If
  //
  // ... In this case, we're good though since Max_Current is less than
  // MaxPossible_I
  //
  // Max_ShuntVoltage = Max_Current_Before_Overflow * RSHUNT
  // Max_ShuntVoltage = 0.131068V
  //
  // If Max_ShuntVoltage >= VSHUNT_MAX
  //    Max_ShuntVoltage_Before_Overflow = VSHUNT_MAX
  // Else
  //    Max_ShuntVoltage_Before_Overflow = Max_ShuntVoltage
  // End If

  // 8. Compute the Maximum Power
  // MaximumPower = Max_Current_Before_Overflow * VBUS_MAX
  // MaximumPower = 1.31068 * 32V
  // MaximumPower = 41.94176W

  // Set multipliers to convert raw current/power values
  ina219_currentDivider_mA = 25;    // Current LSB = 40uA per bit (1000/40 = 25)
  ina219_powerMultiplier_mW = 0.8f; // Power LSB = 800uW per bit

  // Set Calibration register to 'Cal' calculated above
  HAL_I2C_Mem_Write(&hi2c, DevAddress, INA219_REG_CALIBRATION, 1, (uint8_t *)tmp, 2, 1);

  // Set Config register to take into account the settings above
  uint16_t config = INA219_CONFIG_BVOLTAGERANGE_32V |
                    INA219_CONFIG_GAIN_8_320MV | INA219_CONFIG_BADCRES_12BIT |
                    INA219_CONFIG_SADCRES_12BIT_1S_532US |
                    INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;
  HAL_I2C_Mem_Write(&hi2c, DevAddress, INA219_REG_CONFIG, 1, (uint8_t *)config, 2, 1);
}

/*!
 *  @brief set device to alibration which uses the highest precision for
 *     current measurement (0.1mA), at the expense of
 *     only supporting 16V at 400mA max.
 */

void INA219_setCalibration_32V_25A(I2C_HandleTypeDef hi2c, int16_t DevAddress) {

  // Calibration which uses the highest precision for
  // current measurement (0.1mA), at the expense of
  // only supporting 16V at 400mA max.

  // VBUS_MAX = 32V
  // VSHUNT_MAX = 0.04          (Assumes Gain 1, 40mV)
  // RSHUNT = 0.001               (Resistor value in ohms)

  // 1. Determine max possible current
  // MaxPossible_I = VSHUNT_MAX / RSHUNT
  // MaxPossible_I = 40A

  // 2. Determine max expected current
  // MaxExpected_I = 25A

  // 3. Calculate possible range of LSBs (Min = 15-bit, Max = 12-bit)
  // MinimumLSB = MaxExpected_I/32767
  // MinimumLSB = 0.00076294              (763uA per bit)
  // MaximumLSB = MaxExpected_I/4096
  // MaximumLSB = 0.0061              (6.1mA per bit)

  // 4. Choose an LSB between the min and max values
  //    (Preferrably a roundish number close to MinLSB)
  // CurrentLSB = 0.002 (2mA per bit)

  // 5. Compute the calibration register
  // Cal = trunc (0.04096 / (Current_LSB * RSHUNT))
  // Cal = 20480 (0x2000)

  ina219_calValue = 20480;
  int8_t tmp[2];
  tmp[0]=ina219_calValue>>8;
  tmp[1]=(ina219_calValue&0x00FF);

  // 6. Calculate the power LSB
  // PowerLSB = 20 * CurrentLSB
  // PowerLSB = 0.040 (40mW per bit)

  // 7. Compute the maximum current and shunt voltage values before overflow
  //
  // Max_Current = Current_LSB * 32767
  // Max_Current = 65.5340 before overflow
  //
  // If Max_Current > Max_Possible_I then
  //    Max_Current_Before_Overflow = MaxPossible_I
  // Else
  //    Max_Current_Before_Overflow = Max_Current
  // End If
  //
  // Max_Current_Before_Overflow = MaxPossible_I
  // Max_Current_Before_Overflow = 40A
  //
  // Max_ShuntVoltage = Max_Current_Before_Overflow * RSHUNT
  // Max_ShuntVoltage = 0.0655V
  //
  // If Max_ShuntVoltage >= VSHUNT_MAX
  //    Max_ShuntVoltage_Before_Overflow = VSHUNT_MAX
  // Else
  //    Max_ShuntVoltage_Before_Overflow = Max_ShuntVoltage
  // End If
  //
  // Max_ShuntVoltage_Before_Overflow = VSHUNT_MAX
  // Max_ShuntVoltage_Before_Overflow = 0.04V

  // 8. Compute the Maximum Power
  // MaximumPower = Max_Current_Before_Overflow * VBUS_MAX
  // MaximumPower = 40A * 32V
  // MaximumPower = 1.280kW

  // Set multipliers to convert raw current/power values
  ina219_currentDivider_mA = 0.50f;    // Current LSB = 2mA per bit (1000/2000 = 0.5)
  ina219_powerMultiplier_mW = 40.0f; // Power LSB = 1mW per bit

  // Set Calibration register to 'Cal' calculated above
  HAL_I2C_Mem_Write(&hi2c, DevAddress, INA219_REG_CALIBRATION, 1, (uint8_t *)tmp, 2, 1);

  // Set Config register to take into account the settings above
  uint16_t config = INA219_CONFIG_BVOLTAGERANGE_32V |
                    INA219_CONFIG_GAIN_1_40MV | INA219_CONFIG_BADCRES_12BIT |
                    INA219_CONFIG_SADCRES_12BIT_1S_532US |
                    INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;

  HAL_I2C_Mem_Write(&hi2c, DevAddress, INA219_REG_CONFIG, 1, (uint16_t *)config, 2, 1);
}

void INA219_setCalibration_16V_400mA(I2C_HandleTypeDef hi2c, int16_t DevAddress) {

  // Calibration which uses the highest precision for
  // current measurement (0.1mA), at the expense of
  // only supporting 16V at 400mA max.

  // VBUS_MAX = 16V
  // VSHUNT_MAX = 0.04          (Assumes Gain 1, 40mV)
  // RSHUNT = 0.1               (Resistor value in ohms)

  // 1. Determine max possible current
  // MaxPossible_I = VSHUNT_MAX / RSHUNT
  // MaxPossible_I = 0.4A

  // 2. Determine max expected current
  // MaxExpected_I = 0.4A

  // 3. Calculate possible range of LSBs (Min = 15-bit, Max = 12-bit)
  // MinimumLSB = MaxExpected_I/32767
  // MinimumLSB = 0.0000122              (12uA per bit)
  // MaximumLSB = MaxExpected_I/4096
  // MaximumLSB = 0.0000977              (98uA per bit)

  // 4. Choose an LSB between the min and max values
  //    (Preferrably a roundish number close to MinLSB)
  // CurrentLSB = 0.00005 (50uA per bit)

  // 5. Compute the calibration register
  // Cal = trunc (0.04096 / (Current_LSB * RSHUNT))
  // Cal = 8192 (0x2000)

  ina219_calValue = 8192;
  int8_t tmp[2];
  tmp[0]=ina219_calValue>>8;
  tmp[1]=(ina219_calValue&0x00FF);

  // 6. Calculate the power LSB
  // PowerLSB = 20 * CurrentLSB
  // PowerLSB = 0.001 (1mW per bit)

  // 7. Compute the maximum current and shunt voltage values before overflow
  //
  // Max_Current = Current_LSB * 32767
  // Max_Current = 1.63835A before overflow
  //
  // If Max_Current > Max_Possible_I then
  //    Max_Current_Before_Overflow = MaxPossible_I
  // Else
  //    Max_Current_Before_Overflow = Max_Current
  // End If
  //
  // Max_Current_Before_Overflow = MaxPossible_I
  // Max_Current_Before_Overflow = 0.4
  //
  // Max_ShuntVoltage = Max_Current_Before_Overflow * RSHUNT
  // Max_ShuntVoltage = 0.04V
  //
  // If Max_ShuntVoltage >= VSHUNT_MAX
  //    Max_ShuntVoltage_Before_Overflow = VSHUNT_MAX
  // Else
  //    Max_ShuntVoltage_Before_Overflow = Max_ShuntVoltage
  // End If
  //
  // Max_ShuntVoltage_Before_Overflow = VSHUNT_MAX
  // Max_ShuntVoltage_Before_Overflow = 0.04V

  // 8. Compute the Maximum Power
  // MaximumPower = Max_Current_Before_Overflow * VBUS_MAX
  // MaximumPower = 0.4 * 16V
  // MaximumPower = 6.4W

  // Set multipliers to convert raw current/power values
  ina219_currentDivider_mA = 20;    // Current LSB = 50uA per bit (1000/50 = 20)
  ina219_powerMultiplier_mW = 1.0f; // Power LSB = 1mW per bit

  // Set Calibration register to 'Cal' calculated above
  HAL_I2C_Mem_Write(&hi2c, DevAddress, INA219_REG_CALIBRATION, 1, (uint8_t *)tmp, 2, 1);

  // Set Config register to take into account the settings above
  uint16_t config = INA219_CONFIG_BVOLTAGERANGE_16V |
                    INA219_CONFIG_GAIN_1_40MV | INA219_CONFIG_BADCRES_12BIT |
                    INA219_CONFIG_SADCRES_12BIT_1S_532US |
                    INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;

  HAL_I2C_Mem_Write(&hi2c, DevAddress, INA219_REG_CONFIG, 1, (uint16_t *)config, 2, 1);
}
