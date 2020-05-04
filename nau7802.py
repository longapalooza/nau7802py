import smbus, time
from enum import auto, Enum

# Register Map
class Scale_Registers(Enum):
    NAU7802_PU_CTRL = 0x00
    NAU7802_CTRL1 = auto()
    NAU7802_CTRL2 = auto()
    NAU7802_OCAL1_B2 = auto()
    NAU7802_OCAL1_B1 = auto()
    NAU7802_OCAL1_B0 = auto()
    NAU7802_GCAL1_B3 = auto()
    NAU7802_GCAL1_B2 = auto()
    NAU7802_GCAL1_B1 = auto()
    NAU7802_GCAL1_B0 = auto()
    NAU7802_OCAL2_B2 = auto()
    NAU7802_OCAL2_B1 = auto()
    NAU7802_OCAL2_B0 = auto()
    NAU7802_GCAL2_B3 = auto()
    NAU7802_GCAL2_B2 = auto()
    NAU7802_GCAL2_B1 = auto()
    NAU7802_GCAL2_B0 = auto()
    NAU7802_I2C_CONTROL = auto()
    NAU7802_ADCO_B2 = auto()
    NAU7802_ADCO_B1 = auto()
    NAU7802_ADCO_B0 = auto()
    NAU7802_ADC = 0x15          # Shared ADC and OTP 32:24
    NAU7802_OTP_B1 = auto()     # OTP 23:16 or 7:0?
    NAU7802_OTP_B0 = auto()     # OTP 15:8
    NAU7802_PGA = 0x1B
    NAU7802_PGA_PWR = 0x1C
    NAU7802_DEVICE_REV = 0x1F

# Bits within the PU_CRTL register
class PU_CTRL_Bits(Enum):
    NAU7802_PU_CTRL_RR = 0
    NAU7802_PU_CTRL_PUD = auto()
    NAU7802_PU_CTRL_PUA = auto()
    NAU7802_PU_CTRL_PUR = auto()
    NAU7802_PU_CTRL_CS = auto()
    NAU7802_PU_CTRL_CR = auto()
    NAU7802_PU_CTRL_OSCS = auto()
    NAU7802_PU_CTRL_AVDDS = auto()

# Bits within the CTRL1 register
class CTRL1_Bits(Enum):
    NAU7802_CTRL1_GAIN = 2
    NAU7802_CTRL1_VLDO = 5
    NAU7802_CTRL1_DRDY_SEL = 6
    NAU7802_CTRL1_CRP = 7

# Bits within the CTRL2 register
class CTRL2_Bits(Enum):
    NAU7802_CTRL2_CALMOD = 0
    NAU7802_CTRL2_CALS = 2
    NAU7802_CTRL2_CAL_ERROR = 3
    NAU7802_CTRL2_CRS = 4
    NAU7802_CTRL2_CHS = 7

# Bits within the PGA register
class PGA_Bits(Enum):
    NAU7802_PGA_CHP_DIS = 0
    NAU7802_PGA_INV = 3
    NAU7802_PGA_BYPASS_EN = auto()
    NAU7802_PGA_OUT_EN = auto()
    NAU7802_PGA_LDOMODE = auto()
    NAU7802_PGA_RD_OTP_SEL = auto()

# Bits within the PGA PWR register
class PGA_PWR_Bits(Enum):
    NAU7802_PGA_PWR_PGA_CURR = 0
    NAU7802_PGA_PWR_ADC_CURR = 2
    NAU7802_PGA_PWR_MSTR_BIAS_CURR = 4
    NAU7802_PGA_PWR_PGA_CAP_EN = 7

# Allowed Low drop out regulator voltages
class NAU7802_LDO_Values(Enum):
    NAU7802_LDO_2V4 = 0b111
    NAU7802_LDO_2V7 = 0b110
    NAU7802_LDO_3V0 = 0b101
    NAU7802_LDO_3V3 = 0b100
    NAU7802_LDO_3V6 = 0b011
    NAU7802_LDO_3V9 = 0b010
    NAU7802_LDO_4V2 = 0b001
    NAU7802_LDO_4V5 = 0b000

# Allowed gains
class NAU7802_Gain_Values(Enum):
    NAU7802_GAIN_128 = 0b111
    NAU7802_GAIN_64 = 0b110
    NAU7802_GAIN_32 = 0b101
    NAU7802_GAIN_16 = 0b100
    NAU7802_GAIN_8 = 0b011
    NAU7802_GAIN_4 = 0b010
    NAU7802_GAIN_2 = 0b001
    NAU7802_GAIN_1 = 0b000

# Allowed samples per second
class NAU7802_SPS_Values(Enum):
    NAU7802_SPS_320 = 0b111
    NAU7802_SPS_80 = 0b011
    NAU7802_SPS_40 = 0b010
    NAU7802_SPS_20 = 0b001
    NAU7802_SPS_10 = 0b000

# Select between channel values
class NAU7802_Channels(Enum):
    NAU7802_CHANNEL_1 = 0
    NAU7802_CHANNEL_2 = 1

# Sets up the NAU7802 for basic function
# Returns true upon completion
class NAU7802():
    
    def __init__(self):
        self.bus=smbus.SMBus(1)
        self.__i2cPort = False             # This stores the user's requested i2c port
        self.__deviceAddress = 0x2A        # Default unshifted 7-bit address of the NAU7802
        
        # y = mx+b
        self.__zeroOffset = False          # This is b
        self.__calibrationFactor = False   # This is m. User provides this number so that we can output y when requested
    
    # Sets up the NAU7802 for basic function
    # Returns true upon completion
    def begin(self, wirePort):
        
        # Get user's options
        self.__i2cPort = wirePort
        
        # Check if the device ack's over I2C
        if self.isConnected() is False:
            
            # There are rare times when the sensor is occupied and doesn't ack. A 2nd try resolves this.
            if self.isConnected() is False:
                return False
        
        result = True
        
        result &= self.reset() # Reset all registers

        result &= self.powerUp() # Power on analog and digital sections of the scale

        result &= self.setLDO(NAU7802_LDO_Values.NAU7802_LDO_3V3) # Set LDO to 3.3V

        result &= self.setGain(NAU7802_Gain_Values.NAU7802_GAIN_128) # Set gain to 128

        result &= self.setSampleRate(NAU7802_SPS_Values.NAU7802_SPS_80) # Set samples per second to 10

        result &= self.setRegister(Scale_Registers.NAU7802_ADC, 0x30) # Turn off CLK_CHP. From 9.1 power on sequencing.

        result &= self.setBit(PGA_PWR_Bits.NAU7802_PGA_PWR_PGA_CAP_EN, Scale_Registers.NAU7802_PGA_PWR) # Enable 330pF decoupling cap on chan 2. From 9.14 application circuit note.

        result &= self.calibrateAFE() # Re-cal analog front end when we change gain, sample rate, or channel

        return result
    
    # Returns true if device is present
    # Tests for device ack to I2C address
    def isConnected(self):
        try:
            if self.bus.write_quick(self.__deviceAddress) is None:
                return True # All good
        except:
            return False # Sensor did not ACK
    
    # Returns true if Cycle Ready bit is set (conversion is complete)
    def available(self):
      return self.getBit(PU_CTRL_BITS.NAU7802_PU_CTRL_CR, Scale_Registers.NAU7802_PU_CTRL)

    # Calibrate analog front end of system. Returns true if CAL_ERR bit is 0 (no error)
    # Takes approximately 344ms to calibrate
    # It is recommended that the AFE be re-calibrated any time the gain, SPS, or channel number is changed.
    def calibrateAFE(self):
        self.setBit(CTRL2_Bits.NAU7802_CTRL2_CALS, Scale_Registers.NAU7802_CTRL2); # Begin calibration
        counter = 0
        while True:
            if self.getBit(CTRL2_Bits.NAU7802_CTRL2_CALS, Scale_Registers.NAU7802_CTRL2) is False:
                break # Goes to 0 once cal is complete
            delay(1);
            if (counter > 1000):
                return (false)
            counter+=1
        
        if self.getBit(CTRL2_Bits.NAU7802_CTRL2_CAL_ERROR, Scale_Registers.NAU7802_CTRL2) is False:
            return True # No error! Cal is good.
        return False  # Cal error
    
    # Set the readings per second
    # 10, 20, 40, 80, and 320 samples per second is available
    def setSampleRate(self, rate):
        if (rate > 0b111):
            rate = 0b111 # Error check
        
        value = self.getRegister(Scale_Registers.NAU7802_CTRL2)
        value &= 0b10001111 # Clear CRS bits
        value |= rate << 4  # Mask in new CRS bits

        return self.setRegister(Scale_Registers.NAU7802_CTRL2, value);

    # Select between 1 and 2
    def setChannel(self, channelNumber):
        if(channelNumber == NAU7802_CHANNEL_1):
            return self.clearBit(CTRL2_Bits.NAU7802_CTRL2_CHS, Scale_Registers.NAU7802_CTRL2) # Channel 1 (default)
        else:
            return self.setBit(CTRL2_Bits.NAU7802_CTRL2_CHS, Scale_Registers.NAU7802_CTRL2) # Channel 2

    # Power up digital and analog sections of scale
    def powerUp(self):
        self.setBit(PU_CTRL_BITS.NAU7802_PU_CTRL_PUD, Scale_Registers.NAU7802_PU_CTRL);
        self.setBit(PU_CTRL_BITS.NAU7802_PU_CTRL_PUA, Scale_Registers.NAU7802_PU_CTRL);
        
        # Wait for Power Up bit to be set - takes approximately 200us
        counter = 0;
        while True:
            if self.getBit(NAU7802_PU_CTRL_PUR, Scale_Registers.NAU7802_PU_CTRL) is True:
                break # Good to go
            time.sleep(1/1000)
            if (counter > 100):
                return False # Error
            counter+=1
        return True

    # Puts scale into low-power mode
    def powerDown(self):
        self.clearBit(PU_CTRL_BITS.NAU7802_PU_CTRL_PUD, Scale_Registers.NAU7802_PU_CTRL)
        return self.clearBit(PU_CTRL_BITS.NAU7802_PU_CTRL_PUA, Scale_Registers.NAU7802_PU_CTRL)

    # Resets all registers to Power Of Defaults
    def reset(self):
        self.setBit(PU_CTRL_BITS.NAU7802_PU_CTRL_RR, Scale_Registers.NAU7802_PU_CTRL) # Set RR
        time.sleep(1/1000)
        return self.clearBit(PU_CTRL_BITS.NAU7802_PU_CTRL_RR, Scale_Registers.NAU7802_PU_CTRL) # Clear RR to leave reset state

    # Set the onboard Low-Drop-Out voltage regulator to a given value
    # 2.4, 2.7, 3.0, 3.3, 3.6, 3.9, 4.2, 4.5V are available
    def setLDO(self, ldoValue):
        if (ldoValue > 0b111):
            ldoValue = 0b111 # Error check
        
        # Set the value of the LDO
        value = self.getRegister(Scale_Registers.NAU7802_CTRL1)
        value &= 0b11000111    # Clear LDO bits
        value |= ldoValue << 3 # Mask in new LDO bits
        self.setRegister(Scale_Registers.NAU7802_CTRL1, value)
        
        return self.setBit(PU_CTRL_BITS.NAU7802_PU_CTRL_AVDDS, Scale_Registers.NAU7802_PU_CTRL) # Enable the internal LDO

    # Set the gain
    # x1, 2, 4, 8, 16, 32, 64, 128 are avaialable
    def setGain(self, gainValue):
        if (gainValue > 0b111):
            gainValue = 0b111 # Error check
        
        value = self.getRegister(Scale_Registers.NAU7802_CTRL1)
        value &= 0b11111000 # Clear gain bits
        value |= gainValue  # Mask in new bits
        
        return self.setRegister(Scale_Registers.NAU7802_CTRL1, value)

    # Get the revision code of this IC
    def getRevisionCode(self):
        revisionCode = self.getRegister(Scale_Registers.NAU7802_DEVICE_REV)
        return revisionCode & 0x0F

    # Returns 24-bit reading
    # Assumes CR Cycle Ready bit (ADC conversion complete) has been checked to be 1
#    def getReading(self):
#        self.__i2cPort->beginTransmission(self.__deviceAddress)
#        self.__i2cPort->write(NAU7802_ADCO_B2)
#        
#        if (self.__i2cPort->endTransmission() != 0):
#            return False # Sensor did not ACK
#        
#        self.__i2cPort->requestFrom((uint8_t)_deviceAddress, (uint8_t)3)
#        
#        if (self.__i2cPort->available()):
#            valueRaw = (uint32_t)_i2cPort->read() << 16 # MSB
#            valueRaw |= (uint32_t)_i2cPort->read() << 8 # MidSB
#            valueRaw |= (uint32_t)_i2cPort->read()      # LSB
#            
#            # The raw value coming from the ADC is a 24-bit number, so the sign bit now
#            # resides on bit 23 (0 is LSB) of the uint32_t container. By shifting the
#            # value to the left, I move the sign bit to the MSB of the uint32_t container.
#            # By casting to a signed int32_t container I now have properly recovered
#            # the sign of the original value
#            int32_t valueShifted = (int32_t)(valueRaw << 8)
#            
#            # shift the number back right to recover its intended magnitude
#            int32_t value = (valueShifted >> 8)
#            
#            return value
#
#        return 0 # Error














#//Return the average of a given number of readings
#//Gives up after 1000ms so don't call this function to average 8 samples setup at 1Hz output (requires 8s)
#int32_t NAU7802::getAverage(uint8_t averageAmount)
#{
#  long total = 0;
#  uint8_t samplesAquired = 0;
#
#  unsigned long startTime = millis();
#  while (1)
#  {
#    if (available() == true)
#    {
#      total += getReading();
#      if (++samplesAquired == averageAmount)
#        break; //All done
#    }
#    if (millis() - startTime > 1000)
#      return (0); //Timeout - Bail with error
#  }
#  total /= averageAmount;
#
#  return (total);
#}
#

#//Call when scale is setup, level, at running temperature, with nothing on it
#void NAU7802::calculateZeroOffset(uint8_t averageAmount)
#{
#  setZeroOffset(getAverage(averageAmount));
#}
#

#//Sets the internal variable. Useful for users who are loading values from NVM.
#void NAU7802::setZeroOffset(int32_t newZeroOffset)
#{
#  _zeroOffset = newZeroOffset;
#}
#

#int32_t NAU7802::getZeroOffset()
#{
#  return (_zeroOffset);
#}
#

#//Call after zeroing. Provide the float weight sitting on scale. Units do not matter.
#void NAU7802::calculateCalibrationFactor(float weightOnScale, uint8_t averageAmount)
#{
#  int32_t onScale = getAverage(averageAmount);
#  float newCalFactor = (onScale - _zeroOffset) / (float)weightOnScale;
#  setCalibrationFactor(newCalFactor);
#}
#

#//Pass a known calibration factor into library. Helpful if users is loading settings from NVM.
#//If you don't know your cal factor, call setZeroOffset(), then calculateCalibrationFactor() with a known weight
#void NAU7802::setCalibrationFactor(float newCalFactor)
#{
#  _calibrationFactor = newCalFactor;
#}
#

#float NAU7802::getCalibrationFactor()
#{
#  return (_calibrationFactor);
#}
#

#//Returns the y of y = mx + b using the current weight on scale, the cal factor, and the offset.
#float NAU7802::getWeight(bool allowNegativeWeights)
#{
#  int32_t onScale = getAverage(8);
#
#  //Prevent the current reading from being less than zero offset
#  //This happens when the scale is zero'd, unloaded, and the load cell reports a value slightly less than zero value
#  //causing the weight to be negative or jump to millions of pounds
#  if (allowNegativeWeights == false)
#  {
#    if (onScale < _zeroOffset)
#      onScale = _zeroOffset; //Force reading to zero
#  }
#
#  float weight = (onScale - _zeroOffset) / _calibrationFactor;
#  return (weight);
#}
#

#//Set Int pin to be high when data is ready (default)
#bool NAU7802::setIntPolarityHigh()
#{
#  return (clearBit(NAU7802_CTRL1_CRP, NAU7802_CTRL1)); //0 = CRDY pin is high active (ready when 1)
#}
#

#//Set Int pin to be low when data is ready
#bool NAU7802::setIntPolarityLow()
#{
#  return (setBit(NAU7802_CTRL1_CRP, NAU7802_CTRL1)); //1 = CRDY pin is low active (ready when 0)
#}
#

#//Mask & set a given bit within a register
#bool NAU7802::setBit(uint8_t bitNumber, uint8_t registerAddress)
#{
#  uint8_t value = getRegister(registerAddress);
#  value |= (1 << bitNumber); //Set this bit
#  return (setRegister(registerAddress, value));
#}
#

#//Mask & clear a given bit within a register
#bool NAU7802::clearBit(uint8_t bitNumber, uint8_t registerAddress)
#{
#  uint8_t value = getRegister(registerAddress);
#  value &= ~(1 << bitNumber); //Set this bit
#  return (setRegister(registerAddress, value));
#}
#

#//Return a given bit within a register
#bool NAU7802::getBit(uint8_t bitNumber, uint8_t registerAddress)
#{
#  uint8_t value = getRegister(registerAddress);
#  value &= (1 << bitNumber); //Clear all but this bit
#  return (value);
#}
#

#//Get contents of a register
#uint8_t NAU7802::getRegister(uint8_t registerAddress)
#{
#  _i2cPort->beginTransmission(_deviceAddress);
#  _i2cPort->write(registerAddress);
#  if (_i2cPort->endTransmission() != 0)
#    return (-1); //Sensor did not ACK
#
#  _i2cPort->requestFrom((uint8_t)_deviceAddress, (uint8_t)1);
#
#  if (_i2cPort->available())
#    return (_i2cPort->read());
#
#  return (-1); //Error
#}
#

#//Send a given value to be written to given address
#//Return true if successful
#bool NAU7802::setRegister(uint8_t registerAddress, uint8_t value)
#{
#  _i2cPort->beginTransmission(_deviceAddress);
#  _i2cPort->write(registerAddress);
#  _i2cPort->write(value);
#  if (_i2cPort->endTransmission() != 0)
#    return (false); //Sensor did not ACK
#  return (true);
#}