require 'i2c'

# from: https://github.com/adafruit/Adafruit-Raspberry-Pi-Python-Code/blob/master/Adafruit_BMP085/Adafruit_BMP085.py
class I2CDevice::BMP085 < I2CDevice
  # Operating Modes
  ULTRALOWPOWER     = 0
  STANDARD          = 1
  HIGHRES           = 2
  ULTRAHIGHRES      = 3

  # Registers
  CAL_AC1         = 0xAA
  CAL_AC2         = 0xAC
  CAL_AC3         = 0xAE
  CAL_AC4         = 0xB0
  CAL_AC5         = 0xB2
  CAL_AC6         = 0xB4
  CAL_B1          = 0xB6
  CAL_B2          = 0xB8
  CAL_MB          = 0xBA
  CAL_MC          = 0xBC
  CAL_MD          = 0xBE
  CONTROL         = 0xF4
  TEMPDATA        = 0xF6
  PRESSUREDATA    = 0xF6
  READTEMPCMD     = 0x2E
  READPRESSURECMD = 0x34

  def initialize(mode = STANDARD, args={})
    args[:address] ||= 0x77
    super(args)
    @mode = mode
    self.load_calibration_data
  end

  def load_calibration_data
    @cal_ac1 = i2cget(CAL_AC1, 2).unpack('n').pack('v').unpack('s')[0]
    @cal_ac2 = i2cget(CAL_AC2, 2).unpack('n').pack('v').unpack('s')[0]
    @cal_ac3 = i2cget(CAL_AC3, 2).unpack('n').pack('v').unpack('s')[0]
    @cal_ac4 = i2cget(CAL_AC4, 2).unpack('n')[0]
    @cal_ac5 = i2cget(CAL_AC5, 2).unpack('n')[0]
    @cal_ac6 = i2cget(CAL_AC6, 2).unpack('n')[0]
    @cal_b1  = i2cget(CAL_B1, 2).unpack('n').pack('v').unpack('s')[0]
    @cal_b2  = i2cget(CAL_B2, 2).unpack('n').pack('v').unpack('s')[0]
    @cal_mb  = i2cget(CAL_MB, 2).unpack('n').pack('v').unpack('s')[0]
    @cal_mc  = i2cget(CAL_MC, 2).unpack('n').pack('v').unpack('s')[0]
    @cal_md  = i2cget(CAL_MD, 2).unpack('n').pack('v').unpack('s')[0]
  end

  def read_raw_temp
    i2cset(CONTROL, READTEMPCMD)
    sleep 0.005
    bytes = i2cget(TEMPDATA, 2)
    return bytes.unpack('n')[0]
  end

  def read_raw_pressure
    i2cset(CONTROL, READPRESSURECMD + (@mode << 6))
    case @mode
    when ULTRALOWPOWER
      sleep 0.005
    when STANDARD
      sleep 0.008
    when HIGHRES
      sleep 0.014
    when ULTRAHIGHRES
      sleep 0.026
    end
    msb, lsb, xlsb = i2cget(PRESSUREDATA, 3).unpack('C*')
    raw = ((msb << 16) + (lsb << 8) + xlsb) >> (8 - @mode)
    return raw
  end

  def read_temperature
    ut = self.read_raw_temp
    x1 = ((ut - @cal_ac6) * @cal_ac5) >> 15
    x2 = (@cal_mc << 11) / (x1 + @cal_md)
    b5 = x1 + x2
    temp = ((b5 + 8) >> 4) / 10.0
    return temp # Celcius
  end

  def read_pressure
    ut = self.read_raw_temp
    up = self.read_raw_pressure
    
    # True Temperature Calculations
    x1 = ((ut - @cal_ac6) * @cal_ac5) >> 15
    x2 = (@cal_mc << 11) / (x1 + @cal_md)
    b5 = x1 + x2

    # Pressure Calculations
    b6 = b5 - 4000
    x1 = (@cal_b2 * (b6 * b6) >> 12) >> 11
    x2 = (@cal_ac2 * b6) >> 11
    x3 = x1 + x2
    b3 = (((@cal_ac1 * 4 + x3) << @mode) + 2) / 4

    x1 = (@cal_ac3 * b6) >> 13
    x2 = (@cal_b1 * ((b6 * b6) >> 12)) >> 16
    x3 = ((x1 + x2) + 2) >> 2
    b4 = (@cal_ac4 * (x3 + 32768)) >> 15
    b7 = (up - b3) * (50000 >> @mode)

    if (b7 < 0x80000000)
      p = (b7 * 2) / b4
    else
      p = (b7 / b4) * 2
    end

    x1 = (p >> 8) * (p >> 8)
    x1 = (x1 * 3038) >> 16
    x2 = (-7357 * p) >> 16

    p = p + ((x1 + x2 + 3791) >> 4)

    return p # Pa
  end
end
