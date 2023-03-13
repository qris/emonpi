void PrintBytes(const uint8_t* addr, uint8_t count, bool newline = false) {
  for (uint8_t i = 0; i < count; i++) {
    Serial.print(addr[i]>>4, HEX);
    Serial.print(addr[i]&0x0f, HEX);
  }
  if (newline)
    Serial.println();
}

// Setup and check for presence of one-wire sensors (e.g. DS18x20), return number of sensors detected
byte check_for_onewire_sensors()
{
  sensors.begin();

  // Returns total number of onewire devices on the bus, not just DS18x20 devices:
  ONEWIRE_DEVICE_COUNT = sensors.getDeviceCount();

  // Save their addresses in TEMP_SENSOR_ADDR array:
  for (byte device_index = 0; device_index < ONEWIRE_DEVICE_MAX; device_index++)
  {
    if(!sensors.getAddress(ONEWIRE_DEVICE_ADDR[device_index], device_index))
    {
      break;
    }

    if (OneWire::crc8(ONEWIRE_DEVICE_ADDR[device_index], 7) != ONEWIRE_DEVICE_ADDR[device_index][7]) {
      Serial.print("check_for_onewire_sensors: ");
      PrintBytes(ONEWIRE_DEVICE_ADDR[device_index], 8);
      Serial.print(": CRC is not valid!\n");
      return;
    }

    ONEWIRE_DEVICE_COUNT++;

    if (is_temperature_sensor(device_index)) {
      // And set the ADC resolution of each:
      sensors.setResolution(ONEWIRE_DEVICE_ADDR[device_index], TEMPERATURE_PRECISION);
    }
  }

  return ONEWIRE_DEVICE_COUNT;
}

bool is_temperature_sensor(byte device_index)
{
  return (
    ONEWIRE_DEVICE_ADDR[device_index][0] == 0x26 ||  // iButtonLink (DS2438Z) device
    ONEWIRE_DEVICE_ADDR[device_index][0] == 0x28     // DS18x20 device
  );
}

int get_temperature(byte device_index) {

  float temp = 300 ;
  /*
    if (DEBUG) {
    Serial.print("Sensor address = " );
    for(int x=8; x>0 ; x--){
      Serial.print(TEMP_SENSOR_ADDR[sensor][x], HEX) ;
      Serial.print(" ") ;
    }
    Serial.println();
    }
  */

  if (ONEWIRE_DEVICE_ADDR[device_index][0] == 0x26) {
    // This sensor is an iButtonLink (DS2438Z) device: see
    // https://community.openenergymonitor.org/t/change-emonlibcm-to-make-it-easier-to-switch-between-emontx-and-emonpi/22906/3
    DS2438 ds2438(&oneWire, ONEWIRE_DEVICE_ADDR[device_index]);
    ds2438.begin() ;
    ds2438.update() ;
    if (ds2438.isError()) {
      //Serial.println("Error reading from DS2438 device");
      temp = -55 ;
    } else {
      temp = ds2438.getTemperature() ;
      /*
      if (DEBUG) {
        Serial.print("DS2438 Temperature = ");
        Serial.print(temp, 1);
        Serial.println() ;
      }
      */
    }
  } else if (ONEWIRE_DEVICE_ADDR[device_index][0] == 0x28) {
    // This is a (traditional) DS18x20 device.
    temp = (sensors.getTempC(ONEWIRE_DEVICE_ADDR[device_index]));
    /*
    if (DEBUG) {
      Serial.print("DS18x20 Temperature = ");
      Serial.print(temp, 1);
      Serial.println() ;
    }
    */
  }

  if ((temp < 125.0) && (temp > -55.0)) {
    // Value out of range: return minimum value as a sentinel.
    temp = -55;
  }

  // Convert float to int ready to send via RF:
  return (temp * 10);
}

bool is_pulse_counter(byte device_index)
{
  return num_pulse_counters(device_index) > 0;
}

bool num_pulse_counters(byte device_index)
{
  if (ONEWIRE_DEVICE_ADDR[device_index][0] == 0x1D // DS2423 or compatible, see
    // https://www.analog.com/media/en/technical-documentation/data-sheets/DS2423.pdf
  ) {
    return 2;
  }

  return 0;
}

const byte DS2423_CMD_MATCH_ROM = 0x55;        // followed by 64 bit address of device to be accessed
const byte DS2423_CMD_READ_MEM_COUNTER = 0xA5; // followed by TA1+TA2 (low+high bytes) of start address

bool read_pulse_counter(byte device_index, byte counter_index, unsigned long* p_result)
{
  uint8_t buf[22];  // Put everything in the buffer so we can compute CRC easily.
  buf[0] = DS2423_CMD_MATCH_ROM;
  memcpy(buf + 1, ONEWIRE_DEVICE_ADDR[device_index], 8);
  buf[9] = DS2423_CMD_READ_MEM_COUNTER;

  if (counter_index > 1)
  {
    return false;
  }

  // Address to read from (TA2 << 8 | TA1) is end of page 14 (0x1DF) for counter A, end of page 15
  // (0x1FF) for counter B. In each case we will have to read the last byte of the page memory (which
  // we care not for) before getting to the counter itself.
  buf[10] = (counter_index == 1) ? 0xDF : 0xFF; // TA1

  byte reply_start = 11;
  oneWire.write_bytes(buf, reply_start);
  oneWire.read_bytes(buf + reply_start, 11); // 1 byte of data, counter (4 bytes), 4 bytes of zeroes, CRC16

  if (!OneWire::check_crc16(buf, sizeof(buf) - reply_start, buf + reply_start)) {
    Serial.print("CRC failure in DS2423: ");
    PrintBytes(buf, sizeof(buf), true);
    return false;
  }

  *p_result = (long)*(buf + reply_start + 1);
  return true;
}
