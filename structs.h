enum LogType {
  INFO      = 0,
  WARNING   = 1,
  ERROR     = 2,
  PASS      = 3,
  FAIL_t    = 4,
  CRITICAL  = 5,
  DEBUG     = 6,
} ;

enum SensorType {
  HUMIDITY = 0,
  TEMPERATURE = 1,
  HYDROMETER = 2,
  LIGHT = 3,
};


struct bootflags
{
  unsigned char raw_rst_cause : 4;
  unsigned char raw_bootdevice : 4;
  unsigned char raw_bootmode : 4;
  unsigned char rst_normal_boot : 1;
  unsigned char rst_reset_pin : 1;
  unsigned char rst_watchdog : 1;
  unsigned char bootdevice_ram : 1;
  unsigned char bootdevice_flash : 1;
};

struct bootflags bootmode_detect(void) {
  int reset_reason = 0, bootmode = 0;
  asm (
    "movi %0, 0x60000600\n\t"
    "movi %1, 0x60000200\n\t"
    "l32i %0, %0, 0x114\n\t"
    "l32i %1, %1, 0x118\n\t"
    : "+r" (reset_reason), "+r" (bootmode) /* Outputs */
    : /* Inputs (none) */
    : "memory" /* Clobbered */
  );

  struct bootflags flags;
  flags.raw_rst_cause = (reset_reason & 0xF);
  flags.raw_bootdevice = ((bootmode >> 0x10) & 0x7);
  flags.raw_bootmode = ((bootmode >> 0x1D) & 0x7);
  flags.rst_normal_boot = flags.raw_rst_cause == 0x1;
  flags.rst_reset_pin = flags.raw_rst_cause == 0x2;
  flags.rst_watchdog = flags.raw_rst_cause == 0x4;
  flags.bootdevice_ram = flags.raw_bootdevice == 0x1;
  flags.bootdevice_flash = flags.raw_bootdevice == 0x3;

  return flags;
}

/*******************************************************************************************************/

String              TypeNames[4] = {
  "App%5CEntity%5CEvents%5CEventHumidity",
  "App%5CEntity%5CEvents%5CEventTemperature",
  "App%5CEntity%5CEvents%5CEventSoilHydrometer",
  "App%5CEntity%5CEvents%5CEventLight"
};