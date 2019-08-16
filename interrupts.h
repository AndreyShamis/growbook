//=======================================================================
#ifdef ESP32

void IRAM_ATTR onTimer()
{
  // Update uptime if boot time found and not process update_time
  if (boot_time > 0 || !update_time_flag) {
    if (internet_access) {
      uptime = timeClient.getEpochTime() - boot_time;
    }
  }
}

#else
void ICACHE_RAM_ATTR onTimerISR()
{
  // Update uptime if boot time found and not process update_time
  if (boot_time > 0 || !update_time_flag) {
    if (internet_access) {
      uptime = timeClient.getEpochTime() - boot_time;
    }
  }
  // We cannot run timer if there is firmware update process
  if (!firmwareUpdateFlag) {
    timer1_write(INTERRUPT_TIME);//12us
  }
}
#endif