struct ScheduleEvent
{
  uint32_t targetTime;
  void (*callback)();
  bool isActive;
};

ScheduleEvent eventList[MAX_EVENTS];

void main()
{
  Hardware_Init();

  while (true)
  {
    // 1. Process any events that should have fired by now
    ProcessDueEvents();

    // 2. Find the next closest event and schedule the hardware interrupt
    PrepareNextWakeup();

    // 3. Enter Low Power Mode (Sleep)
    // The MCU stays here until the RTC CMP or another IRQ fires
    EnterDeepSleep();
  }
}

// ISR (Interrupt Service Routine)
void RTC_CMP_IRQHandler()
{
  // Just wake up the CPU; the main loop handles the logic
  Clear_RTC_Interrupt_Flag();
}