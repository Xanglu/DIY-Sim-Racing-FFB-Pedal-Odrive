#include "TaskScheduler.h"
//initialized with null handle
TaskScheduler::TaskScheduler() : taskCount(0), periodic_timer_handle(nullptr)
{
    // Initialize task array handles to NULL for safety
    for (int i = 0; i < MAX_TASKS; ++i)
    {
        tasks[i].handle = NULL;
    }
}

// Initializes the scheduler timer
void TaskScheduler::begin(uint8_t timer_id)
{
    // === Replace hw_timer with esp_timer ===
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &TaskScheduler::timerCallback,
        .arg = this, // Pass instance pointer to the callback
        .name = "sched_timer"};

    esp_timer_create(&periodic_timer_args, &periodic_timer_handle);

    // Start periodic timer at BASE_TICK_US interval
    esp_timer_start_periodic(periodic_timer_handle, BASE_TICK_US);
}

// === Scheduler API ===
void TaskScheduler::addScheduledTask(TaskFunction_t fn, const char *name, uint16_t intervalUs, UBaseType_t priority, BaseType_t core, uint32_t stackSize)
{
    if (taskCount >= MAX_TASKS)
        return; // limit reached

    uint16_t intervalTicks = intervalUs / BASE_TICK_US;
    if (intervalTicks == 0)
        intervalTicks = 1; // minimum 1 tick

    // Create task
    xTaskCreatePinnedToCore(fn, name, stackSize, NULL, priority,
                            &tasks[taskCount].handle, core);

    tasks[taskCount].intervalTicks = intervalTicks;
    tasks[taskCount].counter = 0;
    tasks[taskCount].name = name;
    taskCount++;
}

// === Scheduler ISR ===
void IRAM_ATTR TaskScheduler::timerCallback(void *arg)
{
    // The argument is the "this" pointer for the TaskScheduler instance.
    TaskScheduler *instance = static_cast<TaskScheduler *>(arg);
    instance->onTimer();
}

void TaskScheduler::onTimer()
{
    BaseType_t xHigherPriorityWoken = pdFALSE;

    for (int i = 0; i < taskCount; i++)
    {
        tasks[i].counter++;
        if (tasks[i].counter >= tasks[i].intervalTicks)
        {
            tasks[i].counter = 0;
            if (NULL != tasks[i].handle)
            {
                vTaskNotifyGiveFromISR(tasks[i].handle, &xHigherPriorityWoken);
            }
        }
    }

    // Yield if a higher-priority task was woken.
    if (xHigherPriorityWoken)
    {
        portYIELD_FROM_ISR();
    }
}