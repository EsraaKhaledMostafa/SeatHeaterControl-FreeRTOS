/* Kernel includes. */
#include "FreeRTOS/Source/include/FreeRTOS.h"
#include "FreeRTOS/Source/include/task.h"
#include "FreeRTOS/Source/include/event_groups.h"
#include "FreeRTOS/Source/include/semphr.h"

/* MCAL includes. */
#include "MCAL/tm4c123gh6pm_registers.h"
#include "MCAL/UART/uart0.h"
#include "MCAL/GPIO/gpio.h"
#include "MCAL/GPTM/GPTM.h"
#include "HAL/lm35_sensor.h"
#include "Common/delay.h"

#define mainSW1_INTERRUPT_BIT     (1UL << 0UL)   /* Driver seat button    PF4*/
#define mainSW2_INTERRUPT_BIT     (1UL << 1UL)   /* Passenger seat button PF0  */
#define mainSW3_INTERRUPT_BIT     (1UL << 2UL)   /* Steering Wheel Driver seat button PB2*/

typedef enum {OFF,LOW, MED, HIGH}Button_States;
typedef enum {DISABLED, ENABLED, LOW_INTENSITY, MED_INTENSITY, HIGH_INTENSITY}HeaterLevel_States;

Button_States Driver_SW1_State = OFF;
Button_States Driver_SW3_State = OFF;
Button_States Passenger_SW2_State = OFF;

HeaterLevel_States DriverHeaterLevel;
HeaterLevel_States PassengerHeaterLevel;

uint8 CurrentTemperature;
uint8 DriverDesiredTemperature;
uint8 PassengerDesiredTemperature;

uint32 DriverLevelTimeStamp;
uint32 PassengerLevelTimeStamp;


const char pcDriver[] =    "Driver";
const char pcPassenger[] = "Passenger";

uint8 TemperatureSensorState = DISABLED;

uint32 ullTasksOutTime[11];
uint32 ullTasksInTime[11];
uint32 ullTasksTotalTime[11];

uint32 ullTasksExecutionTime[11];

typedef struct
{
    TickType_t acquiredTime;
    TickType_t releaseTime;
}ResourceLockInfo;

ResourceLockInfo TasksResourceLockInfo[10] = {0};


EventGroupHandle_t xButtonEventGroup;
xSemaphoreHandle xTemperatureMutex;
xSemaphoreHandle xButtonsStatesMutex;
xSemaphoreHandle xHeaterLevelMutex;
xSemaphoreHandle xUartMutex;


TaskHandle_t xGetTemperatureValueTaskHandle;
TaskHandle_t xButtonMonitoringHandle;
TaskHandle_t xSettingDesiredTempDriverHandle;
TaskHandle_t xSettingDesiredTempPassengerHandle;
TaskHandle_t xHeaterControlDriverHandle;
TaskHandle_t xHeaterControlPassengerHandle;
TaskHandle_t xHeaterControlSeatOutputHandle;
TaskHandle_t xDisplayingTaskHandle;
TaskHandle_t xRunTimeMeasurementsTaskHandle;
TaskHandle_t xDiagnosticsTaskHandle;


/* The HW setup function */
static void prvSetupHardware( void );

/* FreeRTOS tasks */
void vGetTemperatureValueTask(void *pvParameters);
void vButtonMonitoring(void *pvParameters);
void vSettingDesiredTemperature(void *pvParameters);
void vHeaterControl(void *pvParameters);
void vHeaterControlSeatOutput(void *pvParameters);
void vDisplayingTask(void *pvParameters);
void vDiagnosticsTask(void *pvParameters);
void vRunTimeMeasurementsTask(void *pvParameters);



int main(void){
    /*    Setup the hardware for use with the Tiva C board.*/
    prvSetupHardware();

    xButtonEventGroup = xEventGroupCreate();

    xTemperatureMutex = xSemaphoreCreateMutex();
    xButtonsStatesMutex = xSemaphoreCreateMutex();
    xHeaterLevelMutex = xSemaphoreCreateMutex();
    xUartMutex = xSemaphoreCreateMutex();

    /* Create Tasks here*/
    xTaskCreate(vGetTemperatureValueTask,   "Task 1", 256, NULL,               3, &xGetTemperatureValueTaskHandle    );
    xTaskCreate(vButtonMonitoring,          "Task 2", 256, NULL,               4, &xButtonMonitoringHandle           );
    xTaskCreate(vSettingDesiredTemperature, "Task 3", 256, (void*)pcDriver,    2, &xSettingDesiredTempDriverHandle   );
    xTaskCreate(vSettingDesiredTemperature, "Task 4", 256, (void*)pcPassenger, 2, &xSettingDesiredTempPassengerHandle);
    xTaskCreate(vHeaterControl,             "Task 5", 256, (void*)pcDriver,    2, &xHeaterControlDriverHandle        );
    xTaskCreate(vHeaterControl,             "Task 6", 256, (void*)pcPassenger, 2, &xHeaterControlPassengerHandle     );
    xTaskCreate(vHeaterControlSeatOutput,   "Task 7", 256, NULL,               2, &xHeaterControlSeatOutputHandle    );
    xTaskCreate(vDisplayingTask,            "Task 8", 256, NULL,               2, &xDisplayingTaskHandle             );
    xTaskCreate(vRunTimeMeasurementsTask,   "Task 9", 256, NULL,               1, &xRunTimeMeasurementsTaskHandle    );


    vTaskSetApplicationTaskTag( xGetTemperatureValueTaskHandle,     ( TaskHookFunction_t ) 1  );
    vTaskSetApplicationTaskTag( xButtonMonitoringHandle,            ( TaskHookFunction_t ) 2  );
    vTaskSetApplicationTaskTag( xSettingDesiredTempDriverHandle,    ( TaskHookFunction_t ) 3  );
    vTaskSetApplicationTaskTag( xSettingDesiredTempPassengerHandle, ( TaskHookFunction_t ) 4  );
    vTaskSetApplicationTaskTag( xHeaterControlDriverHandle,         ( TaskHookFunction_t ) 5  );
    vTaskSetApplicationTaskTag( xHeaterControlPassengerHandle,      ( TaskHookFunction_t ) 6  );
    vTaskSetApplicationTaskTag( xHeaterControlSeatOutputHandle,     ( TaskHookFunction_t ) 7  );
    vTaskSetApplicationTaskTag( xDisplayingTaskHandle,              ( TaskHookFunction_t ) 8  );
    vTaskSetApplicationTaskTag( xRunTimeMeasurementsTaskHandle,     ( TaskHookFunction_t ) 9  );
    vTaskSetApplicationTaskTag( xDiagnosticsTaskHandle,             ( TaskHookFunction_t ) 10 );

    /*Now all the tasks have been started - start the scheduler.

    NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
    The processor MUST be in supervisor mode when vTaskStartScheduler is
    called.  The demo applications included in the FreeRTOS.org download switch
    to supervisor mode prior to main being called.  If you are not using one of
    these demo application projects then ensure Supervisor mode is used here.*/
    vTaskStartScheduler();

    /*Should never reach here!  If you do then there was not enough heap
    available for the idle task to be created.*/
    for (;;);
}

/*HW Initialization Functions*/
static void prvSetupHardware( void )
{
    /* Place here any needed HW initialization such as GPIO, UART, etc.  */
    GPIO_BuiltinButtonsLedsInit();
    GPIO_ExternalButtonInit();

    GPIO_SW1EdgeTriggeredInterruptInit();
    GPIO_SW2EdgeTriggeredInterruptInit();
    GPIO_SW3EdgeTriggeredInterruptInit();

    UART0_Init();
    ADC_init();

    GPTM_WTimer0Init();
}

/*Tasks Definitions*/
void vGetTemperatureValueTask(void *pvParameters)
{
    TickType_t xPeriod = pdMS_TO_TICKS(20);
    TickType_t pxPreviousWakeTime = xTaskGetTickCount();

    for(;;)
    {
        vTaskDelayUntil(&pxPreviousWakeTime,xPeriod);

        if(xSemaphoreTake(xTemperatureMutex,portMAX_DELAY) == pdTRUE)
        {
            TasksResourceLockInfo[0].acquiredTime = xTaskGetTickCount();
            CurrentTemperature = LM35_getTemperature();

            if((CurrentTemperature < 5) || (CurrentTemperature > 40))
            {
                TemperatureSensorState = DISABLED;
                xTaskCreate(vDiagnosticsTask, "Task 10", 256, NULL, 1, &xDiagnosticsTaskHandle);
            }
            else
            {
                TemperatureSensorState = ENABLED;
                vTaskResume(xDiagnosticsTaskHandle);
            }

            TasksResourceLockInfo[0].releaseTime = xTaskGetTickCount();
            xSemaphoreGive(xTemperatureMutex);
        }
    }
}

void vButtonMonitoring(void *pvParameters)
{
    static uint8 SW1_Counter = 0;
    static uint8 SW2_Counter = 0;
    static uint8 SW3_Counter = 0;

    EventBits_t SW_Event_Flag = 0;
    const EventBits_t xBitsToWaitFor = (mainSW1_INTERRUPT_BIT | mainSW2_INTERRUPT_BIT | mainSW3_INTERRUPT_BIT);

    TickType_t xPeriod = pdMS_TO_TICKS(10);
    TickType_t pxPreviousWakeTime = xTaskGetTickCount();

    for (;;)
    {
        vTaskDelayUntil(&pxPreviousWakeTime,xPeriod);

        if(xSemaphoreTake(xButtonsStatesMutex, portMAX_DELAY) == pdTRUE)
        {
            TasksResourceLockInfo[1].acquiredTime = xTaskGetTickCount();

            SW_Event_Flag = xEventGroupWaitBits(xButtonEventGroup, xBitsToWaitFor, pdTRUE, pdFALSE, portMAX_DELAY);

            if((SW_Event_Flag & mainSW1_INTERRUPT_BIT) != 0)
            {
                /*                UART0_SendString("PF4 - SW1 is pressed \r\n");*/
                SW1_Counter++;
                if(SW1_Counter == 4)
                {
                    SW1_Counter = 0;
                }
                switch(SW1_Counter)
                {
                case 0:
                    Driver_SW1_State = OFF;
                    break;
                case 1:
                    Driver_SW1_State = LOW;
                    break;
                case 2:
                    Driver_SW1_State = MED;
                    break;
                case 3:
                    Driver_SW1_State = HIGH;
                    break;
                default:
                    break;
                }
                DriverLevelTimeStamp = GPTM_WTimer0Read();
            }
            else if((SW_Event_Flag & mainSW2_INTERRUPT_BIT) != 0)
            {
                /*                UART0_SendString("PF0 - SW2 is pressed \r\n");*/
                SW2_Counter++;
                if(SW2_Counter == 4)
                {
                    SW2_Counter = 0;
                }
                switch(SW2_Counter)
                {
                case 0:
                    Passenger_SW2_State = OFF;
                    break;
                case 1:
                    Passenger_SW2_State = LOW;
                    break;
                case 2:
                    Passenger_SW2_State = MED;
                    break;
                case 3:
                    Passenger_SW2_State = HIGH;
                    break;
                default:
                    break;
                }
                PassengerLevelTimeStamp = GPTM_WTimer0Read();
            }
            else if((SW_Event_Flag & mainSW3_INTERRUPT_BIT) != 0)
            {
                /*                UART0_SendString("PB2 - SW3 is pressed \r\n");*/
                SW3_Counter++;
                if(SW3_Counter == 4)
                {
                    SW3_Counter = 0;
                }
                switch(SW3_Counter)
                {
                case 0:
                    Driver_SW3_State = OFF;
                    break;
                case 1:
                    Driver_SW3_State = LOW;
                    break;
                case 2:
                    Driver_SW3_State = MED;
                    break;
                case 3:
                    Driver_SW3_State = HIGH;
                    break;
                default:
                    break;
                }
                Driver_SW1_State = Driver_SW3_State;  /*To let the 2 switches goes with each other if the switch in steering wheel is pressed*/
                DriverLevelTimeStamp = GPTM_WTimer0Read();
            }
            else
            {
                /*DO Nothing*/
            }
            TasksResourceLockInfo[1].releaseTime = xTaskGetTickCount();

            xSemaphoreGive(xButtonsStatesMutex);
        }
    }
}

void vSettingDesiredTemperature(void *pvParameters)
{
    TickType_t xPeriod = pdMS_TO_TICKS(20);
    TickType_t pxPreviousWakeTime = xTaskGetTickCount();

    for(;;)
    {
        vTaskDelayUntil(&pxPreviousWakeTime,xPeriod);

        if(xSemaphoreTake(xTemperatureMutex, portMAX_DELAY) == pdTRUE)
        {
            TasksResourceLockInfo[2].acquiredTime = xTaskGetTickCount();

            if((char*)pvParameters == pcDriver)
            {
                if((Driver_SW1_State == LOW) || (Driver_SW3_State == LOW))
                {
                    DriverDesiredTemperature = 25;
                }
                else if((Driver_SW1_State == MED) || (Driver_SW3_State == MED))
                {
                    DriverDesiredTemperature = 30;
                }
                else if((Driver_SW1_State == HIGH) || (Driver_SW3_State == HIGH))
                {
                    DriverDesiredTemperature = 35;
                }
                else
                {
                    /*DO Nothing*/
                }
            }
            else if((char*)pvParameters == pcPassenger)
            {
                if(Passenger_SW2_State == LOW)
                {
                    PassengerDesiredTemperature = 25;
                }
                else if(Passenger_SW2_State == MED)
                {
                    PassengerDesiredTemperature = 30;
                }
                else if(Passenger_SW2_State == HIGH)
                {
                    PassengerDesiredTemperature = 35;
                }
                else
                {
                    /*DO Nothing*/
                }
            }
            TasksResourceLockInfo[2].releaseTime = xTaskGetTickCount();

            xSemaphoreGive(xTemperatureMutex);
        }
    }
}

void vHeaterControl(void *pvParameters)
{
    TickType_t xPeriod = pdMS_TO_TICKS(30);
    TickType_t pxPreviousWakeTime = xTaskGetTickCount();

    for(;;)
    {
        vTaskDelayUntil(&pxPreviousWakeTime,xPeriod);

        if(xSemaphoreTake(xTemperatureMutex, portMAX_DELAY) == pdTRUE)
        {
            if(xSemaphoreTake(xHeaterLevelMutex, portMAX_DELAY) == pdTRUE)
            {
                TasksResourceLockInfo[3].acquiredTime = xTaskGetTickCount();

                if((char*)pvParameters == pcDriver)
                {
                    if(DriverDesiredTemperature > CurrentTemperature)
                    {
                        if(DriverHeaterLevel == DISABLED)
                        {
                            if((DriverDesiredTemperature - CurrentTemperature) >= 3)
                            {
                                DriverHeaterLevel = ENABLED;
                            }
                            else
                            {
                                /*Do Nothing*/
                            }
                        }
                        else
                        {
                            /*Do Nothing*/
                        }

                        if((DriverDesiredTemperature - CurrentTemperature) >= 10)
                        {
                            DriverHeaterLevel = LOW_INTENSITY;
                        }
                        else if(((DriverDesiredTemperature - CurrentTemperature) >= 5) && ((DriverDesiredTemperature - CurrentTemperature) <= 10))
                        {
                            DriverHeaterLevel = MED_INTENSITY;
                        }
                        else if(((DriverDesiredTemperature - CurrentTemperature) >= 2) && ((DriverDesiredTemperature - CurrentTemperature) <= 5))
                        {
                            DriverHeaterLevel = HIGH_INTENSITY;
                        }
                        else
                        {
                            /*Do Nothing*/
                        }
                    }
                    else
                    {
                        DriverHeaterLevel = DISABLED;
                    }
                }
                else if((char*)pvParameters == pcPassenger)
                {
                    if(PassengerDesiredTemperature > CurrentTemperature)
                    {
                        if(PassengerHeaterLevel == DISABLED)
                        {
                            if((PassengerDesiredTemperature - CurrentTemperature) >= 3)
                            {
                                PassengerHeaterLevel = ENABLED;
                            }
                            else
                            {
                                /*Do Nothing*/
                            }
                        }
                        else
                        {
                            /*Do Nothing*/
                        }

                        if((PassengerDesiredTemperature - CurrentTemperature) >= 10)
                        {
                            PassengerHeaterLevel = LOW_INTENSITY;
                        }
                        else if(((PassengerDesiredTemperature - CurrentTemperature) >= 5) && ((PassengerDesiredTemperature - CurrentTemperature) <= 10))
                        {
                            PassengerHeaterLevel = MED_INTENSITY;
                        }
                        else if(((PassengerDesiredTemperature - CurrentTemperature) >= 2) && ((PassengerDesiredTemperature - CurrentTemperature) <= 5))
                        {
                            PassengerHeaterLevel = HIGH_INTENSITY;
                        }
                        else
                        {
                            /*Do Nothing*/
                        }
                    }
                    else
                    {
                        PassengerHeaterLevel = DISABLED;
                    }
                }
                TasksResourceLockInfo[3].releaseTime = xTaskGetTickCount();

                xSemaphoreGive(xHeaterLevelMutex);
            }
            xSemaphoreGive(xTemperatureMutex);
        }

    }
}

void vHeaterControlSeatOutput(void *pvParameters)
{
    TickType_t xPeriod = pdMS_TO_TICKS(30);
    TickType_t pxPreviousWakeTime = xTaskGetTickCount();

    for(;;)
    {
        vTaskDelayUntil(&pxPreviousWakeTime,xPeriod);

        if(xSemaphoreTake(xHeaterLevelMutex, portMAX_DELAY) == pdTRUE)
        {
            TasksResourceLockInfo[4].acquiredTime = xTaskGetTickCount();

            if((PassengerHeaterLevel == LOW_INTENSITY) || (DriverHeaterLevel == LOW_INTENSITY))
            {
                GPIO_BlueLedOff();
                GPIO_RedLedOff();
                GPIO_GreenLedOn();
            }
            else if((PassengerHeaterLevel == MED_INTENSITY) || (DriverHeaterLevel == MED_INTENSITY))
            {
                GPIO_RedLedOff();
                GPIO_GreenLedOff();
                GPIO_BlueLedOn();
            }
            else if((PassengerHeaterLevel == HIGH_INTENSITY) || (DriverHeaterLevel == HIGH_INTENSITY))
            {
                GPIO_GreenLedOff();
                GPIO_BlueLedOff();
                GPIO_RedLedOn();
            }
            else
            {
                if(TemperatureSensorState == DISABLED)
                {
                    GPIO_GreenLedOff();
                    GPIO_BlueLedOff();
                    GPIO_RedLedOn();
                }
                else
                {
                    GPIO_GreenLedOff();
                    GPIO_BlueLedOff();
                    GPIO_RedLedOff();
                }

            }
            TasksResourceLockInfo[4].releaseTime = xTaskGetTickCount();

            xSemaphoreGive(xHeaterLevelMutex);
        }
    }
}

void vDisplayingTask(void *pvParameters)
{
    TickType_t xPeriod = pdMS_TO_TICKS(3000);
    TickType_t pxPreviousWakeTime = xTaskGetTickCount();

    for(;;)
    {
        vTaskDelayUntil(&pxPreviousWakeTime,xPeriod);

        if(xSemaphoreTake(xTemperatureMutex, portMAX_DELAY) == pdTRUE)
        {
            if(xSemaphoreTake(xHeaterLevelMutex, portMAX_DELAY) == pdTRUE)
            {
                if(xSemaphoreTake(xUartMutex, portMAX_DELAY) == pdTRUE)
                {
                    TasksResourceLockInfo[5].acquiredTime = xTaskGetTickCount();

                    if(TemperatureSensorState != DISABLED)
                    {
                        UART0_SendString("Current Temperature = ");
                        UART0_SendInteger(CurrentTemperature);
                        UART0_SendString("\r\n");

                        UART0_SendString("Driver Heater Level = ");
                        switch(Driver_SW1_State)
                        {
                        case OFF:
                            UART0_SendString("OFF");
                            break;
                        case LOW:
                            UART0_SendString("LOW");
                            break;
                        case MED:
                            UART0_SendString("MED");
                            break;
                        case HIGH:
                            UART0_SendString("HIGH");
                            break;
                        default:
                            break;
                        }
                        UART0_SendString("\r\n");

                        UART0_SendString("Passenger Heater Level = ");
                        switch(Passenger_SW2_State)
                        {
                        case OFF:
                            UART0_SendString("OFF");
                            break;
                        case LOW:
                            UART0_SendString("LOW");
                            break;
                        case MED:
                            UART0_SendString("MED");
                            break;
                        case HIGH:
                            UART0_SendString("HIGH");
                            break;
                        default:
                            break;
                        }
                        UART0_SendString("\r\n");

                        UART0_SendString("Driver Heater State = ");
                        switch(DriverHeaterLevel)
                        {
                        case DISABLED:
                            UART0_SendString("DISABLED");
                            break;
                        case ENABLED:
                            UART0_SendString("DISABLED");
                            break;
                        case LOW_INTENSITY:
                            UART0_SendString("LOW_INTENSITY");
                            break;
                        case MED_INTENSITY:
                            UART0_SendString("MED_INTENSITY");
                            break;
                        case HIGH_INTENSITY:
                            UART0_SendString("HIGH_INTENSITY");
                            break;
                        default:
                            break;
                        }
                        UART0_SendString("\r\n");

                        UART0_SendString("Passenger Heater State = ");
                        switch(PassengerHeaterLevel)
                        {
                        case DISABLED:
                            UART0_SendString("DISABLED");
                            break;
                        case ENABLED:
                            UART0_SendString("DISABLED");
                            break;
                        case LOW_INTENSITY:
                            UART0_SendString("LOW_INTENSITY");
                            break;
                        case MED_INTENSITY:
                            UART0_SendString("MED_INTENSITY");
                            break;
                        case HIGH_INTENSITY:
                            UART0_SendString("HIGH_INTENSITY");
                            break;
                        default:
                            break;
                        }
                        UART0_SendString("\r\n");


                        UART0_SendString("\r\n");
                        UART0_SendString("\r\n");

                    }
                    else
                    {
                        /*Do Nothing*/
                    }
                    TasksResourceLockInfo[5].releaseTime = xTaskGetTickCount();
                    xSemaphoreGive(xUartMutex);
                }
                xSemaphoreGive(xHeaterLevelMutex);
            }
            xSemaphoreGive(xTemperatureMutex);
        }
    }
}

void vDiagnosticsTask(void *pvParameters)
{
    uint32 FailurTimeStamp = 0;
    TickType_t xPeriod = pdMS_TO_TICKS(3000);
    TickType_t pxPreviousWakeTime = xTaskGetTickCount();

    for(;;)
    {
        vTaskDelayUntil(&pxPreviousWakeTime,xPeriod);

        if(xSemaphoreTake(xTemperatureMutex, portMAX_DELAY) == pdTRUE)
        {
            if(xSemaphoreTake(xUartMutex, portMAX_DELAY) == pdTRUE)
            {
                TasksResourceLockInfo[6].acquiredTime = xTaskGetTickCount();

                if(TemperatureSensorState == DISABLED)
                {
                    FailurTimeStamp = GPTM_WTimer0Read();

                    UART0_SendString("Temperature Sensor Failure recorded at: ");
                    UART0_SendInteger(FailurTimeStamp);
                    UART0_SendString("\r\n");


                    UART0_SendString("Driver Heater Level before Failure = ");
                    switch(Driver_SW1_State)
                    {
                    case OFF:
                        UART0_SendString("OFF ");
                        break;
                    case LOW:
                        UART0_SendString("LOW ");
                        break;
                    case MED:
                        UART0_SendString("MED ");
                        break;
                    case HIGH:
                        UART0_SendString("HIGH ");
                        break;
                    default:
                        break;
                    }
                    UART0_SendString("at: ");
                    UART0_SendInteger(DriverLevelTimeStamp);
                    UART0_SendString("\r\n");

                    UART0_SendString("Passenger Heater Level before Failure = ");
                    switch(Passenger_SW2_State)
                    {
                    case OFF:
                        UART0_SendString("OFF ");
                        break;
                    case LOW:
                        UART0_SendString("LOW ");
                        break;
                    case MED:
                        UART0_SendString("MED ");
                        break;
                    case HIGH:
                        UART0_SendString("HIGH ");
                        break;
                    default:
                        break;
                    }
                    UART0_SendString("at: ");
                    UART0_SendInteger(PassengerLevelTimeStamp);
                    UART0_SendString("\r\n");

                    UART0_SendString("\r\n\r\n");
                }
                else
                {
                    TemperatureSensorState = ENABLED;
                }
                TasksResourceLockInfo[6].releaseTime = xTaskGetTickCount();
                xSemaphoreGive(xUartMutex);
            }
            xSemaphoreGive(xTemperatureMutex);
        }
        vTaskSuspend(NULL);
    }
}


void vRunTimeMeasurementsTask(void *pvParameters)
{
    TickType_t xPeriod = pdMS_TO_TICKS(5000);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    uint8 ucCounter, ucCPULoad;
    uint32 ullTotalTasksTime = 0;


    for(;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xPeriod);

        if(xSemaphoreTake(xUartMutex, portMAX_DELAY) == pdTRUE)
        {
            /*CPU Load*/
            for(ucCounter = 1; ucCounter < 11; ucCounter++)
            {
                ullTotalTasksTime += ullTasksTotalTime[ucCounter];
            }

            ucCPULoad = (ullTotalTasksTime * 100) / GPTM_WTimer0Read();

            taskENTER_CRITICAL();
            UART0_SendString("CPU Load is: ");
            UART0_SendInteger(ucCPULoad);
            UART0_SendString(" %\r\n\r\n");
            taskEXIT_CRITICAL();

            /*Tasks Execution Time*/

            for(ucCounter = 1; ucCounter < 11; ucCounter++)
            {
                UART0_SendString("Task ");
                UART0_SendInteger(ucCounter);
                UART0_SendString(" Execution Time: ");
                UART0_SendInteger(ullTasksExecutionTime[ucCounter]/10);
                UART0_SendString(" msec\r\n\r\n");
            }

            /*Resource Lock Time per Task*/
            UART0_SendString("Task 1 Resource Lock Time: ");
            UART0_SendInteger(TasksResourceLockInfo[0].releaseTime - TasksResourceLockInfo[0].acquiredTime);
            UART0_SendString(" msec\r\n");

            UART0_SendString("Task 2 Resource Lock Time: ");
            UART0_SendInteger(TasksResourceLockInfo[1].releaseTime - TasksResourceLockInfo[1].acquiredTime);
            UART0_SendString(" msec\r\n");

            UART0_SendString("Task 3 & 4 Resource Lock Time: ");
            UART0_SendInteger(TasksResourceLockInfo[2].releaseTime - TasksResourceLockInfo[2].acquiredTime);
            UART0_SendString(" msec\r\n");

            UART0_SendString("Task 5 & 6 Resource Lock Time: ");
            UART0_SendInteger(TasksResourceLockInfo[3].releaseTime - TasksResourceLockInfo[3].acquiredTime);
            UART0_SendString(" msec\r\n");

            UART0_SendString("Task 7 Resource Lock Time: ");
            UART0_SendInteger(TasksResourceLockInfo[4].releaseTime - TasksResourceLockInfo[4].acquiredTime);
            UART0_SendString(" msec\r\n");

            UART0_SendString("Task 8 Resource Lock Time: ");
            UART0_SendInteger(TasksResourceLockInfo[5].releaseTime - TasksResourceLockInfo[5].acquiredTime);
            UART0_SendString(" msec\r\n");

            UART0_SendString("Task 9 Resource Lock Time: ");
            UART0_SendInteger(TasksResourceLockInfo[6].releaseTime - TasksResourceLockInfo[6].acquiredTime);
            UART0_SendString(" msec\r\n");

            UART0_SendString("Task 10 is the Run Time Measurements Task");
            UART0_SendString(" \r\n\r\n");

            xSemaphoreGive(xUartMutex);
        }
    }
}



/*GPIO PORTF & PORTB Interrupt Handlers*/
void GPIOPortF_Handler(void)
{
    BaseType_t pxHigherPriorityTaskWoken = pdFALSE;

    if(GPIO_PORTF_RIS_REG & (1<<0))
    {
        xEventGroupSetBitsFromISR(xButtonEventGroup, mainSW2_INTERRUPT_BIT,&pxHigherPriorityTaskWoken);
        GPIO_PORTF_ICR_REG |= (1<<0);
    }
    else if(GPIO_PORTF_RIS_REG & (1<<4))
    {
        xEventGroupSetBitsFromISR(xButtonEventGroup, mainSW1_INTERRUPT_BIT,&pxHigherPriorityTaskWoken);
        GPIO_PORTF_ICR_REG |= (1<<4);
    }
}

void GPIOPortB_Handler(void)
{
    BaseType_t pxHigherPriorityTaskWoken = pdFALSE;

    if(GPIO_PORTB_RIS_REG & (1<<2))
    {
        xEventGroupSetBitsFromISR(xButtonEventGroup, mainSW3_INTERRUPT_BIT,&pxHigherPriorityTaskWoken);
        GPIO_PORTB_ICR_REG |= (1<<2);
    }
}

/*-----------------------------------------------------------*/
