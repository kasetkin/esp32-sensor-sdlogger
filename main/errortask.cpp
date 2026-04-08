#include "errortask.h"

#include "common_utils.h"
#include "freertos/FreeRTOS.h"


ErrorTask::ErrorTask(ErrorCode code) :
    m_code{code}
{

}

void ErrorTask::execute()
{
    while (true) {
        switch (m_code) {
            case ErrorCode::ecOK:
                sendOK();
                break;
            case ErrorCode::ecGpsUartFail:
                sendGpsUartFail();
                break;
            case ErrorCode::ecTinyGpsFail:
                sendTinyGpsFail();
                break;
            case ErrorCode::ecUM980Fail:
                sendUM980Fail();
                break;
            case ErrorCode::ecSensorsFail:
                sendSensorsFail();
                break;
            case ErrorCode::ecSdCardFilesystemFail:
                sendSdCardFilesystemFail();
                break;
            default:
                sendOK();
                break;
        }

        vTaskDelay(pdMS_TO_TICKS(SEND_PERIOD_MS));
    }
}


void ErrorTask::sendOK()
{

}

void ErrorTask::sendGpsUartFail()
{

}

void ErrorTask::sendTinyGpsFail()
{

}

void ErrorTask::sendUM980Fail()
{

}

void ErrorTask::sendSensorsFail()
{

}

void ErrorTask::sendSdCardFilesystemFail()
{

}