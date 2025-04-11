#include "NetworkThread.h"
#include "Misc/ScopeLock.h"
#include "Runtime/Core/Public/HAL/PlatformProcess.h"

NetworkThread::NetworkThread(FSocket* InSocket) :
    Socket(InSocket),
    bShouldRun(true)
{
}

NetworkThread::~NetworkThread()
{
    Shutdown();
}

bool NetworkThread::Init()
{
    return Socket != nullptr;
}

uint32 NetworkThread::Run()
{
    UE_LOG(LogTemp, Log, TEXT("NetworkingThread started."));

    while (bShouldRun)
    {
        // Lock socket operations to prevent race conditions
        {
            FScopeLock Lock(&socketMutex);

            if (Socket && Socket->Wait(ESocketWaitConditions::WaitForRead, FTimespan::FromMilliseconds(10)))
            {
                uint8 DataBuffer[4096];
                int32 BytesReceived = 0;

                if (!Socket->Recv(DataBuffer, sizeof(DataBuffer), BytesReceived) || BytesReceived == 0)
                {
                    UE_LOG(LogTemp, Error, TEXT("Server disconnected or no data received. Exiting thread."));
                    bShouldRun = false;
                    break;
                }

                // Store received data (Optional: Process received UAV data if needed)
                TArray<uint8> ReceivedData;
                ReceivedData.Append(DataBuffer, BytesReceived);
                ReceiveQueue.Enqueue(ReceivedData);
            }
        }

        // Check if there is data to send
        if (!SendQueue.IsEmpty() && Socket)
        {
            TArray<uint8> DataToSend;
            if (SendQueue.Dequeue(DataToSend) && DataToSend.Num() > 0)
            {
                int32 BytesSent = 0;
                bool bSent = Socket->Send(DataToSend.GetData(), DataToSend.Num(), BytesSent);

                if (bSent)
                {
                    UE_LOG(LogTemp, Log, TEXT("Sent frame data: %d bytes"), BytesSent);
                }
                else
                {
                    UE_LOG(LogTemp, Error, TEXT("Failed to send frame data!"));
                }
            }
        }

        // **Reduce CPU usage** (Prevents 100% utilization)
        FPlatformProcess::Sleep(0.005f); // Adjust as needed
    }

    UE_LOG(LogTemp, Warning, TEXT("NetworkingThread stopping."));
    return 0;
}

void NetworkThread::Stop()
{
    UE_LOG(LogTemp, Log, TEXT("Stopping NetworkThread..."));
    bShouldRun = false;

    if (Socket)
    {
        Socket->Close();  // Forces `Wait()` to return early
    }
}

void NetworkThread::EnqueueSendData(const TArray<uint8>& Data)
{
    if (Data.Num() > 0)
    {
        SendQueue.Enqueue(Data);
    }
}

bool NetworkThread::DequeueReceivedData(TArray<uint8>& OutData)
{
    OutData.Empty();
    return ReceiveQueue.Dequeue(OutData);
}

void NetworkThread::Shutdown()
{
    UE_LOG(LogTemp, Log, TEXT("Shutting down NetworkingThread..."));
    bShouldRun = false;
    {
        FScopeLock Lock(&socketMutex);
        if (Socket)
        {
            Socket->Close();
        }
        
    }
   /* if (Socket)
    {
        ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->DestroySocket(Socket);
        Socket = nullptr;
    }*/
    
}
