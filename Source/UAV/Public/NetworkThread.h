// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "HAL/Runnable.h"
#include "HAL/RunnableThread.h"
#include "Sockets.h"
#include "SocketSubsystem.h"
#include "Networking.h"
#include "Containers/Queue.h"

#include "CoreMinimal.h"

/**
 * 
 */
class UAV_API NetworkThread : public FRunnable
{
private:
    FSocket* Socket;
    TQueue<TArray<uint8>> SendQueue; // Queue for sending data
    TQueue<TArray<uint8>> ReceiveQueue; // Queue for receiving data
    std::atomic<bool> bShouldRun;

    FCriticalSection socketMutex; // Protect access to the socker


public:
    NetworkThread(FSocket* InSocket);
    virtual ~NetworkThread();

    // FRunnable overrides
        virtual bool Init() override;
    virtual uint32 Run() override;
    virtual void Stop() override;

    // Thread control
    void EnqueueSendData(const TArray<uint8>& Data);
    bool DequeueReceivedData(TArray<uint8>& OutData);

    // Shutdown
    void Shutdown();
};
