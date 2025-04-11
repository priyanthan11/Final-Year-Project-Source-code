// Fill out your copyright notice in the Description page of Project Settings.


//#pragma warning(push)
//#pragma warning(disable: 4263 4264) // Disable OpenCV override warnings
//#include <opencv2/stitching/detail/exposure_compensate.hpp>
//#pragma warning(pop)


#include "Base_UAV.h"
#include "Camera/CameraComponent.h"
#include "Components/SceneCaptureComponent2D.h"
#include "Kismet/KismetRenderingLibrary.h"
#include "Kismet/GameplayStatics.h"
//#include "opencv2/opencv.hpp"
//#include <opencv2/dnn.hpp>
#include "EngineUtils.h"
#include <fstream>
#include "Async/Async.h"

#include "ImageUtils.h"

#include "Engine/TextureRenderTarget2D.h"
#include "ImageUtils.h"
#include "IImageWrapper.h"
#include "IImageWrapperModule.h"
#include "Modules/ModuleManager.h"
#include "UAVSpawner.h"

#include "Engine/EngineTypes.h"
#include "Engine/OverlapResult.h"
#include "CollisionQueryParams.h"

#include "GameFramework/Character.h"
//#include "GameFramework/CharacterMovementComponent.h"
#include "Components/CapsuleComponent.h"

// For logging
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "HAL/PlatformFileManager.h"
#include "Misc//DateTime.h"


#define UDP_RECEIVE_PORT 7000 // Port from the Object Detection server

//#include "CVModel.h"

/*
* TODO: 1) Path finding
* TODO: 2) Obstical Avoidance - Done 
* TODO: 3) Information Share ( Covered Map details)


*/

TArray<FSearch_Cell> ABase_UAV::GlobalSearchGrid;
FCriticalSection ABase_UAV::GridLock;
int32 NumOfSearchedCell;


// Sets default values
ABase_UAV::ABase_UAV():
    RollController(nullptr),
    PitchController(nullptr),
    YawController(nullptr),
    AltitudeController(nullptr),
    RollKp(1.0f), RollKi(0.1f), RollKd(0.05f),
    PitchKp(1.0f), PitchKi(0.1f), PitchKd(0.05f),
    YawKp(1.0f), YawKi(0.1f), YawKd(0.05f),
    AltitudeKp(1.0f), AltitudeKi(0.1f), AltitudeKd(0.05f),
    ForwardKp(1.0f), ForwardKi(0.1f), ForwardKd(0.05f),
    RightKp(1.0f), RightKi(0.1f), RightKd(0.05f),
    CurrentVelocity(FVector::ZeroVector),
    TargetVelocity(FVector::ZeroVector),
    PerceptionRadius(300.0f), // Default perception radius
    SeparationWeight(1.5f),   // Default weights for behaviors
    AlignmentWeight(1.0f),
    CohesionWeight(1.0f),
    RandomTargetRange(1000.0f), // Default range for random target points
    TargetPosition(FVector(0.f)),
    

    AvoidanceRadius(400.f), // Default avoidance radius
    TargetReachedTolerance(25.0f), // Default tolerance for reaching the target
    RepulsionStrength(1000.f),// Default repulsion strength
    MaxRollRate(45.0f),  // Maximum roll rate: 45 degrees per second
    MaxPitchRate(45.0f), // Maximum pitch rate: 45 degrees per second
    MaxYawRate(30.0f),
    //net(nullptr)
    uav_State(EUAV_State::EUS_IDEL),
    UDPSocket(nullptr),
    bIsSearching(false),
    bIsTargetIsFound(false),
    GridSizeX(2),
    GridSizeY(2),
    CellSize(200.f),
    bIsKickedStarted(false),
    AttachedActor(nullptr),
    bRopeAttached(false),
    bRopeDropped(false),
    bCanRopDrop(false)
    
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

	UAVMesh = CreateDefaultSubobject<USkeletalMeshComponent>(TEXT("UAVDrone"));
	SetRootComponent(UAVMesh);

	UAVCamera = CreateDefaultSubobject<UCameraComponent>(TEXT("UAVCamera"));
	UAVCamera->SetupAttachment(UAVMesh);

    // Initialize camera component
    SceneComponent = CreateDefaultSubobject<USceneCaptureComponent2D>(TEXT("SceneComponent"));
    SceneComponent->SetupAttachment(UAVCamera);

    //Create a Spline Component
    SplinePath = CreateDefaultSubobject<USplineComponent>(TEXT("SplinePath"));
    SplinePath->SetupAttachment(RootComponent);

    // Create Text Component
    RoleText = CreateDefaultSubobject<UTextRenderComponent>(TEXT("RoleText"));
    RoleText->SetupAttachment(RootComponent);


    // Initialize the rope component
    /*RopeComponent = CreateDefaultSubobject<UCableComponent>(TEXT("RopeComponent"));
    RopeComponent->SetupAttachment(RootComponent);
    RopeComponent->EndLocation = FVector(0.0f, 0.0f, -150.f);
    RopeComponent->CableWidth = 2.0f;
    RopeComponent->NumSegments = 20;
    RopeComponent->SolverIterations = 8;
    RopeComponent->SetVisibility(false);*/


    // Initialize Attachment Sphere
    AttachmentSphere = CreateDefaultSubobject<USphereComponent>(TEXT("AttachmentSphere"));
    AttachmentSphere->SetupAttachment(RootComponent);
    AttachmentSphere->SetSphereRadius(80.0f);
    //AttachmentSphere->SetRelativeLocation(RopeComponent->EndLocation);
    AttachmentSphere->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
    AttachmentSphere->SetCollisionResponseToAllChannels(ECR_Overlap);



}

ABase_UAV::~ABase_UAV()
{
    //if (net)
    //{
    //    delete net;  // Free memory
    //    net = nullptr;
    //    UE_LOG(LogTemp, Log, TEXT("YOLO model deleted."));
    //}

    if (UDPSocket)
    {
        UDPSocket->Close();
        ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->DestroySocket(UDPSocket);
    }

}

// Drop rope when UAV allign with target with formation
void ABase_UAV::DropeRope()
{
    bRopeDropped = true;
    //RopeComponent->SetVisibility(true);
}

void ABase_UAV::AttachedToTarget(AActor* Target)
{

    if (bRopeAttached || !Target) return;

    bRopeAttached = true;
    AttachedActor = Target;
    //RopeComponent->AttachToComponent(Target->GetRootComponent(), FAttachmentTransformRules::KeepWorldTransform);
    
    // Attach using capsule or mesh
    ACharacter* TargetCharacter = Cast<ACharacter>(AttachedActor);
    if (TargetCharacter)
    {
        UCapsuleComponent* Capsule = TargetCharacter->GetCapsuleComponent();
        if (Capsule)
        {
            // Disable collision if needed
            Capsule->SetCollisionEnabled(ECollisionEnabled::NoCollision);

            // Attach to sphere
            TargetCharacter->AttachToComponent(AttachmentSphere, FAttachmentTransformRules::SnapToTargetNotIncludingScale);

            // Offset upward after attaching
            float CapsuleHalfHeight = Capsule->GetScaledCapsuleHalfHeight();
            FVector NewLocation = TargetCharacter->GetActorLocation() + FVector(0, 0, CapsuleHalfHeight);
            TargetCharacter->SetActorLocation(NewLocation);
        }
    }
}

void ABase_UAV::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    // Stop and clean up the networking thread
    if (NetworkingThread)
    {
        UE_LOG(LogTemp, Warning, TEXT("Shutting down NetworkingThread..."));
        NetworkingThread->Stop();
        //NetworkingThread->Shutdown();

        if (NetworkingRunnableThread)
        {
            NetworkingRunnableThread->WaitForCompletion();
            delete NetworkingRunnableThread;
            NetworkingRunnableThread = nullptr;
        }

        delete NetworkingThread;
        NetworkingThread = nullptr;
    }

    // Close and clean up the socket
    if (Socket)
    {
        UE_LOG(LogTemp, Warning, TEXT("Closing and destroying the socket..."));

        if (Socket->GetConnectionState() == ESocketConnectionState::SCS_Connected)
        {
            Socket->Close();
        }

        ISocketSubsystem* SocketSubsystem = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM);
        if (SocketSubsystem)
        {
            SocketSubsystem->DestroySocket(Socket);
        }

        Socket = nullptr;  // Prevent dangling pointer issues

        /*if (Socket->Close())
        {
            ISocketSubsystem* SocketSubsystem = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM);
            if (SocketSubsystem)
            {
                SocketSubsystem->DestroySocket(Socket);
                Socket = nullptr;
            }
            else
            {
                UE_LOG(LogTemp, Error, TEXT("SocketSubsystem is null. Cannot destroy socket."));
            }

        }
        else
        {
            UE_LOG(LogTemp, Error, TEXT("Failed to close the socket."));
        }*/
      /*  if (SocketSubsystem)
        {
            UE_LOG(LogTemp, Warning, TEXT("Closing and destroying the socket..."));
            Socket->Close();
            
        }
        else
        {
            UE_LOG(LogTemp, Error, TEXT("SocketSubsystem is null. Cannot destroy socket."));
        }*/

       
    }
    else
    {
        UE_LOG(LogTemp, Warning, TEXT("Socket is already null. Skipping cleanup."));
    }

    Super::EndPlay(EndPlayReason);

    if (HasAuthority())
        GlobalSearchGrid.Empty(); // Force reset the Global Serch grid each simulation

}

// Called when the game starts or when spawned
void ABase_UAV::BeginPlay()
{
	Super::BeginPlay();

    AssignFormationOffsets(); // Ensure correct positions are assigned
    DetermineLeader();  // Ensure leader is assigned

    SetupUDPReceiver();
    SetupTriggerSocket();

    // Sent the number of uav to server
    //NotifyPythonServer();

    // Initialize Grid
    //InitializeGrid();
    //AssignSearchCell();
    
    // Initialize PID controllers
    RollController = MakeUnique<PIDController>(RollKp, RollKi, RollKd);
    PitchController = MakeUnique<PIDController>(PitchKp, PitchKi, PitchKd);
    YawController = MakeUnique<PIDController>(YawKp, YawKi, YawKd);
    AltitudeController = MakeUnique<PIDController>(AltitudeKp, AltitudeKi, AltitudeKd);
    ForwardController = MakeUnique<PIDController>(ForwardKp, ForwardKi, ForwardKd);
    RightController = MakeUnique<PIDController>(RightKp, RightKi, RightKd);

	CurrentPosition = GetActorLocation();
	CurrentRotation = GetActorRotation();

    // Generate evenly distributed ray directions
    RayDirections = GenerateSpiralRayDirections(50, 0.618033f, 1.0f); // Golden ratio turn fraction
	

    // Set integral limits
    RollController->SetIntegralLimits(-50.0f, 50.0f);
    PitchController->SetIntegralLimits(-50.0f, 50.0f);
    YawController->SetIntegralLimits(-50.0f, 50.0f);
    AltitudeController->SetIntegralLimits(-100.0f, 100.0f);
    ForwardController->SetIntegralLimits(-100.0f, 100.0f);
    RightController->SetIntegralLimits(-100.0f, 100.0f);

    // Set output limits to ensure stable UAV behavior
    RollController->SetOutputLimits(-300.0f, 300.0f);
    PitchController->SetOutputLimits(-300.0f, 300.0f);
    YawController->SetOutputLimits(-300.0f, 300.0f);
    AltitudeController->SetOutputLimits(-800.0f, 800.0f);
    ForwardController->SetOutputLimits(-600.0f, 600.0f);
    RightController->SetOutputLimits(-600.0f, 600.0f);


    FVector2D ViewportRes = FVector2D();
    if (GEngine && GEngine->GameViewport)
    {
        GEngine->GameViewport->GetViewportSize(ViewportRes);
    }

    UE_LOG(LogTemp, Warning, TEXT(" Original Viewport SizeX :%f"), ViewportRes.X);
    UE_LOG(LogTemp, Warning, TEXT(" Original Viewport SizeY: %f"), ViewportRes.Y);


    UE_LOG(LogTemp, Warning, TEXT(" Original Resolution SizeX : % d"), GSystemResolution.ResX);
    UE_LOG(LogTemp, Warning, TEXT(" Original Resolution SizeY: %d"), GSystemResolution.ResY);


    // Make sure target is set
    if (SceneComponent && RenderTarget)
    {
        
        RenderTarget->ResizeTarget(640, 640); // Set to 640x640 for efficient YOLO detection
        UE_LOG(LogTemp, Warning, TEXT("Fixed RenderTarget resolution to 640x640 for Object Detection"));
        SceneComponent->TextureTarget = RenderTarget;
    }


    // Networking -> BackEnd
    // Generate uniq ID based on spawn order
    //ServerPort = 5000;
    // Create Connection to Back End Server
    ConnectToServer(UAV_ID,5000);
   /* FString ServerIPAddress = TEXT("127.0.0.1");
    ServerPort = 5000;

    FIPv4Address::Parse(ServerIPAddress, ServerIP);

    ISocketSubsystem* SocketSubsystem = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM);
    Socket = SocketSubsystem->CreateSocket(NAME_Stream, TEXT("NetworkSocket"), false);

    FIPv4Endpoint Endpoint(ServerIP, ServerPort);
    if (Socket->Connect(*Endpoint.ToInternetAddr()))
    {
        UE_LOG(LogTemp, Warning, TEXT("Connected to backend server!"));

        NetworkingThread = new NetworkThread(Socket);
        if (NetworkingThread->Init())
        {
            NetworkingRunnableThread = FRunnableThread::Create(NetworkingThread, TEXT("NetworkingThread"));
        }
        else
        {
            UE_LOG(LogTemp, Error, TEXT("Failed to initialize NetworkingThread."));
            delete NetworkingThread;
            NetworkingThread = nullptr;
        }
    }
    else
    {
        UE_LOG(LogTemp, Error, TEXT("Failed to connect to backend server."));
        if (Socket)
        {
            Socket->Close();
            SocketSubsystem->DestroySocket(Socket);
            Socket = nullptr;
        }
    }*/
    

    
    // Call StreamRawCameraFeed every 0.033 seconds (30 FPS)
    //GetWorldTimerManager().SetTimer(
    //    StreamTimerHandle,
    //    this,
    //    &ABase_UAV::CaptureFrame,
    //    0.033f,
    //    true
    //);

    //// Call ReceiveProcessedVideo every 0.033 seconds
    //GetWorldTimerManager().SetTimer(
    //    ReceiveTimerHandle,
    //    this,
    //    &ABase_UAV::LoadDetectionResults,
    //    0.033f,
    //    true
    //);


    // Pre load the CVModel
    //UCVModel::GetInstance()->LoadModelAsync();
    //LoadYOLOModel();
}

void ABase_UAV::SetTargetPosition(FVector NewTarget)
{
    TargetPosition = NewTarget;
}

void ABase_UAV::SetTargetRotation(FRotator NewTarget)
{
    TargetRotation = NewTarget;
}

void ABase_UAV::DrawPathLine()
{
    if (!GetWorld()) return;

    DrawDebugLine(
        GetWorld(),
        GetActorLocation(),
        TargetPosition,
        FColor::Green,  // Line color
        false,          // Persistent (false means it disappears)
        -1,             // Lifetime (-1 means forever)
        0,              // Depth priority
        5.0f            // Line thickness
    );
}

void ABase_UAV::UpdateSplinePath()
{
    if (!SplinePath) return;

    // Clear old points
    SplinePath->ClearSplinePoints();

    // Add Start Point (UAV's Position)
    SplinePath->AddSplinePoint(GetActorLocation(), ESplineCoordinateSpace::World, true);

    // Add Midpoint for curve (optional, can be dynamically calculated)
    FVector MidPoint = (GetActorLocation() + TargetPosition) / 2 + FVector(0, 0, 100);
    SplinePath->AddSplinePoint(MidPoint, ESplineCoordinateSpace::World, true);

    // Add End Point (Target Position)
    SplinePath->AddSplinePoint(TargetPosition, ESplineCoordinateSpace::World, true);

    // Update the Spline
    SplinePath->UpdateSpline();
}

void ABase_UAV::UpdateRoleText()
{
    if (!RoleText) return;

    if (bIsLeader)
    {
        RoleText->SetText(FText::FromString("LEADER"));
        RoleText->SetTextRenderColor(FColor::Red);
    }
    else
    {
        RoleText->SetText(FText::FromString("FOLLOWER"));
        RoleText->SetTextRenderColor(FColor::Blue);
    }

    // Position the text above the UAV
    RoleText->SetRelativeLocation(FVector(0, 0, 100));
    RoleText->SetHorizontalAlignment(EHTA_Center);
    RoleText->SetWorldSize(50);
}

void ABase_UAV::DrawFormationDebug()
{
    if (!bIsLeader) return;

    for (int32 i = 0; i < SquareOffsets.Num(); i++)
    {
        if (!SquareOffsets.IsValidIndex(i)) continue; // Prevent out-of-bounds access

        FVector TargetPos = GetActorLocation() + SquareOffsets[i];

        DrawDebugSphere(GetWorld(), TargetPos, 20.0f, 12, FColor::Green, false, 5.0f, 0, 2.0f);
     
    }
}

void ABase_UAV::SetUAV_ID(int32 NewID)
{
    UAV_ID = NewID;
    UE_LOG(LogTemp, Warning, TEXT("UAV ID Set: %d"), UAV_ID);
}

FVector ABase_UAV::ComputeSeparationForce() const
{
    FVector SeparationForce = FVector::ZeroVector;
    int NeighborCount = 0;

    for (AActor* UAV : OtherUAVs) {
        if (!UAV) continue;

        FVector OtherPosition = UAV->GetActorLocation();
        float Distance = FVector::Dist(GetActorLocation(), OtherPosition);

        if (Distance < PerceptionRadius && Distance > 0.0f) {
            SeparationForce += (GetActorLocation() - OtherPosition).GetSafeNormal() / Distance;
            NeighborCount++;
        }
    }

    if (NeighborCount > 0) {
        SeparationForce /= NeighborCount; // Average the force
    }

    return SeparationForce;
}

FVector ABase_UAV::ComputeAlignmentForce() const
{
    FVector AverageVelocity = FVector::ZeroVector;
    int NeighborCount = 0;

    for (AActor* UAV : OtherUAVs) {
        if (!UAV) continue;

        float Distance = FVector::Dist(GetActorLocation(), UAV->GetActorLocation());

        if (Distance < PerceptionRadius) {
            ABase_UAV* OtherUAVController = Cast<ABase_UAV>(UAV);
            if (OtherUAVController) {
                AverageVelocity += OtherUAVController->CurrentVelocity;
                NeighborCount++;
            }
        }
    }

    if (NeighborCount > 0) {
        AverageVelocity /= NeighborCount; // Average velocity of neighbors
    }

    return (AverageVelocity - CurrentVelocity).GetClampedToMaxSize(100.0f); // Steer towards alignment

}

FVector ABase_UAV::ComputeCohesionForce() const
{
    FVector CenterOfMass = FVector::ZeroVector;
    int NeighborCount = 0;

    for (AActor* UAV : OtherUAVs) {
        if (!UAV) continue;

        float Distance = FVector::Dist(GetActorLocation(), UAV->GetActorLocation());

        if (Distance < PerceptionRadius) {
            CenterOfMass += UAV->GetActorLocation();
            NeighborCount++;
        }
    }

    if (NeighborCount > 0) {
        CenterOfMass /= NeighborCount; // Calculate the center of mass
    }

    return (CenterOfMass - GetActorLocation()).GetSafeNormal(); // Steer towards the center

}

FVector ABase_UAV::ComputeObstacleAvoidanceForce() const
{
    int32 NumRays = 100;         // Number of directions
    //float ViewRadiuss = 500.0f;   // Raycast length (radius for obstacle avoidance)
    //TArray<FVector> RayDirections = GenerateFibonacciSpherePoints(NumRays);

    FVector BestDirection = GetActorForwardVector(); // Default direction
    float FurthestDistance = 50.0f;
    float BestAalignment = -1.f;
    bool ObstacleDetected = false;

    for (const FVector& Direction : RayDirections) {
        // Scale the unit direction vector by ViewRadius
        FVector WorldDirection = Direction * ViewRadius;

        FVector Start = GetActorLocation();
        FVector End = Start + WorldDirection;

        FHitResult Hit;
        bool bHit = GetWorld()->LineTraceSingleByChannel(
            Hit,
            Start,
            End,
            ECC_Visibility
        );

        // Debugging
        DrawDebugLine(GetWorld(), Start, End, bHit ? FColor::Red : FColor::Green, false, 0.0f, 0, 1.0f);


        if (bHit)
        {
            ObstacleDetected = true;
        }
        else
        {
            // If the ray dosen't hit an obsticale
            // Compute alignment with forward direction
            float alignment = FVector::DotProduct(Direction, GetActorForwardVector());
            if (alignment > BestAalignment)
            {
                BestAalignment = alignment; // Update best alignment
                BestDirection = Direction; // Update to best direction
            }
        }

        // If an obstical detected, move in the best unhit direction
        if (ObstacleDetected)
        {
            return BestDirection * ViewRadius;
        }


        //if (bHit) 
        //{

        //    if (Hit.Distance > FurthestDistance) {
        //        BestDirection = WorldDirection.GetSafeNormal(); // Update to best direction
        //        FurthestDistance = Hit.Distance;
        //    }
        //}
    }
    // Return the best direction scaled by a desired movement magnitude
    //return BestDirection * FurthestDistance;

    return GetActorForwardVector() * ViewRadius;

}

TArray<FVector> ABase_UAV::GenerateSpiralRayDirections(int32 NumPoints, float TurnFraction, float Power)
{
    TArray<FVector> Directions;

    float GoldenRatio = (1.0f + FMath::Sqrt(5.0f)) / 2.0f; // Golden ratio
    float AngleIncrement = 2.0f * PI * GoldenRatio;        // Angle increment for spiral

    for (int32 i = 0; i < NumPoints; i++) {
        float t = (float)i / (NumPoints - 1);              // Normalize i to range [0, 1]
        float Inclination = FMath::Acos(1.0f - 2.0f * t);  // Polar angle (latitude)
        float Azimuth = AngleIncrement * i;               // Azimuthal angle (longitude)

        // Convert spherical coordinates to Cartesian
        float x = FMath::Sin(Inclination) * FMath::Cos(Azimuth);
        float y = FMath::Sin(Inclination) * FMath::Sin(Azimuth);
        float z = FMath::Cos(Inclination);

        Directions.Add(FVector(x, y, z)); // Unit sphere points
    }

    return Directions;
}


// Worked ( each drones avoid each other)
FVector ABase_UAV::AvoidCollisions()
{
    FVector RepulsionForce1 = FVector::ZeroVector;
    // Iterate through all actors of type ABase_Uav in the world
    for (TActorIterator<ABase_UAV> It(GetWorld()); It; ++It)
    {
        ABase_UAV* OtherUAV = *It;

        // Ensure we are not calculating repulsion itself
        if (OtherUAV && OtherUAV != this)
        {
            // Get the distance between this uav and next uav
            float Distance = FVector::Dist(OtherUAV->GetActorLocation(), GetActorLocation());
            float MinDistance = 10.0f;
            if (Distance < AvoidanceRadius && Distance > MinDistance)
            {
                FVector Direction = (GetActorLocation() - OtherUAV->GetActorLocation()).GetSafeNormal();
                RepulsionForce1 += Direction * (RepulsionStrength / FMath::Square(FMath::Max(Distance, MinDistance)));
            }
        }
    }

    // Avoid environmental obstacle using sphereTrace
    FCollisionQueryParams Traceparams;
    Traceparams.AddIgnoredActor(this);
    Traceparams.bTraceComplex = true;
    Traceparams.bReturnPhysicalMaterial = false;

    TArray<FHitResult> HitResults;
    FVector StartLocation = GetActorLocation();
    FVector EndLocation = StartLocation + GetActorForwardVector() * AvoidanceRadius; // Static avoidance, cast around the uav
    float SphereRadius = AvoidanceRadius;

    // Perform a sphere trace 
    if (GetWorld()->SweepMultiByChannel(
        HitResults,
        StartLocation,
        EndLocation,
        FQuat::Identity,
        ECC_Visibility, // Collision channel
        FCollisionShape::MakeSphere(SphereRadius),
        Traceparams
    ))
    {
        for (const FHitResult& Hit : HitResults)
        {
            // Check if the hit is other actors in the environment
            if (AActor* HitActor = Hit.GetActor())
            {
                // Ensure the detected actor is an obstcale and not a UAV
                if (HitActor != this && !HitActor->IsA(ABase_UAV::StaticClass()))
                {
                    FVector Direction = (GetActorLocation() - Hit.ImpactPoint).GetSafeNormal();
                    float Distance = FVector::Dist(Hit.ImpactPoint, GetActorLocation());
                    float MinDistance = 10.0f;

                    if (Distance < AvoidanceRadius && Distance > MinDistance)
                    {
                        RepulsionForce1 += Direction * (RepulsionStrength / FMath::Square(FMath::Max(Distance, MinDistance)));
                    }
                }
            }


            // Check if the hit is another statick mesh or skeletal mesh
            if (UPrimitiveComponent* HitComponent = Hit.GetComponent())
            {
                // Check if the hit is a static or skeletal component
                if (HitComponent->IsA(UStaticMeshComponent::StaticClass()) ||
                    HitComponent->IsA(USkeletalMeshComponent::StaticClass()))
                {
                    FVector DirectionToHit = (Hit.ImpactPoint - GetActorLocation()).GetSafeNormal();
                    FVector AvoidanceDirection = FVector::CrossProduct(DirectionToHit, FVector::UpVector).GetSafeNormal();

                    RepulsionForce1 += RepulsionForce1 * RepulsionStrength;
                    // Debug: Visualize the avoidance direction
                    //DrawDebugLine(GetWorld(), GetActorLocation(), GetActorLocation() + AvoidanceDirection * 200.0f, FColor::Blue, false, 0.1f, 0, 2.0f);
                    DrawDebugLine(GetWorld(), GetActorLocation(), Hit.ImpactPoint, FColor::Black, false, 0.1f, 0, 2.0f);
                }
            }

            DrawDebugSphere(GetWorld(), Hit.ImpactPoint, 10.0f, 12, FColor::Red, false, 0.1f);
            //DrawDebugSphere(GetWorld(), GetActorLocation(), AvoidanceRadius, 12, FColor::Green, false, 0.1f);

        }
    }


    float MaxForce = 10.f;
    RepulsionForce1 = RepulsionForce1.GetClampedToMaxSize(MaxForce);
    return RepulsionForce1;
}

void ABase_UAV::LoadYOLOModel()
{
    // Load the yolo model and names
   // FString ModelPath = FPaths::ProjectConfigDir() + TEXT("Models/yolov5x.onnx");
    FString NamesPath = FPaths::ProjectConfigDir() + TEXT("Models/coco.names");

    // Load Model
    //cv::dnn::Net LoadedNet(cv::dnn::readNet(TCHAR_TO_UTF8(*ModelPath)));
    //net->setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    //net->setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

    //// Load class name
    //std::ifstream ifs(TCHAR_TO_UTF8(*NamesPath));
    //std::string line;
    //while (std::getline(ifs, line)) { classNames.push_back(line); }

    // UE_LOG(LogTemp, Warning, TEXT("Starting YOLO model load asynchronously..."));

    //AsyncTask(ENamedThreads::AnyBackgroundThreadNormalTask, [this]()
    //    {
    //        FString ModelPath = FPaths::ProjectContentDir() + TEXT("Models/yolov5x.onnx");

    //        UE_LOG(LogTemp, Log, TEXT("Loading YOLO model from: %s"), *ModelPath);

    //        // Load the model in a background thread
    //        cv::dnn::Net* LoadedNet = new cv::dnn::Net(cv::dnn::readNet(TCHAR_TO_UTF8(*ModelPath)));

    //        // Move back to the Game Thread to store the loaded model safely
    //        AsyncTask(ENamedThreads::GameThread, [this, LoadedNet]()
    //            {
    //                net = LoadedNet;
    //                net->setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    //                net->setPreferableTarget(cv::dnn::DNN_TARGET_CPU);

    //                UE_LOG(LogTemp, Log, TEXT("YOLO model loaded successfully!"));
    //            });
    //});



}

void ABase_UAV::PostProcessYOLO()
{

}

bool ABase_UAV::IsServerAvilable()
{

    if (!Socket) return false;

    const char* HeartbeatMessage = "ping";
    int32 BytesSent = 0;

    // Attempt to send a heartbeat message
    bool bSent = Socket->Send((uint8*)HeartbeatMessage, strlen(HeartbeatMessage), BytesSent);

    if (bSent && BytesSent > 0)
    {
        UE_LOG(LogTemp, Warning, TEXT("Server is available."));
        return true;
    }

    UE_LOG(LogTemp, Error, TEXT("Server is not reachable."));
    return false;
}

void ABase_UAV::CheckServerAvilability()
{
    if (!IsServerAvilable())
    {
        GEngine->AddOnScreenDebugMessage(
            -1,
            5.0f,
            FColor::Red,
            TEXT("Backend server is not running. Please start the server!")
        );
    }
}

void ABase_UAV::ReconnectToServer()
{
    if (Socket->GetConnectionState() != SCS_Connected)
    {
        UE_LOG(LogTemp, Warning, TEXT("Attempting to reconnect to the server..."));

        FIPv4Endpoint Endpoint(ServerIP, ServerPort);
        bool bReconnected = Socket->Connect(*Endpoint.ToInternetAddr());

        if (bReconnected)
        {
            UE_LOG(LogTemp, Warning, TEXT("Reconnected to the server!"));
        }
        else
        {
            UE_LOG(LogTemp, Error, TEXT("Reconnection failed. Retrying..."));
        }
    }
}

void ABase_UAV::CaptureFrame()
{
    

    if (!SceneComponent || !RenderTarget || !NetworkingThread) return;

    int32 Width = RenderTarget->SizeX;
    int32 Height = RenderTarget->SizeY;

    UE_LOG(LogTemp, Warning, TEXT("Capturing Frame - RenderTarget Size: %d x %d"), Width, Height);

    // Access RenderTarget resource
    FTextureRenderTargetResource* Resource = RenderTarget->GameThread_GetRenderTargetResource();
    TArray<FColor> Pixels;
    Resource->ReadPixels(Pixels);

    if (Pixels.Num() == 0)
    {
        UE_LOG(LogTemp, Error, TEXT("Captured frame is empty!"));
        return;
    }

    // Convert Pixels to JPEG using UE's Image Wrapper Module
    TArray<uint8> CompressedData;
    IImageWrapperModule& ImageWrapperModule = FModuleManager::LoadModuleChecked<IImageWrapperModule>(FName("ImageWrapper"));
    TSharedPtr<IImageWrapper> ImageWrapper = ImageWrapperModule.CreateImageWrapper(EImageFormat::JPEG);

    if (ImageWrapper.IsValid() && ImageWrapper->SetRaw(Pixels.GetData(), Pixels.GetAllocatedSize(), Width, Height, ERGBFormat::BGRA, 8))
    {
        CompressedData = ImageWrapper->GetCompressed(100);  // 100 = Max JPEG Quality
    }

    if (CompressedData.Num() == 0)
    {
        UE_LOG(LogTemp, Error, TEXT("Failed to compress frame."));
        return;
    }

    UE_LOG(LogTemp, Warning, TEXT("Compressed frame size: %d bytes"), CompressedData.Num());

    //Send compressed data to networking thread asynchronously
    NetworkingThread->EnqueueSendData(CompressedData);
    UE_LOG(LogTemp, Warning, TEXT("Compressed frame sent to networking thread"));



    //if (!SceneComponent || !RenderTarget || !NetworkingThread) return;

    //int32 Width = RenderTarget->SizeX;
    //int32 Height = RenderTarget->SizeY;

    //UE_LOG(LogTemp, Warning, TEXT("Capturing Frame - RenderTarget Size: %d x %d"), Width, Height);

    //// Access the RenderTarget's resource
    //FTextureRenderTargetResource* Resource = RenderTarget->GameThread_GetRenderTargetResource();
    //TArray<FColor> Pixels;
    //Resource->ReadPixels(Pixels);

    //if (Pixels.Num() == 0)
    //{
    //    UE_LOG(LogTemp, Error, TEXT("Captured frame is empty!"));
    //    return;
    //}

    //// Compress Pixels to JPEG
    //TArray<uint8> CompressedData;
    //FImageUtils::CompressImageArray(Width, Height, Pixels, CompressedData);

    //UE_LOG(LogTemp, Warning, TEXT("Compressed frame size: %d bytes"), CompressedData.Num());

    //// Send compressed data to networking thread
    //NetworkingThread->EnqueueSendData(CompressedData);

    //UE_LOG(LogTemp, Warning, TEXT("Compressed frame sent to networking thread"));



    //if (!NetworkingThread) return;
    //TArray<uint8> ReceivedData;
    //if (NetworkingThread->DequeueReceivedData(ReceivedData))
    //{
    //    // Process the received data
    //    int32 Width = 1280;
    //    int32 Height = 720;

    //    UTexture2D* ProcessedTexture = UTexture2D::CreateTransient(Width, Height);

    //    if (ProcessedTexture)
    //    {
    //        FTexture2DMipMap& Mip = ProcessedTexture->GetPlatformData()->Mips[0];
    //        void* TextureData = Mip.BulkData.Lock(LOCK_READ_WRITE);
    //        FMemory::Memcpy(TextureData, ReceivedData.GetData(), ReceivedData.Num());
    //        Mip.BulkData.Unlock();
    //        ProcessedTexture->UpdateResource();

    //        UE_LOG(LogTemp, Warning, TEXT("Processed Video Received and Loaded into Texture!"));
    //    }
    //}
    //else
    //{
    //    UE_LOG(LogTemp, Warning, TEXT("No data received from the server."));
    //}
    



    //if (!SceneComponent || !RenderTarget || !Socket) return;

    //if (!Socket)
    //{
    //    UE_LOG(LogTemp, Error, TEXT("Socket is not initialized!"));
    //    return;
    //}

    //if (Socket->GetConnectionState() != SCS_Connected)
    //{
    //    UE_LOG(LogTemp, Error, TEXT("Socket is not connected to the server!"));
    //    return;
    //}
    //// Get UAV Camera Render Target
    //FTextureRenderTargetResource* Resource = RenderTarget->GameThread_GetRenderTargetResource();
    //TArray<FColor> Pixels;
    //FIntPoint Size(RenderTarget->SizeX, RenderTarget->SizeY); // 2048x1080
    //Resource->ReadPixels(Pixels);

    //// Check if Pixels array is valid
    //if (Pixels.Num() == 0)
    //{
    //    UE_LOG(LogTemp, Error, TEXT("Captured frame is empty!"));
    //    return;
    //}

    //// Compress Pixels to JPEG
    //TArray<uint8> CompressedData;
    //FImageUtils::CompressImageArray(Size.X, Size.Y, Pixels, CompressedData);


    //// Ensure data size matches 2K resolution
    //int32 ExpectedSize = Size.X * Size.Y * sizeof(FColor);
    //if (Pixels.Num() * sizeof(FColor) != ExpectedSize)
    //{
    //    UE_LOG(LogTemp, Error, TEXT("Unexpected data size: %d bytes (Expected: %d bytes)"), Pixels.Num() * sizeof(FColor), ExpectedSize);
    //    return;
    //}

    //int32 DataSize = Pixels.Num() * sizeof(FColor); // Calculate the data size
    //UE_LOG(LogTemp, Warning, TEXT("Data size being sent: %d bytes"), DataSize);

    //// Send Raw Pixel Data via UDP
    //int32 BytesSent = 0;
    //bool bSent = Socket->Send((uint8*)Pixels.GetData(), Pixels.Num() * sizeof(FColor), BytesSent);

    //if (!bSent || BytesSent <= 0)
    //{
    //    UE_LOG(LogTemp, Error, TEXT("Failed to send data. Bytes sent: %d"), BytesSent);
    //}
    //else
    //{
    //    UE_LOG(LogTemp, Warning, TEXT("Data successfully sent: %d bytes"), BytesSent);
    //}

}

void ABase_UAV::LoadDetectionResults()
{

    if (!NetworkingThread) return;

    TArray<uint8> ReceivedData;
    if (NetworkingThread->DequeueReceivedData(ReceivedData))
    {
        // Process received data (e.g., display processed video)
        UE_LOG(LogTemp, Warning, TEXT("Processed data received from server."));
    }
    else
    {
        UE_LOG(LogTemp, Warning, TEXT("No processed data available yet."));
    }


    //if(!Socket || !NetworkingThread) return;

    //uint8 DataBuffer[921600]; // Buffer for image
    //int32 BytesReceived = 0;

    //if (Socket->Recv(DataBuffer, sizeof(DataBuffer), BytesReceived))
    //{
    //    UE_LOG(LogTemp, Warning, TEXT("Processed video received."));
    //    // Create a UTexture2D dynamically
    //    int32 Width = 1280; // Set your frame width
    //    int32 Height = 720; // Set your frame height
    //    UTexture2D* ProcessedTexture = UTexture2D::CreateTransient(Width, Height);

    //    if (ProcessedTexture)
    //    {
    //        // Lock the texture for writing
    //        FTexture2DMipMap& Mip = ProcessedTexture->GetPlatformData()->Mips[0];
    //        void* TextureData = Mip.BulkData.Lock(LOCK_READ_WRITE);

    //        // Copy pixel data into texture
    //        FMemory::Memcpy(TextureData, DataBuffer, BytesReceived);

    //        // Unlock and update texture
    //        Mip.BulkData.Unlock();
    //        ProcessedTexture->UpdateResource();

    //        // Use the texture in your RenderTarget or elsewhere
    //        UE_LOG(LogTemp, Warning, TEXT("Processed Video Received and Loaded into Texture!"));
    //    }
    //}
    //else
    //{
    //    UE_LOG(LogTemp, Error, TEXT("Failed to receive video from the server."));
    //}
    //

}

void ABase_UAV::ConnectToServer(int32 r_UAV_ID, int32 r_ServerPort)
{
    FString ServerIPAddress = TEXT("127.0.0.1");
    const int32 MaxRetries = 20;  // Number of times to retry connection
    const float RetryDelay = 3.0f;  // Delay between retries in seconds
    int32 CurrentRetry = 0;
    UE_LOG(LogTemp, Error, TEXT("Create Connection Callled"));

    if (SocketMap.Contains(r_UAV_ID))
    {
        FSocket* ExisitingSocket = SocketMap[r_UAV_ID];
        if (ExisitingSocket)
        {
            ExisitingSocket->Close();
            ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->DestroySocket(ExisitingSocket);
        }
        SocketMap.Remove(r_UAV_ID);
    }



    FIPv4Address IP;
    if (!FIPv4Address::Parse(ServerIPAddress, IP))
    {
        UE_LOG(LogTemp, Error, TEXT("Invalid IP format"));
        return;
    }

    TSharedPtr<FInternetAddr> Endpoint = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->CreateInternetAddr();
    Endpoint->SetIp(IP.Value);
    Endpoint->SetPort(r_ServerPort);

    ISocketSubsystem* SocketSubsystem = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM);
    FSocket* NewSocket = SocketSubsystem->CreateSocket(NAME_Stream, TEXT("NetworkSocket"), false);

    if (!NewSocket)
    {
        UE_LOG(LogTemp, Error, TEXT("Failed to create socket."));
        return;
    }

    // Run connection attempt asynchronously to avoid blocking game startup
    /*AsyncTask(ENamedThreads::AnyBackgroundThreadNormalTask, [this, Endpoint]()
        {
            bool bConnected = Socket->Connect(*Endpoint);

            AsyncTask(ENamedThreads::GameThread, [this, bConnected]()
                {
                    if (bConnected)
                    {
                        UE_LOG(LogTemp, Warning, TEXT("Connected to backend server!"));

                        NetworkingThread = new NetworkThread(Socket);
                        if (NetworkingThread->Init())
                        {
                            NetworkingRunnableThread = FRunnableThread::Create(NetworkingThread, TEXT("NetworkingThread"));
                        }
                        else
                        {
                            UE_LOG(LogTemp, Error, TEXT("Failed to initialize NetworkingThread."));
                            delete NetworkingThread;
                            NetworkingThread = nullptr;
                        }
                    }
                    else
                    {
                        UE_LOG(LogTemp, Error, TEXT("Failed to connect to backend server."));
                        if (Socket)
                        {
                            ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->DestroySocket(Socket);
                            Socket = nullptr;
                        }
                    }
                });
        });*/

    SocketMap.Add(r_UAV_ID, NewSocket);
        // Run connection attempt asynchronously to avoid blocking game startup
    AsyncTask(ENamedThreads::AnyBackgroundThreadNormalTask, [this, Endpoint, MaxRetries, RetryDelay, &CurrentRetry, r_UAV_ID, NewSocket]()
        {
            bool bConnected = false;

            while (!bConnected && CurrentRetry < MaxRetries)
            {
                bConnected = NewSocket->Connect(*Endpoint);

                if (bConnected)
                {
                    UE_LOG(LogTemp, Warning, TEXT("UAV %d: Connected to backend server!"), UAV_ID);
                    AsyncTask(ENamedThreads::GameThread, [this, r_UAV_ID]()
                        {
                            NetworkingThread = new NetworkThread(SocketMap[r_UAV_ID]);
                            if (NetworkingThread->Init())
                            {
                                NetworkingRunnableThread = FRunnableThread::Create(NetworkingThread, TEXT("NetworkingThread"));
                            }
                            else
                            {
                                UE_LOG(LogTemp, Error, TEXT("UAV %d: Failed to initialize NetworkingThread."), r_UAV_ID);
                                delete NetworkingThread;
                                NetworkingThread = nullptr;
                            }

                            
                        });
                    break; // Exit the retry loop on successful connection
                }
                else
                {
                    UE_LOG(LogTemp, Warning, TEXT("UAV %d: Failed to connect to backend server. Retrying in %.1f seconds..."), UAV_ID, RetryDelay);
                    CurrentRetry++;
                    FPlatformProcess::Sleep(RetryDelay);  // Wait before retrying
                }
            }

            if (!bConnected)
            {
                UE_LOG(LogTemp, Error, TEXT("UAV %d: Failed to connect to backend server after %d attempts."), UAV_ID, MaxRetries);

                AsyncTask(ENamedThreads::GameThread, [this, r_UAV_ID]()
                    {
                        if (SocketMap.Contains(r_UAV_ID))
                        {
                            FSocket* FailedSocket = SocketMap[r_UAV_ID];
                            if (FailedSocket)
                            {
                                ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->DestroySocket(FailedSocket);
                                SocketMap.Remove(r_UAV_ID);
                            }
                        }
                    });
            }
        });




    /*FString ServerIPs;
    int32 serverPort = 5000;

    FIPv4Address IP;
    FIPv4Address::Parse(ServerIPs, IP);

    TSharedPtr<FInternetAddr> Addr = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->CreateInternetAddr();
    Addr->SetIp(IP.Value);
    Addr->SetPort(serverPort);

    UAVSocket = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->CreateSocket(NAME_Stream, TEXT("UAVSocket"), false);
    if (!UAVSocket)
    {
        UE_LOG(LogTemp, Error, TEXT("Failed to create UAV socket!"));
        return;
    }
    if (UAVSocket->Connect(*Addr))
    {
        UE_LOG(LogTemp, Log, TEXT("UAV %d connected to server at %s:%d"), UAV_ID, *ServerIPs, ServerPort);
    }
    else
    {
        UE_LOG(LogTemp, Warning, TEXT("UAV %d failed to connect to server"), UAV_ID);
    }*/

}

void ABase_UAV::isReached(bool bisSearch)
{
    float DistanceToTarget = FVector::Dist(CurrentPosition, TargetPosition);
    if (DistanceToTarget <= TargetReachedTolerance && !bisSearch)
    {
        // changing the state of the uave to Idel
        uav_State = EUAV_State::EUS_IDEL;
    }
    else if (/*DistanceToTarget <= TargetReachedTolerance &&*/ bisSearch)
    {
        // If not reached Keep the state to Travel
        uav_State = EUAV_State::EUS_SEARCH;
    }
    else
    {
        uav_State = EUAV_State::EUS_TRAVEL;
    }
    
}

void ABase_UAV::resetPIDController()
{
    // Stop Movement
    CurrentVelocity = FVector::ZeroVector;

    // Reset all controllers when stopping
    RollController->Reset(TargetRotation.Roll - CurrentRotation.Roll);
    PitchController->Reset(TargetRotation.Pitch - CurrentRotation.Pitch);
    YawController->Reset(TargetRotation.Yaw - CurrentRotation.Yaw);
    AltitudeController->Reset(TargetPosition.Z - CurrentPosition.Z);
    ForwardController->Reset(TargetPosition.X - CurrentPosition.X);
    RightController->Reset(TargetPosition.Y - CurrentPosition.Y);

    
}

void ABase_UAV::CalculateTargetLocation()
{
    // Compute PID outputs
    RollOutput = RollController->Compute(TargetRotation.Roll, CurrentRotation.Roll, DeltaX);
    PitchOutput = PitchController->Compute(TargetRotation.Pitch, CurrentRotation.Pitch, DeltaX);
    YawOutput = YawController->Compute(TargetRotation.Yaw, CurrentRotation.Yaw, DeltaX);

    float AltitudeOutput = AltitudeController->Compute(TargetPosition.Z, CurrentPosition.Z, DeltaX);
    float ForwardOutput = ForwardController->Compute(TargetPosition.X, CurrentPosition.X, DeltaX);
    float RightOutput = RightController->Compute(TargetPosition.Y, CurrentPosition.Y, DeltaX);

    // Orientation

    // Clamp the rotation to avoid jittery
    RollOutput = FMath::Clamp(RollOutput, -MaxRollRate, MaxRollRate);
    PitchOutput = FMath::Clamp(PitchOutput, -MaxPitchRate, MaxPitchRate);
    YawOutput = FMath::Clamp(YawOutput, -MaxYawRate, MaxYawRate);



    // Combine forces
    MovementForce = FVector(ForwardOutput, RightOutput, AltitudeOutput);

    // cap speed
    float maximumSpeed = 800.f;
    float forceMagnitude = MovementForce.Size();

    if (forceMagnitude > maximumSpeed)
    {
        MovementForce = MovementForce.GetSafeNormal() * maximumSpeed;
    }

    RepulsionForce = AvoidCollisions();
    bool bIsBlocked = RepulsionForce.Size() > 0.1f;
    
    // Log PID performance data to file
    FString FilePath = FPaths::ProjectDir() + "PID_Log.csv";

    // Write headers if file dosent exits
    if (!FPlatformFileManager::Get().GetPlatformFile().FileExists(*FilePath))
    {
        FString Header = "Time, TargetValue_Roll, CurrentValue_Roll, Error_Roll, PIDOutput_Roll, TargetValue_Pitch, CurrentValue_Pitch, Error_Pitch, PIDOutput_Pitch, TargetValue_Yaw, CurrentValue_Yaw, Error_Yaw, PIDOutput_Yaw, TargetValue_Altitude, CurrentValue_Altitude, Error_Altitude, PIDOutput_Altitude, TargetValue_Forward, CurrentValue_Forward, Error_Forward, PIDOutput_Forward, TargetValue_Right, CurrentValue_Right, Error_Right, PIDOutput_Right\n";

        FFileHelper::SaveStringToFile(Header, *FilePath);
    }

    float CurrentTime = GetWorld()->GetDeltaSeconds();

    // Log data for all controllers
    FString Line = FString::Printf(TEXT("%f,%f,%f,%f,%f,"
        "%f,%f,%f,%f,"
        "%f,%f,%f,%f,"
        "%f,%f,%f,%f,"
        "%f,%f,%f,%f,"
        "%f,%f,%f,%f\n"),
        CurrentTime,
        TargetRotation.Roll, CurrentRotation.Roll, TargetRotation.Roll - CurrentRotation.Roll, RollOutput,
        TargetRotation.Pitch, CurrentRotation.Pitch, TargetRotation.Pitch - CurrentRotation.Pitch, PitchOutput,
        TargetRotation.Yaw, CurrentRotation.Yaw, TargetRotation.Yaw - CurrentRotation.Yaw, YawOutput,
        TargetPosition.Z, CurrentPosition.Z, TargetPosition.Z - CurrentPosition.Z, AltitudeOutput,
        TargetPosition.X, CurrentPosition.X, TargetPosition.X - CurrentPosition.X, ForwardOutput,
        TargetPosition.Y, CurrentPosition.Y, TargetPosition.Y - CurrentPosition.Y, RightOutput
    );

    bool bFileSaved = FFileHelper::SaveStringToFile(Line, *FilePath, FFileHelper::EEncodingOptions::AutoDetect, &IFileManager::Get(), FILEWRITE_Append);
    if (!bFileSaved)
    {
        UE_LOG(LogTemp, Error, TEXT("Failed to save line to file: %s"), *FilePath);
    }



    Move(bIsBlocked,RepulsionForce);
    
}

void ABase_UAV::Move(const bool bIsBlocked, const FVector& targetLocation)
{
    // Reduce goal force if blocked
    if (bIsBlocked)
    {
        MovementForce *= 0.7f; // Reduce to 20% of its strength
    }
    // Update the position
    FVector NewPosition = (CurrentPosition + RepulsionForce) + MovementForce * DeltaX;
    SetActorLocation(NewPosition);

    if (FVector::Dist(GetActorLocation(), TargetPosition) < 50.0f)
    {
        bHasReachedTarget = true;
        bisSearchPhase = true;
        //uav_State = EUAV_State::EUS_SEARCH;
    }

    // Apply rotation
    FRotator NewRotation = FRotator(
        CurrentRotation.Pitch + PitchOutput,
        CurrentRotation.Yaw + YawOutput,
        CurrentRotation.Roll + RollOutput);

    // Smooth the rotation
    FRotator SmoothRotation = FMath::RInterpTo(CurrentRotation, TargetRotation, DeltaX, 60.f);

    SetActorRotation(NewRotation);

    CurrentRotation = GetActorRotation();

    DrawDebugLine(GetWorld(), GetActorLocation(), GetActorLocation() + RepulsionForce, FColor::Red, false, -1, 0, 1.0f);
    DrawDebugLine(GetWorld(), GetActorLocation(), GetActorLocation() + MovementForce, FColor::Blue, false, -1, 0, 1.0f);

    if (GetActorLocation() == targetLocation)
    {
        
        //uav_State = EUAV_State::EUS_SEARCH;
    }


    //ProcessCameraFeed();



}

void ABase_UAV::searchPhase()
{
    // Perfrom Computer vision
    /*
    * Recive the posstion of the person relative to screen space from the back-end server
    * Perfome linetrace based on the screen space get 
    *   * Get the Hit direction and calculate the distance of the person
    *   * Add the distance with the UAV which is found the person
    *       * Perfrom formation calculation
    *           * set the postitions of other UAVs
    *   * Sshare the coordinate with other UAVs
    * 
    * Set the UAV State to EUS_EXTRACT
    
    */

    

    

    if (!bIsKickedStarted)
    {
        AssignSearchCell();
        bIsKickedStarted = true;
        LogSearchTime(NumOfUAVs, UAV_ID, TEXT("Start"), 0);
    }
    // Do the Screen Capture
    static float StreamTimeAccumulator = 0.0f;
    StreamTimeAccumulator += DeltaX;

    // Send frames every 0.033 seconds (~30 FPS)
    if (StreamTimeAccumulator >= 0.033f)
    {
        if (NetworkingThread)
        {
            CaptureFrame();
        }
        StreamTimeAccumulator = 0.0f;
    }
    // Search 
    UpdateSearch();
    
    // Draw Searching Grid
    DrawSearchGrid();
   

    //UE_LOG(LogTemp, Warning, TEXT("UAV_ID: %d,NumOfSearchedCell: %d"),UAV_ID, NumOfSearchedCell);

    if (NumOfSearchedCell == GlobalSearchGrid.Num() && uav_State == EUAV_State::EUS_SEARCH)
    {
        UE_LOG(LogTemp, Warning, TEXT("Search Completed!"));
        uav_State = EUAV_State::EUS_IDEL;
        LogSearchTime(NumOfUAVs, UAV_ID, TEXT("End"), 0);
    }
    


    
    // Between Changing the grid need Delay to Do the computer vision 
    /* Means Need time to do the search to find the target
    
     Current Issue: UAV Instenly swithching bIsSearching true and false
         Solution: Add Delay to do the computer vision to find target

    
    */


    // Set bIsTarget found when shearch Grid all Searched (For Testing Purpose)
    

    // Receive the Missing Person CoordinatesLogSearchTime
   
    
   

    // Line Trace to the target position to determine the distance 

    // Move the UAV to target postion

    // Set the UAV_Stat = Extract




    /*
    * 
    *  TODO: Improvements: When cell allocated its instealy go red, but it should be nicer when UAV in the cell and turn the grid red
    
    */



}

void ABase_UAV::extractPhase()
{
    /*
    * Move all other UAV to the shared Target (Person) location
    * Drop the net ( extraction thing)
    * Get the Safe location coordinates
    * Move to the Location
    * 
    * change the UAV State to EUS_IDLE
    * 
    
    */

    // DO formatoion
    ApplyConsensusFormationControl();

    FVector Start = GetActorLocation();
    FVector End = Start + FVector(0, 0, -500); // 500 units downward

    FHitResult HitResult;
    FCollisionQueryParams Params;
    Params.AddIgnoredActor(this); // ignore self

    bool bHit = GetWorld()->LineTraceSingleByChannel(HitResult, Start, End, ECC_Visibility, Params);

    // Move sphere to end of trace
    if (AttachmentSphere)
    {
        FVector NewTargetLocation = bHit ? HitResult.Location : End;
        AttachmentSphere->SetWorldLocation(NewTargetLocation);
    }

    // (Optional) Draw debug line as rope
    DrawDebugLine(GetWorld(), Start, End, FColor::Orange, false, -1, 0, 3.0f);



    if (bRopeDropped && !bRopeAttached)
    {
        TArray<AActor*> OverlappingActor;
        AttachmentSphere->GetOverlappingActors(OverlappingActor);
        for (AActor* Actor : OverlappingActor)
        {
            if (Actor->ActorHasTag("Target"))
            {
                UE_LOG(LogTemp, Warning, TEXT("RA: Rope Attached to Target!"));
                AttachedToTarget(Actor);
                TargetPosition = FVector(2000.f, 500.f, 1000.f);
                break;
            }
        }

    }
    if (bRopeAttached && AttachedActor)
    {
        //FVector UAVLocation = GetActorLocation();

        //// Step 1: Ascend
        //FVector AscendTarget = UAVLocation + FVector(0, 0, 600.0f); // Move up 300 units
        //FVector NewLocation = FMath::VInterpTo(UAVLocation, AscendTarget, DeltaX, 1.5f);
        //SetActorLocation(NewLocation);


        Move(false, TargetPosition);
    }
}

void ABase_UAV::ApplyConsensusFormation()
{
  
   

    if (bIsLeader)
        return; // Leader stays in place

    FVector TargetFormationPos = GetDesiredFormationPosition();
    FVector Direction = (TargetFormationPos - GetActorLocation()).GetSafeNormal();

    // Move UAV smoothly to formation position
    SetActorLocation(FMath::VInterpTo(GetActorLocation(), TargetFormationPos, GetWorld()->GetDeltaSeconds(), 2.0f));

    AvoidCollision();
    
}
void ABase_UAV::AvoidCollision()
{
    TArray<ABase_UAV*> Neighbors = GetNearbyUAVs();
    FVector RepulsionForcex = FVector::ZeroVector;

    for (ABase_UAV* Neighbor : Neighbors)
    {
        FVector Difference = GetActorLocation() - Neighbor->GetActorLocation();
        float Distance = Difference.Size();

        if (Distance < CollisionRadius && Distance > 0)
        {
            RepulsionForcex += Difference.GetSafeNormal() * (1.0f / Distance);
        }
    }

    SetActorLocation(GetActorLocation() + RepulsionForcex * 10.0f);
}

FVector ABase_UAV::ComputeFormationCenter()
{
    TArray<ABase_UAV*> Neighbors = GetNearbyUAVs();
    FVector SumPosition = GetActorLocation();

    for (ABase_UAV* UAV : Neighbors)
    {
        SumPosition += UAV->GetActorLocation();
    }

    return SumPosition / (Neighbors.Num() + 1); // Include itself
}

void ABase_UAV::ApplyConsensusFormationControl()
{
    static bool bFormationComputed = false;  // Ensure we only compute once
    static TArray<AActor*> CachedUAVs;
    static FVector CachedFormationPositions[7];
    static TArray<ABase_UAV*> Followers;
    static TArray<FFormationSlot> FormationSlots;

    // Ensure a valid world exists
    if (!GetWorld()) return;

    // Step 1: Get and sort UAVs only once
    if (!bFormationComputed)
    {
        UGameplayStatics::GetAllActorsOfClass(GetWorld(), ABase_UAV::StaticClass(), CachedUAVs);

        // Sort UAVs based on their X-coordinate or spawn order
        CachedUAVs.Sort([](const AActor& A, const AActor& B) {
            return A.GetActorLocation().X < B.GetActorLocation().X;
        });


        FormationSlots = {

            // Define 2D Formation Positions
            FFormationSlot(FVector(-200, -200, 0)),  // Front Left
            FFormationSlot(FVector(200, -200, 0)),  // Front Right
            FFormationSlot(FVector(-200,    0, 0)),  // Middle Left
            FFormationSlot(FVector(0,  0, 0)),  // **Leader (Center)**
            FFormationSlot(FVector(200,    0, 0)),  // Middle Right
            FFormationSlot(FVector(-200,  200, 0)),  // Back Left
            FFormationSlot(FVector(200,  200, 0))  // Back Right
        };


        // Define 2D Formation Positions
        CachedFormationPositions[0] = FVector(-200, -200, 0);  // Front Left
        CachedFormationPositions[1] = FVector( 200, -200, 0);  // Front Right
        CachedFormationPositions[2] = FVector(-200,    0, 0);  // Middle Left
        CachedFormationPositions[3] = FVector(   0,    0, 0);  // **Leader (Center)**
        CachedFormationPositions[4] = FVector( 200,    0, 0);  // Middle Right
        CachedFormationPositions[5] = FVector(-200,  200, 0);  // Back Left
        CachedFormationPositions[6] = FVector( 200,  200, 0);  // Back Right

        bFormationComputed = true;
    }

    // Check if there are enough UAVs
    if (CachedUAVs.Num() < 7)
    {
        UE_LOG(LogTemp, Error, TEXT("Not enough UAVs! %d UAVs found, but 7 are required."), CachedUAVs.Num());
        return;
    }

    // check for leader
    ABase_UAV* LeaderUAV = nullptr;
    for (AActor* Actor : CachedUAVs)
    {
        ABase_UAV* UAV = Cast<ABase_UAV>(Actor);
        {
            if (UAV && UAV->bIsLeader)
            {
                LeaderUAV = UAV;
                break;
            }
        }
    }


    // Assign leader
    //ABase_UAV* LeaderUAV = Cast<ABase_UAV>(CachedUAVs[3]); // Middle UAV (Index 3)
    
    // If no leader selected, autimatically select the UAV at index 3 
    if (!LeaderUAV && CachedUAVs.Num() > 3)
    {
        LeaderUAV = Cast<ABase_UAV>(CachedUAVs[3]); // Middle UAV (Index 3)
        LeaderUAV->bIsLeader = true;
        //LeaderUAV->TargetLocation = FVector(2500, 1500, 2000);
    }
    if (!LeaderUAV)
    {
        UE_LOG(LogTemp, Error, TEXT("No Leader UAV found or selected."));
        return;
    }
    

    // Populate followers array
    Followers.Empty();
    for (AActor* Actor :CachedUAVs)
    {
        ABase_UAV* UAV = Cast<ABase_UAV>(Actor);
       
        if (UAV && UAV != LeaderUAV)
        {
            UAV->bIsLeader = false;
            Followers.Add(UAV);
        }
        

        //// Assign formation position if within range
        //if (PositionIndex < 7)
        //{
        //    // Store the target position in the UAV
        //    UAV->TargetFormationPosition = LeaderUAV->GetActorLocation() + CachedFormationPositions[PositionIndex];
        //    
        //    PositionIndex++; // Move to next position
        //}


    }



    // Reset the slot occupation status
    for (FFormationSlot& Slot : FormationSlots)
    {
        Slot.bIsOccupied = false; // Reset all slots to be available for new assignment
    }
    // Assign UAVs to slots (Ensuring no duplicates)
    int32 PositionIndex = 0;
    for (ABase_UAV* UAV : Followers)
    {
        if (!UAV) continue;
        bool AssignedSlot = false;
        while (PositionIndex < FormationSlots.Num() && !AssignedSlot)
        {
            if (!FormationSlots[PositionIndex].bIsOccupied)
            {
                FVector TargetFormationPos = LeaderUAV->GetActorLocation() + FormationSlots[PositionIndex].Position;
                UAV->TargetFormationPosition = TargetFormationPos;

                // mark slot as occupied
                FormationSlots[PositionIndex].bIsOccupied = true;
                AssignedSlot = true;
                //PositionIndex++;
                break; // move to the next UAV
            }
            PositionIndex++;
    
        }

        // Search for the next available slot
        //for (int32 i = 0; i < FormationSlots.Num(); i++)
        //{
        //    if (!FormationSlots[i].bIsOccupied)
        //    {
        //        FVector TargetFormationPos = LeaderUAV->GetActorLocation() + FormationSlots[i].Position;
        //        UAV->TargetFormationPosition = TargetFormationPos;

        //        // Mark the slot as occupied
        //        FormationSlots[i].bIsOccupied = true;
        //        AssignedSlot = true;
        //        break; // Exit the slot search loop once assigned
        //    }
        //}

    }

    // Consensus Control Constants
    float Kp = 5.f;
    float Kc = 3.0f;
    float DesiredSpacing = 200.f;

    

    for (ABase_UAV* UAV : Followers)
    {
        if (!UAV) continue;

        // Get this UAV's target position in formation
        /*int32 Index = Followers.Find(UAV);
        FVector TargetFormationPos = LeaderUAV->GetActorLocation() + CachedFormationPositions[Index];*/
        FVector TargetFormationPos = UAV->TargetFormationPosition;

        // Concensus Force calculations
        FVector ConcensusForce = FVector::ZeroVector;
        TArray<ABase_UAV*> Neighbors = UAV->GetNearbyUAVs();


        for (ABase_UAV* Neighbor : Neighbors)
        {
            if (Neighbor == UAV || !Neighbor) continue;


            FVector Difference = Neighbor->GetActorLocation() - UAV->GetActorLocation();
            float Distance = Difference.Size();
            float Error = Distance - DesiredSpacing;

            // Compute the correction force based on spacing error
            FVector CorrectionForce = Difference.GetSafeNormal() * Error;
            //ConcensusForce += CorrectionForce;
            ConcensusForce += ConcensusForce;
            UE_LOG(LogTemp, Warning, TEXT("UAV ID: %d,ConcensusForce: %s"), UAV->GetUAV_ID(), *ConcensusForce.ToString());

        }
        
        if (Neighbors.Num() > 0)
        {
            ConcensusForce /= Neighbors.Num(); // normalize concensus force
        }

        //Compute desired velocity using consensus algorithm
        FVector DesiredDirection = (TargetFormationPos - UAV->GetActorLocation()).GetSafeNormal();
        FVector PositionForce = Kp * DesiredDirection;
        FVector ConsensusInfluence = Kc * ConcensusForce;

        FVector NewVelocity = PositionForce + ConsensusInfluence;

        // Apply Collision Avoidance if within detection range
        FVector AvoidanceForce = FVector::ZeroVector;
        if (FVector::Dist(UAV->GetActorLocation(), TargetFormationPos) < 300.f)
        {
            AvoidanceForce = UAV->AvoidCollisions();
        }

        if (!AvoidanceForce.IsZero())
        {
            NewVelocity = FMath::Lerp(NewVelocity, AvoidanceForce, 0.5f);
        }

        // Smooth Movement Update
        FVector NewPosition = FMath::VInterpTo(UAV->GetActorLocation(), UAV->GetActorLocation() + NewVelocity, GetWorld()->GetDeltaSeconds(), 5.0f);
        UAV->TargetLocation = FVector(0.f);

        UAV->SetActorLocation(NewPosition);

        // Debugging: Show UAV target positions
        DrawDebugSphere(GetWorld(), TargetFormationPos, 20.0f, 12, FColor::Green, false, 1.0f);
        DrawDebugLine(GetWorld(), UAV->GetActorLocation(), TargetFormationPos, FColor::Blue, false, 1.0f);


    }



    //// Step 2: Assign positions to followers
    //for (int32 i = 0; i < 7; i++)
    //{
    //    ABase_UAV* UAV = Cast<ABase_UAV>(CachedUAVs[i]);
    //    if (!UAV) continue;

    //    UAV->bIsLeader = (i == 3); // Leader is at index 3
    //    FVector TargetFormationPos = LeaderUAV->GetActorLocation() + CachedFormationPositions[i];

    //    // Move UAV **only if necessary** to avoid unnecessary processing
    //    if (FVector::Dist(UAV->GetActorLocation(), TargetFormationPos) > 10.0f)
    //    {
    //        FVector DesiredDirection = (TargetFormationPos - UAV->GetActorLocation()).GetSafeNormal();
    //        FVector NewVelocity = DesiredDirection * 150.f; // Reduced Speed for stability

    //        // Apply Collision Avoidance **only if close to another UAV**
    //        FVector AvoidanceForce = FVector::ZeroVector;
    //        if (FVector::Dist(UAV->GetActorLocation(), TargetFormationPos) < 300.0f)
    //        {
    //            AvoidanceForce = UAV->AvoidCollisions();
    //        }

    //        if (!AvoidanceForce.IsZero())
    //        {
    //            NewVelocity += AvoidanceForce;
    //        }

    //        // Smooth Movement Update
    //        FVector NewPosition = FMath::VInterpTo(UAV->GetActorLocation(), TargetFormationPos, GetWorld()->GetDeltaSeconds(), 2.0f);
    //        UAV->SetActorLocation(NewPosition);

    //        // Debugging: Show UAV target positions
    //        DrawDebugSphere(GetWorld(), TargetFormationPos, 20.0f, 12, FColor::Green, false, 1.0f);
    //        DrawDebugLine(GetWorld(), UAV->GetActorLocation(), TargetFormationPos, FColor::Blue, false, 1.0f);
    //    }
    //}
}


TArray<ABase_UAV*> ABase_UAV::GetNearbyUAVs()
{
    TArray<AActor*> FoundUAVs;
    UGameplayStatics::GetAllActorsOfClass(GetWorld(), ABase_UAV::StaticClass(), FoundUAVs);

    TArray<ABase_UAV*> Neighbors;
    for (AActor* Actor : FoundUAVs)
    {
        ABase_UAV* UAV = Cast<ABase_UAV>(Actor);
        if (UAV && UAV != this && FVector::Dist(UAV->GetActorLocation(), GetActorLocation()) < NeighborRadius)
        {
            Neighbors.Add(UAV);
        }
    }
    return Neighbors;
}

void ABase_UAV::DetermineLeader()
{
    TArray<AActor*> FoundUAVs;
    UGameplayStatics::GetAllActorsOfClass(GetWorld(), ABase_UAV::StaticClass(), FoundUAVs);

    FoundUAVs.Sort([](const AActor& A, const AActor& B) {
        return A.GetActorLocation().X < B.GetActorLocation().X; // Sort UAVs by position (X-axis)
        });

    if (FoundUAVs.Num() > 0)
    {
        ABase_UAV* FirstUAV = Cast<ABase_UAV>(FoundUAVs[0]);
        if (FirstUAV == this)
        {
            bIsLeader = true;
            UE_LOG(LogTemp, Warning, TEXT("UAV Leader Selected at: %s"), *GetActorLocation().ToString());
        }
    }


    //UpdateRoleText();
}

void ABase_UAV::AssignFormationOffsets()
{
    // Assign Square Formation Offsets
   /* if (bIsLeader)
        return;*/ // Leader stays in place

    SquareOffsets.Add( FVector(-200, -200, 0)); // Top-left
    SquareOffsets.Add(FVector(200, -200, 0)); // Top-right
    SquareOffsets.Add(FVector(-200, 200, 0)); // Bottom-left
    SquareOffsets.Add(FVector(200, 200, 0)); // Bottom-right
    SquareOffsets.Add(FVector(0, -200, 0)); // Middle-top
    SquareOffsets.Add(FVector(0, 200, 0)); // Middle-bottom
    SquareOffsets.Add(FVector(-200, 0, 0)); // Middle-left
    SquareOffsets.Add(FVector(200, 0, 0)); // Middle-right


}

FVector ABase_UAV::GetDesiredFormationPosition()
{
    if (bIsLeader)
        return GetActorLocation(); // Leader stays in place

    ABase_UAV* LeaderUAV = nullptr;
    TArray<AActor*> FoundUAVs;
    UGameplayStatics::GetAllActorsOfClass(GetWorld(), ABase_UAV::StaticClass(), FoundUAVs);

    FoundUAVs.Sort([](const AActor& A, const AActor& B) {
        return A.GetActorLocation().X < B.GetActorLocation().X;
        });

    if (FoundUAVs.Num() > 0)
    {
        LeaderUAV = Cast<ABase_UAV>(FoundUAVs[0]); // First UAV is leader
    }

    if (LeaderUAV)
    {
        int32 UAVIndex = FoundUAVs.Find(this) - 1; // Exclude leader
        if (UAVIndex >= 0 && UAVIndex < SquareOffsets.Num())
        {
            return LeaderUAV->GetActorLocation() + SquareOffsets[UAVIndex];
        }
    }

    return GetActorLocation(); // Default return if no leader is found
}

void ABase_UAV::LogSearchTime(int32 UAV_Count, int32 UAV_IDs, const FString& Event, float searchduration)
{
    FString FilePath = FPaths::ProjectDir() + "UAV_SearchTimes_1.csv";
    FString Header = "UAV_Count,UAV_ID,Event,Timestamp\n";

    FString CurrentTime = FDateTime::Now().ToString();
    FString Line = FString::Printf(TEXT("%d,%d,%s,%s\n"), UAV_Count, UAV_ID, *Event, *CurrentTime);

    if (!FPlatformFileManager::Get().GetPlatformFile().FileExists(*FilePath))
    {
        FFileHelper::SaveStringToFile(Header, *FilePath);
    }

    FFileHelper::SaveStringToFile(Line, *FilePath, FFileHelper::EEncodingOptions::AutoDetect, &IFileManager::Get(), FILEWRITE_Append);

    UE_LOG(LogTemp, Log, TEXT("Logged %s event for UAV %d at %s"), *Event, UAV_ID, *CurrentTime);
}



void ABase_UAV::SetupUDPReceiver()
{

    ISocketSubsystem* socketSubsystem = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM);
    UDPSocket = FUdpSocketBuilder(TEXT("UAVUDPReceiver"))
        .AsNonBlocking()
        .AsReusable()
        .BoundToPort(UDP_RECEIVE_PORT)  // Ensure it binds to 7000
        .WithReceiveBufferSize(512);

    if (UDPSocket && UDPSocket->GetConnectionState() != ESocketConnectionState::SCS_Connected)
    {
        UDPSocket->Close();
        ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->DestroySocket(UDPSocket);
    }

    if (UDPSocket)
    {
        UE_LOG(LogTemp, Log, TEXT("UAV %d is listening for person coordinates on UDP port %d"), UAV_ID, UDP_RECEIVE_PORT);
        GetWorld()->GetTimerManager().SetTimer(UDPReceiverTimer, this, &ABase_UAV::ReceivePersonLocation, 0.1f, true);
    }
    else
    {
        UE_LOG(LogTemp, Error, TEXT("Failed to create UDP receiver socket."));
    }


    /*ISocketSubsystem* socketSubsystem = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM);
    UDPSocket = FUdpSocketBuilder(TEXT("UAVUDPReceiver"))
        .AsNonBlocking()
        .AsReusable()
        .BoundToAddress(FIPv4Address::Any)
        .BoundToPort(UDP_RECEIVE_PORT)
        .WithReceiveBufferSize(512);

    if (UDPSocket)
    {
        UE_LOG(LogTemp, Log, TEXT("UAV %d is listening for person coordinates on UDP port %d"), UAV_ID, UDP_RECEIVE_PORT);
        GetWorld()->GetTimerManager().SetTimer(UDPReceiverTimer, this, &ABase_UAV::ReceivePersonLocation, 0.1f, true);

    }
    else
    {
        UE_LOG(LogTemp,Error,TEXT("Failed to create UDP receiver socket."))
    }*/

}

void ABase_UAV::ReceivePersonLocation()
{


    if (!UDPSocket) return;
    uint8 Buffer[640];
    int32 BytesRead = 0;
    TSharedPtr<FInternetAddr> Sender = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->CreateInternetAddr();

    // Listion on a specific port assigned to this uav
    int32 UAVPort = 5000 + UAV_ID;
    FIPv4Endpoint Endpoint(FIPv4Address::Any, UAVPort);

    if (!UDPSocket->Bind(*Endpoint.ToInternetAddr()))
    {
        UE_LOG(LogTemp, Error, TEXT("Failed to bind socket on port %d for UAV %d"), UAVPort, UAV_ID);
        return;
    }
    UE_LOG(LogTemp, Log, TEXT("UAV %d listening on port %d"), UAV_ID, UAVPort);

    if (UDPSocket->RecvFrom(Buffer, sizeof(Buffer), BytesRead, *Sender))
    {
        int32 ReceivedUAV_ID;
        int32 ScreenX, ScreenY;

        FMemory::Memcpy(&ReceivedUAV_ID, Buffer, sizeof(int32));
        FMemory::Memcpy(&ScreenX, Buffer + sizeof(int32), sizeof(int32));
        FMemory::Memcpy(&ScreenY, Buffer + 2 * sizeof(int32), sizeof(int32));

        //UE_LOG(LogTemp, Warning, TEXT("[UDP] Received %d bytes from %s"), BytesRead, *Sender->ToString());
        //UE_LOG(LogTemp, Warning, TEXT("UAV %d: Person detected at screen position (%d, %d)"), ReceivedUAV_ID, ScreenX, ScreenY);

        FVector WorldLocation;
    	FVector WorldDirection;

        UE_LOG(LogTemp,Warning, TEXT("Recieved_UAV_ID: %d, This UAV_ID: %d"), ReceivedUAV_ID,UAV_ID)

            // Process only if this packet is meant for this UAV
            if (ReceivedUAV_ID != UAV_ID)
            {
                UE_LOG(LogTemp, Warning, TEXT("UAV %d: Ignored packet meant for UAV %d."), UAV_ID, ReceivedUAV_ID);
                return;
            }

        UE_LOG(LogTemp, Warning, TEXT("UAV %d: Processing packet for UAV %d at (%d, %d)"), UAV_ID, ReceivedUAV_ID, ScreenX, ScreenY);

        // Get player controller that responsible for rendering the view
        /*APlayerController* PC = UGameplayStatics::GetPlayerController(GetWorld(), 0);
        if (!PC)
        {
            UE_LOG(LogTemp, Error, TEXT("No PlayerController found."));
            
        }*/

        if(UAVCamera)
        {
            const float FOV = UAVCamera->FieldOfView;
            FVector2D ScreenPos(ScreenX, ScreenY);
            const int32 RenderWidth = 640;
            const int32 RenderHeight = 640;
           

            // Normalize screen coordinates to [-1, 1]
            float NormalizedX = (ScreenX / float(RenderWidth)) * 2.f - 1.f;
            float NormalizedY = 1.f - (ScreenY / float(RenderHeight)) * 2.f; // Flip Y axis

            // Convert FOV to radians
            float TanFOV = FMath::Tan(FMath::DegreesToRadians(FOV) * 0.5f);

            // Aspect ratio (usually width / height)
            float AspectRatio = UAVCamera->AspectRatio;

            // Get direction
            FVector CameraDirection = FVector(1.f, NormalizedX * TanFOV * AspectRatio, NormalizedY * TanFOV);
            CameraDirection = CameraDirection.GetSafeNormal();

            // Transform direction from camera local space to world space
            WorldDirection = UAVCamera->GetComponentTransform().TransformVector(CameraDirection);



            // Ray trace
            FVector TraceStart = UAVCamera->GetComponentLocation();
            FVector TraceEnd = TraceStart + WorldDirection * 50000.f;


            DrawDebugLine(GetWorld(), TraceStart, TraceEnd, FColor::Blue, false, 5.0f, 0.f, 1.f);

            FHitResult HitResult;
            FCollisionQueryParams Params;
            Params.bTraceComplex = true;
            Params.AddIgnoredActor(this);

            if (GetWorld()->LineTraceSingleByChannel(HitResult, TraceStart, TraceEnd, ECC_Visibility, Params))
            {
                DrawDebugLine(GetWorld(), TraceStart, HitResult.Location, FColor::Green, false, 5.0f);
                DrawDebugSphere(GetWorld(), HitResult.Location, 20.0f, 12, FColor::Red, false, 5.0f);
                UE_LOG(LogTemp, Log, TEXT("Hit at location: %s"), *HitResult.Location.ToString());
                UE_LOG(LogTemp, Warning, TEXT("UAV %d: Person detected at screen position and Linetraced Location: (%d, %d),(%f,%f)"), ReceivedUAV_ID, ScreenX, ScreenY, HitResult.Location.X, HitResult.Location.Y);
            }
            else
            {
                UE_LOG(LogTemp, Warning, TEXT("Ray did not hit anything"));
            }


        }



        //// De project screen to world
        //if (PC->DeprojectScreenPositionToWorld(ScreenX,ScreenY,WorldLocation,WorldDirection))
        //{
        //    UE_LOG(LogTemp, Warning, TEXT("WorldLocation ScrrenX: %d | WorldDirection ScreenY: %d"), ScreenX, ScreenY);
        //    UE_LOG(LogTemp, Warning, TEXT("WorldLocation: %s | WorldDirection: %s"), *WorldLocation.ToString(), *WorldDirection.ToString());


	       // // Perform line trace from the camera into the world to find the 3d point
        //    //Fector TraceStart = WorldLocation + WorldDirection * 100.0f;
        //    FVector TraceStart = GetActorLocation();
        //    FVector TraceEnd = TraceStart + (WorldDirection * 100000.f);

        //    FHitResult HitResult;
        //    FCollisionQueryParams Params;
        //    Params.bTraceComplex = true;
        //    Params.AddIgnoredActor(this);
        //    DrawDebugLine(GetWorld(), TraceStart, TraceEnd, FColor::Blue, false, 5.0f, 0, 1.0f);


        //    if (GetWorld()->LineTraceSingleByChannel(HitResult,TraceStart,TraceEnd,ECC_Visibility,Params))
        //    {
        //        // Draw debug line and impact point
        //        DrawDebugLine(GetWorld(), TraceStart, HitResult.Location, FColor::Green, false, 5.0f, 0, 2.0f);
        //        DrawDebugSphere(GetWorld(), HitResult.Location, 20.0f, 12, FColor::Red, false, 5.0f);

        //        // Hit something, return the location
        //        UE_LOG(LogTemp, Log, TEXT("Hit world at location: %s"), *HitResult.Location.ToString());
        //    }
        //    else
        //    {
        //        UE_LOG(LogTemp, Warning, TEXT("Deproject succeeded, but line trace did not hit."));
        //    }

        //}

    }
    else
    {
        UE_LOG(LogTemp, Error, TEXT("[UDP] No data received!"));
    }




   /* if (!UDPSocket) return;
    uint8 Buffer[512];
    int32 BytesRead = 0;
    TSharedPtr<FInternetAddr> Sender = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->CreateInternetAddr();

    if (UDPSocket->RecvFrom(Buffer, sizeof(Buffer), BytesRead, *Sender))
    {
        UE_LOG(LogTemp, Warning, TEXT("Received UDP packet of %d bytes"), BytesRead);

        int32 ReceivedUAV_ID;
        int32 ScreenX, ScreenY;


        if (BytesRead >= 12)
        {
            FMemory::Memcpy(&ReceivedUAV_ID, Buffer, sizeof(int32));
            FMemory::Memcpy(&ScreenX, Buffer + sizeof(int32), sizeof(int32));
            FMemory::Memcpy(&ScreenY, Buffer + 2 * sizeof(int32), sizeof(int32));


            UE_LOG(LogTemp, Warning, TEXT("Received -> UAV_ID: %d, X: %d, Y: %d"),
                ReceivedUAV_ID, ScreenX, ScreenY);

            if (ReceivedUAV_ID == UAV_ID)
            {
                PersonScreenPosition = FVector2D(ScreenX, ScreenY);
                UE_LOG(LogTemp, Warning, TEXT("UAV %d: Person detected at screen position (%d, %d)"), UAV_ID, ScreenX, ScreenY);
            }
        }
        else
        {
            UE_LOG(LogTemp, Error, TEXT("Received UDP data is too short! BytesRead: %d"), BytesRead);
        }

        
    }
    else
    {
        UE_LOG(LogTemp, Warning, TEXT("No UDP data received."));
    }*/

}

int32 ABase_UAV::GetUAVNum()
{
    if (!GetWorld()) return 0;

    TArray<AActor*> FoundedUav;
    UGameplayStatics::GetAllActorsOfClass(GetWorld(), ABase_UAV::StaticClass(), FoundedUav);

    int32 UAVCount = FoundedUav.Num();

    UE_LOG(LogTemp, Warning, TEXT("Number of UAVs Spawned: %d"), UAVCount);

    return UAVCount;
}

void ABase_UAV::NotifyPythonServer()
{
    FString ServerIPAddress = TEXT("127.0.0.1");
    int32 PythonServerPort = 6000; // Port for sending UAV count

    ISocketSubsystem* socketSubsystem = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM);
    TSharedPtr<FInternetAddr> PythonServerAddr = socketSubsystem->CreateInternetAddr();

    FIPv4Address IP;
    if (FIPv4Address::Parse(ServerIPAddress, IP))
    {
        PythonServerAddr->SetIp(IP.Value);
        PythonServerAddr->SetPort(PythonServerPort);
    }
    else
    {
        UE_LOG(LogTemp, Error, TEXT("Invalid Python server IP!"));
        return;
    }

    FSocket* UDPClientSocket = FUdpSocketBuilder(TEXT("PythonNotificaton"))
        .AsReusable();

    if (!UDPClientSocket)
    {
        UE_LOG(LogTemp, Error, TEXT("Failed to create UDP client socket!"));
        return;
    }

    // Send Number of UAVs
    int32 numUAV = GetUAV_ID();  
    int32 BytesSent = 0;
    bool bSent = UDPClientSocket->SendTo(reinterpret_cast<uint8*>(&numUAV), sizeof(int32), BytesSent, *PythonServerAddr);

    if (bSent)
    {
        UE_LOG(LogTemp, Log, TEXT("Sent UAV count: %d to Python Server"), numUAV);
    }
    else
    {
        UE_LOG(LogTemp, Error, TEXT("Failed to send UAV count to Python Server"));
    }

    // Cleanup
    UDPClientSocket->Close();
    socketSubsystem->DestroySocket(UDPClientSocket);

}

void ABase_UAV::InitializeGrid()
{

    GlobalSearchGrid.Empty();
    //if (GlobalSearchGrid.Num() > 0) return;

    FVector GridStart = GetTargetPosition();
    UE_LOG(LogTemp, Warning, TEXT("TaG: %s"), *TargetPosition.ToString());
    UE_LOG(LogTemp, Warning, TEXT("TaG: %s"), *GridStart.ToString());


     for (int32 X = 0; X < GridSizeX; X++)
     {
        for (int32 Y = 0; Y < GridSizeY; Y++)
        {
            // Convert cell indices to world location
            FVector CellLocation = GridStart + FVector(X * CellSize, Y * CellSize,0.f);
            
            GlobalSearchGrid.Add(FSearch_Cell(X, Y, CellLocation));
        }
     }
    
}

void ABase_UAV::AssignSearchCell()
{
    bisSearchPhase = false;
    UE_LOG(LogTemp, Error, TEXT("Inside AssignSearch Cell"));
    FVector ProposedCell = FindUnsearchedCell();

    if (ProposedCell.IsZero())
    {
        UE_LOG(LogTemp, Warning, TEXT(" UAV %d: No Unsearched cell left! "), UAV_ID);
        return;
    }
    // Brodcast Request to other UAV
    //RequestSearchCell(); 

    FPlatformProcess::Sleep(0.1f);
    
    if (!IsCellOccupide(ProposedCell))
    {
        TargetPosition = ProposedCell;
        bIsSearching = true;
        //NumOfSearchedCell++;
        // Lock Grid to prevent race condition
        FScopeLock Lock(&GridLock);
        for (FSearch_Cell& Cell : GlobalSearchGrid)
        {
            if (FMath::IsNearlyEqual(Cell.WorldLocation.X, TargetPosition.X, KINDA_SMALL_NUMBER) &&
                FMath::IsNearlyEqual(Cell.WorldLocation.Y, TargetPosition.Y, KINDA_SMALL_NUMBER))
            {
                Cell.bIsSearched = true;
                Cell.searchedGrid++;
                NumOfSearchedCell++;
                
                //this->SetActorLocation(TargetPosition);
                break;
            }
        }

    }
    UE_LOG(LogTemp, Warning, TEXT("UAV %d: Assigned Cell at %s"), UAV_ID, *TargetPosition.ToString());
    bisSearchPhase = false;
   /* bool bIsBlocked = RepulsionForce.Size() > 0.1f;
    Move(bIsBlocked,TargetPosition);*/
}

void ABase_UAV::OnSearchComplete()
{
    UE_LOG(LogTemp, Error, TEXT("Inside OnSearchComplete"));
    bIsSearching = false; 
    bisSearchPhase = true;
    //uav_State = EUAV_State::EUS_SEARCH;
    GetWorld()->GetTimerManager().SetTimer(UavStateSwitcher, this, &ABase_UAV::AssignSearchCell, 5.0f, false);


    // Request another cell 
    //AssignSearchCell();
}

void ABase_UAV::RequestSearchCell()
{
    NearbyUAVs = GetNearbyUAVs_bySphere();
    for (ABase_UAV* UAV : NearbyUAVs)
    {
        if (UAV)
        {
            UAV->ReciveSearchRequest(FSearchMessage(UAV_ID, TargetPosition, false));
        }
    }
}

void ABase_UAV::ReciveSearchRequest(FSearchMessage Message)
{
    UE_LOG(LogTemp, Error, TEXT("Inside ReciveSearchRequest"));
    if (FVector::Dist(Message.RequestedCell, TargetPosition) < CellSize * 0.5f)
    {
        if (Message.UAV_ID < UAV_ID)
        {
            Message.bAccepted = true; // This UAV loses the claim
        }
    }


}

bool ABase_UAV::IsCellOccupide(FVector CellLocation)
{
    UE_LOG(LogTemp, Error, TEXT("Inside IsCellOccupide"));
    NearbyUAVs = GetNearbyUAVs_bySphere();
    for (ABase_UAV* UAV : NearbyUAVs)
    {
        if (UAV && FVector::Dist(UAV->TargetPosition, CellLocation) < CellSize * 0.75f)
        {
            UE_LOG(LogTemp, Warning, TEXT("Cell at %s is occupied by UAV %d"), *CellLocation.ToString(), UAV->UAV_ID);
            return true;
        }
    }

    return false;
}

FVector ABase_UAV::FindUnsearchedCell()
{
    UE_LOG(LogTemp, Error, TEXT("Inside FIndUnsearchedCell"));

    for (FSearch_Cell& Cell : GlobalSearchGrid)
    {
        UE_LOG(LogTemp, Log, TEXT("Cell [%d, %d] - Searched: %s"),
            Cell.X, Cell.Y, Cell.bIsSearched ? TEXT("Yes") : TEXT("No"));
    }

    FScopeLock Lock(&GridLock);

    TArray<FSearch_Cell*> AvailableCells;

    //Collect unsearched cells
    for (FSearch_Cell& Cell : GlobalSearchGrid)
    {
        if (!Cell.bIsSearched)
        {
            AvailableCells.Add(&Cell);
        }
    }
    if (AvailableCells.Num() == 0)
    {
        UE_LOG(LogTemp, Warning, TEXT("No available Cells are found!"));
        return FVector::ZeroVector;
    }

    // Sort Avilable Cells Based on a structerd order
    AvailableCells.Sort([](const FSearch_Cell& A, const FSearch_Cell& B)
        {
            if (A.X == B.X)
            {
                return A.Y < B.Y; // Sort by Y if X is the same
            }
            return A.X < B.X; // Sort by X first
        }
    );



    // Use UAV_ID to distribute different cells instead of always picking the first
    //int32 SelectedIndex = FMath::RandRange(0, AvailableCells.Num() - 1);// This makes UAVs pick different cells

    // Assign UAV based on UAV_ID, ensureing sequencal order
    // Distribute UAV in a sequencel order
    int32 SelectedIndex = UAV_ID % AvailableCells.Num();


    FSearch_Cell* SelectedCell = AvailableCells[SelectedIndex]; // Always select first element
    SelectedCell->bIsSearched = true;

    //FVector GridStart = TargetPosition - FVector((GridSizeX * CellSize) * 0.5f, (GridSizeY * CellSize) * 0.5f, 0.f);
    /*FVector GridStart = FVector(-GridSizeX * CellSize * 0.5f, -GridSizeY * CellSize * 0.5f, 1000.f);
    FVector AssignedCellLocation = GridStart + FVector(SelectedCell->X * CellSize, SelectedCell->Y * CellSize, 0.0f);*/

    UE_LOG(LogTemp, Log, TEXT("UAV %d assigned to cell [%d, %d] at location %s"), UAV_ID, SelectedCell->X, SelectedCell->Y, *SelectedCell->WorldLocation.ToString());
    //return GridStart + FVector(SelectedCell->X * CellSize, SelectedCell->Y * CellSize - (GridSizeY - 1), 0.0f);
    return SelectedCell->WorldLocation;

   /* for (FSearch_Cell& Cell : GlobalSearchGrid)
    {
        if (!Cell.bIsSearched)
        {
        }
    }*/



    return FVector::ZeroVector;
}

TArray<ABase_UAV*> ABase_UAV::GetNearbyUAVs_bySphere()
{
    UE_LOG(LogTemp, Error, TEXT("Inside GetNearbyUAVs_bySphere"));
    TArray<FOverlapResult> OverlappingResults;
    TArray<ABase_UAV*> Nearby_UAVs;


    // Define teh collision shpere parameter
    FVector SearchOrgin = GetActorLocation();
    float SearchReadius = PerceptionRadius;

    // Define collision overlap quary (sphere)

    FCollisionShape CollisionShpere = FCollisionShape::MakeSphere(SearchReadius);
    FCollisionQueryParams QueryParams;
    QueryParams.AddIgnoredActor(this);


    // Perfomr shpere overlap query
    bool bHit = GetWorld()->OverlapMultiByChannel(
        OverlappingResults, // Output
        SearchOrgin,
        FQuat::Identity,
        ECC_Pawn,
        CollisionShpere,
        QueryParams
        );

    if (bHit)
    {
        for (const FOverlapResult& Result : OverlappingResults)
        {
            ABase_UAV* UAV = Cast<ABase_UAV>(Result.GetActor());
            if (UAV && UAV != this)
            {
                Nearby_UAVs.Add(UAV);
            }
        }
    }

    return Nearby_UAVs;
}

void ABase_UAV::UpdateSearch()
{
    UE_LOG(LogTemp, Warning, TEXT("bIsSearching: %s"), bIsSearching ? TEXT("true") : TEXT("false"));
    UE_LOG(LogTemp, Error, TEXT("Inside UpdateSearch"));
    if (bIsSearching && FVector::Dist(GetActorLocation(), TargetPosition) < 50.f)
    {
        UE_LOG(LogTemp, Log, TEXT("UAV %d completed searching at %s"), UAV_ID, *TargetLocation.ToString());
        OnSearchComplete();
    }

}

//void ABase_UAV::DrawSearchGrid()
//{
//    UWorld* World = GetWorld();
//    if (!World) return;
//
//    // Calculate the starting point based on the search area center
//    FVector GridStart = TargetPosition - FVector((GridSizeX * CellSize) * 0.5f, (GridSizeY * CellSize) * 0.5f, 0.0f);
//
//    for (int32 X = 0; X <= GridSizeX; ++X)
//    {
//        FVector Start = GridStart + FVector(X * CellSize, 0, 0);
//        FVector End = Start + FVector(0, GridSizeY * CellSize, 0);
//
//        DrawDebugLine(World, Start, End, FColor::Green, false, 5.0f, 0, 5.0f);
//    }
//
//    for (int32 Y = 0; Y <= GridSizeY; ++Y)
//    {
//        FVector Start = GridStart + FVector(0, Y * CellSize, 0);
//        FVector End = Start + FVector(GridSizeX * CellSize, 0, 0);
//
//        DrawDebugLine(World, Start, End, FColor::Green, false, 5.0f, 0, 5.0f);
//    }
//}
void ABase_UAV::DrawSearchGrid()
{
    UWorld* World = GetWorld();
    if (!World) return;

    //FVector GridStart = FVector(-GridSizeX * CellSize * 0.5f, -GridSizeY * CellSize * 0.5f, 1000.0f);

    for (const FSearch_Cell& Cell : GlobalSearchGrid)
    {
        /*FVector BottomLeft = GridStart + FVector(Cell.X * CellSize, Cell.Y * CellSize, 0);
        FVector BottomRight = BottomLeft + FVector(CellSize, 0, 0);
        FVector TopLeft = BottomLeft + FVector(0, CellSize, 0);
        FVector TopRight = BottomLeft + FVector(CellSize, CellSize, 0);*/
        FVector BottomLeft = Cell.WorldLocation;
        FVector BottomRight = BottomLeft + FVector(CellSize, 0, 0);
        FVector TopLeft = BottomLeft + FVector(0, CellSize, 0);
        FVector TopRight = BottomLeft + FVector(CellSize, CellSize, 0);

        FColor LineColor = Cell.bIsSearched ? FColor::Red : FColor::Green;

        DrawDebugLine(World, BottomLeft, BottomRight, LineColor, false, 5.0f, 0, 3.0f);
        DrawDebugLine(World, BottomRight, TopRight, LineColor, false, 5.0f, 0, 3.0f);
        DrawDebugLine(World, TopRight, TopLeft, LineColor, false, 5.0f, 0, 3.0f);
        DrawDebugLine(World, TopLeft, BottomLeft, LineColor, false, 5.0f, 0, 3.0f);
    }
}

void ABase_UAV::SwitchToTravel()
{
    UE_LOG(LogTemp, Warning, TEXT("UAV: %d uav_State switching to Travel"),UAV_ID);
    uav_State = EUAV_State::EUS_TRAVEL;
}

void ABase_UAV::SetupTriggerSocket()
{

    /*if (TriggerSocket)
    {
        TriggerSocket->Close();
        ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->DestroySocket(TriggerSocket);
        TriggerSocket = nullptr;
        UE_LOG(LogTemp, Warning, TEXT("TriggerSocket has been closed and destroyed before binding."));
    }

    FString ServerIPAddress = TEXT("127.0.0.1");
    int32 TriggerPort = 7000;

    ISocketSubsystem* socketSubsystem = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM);
    TriggerSocket = socketSubsystem->CreateSocket(NAME_DGram, TEXT("TriggerSocket"), true);

    TSharedPtr<FInternetAddr> PythonAddr = socketSubsystem->CreateInternetAddr();
    PythonAddr->SetAnyAddress();
    PythonAddr->SetPort(TriggerPort);

    bool bBind = TriggerSocket->Bind(*PythonAddr);

    if (bBind)
    {
        UE_LOG(LogTemp, Warning, TEXT("TriggerSocket successfully bound to port %d"), TriggerPort);
    }
    else
    {
        UE_LOG(LogTemp, Error, TEXT("Failed to bind TriggerSocket."));
        return;
    }

    TriggerSocket->SetNonBlocking(true);
    TriggerSocket->SetReuseAddr(true);*/

    FString TriggerIPAddress = TEXT("127.0.0.1");
    int32 TriggerPort = 7000 + UAV_ID;

    ISocketSubsystem* SocketSubsystem = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM);
    TriggerSocket = FUdpSocketBuilder(TEXT("TriggerSoket"))
        .AsNonBlocking()
        .AsReusable()
        .BoundToAddress(FIPv4Address::Any)
        .BoundToPort(TriggerPort)
        .WithReceiveBufferSize(2 * 1024 * 1024);
    if (TriggerSocket)
    {
        UE_LOG(LogTemp, Warning, TEXT("TriggerSocket successfully created and bound to port %d."), TriggerPort);
    }
    else
    {
        UE_LOG(LogTemp, Error, TEXT("Failed to create TriggerSocket."));
    }

}

void ABase_UAV::CheckForTriggerMessage()
{
    if (!TriggerSocket)
    {
        // Create the triggersocket of doesn't exist
        TriggerSocket = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->CreateSocket(NAME_Stream,TEXT("TriggerSocket"),false);
        if (!TriggerSocket)
        {
            UE_LOG(LogTemp, Error, TEXT("CFTM: Failed to create TriggerSocket."));
            return;
        }

        // Set Socket Option
        TriggerSocket->SetNonBlocking(false);
        TriggerSocket->SetReuseAddr(true);
        //TriggerSocket->SetRecvErr(true);

        // Bind the socket to listion for incoming messages on port 7000
        FIPv4Address IP; // Listion all incoming IP address
        FIPv4Address::Parse(TEXT("127.0.0.1"), IP);
        TSharedPtr<FInternetAddr> Endpoint = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->CreateInternetAddr();
        Endpoint->SetIp(IP.Value);
        Endpoint->SetPort(8000);
        bool bBound = TriggerSocket->Bind(*Endpoint);

        if (!bBound)
        {
            UE_LOG(LogTemp, Error, TEXT("CFTM: Failed to bind TriggerSocket to port 8000"));
            ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->DestroySocket(TriggerSocket);
            TriggerSocket = nullptr;
            return;

        }
        else
        {
            UE_LOG(LogTemp, Warning, TEXT("CFTM: TriggerSocket successfully bound to port 7000 and ready to receive messages."));
        }
        // Start listening for incoming connections
        if (!TriggerSocket->Listen(1))
        {
            UE_LOG(LogTemp, Error, TEXT("CFTM: Failed to start listening on TriggerSocket."));
            return;
        }

    }


    // Accept incoming connection
    bool Pending;
    if (TriggerSocket->HasPendingConnection(Pending) && Pending)
    {
        TSharedPtr<FInternetAddr> Sender = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->CreateInternetAddr();
        FSocket* ClientSocket = TriggerSocket->Accept(*Sender, TEXT("AcceptedConnection"));

        if (ClientSocket)
        {
            UE_LOG(LogTemp, Warning, TEXT("CFTM: Accepted connection from client."));

            // Read data from the client
            uint8 Buffer[128];
            int32 BytesRead = 0;

            if (ClientSocket->Recv(Buffer, sizeof(Buffer), BytesRead, ESocketReceiveFlags::None))
            {
                if (BytesRead >= sizeof(int32))
                {
                    int32 ReceivedUAV_ID = 0;
                    FMemory::Memcpy(&ReceivedUAV_ID, Buffer, sizeof(int32));
                    UE_LOG(LogTemp, Warning, TEXT("CFTM: Trigger message received for UAV ID: %d"), ReceivedUAV_ID);

                    // Trigger connection for this UAV
                    ConnectToServer(ReceivedUAV_ID, ServerPort);
                    return;
                }
                else
                {
                    UE_LOG(LogTemp, Error, TEXT("CFTM: Failed to read valid data from client."));
                }
            }
            else
            {
                UE_LOG(LogTemp, Error, TEXT("CFTM: Failed to receive data from client."));
            }

            ClientSocket->Close();
            ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->DestroySocket(ClientSocket);
        }
    }

    //uint8 Buffer[128];
    //int32 BytesRead = 0;
    //TSharedPtr<FInternetAddr> Sender = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->CreateInternetAddr();

    //int32 BufferSize = 65507;  // Max size for a UDP packet
    //TriggerSocket->SetReceiveBufferSize(BufferSize, BufferSize);

    //uint32 bHasPendingData = 0;
    //if (TriggerSocket->HasPendingData(bHasPendingData) && bHasPendingData > 0)
    //{
    //    bool bSuccess = TriggerSocket->RecvFrom(Buffer, sizeof(Buffer), BytesRead, *Sender);
    //
    //    if (!bSuccess || BytesRead <= 0)
    //    {
    //        UE_LOG(LogTemp, Error, TEXT("CFTM: Failed to receive data or received zero bytes."));
    //        return;
    //    }
    //    
    //    if (BytesRead >= sizeof(int32))
    //    {
    //        // Process the recived message
    //        int32 ReceivedUAV_ID = 0;
    //        FMemory::Memcpy(&ReceivedUAV_ID, Buffer, sizeof(int32));

    //        UE_LOG(LogTemp, Warning, TEXT("CFTM: Trigger message received for UAV ID: %d"), ReceivedUAV_ID);

    //        // Trigger connection for this UAV
    //        ConnectToServer(ReceivedUAV_ID, ServerPort);

    //    }
    //   

    //}
    //else
    //{
    //    UE_LOG(LogTemp, Warning, TEXT("CFTM: No pending data available to read from TriggerSocket."));
    //}
    //


    //if (!TriggerSocket) return;

    //uint8 Buffer[128];
    //int32 BytesRead = 0;
    //TSharedPtr<FInternetAddr> Sender = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->CreateInternetAddr();

    //if (TriggerSocket->RecvFrom(Buffer, sizeof(Buffer), BytesRead, *Sender))
    //{
    //    int32 ReceivedUAV_ID;
    //    FMemory::Memcpy(&ReceivedUAV_ID, Buffer, sizeof(int32));

    //    UE_LOG(LogTemp, Warning, TEXT("Trigger message received for UAV ID: %d"), ReceivedUAV_ID);

    //    // Now trigger connection for this UAV
    //    ConnectToServer(ReceivedUAV_ID, ServerPort);
    //}




    //if (!TriggerSocket) return;
    //UE_LOG(LogTemp, Warning, TEXT("In TriggerSocket."));
    //uint8 Buffer[512];
    //int32 BytesRead = 0;
    //TSharedPtr<FInternetAddr> Sender = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->CreateInternetAddr();

    //while (TriggerSocket->RecvFrom(Buffer, sizeof(Buffer), BytesRead, *Sender))
    //{
    //    int32 TriggerUAV_ID;
    //    FMemory::Memcpy(&TriggerUAV_ID, Buffer, sizeof(int32));

    //    UE_LOG(LogTemp, Warning, TEXT("Received trigger message for UAV %d"), TriggerUAV_ID);

    //    if (TriggerUAV_ID == GetUAV_ID())  // Check if the message is meant for this UAV
    //    {
    //        ConnectToServer(TriggerUAV_ID,5000 + TriggerUAV_ID);  // Trigger connection
    //    }
    //}



}

void ABase_UAV::SetUAV_ID_Text()
{
    
    RoleText->SetText(FText::AsNumber(UAV_ID));

}




// Called every frame
void ABase_UAV::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
    DeltaX = DeltaTime;

    
    // Call Consensus Formation Control
    //ApplyConsensusFormationControl();

    // Check for Distance of th taraget
    isReached(bisSearchPhase);
    // Get current state
    CurrentPosition = GetActorLocation();
    CurrentRotation = GetActorRotation();

    CheckForTriggerMessage();

    // Path visualization
    //DrawPathLine();
    //UpdateSplinePath();

    

    switch (uav_State)
    {
    case EUS_IDEL:
        // floating on the Position
        resetPIDController();
        break;
    case EUS_TRAVEL:
        // Go to the GEO Location
        CalculateTargetLocation();
        break;
    case EUS_SEARCH:
        // Look for the Target
        searchPhase();
        

        break;
    case EUS_EXTRACT:
        //  Extract
        extractPhase();

        break;
    default:
        break;
    }

    //extractPhase();

    if (bCanRopDrop)
    {
        
        DropeRope();
        //bCanRopDrop = false;
    }


   /* if (bisSearchPhase)
    {
        ApplyConsensusFormation();
        DrawFormationDebug();
    }*/

    //ApplyConsensusFormation();
    //DrawFormationDebug();
  

    //float DistanceToTarget = FVector::Dist(CurrentPosition, TargetPosition);
    //if (DistanceToTarget <= TargetReachedTolerance)
    //{
    //    // Stop Movement
    //    CurrentVelocity = FVector::ZeroVector;

    //    // Reset all controllers when stopping
    //    RollController->Reset(TargetRotation.Roll - CurrentRotation.Roll);
    //    PitchController->Reset(TargetRotation.Pitch - CurrentRotation.Pitch);
    //    YawController->Reset(TargetRotation.Yaw - CurrentRotation.Yaw);
    //    AltitudeController->Reset(TargetPosition.Z - CurrentPosition.Z);
    //    ForwardController->Reset(TargetPosition.X - CurrentPosition.X);
    //    RightController->Reset(TargetPosition.Y - CurrentPosition.Y);

    //    return; // Exit early to prevent furthe updates
    //}

    

    //// Compute PID outputs
    //float RollOutput = RollController->Compute(TargetRotation.Roll, CurrentRotation.Roll, DeltaTime);
    //float PitchOutput = PitchController->Compute(TargetRotation.Pitch, CurrentRotation.Pitch, DeltaTime);
    //float YawOutput = YawController->Compute(TargetRotation.Yaw, CurrentRotation.Yaw, DeltaTime);

    //float AltitudeOutput = AltitudeController->Compute(TargetPosition.Z, CurrentPosition.Z, DeltaTime);
    //float ForwardOutput = ForwardController->Compute(TargetPosition.X, CurrentPosition.X, DeltaTime);
    //float RightOutput = RightController->Compute(TargetPosition.Y, CurrentPosition.Y, DeltaTime);

    //// Combine forces
    //FVector MovementForce = FVector(ForwardOutput, RightOutput, AltitudeOutput);

    //// cap speed
    //float maximumSpeed = 800.f;
    //float forceMagnitude = MovementForce.Size();

    //if (forceMagnitude > maximumSpeed)
    //{
    //    MovementForce = MovementForce.GetSafeNormal() * maximumSpeed;
    //}

    //FVector RepulsionForce = AvoidCollisions();
    //bool bIsBlocked = RepulsionForce.Size() > 0.1f;
    //// Reduce goal force if blocked
    //if (bIsBlocked)
    //{
    //    MovementForce *= 0.7f; // Reduce to 20% of its strength
    //}
    //// Update the position
    //FVector NewPosition = (CurrentPosition + RepulsionForce) + MovementForce * DeltaTime;
    //SetActorLocation(NewPosition);


    //// Clamp the rotation to avoid jittery
    //RollOutput = FMath::Clamp(RollOutput, -MaxRollRate, MaxRollRate);
    //PitchOutput = FMath::Clamp(PitchOutput, -MaxPitchRate, MaxPitchRate);
    //YawOutput = FMath::Clamp(YawOutput, -MaxYawRate, MaxYawRate);


    //// Apply rotation
    //FRotator NewRotation = FRotator(
    //    CurrentRotation.Pitch + PitchOutput ,
    //    CurrentRotation.Yaw + YawOutput ,
    //    CurrentRotation.Roll + RollOutput);

    //// Smooth the rotation
    //FRotator SmoothRotation = FMath::RInterpTo(CurrentRotation, TargetRotation, DeltaTime,60.f);

    //SetActorRotation(NewRotation);

    //CurrentRotation = GetActorRotation();

    //DrawDebugLine(GetWorld(), GetActorLocation(), GetActorLocation() + RepulsionForce, FColor::Red, false, -1, 0, 1.0f);
    //DrawDebugLine(GetWorld(), GetActorLocation(), GetActorLocation() + MovementForce, FColor::Blue, false, -1, 0, 1.0f);

    //ProcessCameraFeed();


}

void ABase_UAV::RegisterOtherUAV(AActor* UAV)
{
    if (UAV && UAV != this) 
    {
        OtherUAVs.Add(UAV);
    }
}

void ABase_UAV::Boids(float DeltaTime)
{

    // Compute forces
    FVector Separation = ComputeSeparationForce() * SeparationWeight;
    FVector Alignment = ComputeAlignmentForce() * AlignmentWeight;
    FVector Cohesion = ComputeCohesionForce() * CohesionWeight;
    FVector ObstacleAvoidance = ComputeObstacleAvoidanceForce() * ObstacleAvoidanceWeight;


    // Combine forces to determine target velocity
    TargetVelocity = Separation  + Alignment + Cohesion + ObstacleAvoidance;

    //UE_LOG(LogTemp, Warning, TEXT("UAV ID: %s, TargetVelocity: %f"), *this->GetFName().ToString(), TargetVelocity.Size());


    // Clamp the velocity to a maximum speed (e.g., 500 units/second)
    TargetVelocity = TargetVelocity.GetClampedToMaxSize(1000.0f);

    //Check if the UAV is close enough to the target
    
  
   // Update position and velocity
   CurrentVelocity = FMath::VInterpTo(CurrentVelocity, TargetVelocity, DeltaTime, 2.5f);
    


   
    FVector NewPosition = GetActorLocation() + CurrentVelocity * DeltaTime;
    SetActorLocation(NewPosition);

    // Debug visualization
   // DrawDebugSphere(GetWorld(), GetActorLocation(), PerceptionRadius, 12, FColor::Green, false, -1, 0, 1.0f);
    DrawDebugLine(GetWorld(), GetActorLocation(), GetActorLocation() + CurrentVelocity, FColor::Blue, false, -1, 0, 1.0f);
}

void ABase_UAV::UpdateTargetPointIfNeeded()
{
    // Check if the UAV is close enough to the target
    float DistanceToTarget = FVector::Dist(GetActorLocation(), TargetPosition);
    if (DistanceToTarget <= TargetReachedTolerance) {
        // Generate a new random target point within the specified range
        FVector RandomOffset = FVector(
            FMath::RandRange(0.f, RandomTargetRange),
            FMath::RandRange(0.f, RandomTargetRange),
            FMath::RandRange(0.f / 2.0f, RandomTargetRange / 2.0f) // Randomize height (optional)
        );

        TargetPosition = GetActorLocation() + RandomOffset;

        // Ensure the new target is within world bounds if needed (optional)
        UE_LOG(LogTemp, Log, TEXT("UAV %s reached target. New Target: %s"), *GetName(), *TargetPosition.ToString());
    }
}

// Computer Vision
void ABase_UAV::ProcessCameraFeed()
{
    // Ensure the Render Target exists
    if (!RenderTarget)
    {
        UE_LOG(LogTemp, Warning, TEXT("RenderTarget does not exist!"));
        return;
    }

    // Access the Render Target Resource
    FTextureRenderTargetResource* RenderTargetResource = RenderTarget->GameThread_GetRenderTargetResource();
    
    if (!RenderTargetResource)
    {
        UE_LOG(LogTemp, Warning, TEXT("RenderTargetResource does not exist!"));
        return;
    }

    // Throttle Processing (Process every 5th frame)
    static int FrameCount = 0;
    FrameCount++;
    if (FrameCount % 10 != 0) // Skip processing
    {
        return;
    }

    // Read Pixels into Bitmap
    TArray<FColor> Bitmap;
    RenderTargetResource->ReadPixels(Bitmap);

    int Width = RenderTarget->SizeX;
    int Height = RenderTarget->SizeY;

    if (Bitmap.Num() != Width * Height)
    {
        UE_LOG(LogTemp, Warning, TEXT("Bitmap size mismatch with RenderTarget size!"));
        return;
    }


    UE_LOG(LogTemp, Warning, TEXT("RenderTarget exists"));
    UE_LOG(LogTemp, Warning, TEXT("RenderTargetResource exists"));
    UE_LOG(LogTemp, Warning, TEXT("Width: %d, Height: %d"), Width, Height);
    UE_LOG(LogTemp, Warning, TEXT("Bitmap size: %d"), Bitmap.Num());
    UE_LOG(LogTemp, Warning, TEXT("PixelData buffer allocated"));


    // preallocate static buffer for reuse
    static std::vector<uint8> PixelData(Bitmap.Num() * 4); 
    if (PixelData.size() != Bitmap.Num() * 4)
    {
        PixelData.resize(Bitmap.Num() * 4); // resize when its necessary
    }


    // convert bitmap to BGRA format
    for (int i = 0; i < Bitmap.Num(); ++i)
    {
        PixelData[i * 4 + 0] = Bitmap[i].B; // Blue
        PixelData[i * 4 + 1] = Bitmap[i].G; // Green
        PixelData[i * 4 + 2] = Bitmap[i].R; // Red
        PixelData[i * 4 + 3] = Bitmap[i].A; // Alpha
    }


    //// Convert to OpenCV Mat
    //cv::Mat Frame(Height, Width, CV_8UC4, PixelData.data());

    
    //// Access the first pixel to confirm OpenCV integration
    //cv::Vec4b FirstPixel = Frame.at<cv::Vec4b>(0, 0);
    //UE_LOG(LogTemp, Warning, TEXT("First Pixel - Blue: %d, Green: %d, Red: %d, Alpha: %d"),
    //    FirstPixel[0], FirstPixel[1], FirstPixel[2], FirstPixel[3]);

}