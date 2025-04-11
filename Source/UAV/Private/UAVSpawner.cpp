// Fill out your copyright notice in the Description page of Project Settings.


#include "UAVSpawner.h"
#include "Kismet/GameplayStatics.h"
#include "Engine/World.h"
#include "DrawDebugHelpers.h"

// Sets default values
AUAVSpawner::AUAVSpawner()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;
    NumUAVs = 2; // Default number of UAVs
    SpawnAreaMin = FVector(-500, -500, 300); // Default spawn area
    SpawnAreaMax = FVector(500, 500, 500);
    GeoLocation = FVector(100, 100, 100); // Default target location
    Spacing = 200.0f; // Default spacing between drones

    GridSizeX = 4;
    GridSizeY = 4;
    CellSize = 200.f;


}

FVector AUAVSpawner::CalculateOffset(int32 X, int32 Y, int32 Z, int32 GridSize) const
{
    // Center the grid around the target location
    float HalfGridSize = (GridSize - 1) * Spacing * 0.5f;

    float OffsetX = X * Spacing - HalfGridSize;
    float OffsetY = Y * Spacing - HalfGridSize;
    float OffsetZ = Z * Spacing - HalfGridSize;

    return FVector(OffsetX, OffsetY, OffsetZ);
}

void AUAVSpawner::SpawnUAVs()
{
    if (!UAVClass) {
        UE_LOG(LogTemp, Warning, TEXT("UAVClass is not set in UAVSpawner."));
        return;
    }

    for (int32 i = 0; i < NumUAVs; i++) {
        FVector SpawnLocation = FMath::RandPointInBox(FBox(SpawnAreaMin, SpawnAreaMax));
        FRotator SpawnRotation = FRotator::ZeroRotator;

        ABase_UAV* UAV = GetWorld()->SpawnActor<ABase_UAV>(UAVClass, SpawnLocation, SpawnRotation);

        if (UAV)
        {
            UAV->SetUAV_ID(i);
            UAVs.Add(UAV);
            //UAV->UAV_ID = i + 0;  // Assign a unique UAV ID (1,2,3...)
            UAV->ServerPort = 5000 + UAV->UAV_ID; // Assign a unique port
            UAV->SetTargetLocation(GeoLocation);
            UAV->InitializeGrid();
            UAV->SetUAV_ID_Text();
            //UAV->ConnectToServer(i,UAV->ServerPort);
            UAV->NotifyPythonServer();
            UAV->SetNumOfUAVs(NumUAVs);
           
            

            UE_LOG(LogTemp, Log, TEXT("Spawned UAV: %s, Spawned UAV: %d,Assigned Port: %d"), *UAV->GetName(), UAV->GetUAV_ID(), UAV->ServerPort);
        }
        else {
            UE_LOG(LogTemp, Warning, TEXT("Failed to spawn UAV."));
        }
    }
}

void AUAVSpawner::RegisterAllUAVs()
{
    TArray<AActor*> FoundUAVs;
    UGameplayStatics::GetAllActorsOfClass(GetWorld(), ABase_UAV::StaticClass(), FoundUAVs);

    for (AActor* UAV : FoundUAVs) {
        ABase_UAV* UAVController = Cast<ABase_UAV>(UAV);

        if (UAVController) {
            for (AActor* OtherUAV : FoundUAVs) {
                if (OtherUAV != UAV) {
                    UAVController->RegisterOtherUAV(OtherUAV);
                }
            }
        }
    }

    UE_LOG(LogTemp, Log, TEXT("Registered all UAVs with each other."));
}

void AUAVSpawner::InitializeSearchGrid()
{
    SearchGrid.Empty(); // Clear any existing grid

    UE_LOG(LogTemp, Log, TEXT("Initializing Search Grid at GeoLocation: %s"), *GeoLocation.ToString());


    for (int32 x = 0; x < GridSizeX; x++)
    {
        for (int32 y = 0; y < GridSizeY; y++)
        {
            SearchGrid.Add(FSearchCell(x, y)); // Mark all as unsearched
            FVector CellLocation = CalculateCellLocation(x, y);
            UE_LOG(LogTemp, Log, TEXT("Cell [%d, %d] -> Location: %s"), x, y, *CellLocation.ToString());

        }
    }
}

void AUAVSpawner::AssignSearchCells()
{

    for (ABase_UAV* UAV : UAVs)
    {
        if (UAV->bIsAssigned) continue;
        AssignNextCell(UAV);
    }
}

void AUAVSpawner::AssignNextCell(ABase_UAV* UAV)
{
    for (int32 i = 0; i < SearchGrid.Num(); i++) // Use index to modify the actual array element
    {
        if (!SearchGrid[i].bSearched) // Find the first unsearched cell
        {

            if (UAV->bIsAssigned) continue;// skip already asigned uavs

            // Mark UAV as assigned
            UAV->bIsAssigned = true;

            // Assign target location
            FVector TargetLocation = CalculateCellLocation(SearchGrid[i].X, SearchGrid[i].Y);
            UAV->SetTargetLocation(TargetLocation);
            
            SearchGrid[i].bSearched = true; // Correctly mark the actual array element
            
            
            UE_LOG(LogTemp, Log, TEXT("Assigned UAV %s to Cell [%d, %d] -> Location: %s"), 
                   *UAV->GetName(), SearchGrid[i].X, SearchGrid[i].Y, *TargetLocation.ToString());
            return;
        }
    }

    UE_LOG(LogTemp, Warning, TEXT("No unsearched cells left to assign!"));
}

FVector AUAVSpawner::CalculateCellLocation(int32 X, int32 Y) const
{

    FVector GridStart = GeoLocation - FVector((GridSizeX * CellSize) * 0.5f, (GridSizeY * CellSize) * 0.5f, 0.f);

    return GridStart + FVector(X* CellSize,Y*CellSize - (GridSizeY -1), 0);
}

void AUAVSpawner::OnUAVSearchComplete(ABase_UAV* UAV)
{
    AssignNextCell(UAV);
}

void AUAVSpawner::DrawSearchGrid()
{
    UWorld* World = GetWorld();
    if (!World) return;

    // Calculate the starting point based on the search area center
    FVector GridStart = GeoLocation - FVector((GridSizeX * CellSize) * 0.5f, (GridSizeY * CellSize) * 0.5f, 0.0f);

    for (int32 X = 0; X <= GridSizeX; ++X)
    {
        FVector Start = GridStart + FVector(X * CellSize, 0, 0);
        FVector End = Start + FVector(0, GridSizeY * CellSize, 0);

        DrawDebugLine(World, Start, End, FColor::Green, false, 5.0f, 0, 5.0f);
    }

    for (int32 Y = 0; Y <= GridSizeY; ++Y)
    {
        FVector Start = GridStart + FVector(0, Y * CellSize, 0);
        FVector End = Start + FVector(GridSizeX * CellSize, 0, 0);

        DrawDebugLine(World, Start, End, FColor::Green, false, 5.0f, 0, 5.0f);
    }
}

// Called when the game starts or when spawned
void AUAVSpawner::BeginPlay()
{
	Super::BeginPlay();

    //GetTargetLocation() = GeoLocation;
	
    SpawnUAVs();
    
    RegisterAllUAVs();
   /* InitializeSearchGrid();
    AssignSearchCells();*/
}

// Called every frame
void AUAVSpawner::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);
    
    //DrawSearchGrid();
}

