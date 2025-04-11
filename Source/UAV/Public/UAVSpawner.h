// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Base_UAV.h"
//#include "opencv2/core.hpp"
#include "UAVSpawner.generated.h"


USTRUCT(BlueprintType)
struct FSearchCell
{
    GENERATED_BODY()

    UPROPERTY()
    int32 X;
    UPROPERTY()
    int32 Y;
    UPROPERTY()
    bool bSearched; // Trach weather the cell has been searched or not
   

    FSearchCell() : X(0), Y(0), bSearched(false){}
    FSearchCell(int32 InX, int32 InY) : X(InX), Y(InY), bSearched(false){}
  
};


UCLASS()
class UAV_API AUAVSpawner : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	AUAVSpawner();

    // UAV class to spawn
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UAV Spawning")
    TSubclassOf<ABase_UAV> UAVClass;

    // Number of UAVs to spawn
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UAV Spawning")
    int32 NumUAVs;

    // Spawn area bounds
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UAV Spawning")
    FVector SpawnAreaMin;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UAV Spawning")
    FVector SpawnAreaMax;

    UPROPERTY(EditAnywhere, Category = "UAV")
    FVector GeoLocation;

    float Spacing;


    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UAV Mapping")
    int32 GridSizeX;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UAV Mapping")
    int32 GridSizeY;
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UAV Mapping")
    float CellSize; // Size of each cell

    TArray<FSearchCell> SearchGrid; // Stores the search cells
    TArray<ABase_UAV*> UAVs;



private:
    // Spawn UAVs
    void SpawnUAVs();

    FVector CalculateOffset(int32 X, int32 Y, int32 Z, int32 GridSize) const;

    // Register UAVs with each other
    void RegisterAllUAVs();

    // Functions
    void InitializeSearchGrid();
    void AssignSearchCells();
    void AssignNextCell(ABase_UAV* UAV);
    FVector CalculateCellLocation(int32 X, int32 Y) const;
    void OnUAVSearchComplete(ABase_UAV* UAV);
    void DrawSearchGrid();
protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

    FORCEINLINE int32 GetUAVNum() const { return NumUAVs; }

};
