// Fill out your copyright notice in the Description page of Project Settings.

#pragma once


//#include "opencv2/core/utility.hpp"
//#include "opencv2/core.hpp"
//#include <opencv2/dnn.hpp> 
//#include <opencv2/core.hpp>       // Core functionalities
//#include <opencv2/imgproc.hpp>    // Image processing
//#include <opencv2/highgui.hpp>    // GUI (imshow, waitKey)
//#include <opencv2/dnn.hpp>        // Deep learning (YOLO, SSD)
//#include <opencv2/videoio.hpp>    // Video processing (optional)


#include <vector>
#include <string>
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "PIDController.h"

#include "Components/TextRenderComponent.h"
#include "Sockets.h"
#include "Networking.h"

//#include "Engine/TextureRenderTarget2D.h"
#include "Components/SplineComponent.h"
#include "CableComponent.h"
#include "Components/SphereComponent.h"


#include "NetworkThread.h"
#include "Base_UAV.generated.h"

UENUM()
enum EUAV_State
{
	EUS_IDEL UMETA(DisplayName = "Idel"),
	EUS_TRAVEL UMETA(DisplayName = "Travel"),
	EUS_SEARCH UMETA(DisplayName = "Search"),
	EUS_EXTRACT UMETA(DisplayName = "Extract")

};


USTRUCT()
struct FSearch_Cell
{
	GENERATED_BODY()
public:
	UPROPERTY()
	int32 X;
	UPROPERTY()
	int32 Y;
	UPROPERTY()
	bool bIsSearched; // Trach weather the cell has been searched or not
	UPROPERTY()
	FVector WorldLocation;
	UPROPERTY()
	int32 searchedGrid;
	FSearch_Cell() : X(0), Y(0), bIsSearched(false), WorldLocation(FVector::ZeroVector), searchedGrid(0){}
	FSearch_Cell(int32 InX, int32 InY) : X(InX), Y(InY), bIsSearched(false), WorldLocation(FVector::ZeroVector), searchedGrid(0) {}
	FSearch_Cell(int32 InX, int32 InY, FVector InLocation) : X(InX), Y(InY), bIsSearched(false), WorldLocation(InLocation), searchedGrid(0) {}
};
// Define a struct for Formation Slot
struct FFormationSlot
{
	FVector Position;  // The position offset relative to the leader
	bool bIsOccupied;  // Whether this slot is occupied by a UAV

	FFormationSlot(FVector InPosition) : Position(InPosition), bIsOccupied(false) {}
};
USTRUCT()
struct FSearchMessage
{
	GENERATED_BODY()
public:
	UPROPERTY()
	int32 UAV_ID;
	UPROPERTY()
	FVector RequestedCell;
	UPROPERTY()
	bool bAccepted;

	FSearchMessage() : UAV_ID(0), RequestedCell(FVector::ZeroVector), bAccepted(false) {}
	FSearchMessage(int32 ID, FVector Cell, bool Accepted) : UAV_ID(ID), RequestedCell(Cell), bAccepted(Accepted) {}


};

UCLASS()
class UAV_API ABase_UAV : public AActor
{
	GENERATED_BODY()
	
public:	
	// Sets default values for this actor's properties
	ABase_UAV();
	virtual ~ABase_UAV();

	

private:

	float DeltaX;

	UPROPERTY(EditAnywhere, Category = "UAV")
	USkeletalMeshComponent* UAVMesh;
	UPROPERTY(VisibleAnywhere, Category = "UAV")
	class UCameraComponent* UAVCamera;


	// Rope Component
	/*UPROPERTY(VisibleAnywhere, Category = "UAV| Extraction")
	UCableComponent* RopeComponent;*/
	UPROPERTY(VisibleAnywhere, Category = "UAV| Extraction")
	USphereComponent* AttachmentSphere;
	UPROPERTY(VisibleAnywhere, Category = "UAV | Extraction")
	AActor* AttachedActor;
	UPROPERTY(VisibleAnywhere, Category = "UAV | Extraction")
	bool bRopeAttached;
	UPROPERTY(VisibleAnywhere, Category = "UAV | Extraction")
	bool bRopeDropped;
	UFUNCTION(Category = "UAV | Extraction")
	void DropeRope();
	UFUNCTION(Category = "UAV | Extraction")
	void AttachedToTarget(AActor*Target);
public:
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UAV | Extraction")
	bool bCanRopDrop;

	// =========================================================================

private:
	// PID controller smart pointers
	TUniquePtr<PIDController> RollController;
	TUniquePtr<PIDController> PitchController;
	TUniquePtr<PIDController> YawController;
	TUniquePtr<PIDController> AltitudeController;
	TUniquePtr<PIDController> ForwardController;
	TUniquePtr<PIDController> RightController;

	FVector CurrentPosition;
	FRotator CurrentRotation;

	float MaxRollRate;  // Maximum allowable roll rate (degrees per second)
	float MaxPitchRate; // Maximum allowable pitch rate (degrees per second)
	float MaxYawRate;   // Maximum allowable yaw rate (degrees per second)

	//UPROPERTY(EditAnywhere, Category = "UAV")
	FVector TargetPosition;
	FRotator TargetRotation;


	bool isTargetReached;
	bool bisSearchPhase;

	NetworkThread* NetworkingThread;
	FRunnableThread* NetworkingRunnableThread; // Thread manager for the networking thread
	virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

	UPROPERTY(VisibleAnywhere, Category = "Path Visualization")
	USplineComponent* SplinePath;

	UPROPERTY(EditAnywhere, Category = "UAV Appearance")
	UMaterialInstanceDynamic* DynamicMaterial;
	UPROPERTY(VisibleAnywhere, Category = "UAV Appearance")
	UTextRenderComponent* RoleText;

	UPROPERTY(VisibleAnywhere, Category = "UAV | Extraction")
	UStaticMeshComponent* RopeMesh;
	UPROPERTY(EditAnywhere, Category = "UAV | Extraction")
	UStaticMesh* RopeStaticMesh;
protected:
	// Called when the game starts or when spawned
	virtual void BeginPlay() override;

	void SetTargetPosition(FVector NewTarget);
	void SetTargetRotation(FRotator NewTarget);

	void DrawPathLine();
	void UpdateSplinePath();
	void UpdateRoleText();
	void DrawFormationDebug();
	
	

	// PID parameters for editor
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PID|Roll")
	float RollKp;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PID|Roll")
	float RollKi;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PID|Roll")
	float RollKd;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PID|Pitch")
	float PitchKp;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PID|Pitch")
	float PitchKi;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PID|Pitch")
	float PitchKd;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PID|Yaw")
	float YawKp;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PID|Yaw")
	float YawKi;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PID|Yaw")
	float YawKd;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PID|Altitude")
	float AltitudeKp;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PID|Altitude")
	float AltitudeKi;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PID|Altitude")
	float AltitudeKd;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PID|Forward")
	float ForwardKp;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PID|Forward")
	float ForwardKi;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PID|Forward")
	float ForwardKd;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PID|Right")
	float RightKp;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PID|Right")
	float RightKi;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PID|Right")
	float RightKd;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PID|Right")
	float RollOutput;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PID|Right")
	float PitchOutput;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PID|Right")
	float YawOutput;


	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PID|Right")
	FVector MovementForce;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "PID|Right")
	FVector RepulsionForce;

private:
	FVector CurrentVelocity; // Current velocity of the UAV
	FVector TargetVelocity;  // Target velocity after applying Boids rules

	TArray<AActor*> OtherUAVs; // List of other UAVs in the scene

	UPROPERTY(EditAnywhere, Category = "Boids")
	float PerceptionRadius; // Radius for detecting nearby UAVs

	UPROPERTY(EditAnywhere, Category = "Boids")
	float SeparationWeight; // Weight for separation force

	UPROPERTY(EditAnywhere, Category = "Boids")
	float AlignmentWeight; // Weight for alignment force

	UPROPERTY(EditAnywhere, Category = "Boids")
	float CohesionWeight; // Weight for cohesion forc

	UPROPERTY(EditAnywhere, Category = "Obstacle Avoidance")
	float ViewRadius; // How far the UAV can "see" for obstacles

	UPROPERTY(EditAnywhere, Category = "Obstacle Avoidance")
	float ObstacleAvoidanceWeight; // Weight for obstacle avoidance force

	UPROPERTY(EditAnywhere, Category = "Obstacle Avoidance")
	TArray<FVector> RayDirections; // Directions to cast rays for obstacle detection

	


	FVector ComputeSeparationForce() const;
	FVector ComputeAlignmentForce() const;
	FVector ComputeCohesionForce() const;
	FVector ComputeObstacleAvoidanceForce() const; // New method for obstacle avoidance
	TArray<FVector> GenerateSpiralRayDirections(int32 NumPoints, float TurnFraction, float Power);
	FVector ComputeBoidForce(float DeltaTime);





	// Collision
	UPROPERTY(EditAnywhere, Category = "UAV")
	float AvoidanceRadius;

	UPROPERTY(EditAnywhere,  Category = "UAV")
	float RepulsionStrength;


	/* Calculate repulsive forces using the potential field method*/
	FVector AvoidCollisions();
public:
		UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UAV|Camera")
		class USceneCaptureComponent2D* SceneComponent;

		UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "UAV|Cameara")
		UTextureRenderTarget2D* RenderTarget;

		UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "UAV State")
		TEnumAsByte<EUAV_State> uav_State;

private:
	// Range for generating a new random target point
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Target", meta = (AllowPrivateAccess = "true"))
	float RandomTargetRange;

	// Tolerance distance to consider as "reached" the target
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Target", meta = (AllowPrivateAccess = "true"))
	float TargetReachedTolerance;

	// Concensus formation
	bool bHasReachedTarget = false;
	bool bIsLeader = false; // Determines if UAV is leader
	TArray<FVector> SquareOffsets; // Stores offsets for square formation


	// Consensus Formation Control Variables
	UPROPERTY(EditAnywhere, Category = "Consensus Formation")
	float FormationStrength = 0.1f;

	UPROPERTY(EditAnywhere, Category = "Consensus Formation")
	float CollisionRadius = 150.0f;

	UPROPERTY(EditAnywhere, Category = "Consensus Formation")
	float NeighborRadius = 500.0f;

	// Conncensus formation controll
	void ApplyConsensusFormation();
	void AvoidCollision();
	FVector ComputeFormationCenter();
	void ApplyConsensusFormationControl();


	// Other UAVs in the Scene
	TArray<ABase_UAV*> GetNearbyUAVs();

	// Functions
	void DetermineLeader();
	void AssignFormationOffsets();
	FVector GetDesiredFormationPosition();

	
	// LOgging
	void LogSearchTime(int32 UAV_Count, int32 UAV_IDs, const FString& Event,float searchduration);


private:
	
	// Recive the input frpm python server 
	// Coordinates respect to screen space

	void SetupUDPReceiver();
	void ReceivePersonLocation();

	FSocket* UDPSocket;
	FIPv4Endpoint RemoteEndpoint;
	FTimerHandle UDPReceiverTimer;
	FTimerHandle UavStateSwitcher;
	

	TMap<int32, FSocket*> SocketMap;


	FSocket* TriggerSocket;
	TSharedPtr<FInternetAddr> TriggerAddr;
	void SetupTriggerSocket();
	void CheckForTriggerMessage();

	FVector2D PersonScreenPosition;


	FVector TargetFormationPosition;

public:
	// Object detection with yolo + open cv

	FTimerHandle StreamTimerHandle, ReceiveTimerHandle, ReconnectTimerHandle;
	//cv::Mat cvMat;
	//cv::dnn::Net* net;
	std::vector<std::string> classNames;

	FSocket* Socket;
	FIPv4Address ServerIP;
	int32 ServerPort;


	// Is already assigned serch area
	bool bIsAssigned;

	void LoadYOLOModel();
	void PostProcessYOLO();

	bool IsServerAvilable();
	void CheckServerAvilability();
	void ReconnectToServer();

	void CaptureFrame();
	void LoadDetectionResults();
	void ConnectToServer(int32 r_UAV_ID, int32 r_ServerPort);
	
	// reached the GEO postion
	void isReached(bool bisSearch);
	// Reset the PID controler
	void resetPIDController();
	// Calculate Distance to target location
	void CalculateTargetLocation();
	// Move 
	void Move(const bool bIsBlocked,const FVector& targetLocation);

	// Perform Search phase
	void searchPhase();
	// Perform Extract phase
	void extractPhase();


	void SetUAV_ID(int32 NewID);

	


	// UAV Socket
	FSocket* UAVSocket;

	UPROPERTY(EditAnywhere, Category = "UAV Settings")
	// UAV Uniq ID
	int32 UAV_ID;
	// Notify to backed_server how many UAVs are spawned
	int32 GetUAVNum();
	void NotifyPythonServer();


public:

	// Searcj phase
	static TArray<FSearch_Cell> GlobalSearchGrid; // Search grid
	static FCriticalSection GridLock; // Prevent race conditions

	UPROPERTY()
	TArray<ABase_UAV*> NearbyUAVs;
	UPROPERTY()
	bool bIsSearching; 
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly)
	bool bIsTargetIsFound;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Map Parameter")
	int32 GridSizeX;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Map Parameter")
	int32 GridSizeY;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Map Parameter")
	float CellSize;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Map Parameter")
	FVector TargetLocation;

	bool bIsKickedStarted;

	void InitializeGrid();
	void AssignSearchCell();
	void OnSearchComplete();
	void RequestSearchCell();
	void ReciveSearchRequest(FSearchMessage Message);
	bool IsCellOccupide(FVector CellLocation);
	FVector FindUnsearchedCell();
	TArray<ABase_UAV*> GetNearbyUAVs_bySphere();



	void UpdateSearch();
	void DrawSearchGrid();
	void SwitchToTravel();
	

	// Log
	int32 NumOfUAVs;
	//int32 NumOfSearchedCell;
	

public:	
	// Called every frame
	virtual void Tick(float DeltaTime) override;

	void RegisterOtherUAV(AActor* UAV); // Register another UAV for flocking
	void Boids(float DeltaTime);

	// Function to check if the UAV reached the target and generate a new random target
	void UpdateTargetPointIfNeeded();

	void ProcessCameraFeed();


	void SetUAV_ID_Text();

	UFUNCTION(BlueprintPure)
	FORCEINLINE FVector GetTargetPosition() const { return TargetPosition; }
	FORCEINLINE int32 GetNumOfUAVs() const { return NumOfUAVs; }
	UFUNCTION(BlueprintPure)
	FORCEINLINE int32 GetUAV_ID() const { return UAV_ID; }
	FORCEINLINE void SetTargetLocation(FVector targetLocation) { TargetPosition = targetLocation; }

	FORCEINLINE void SetServerPort(int32 targetLocation) { ServerPort = targetLocation; }
	FORCEINLINE void SetNumOfUAVs(int32 numberofUAV) { NumOfUAVs = numberofUAV; }

	UFUNCTION(BlueprintCallable, BlueprintPure)
	FORCEINLINE FVector2D GetPersonScreenPosition() const { return PersonScreenPosition; }


};
