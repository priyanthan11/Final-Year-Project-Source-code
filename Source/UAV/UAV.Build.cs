// Copyright Epic Games, Inc. All Rights Reserved.

using System.IO;
using UnrealBuildTool;

public class UAV : ModuleRules
{
	string OPENCV_VERSION = "4110";
    private string ThirdPartyPath
    {
        get
        {
            return Path.GetFullPath(Path.Combine(ModuleDirectory, "../../ThirdParty/"));
        }
    }
    public bool LoadOpenCV(ReadOnlyTargetRules Target)
    {
        // only set up for Win64
        bool isLibrarySupported = false;

        // Create OpenCV Path 
        string OpenCVPath = Path.Combine(ThirdPartyPath, "OpenCV");

        // Get Library Path 
        string LibPath = "";
        bool isdebug = Target.Configuration == UnrealTargetConfiguration.Debug;
        if (Target.Platform == UnrealTargetPlatform.Win64)
        {
            LibPath = Path.Combine(OpenCVPath, "lib", "Win64");
            isLibrarySupported = true;
        }
        else
        {
            string Err = string.Format("{0} dedicated server is made to depend on {1}. We want to avoid this, please correct module dependencies.", Target.Platform.ToString(), this.ToString());
            System.Console.WriteLine(Err);
        }

        if (isLibrarySupported)
        {
            //Add Include path 
            PublicIncludePaths.AddRange(new string[] { Path.Combine(OpenCVPath, "Include") });
            //Add Static Libraries
            PublicAdditionalLibraries.Add(Path.Combine(LibPath, "opencv_world" + OPENCV_VERSION + ".lib"));
            //Add Dynamic Libraries
            PublicDelayLoadDLLs.Add("opencv_world" + OPENCV_VERSION + ".dll");
            PublicDelayLoadDLLs.Add("opencv_videoio_ffmpeg" + OPENCV_VERSION + "_64.dll");
        }
        PublicDefinitions.Add(string.Format("WITH_OPENCV_BINDING={0}", isLibrarySupported ? 1 : 0));
        return isLibrarySupported;
    }
    public UAV(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

        // Add OpenCV include paths
        //PublicIncludePaths.Add(Path.Combine(ModuleDirectory, "ThirdParty", "OpenCV", "include"));

        // Add OpenCV libraries
        //PublicSystemLibraryPaths.Add(Path.Combine(ModuleDirectory, "ThirdParty", "OpenCV", "lib", "Win64", "opencv_world4110.lib")); // Adjust version and path

        //      PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine", "InputCore", "EnhancedInput", "OpenCV", "OpenCVHelper", "Media", "MediaAssets", "Sockets","Networking" });

        //PrivateDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine", "Renderer", "RenderCore", "RHI", "RHICore", "D3D12RHI", "OpenCV", "OpenCVHelper" });

        PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine", "InputCore", "EnhancedInput", "Media", "MediaAssets", "Sockets", "Networking", "CableComponent" });

        PrivateDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine", "Renderer", "RenderCore", "RHI", "RHICore", "D3D12RHI" });


        // Uncomment if you are using Slate UI
        PrivateDependencyModuleNames.AddRange(new string[] { "Slate", "SlateCore" });

        // Uncomment if you are using online features
        // PrivateDependencyModuleNames.Add("OnlineSubsystem");


        // To include OnlineSubsystemSteam, add it to the plugins section in your uproject file with the Enabled attribute set to true opencv_world4110


        //PublicDependencyModuleNames.AddRange(new string[] { /*, "RenderCore", "Media", "MediaAssets" */});
        //LoadOpenCV(Target);

    }
}
