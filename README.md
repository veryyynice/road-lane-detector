# Autonomous Lane Tracker & Steering Demo

This is our final project for Computer Vision. The system processes a raw driving video, applies a perspective warp to generate a bird's-eye view, uses a sliding window polynomial fit to calculate the road curve, and dynamically turns a dashboard steering wheel.

## 🛠️ Visual Studio Setup Instructions for Teammates

Because OpenCV relies on massive `.dll` (Dynamic Link Library) files, those files are too large for GitHub and are ignored by our `.gitignore`. You must link OpenCV and copy the `.dll` files manually on your machine.

### Step 1: Open the Project
1. Clone this repository.
2. Open the `LaneDetector/LaneDetector.sln` file in Visual Studio 2019/2022.
3. Ensure the build configuration at the top is set to **Debug** and **x64**.

### Step 2: Link OpenCV
1. Right-click the `LaneDetector` project in the Solution Explorer -> **Properties**.
2. **C/C++ -> General -> Additional Include Directories:** Add your OpenCV include path (e.g., `C:\opencv\build\include`).
3. **Linker -> General -> Additional Library Directories:** Add your OpenCV lib path (e.g., `C:\opencv\build\x64\vc16\lib`).
4. **Linker -> Input -> Additional Dependencies:** Type `opencv_world4110d.lib` (match your version number).

### Step 3: The Missing DLLs (CRITICAL)
If the project builds but crashes instantly, Windows cannot find the OpenCV runtime files. You MUST copy these two files from your `C:\opencv\build\x64\vc16\bin` folder and paste them directly into the inner `LaneDetector/LaneDetector/` folder (right next to `main.cpp` and the `.mp4` video):

1. `opencv_world4110d.dll` (The core computer vision library)
2. `opencv_videoio_ffmpeg4110_64.dll` (The FFMPEG plugin required to decode .mp4 files. Note: use the non-'d' release version of this specific file).

Hit **F5** to run the simulation.
