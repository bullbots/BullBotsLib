# BullBotsLib Vendor Repository Setup

This document describes the conversion of BullBotsLib into a WPILib vendor repository.

## What Was Done

### 1. Build System Configuration
- **build.gradle**: Converted from simple Java library to WPILib vendor structure with C++/JNI support
  - Added vendor plugins (NativeUtils, GradleJni, GradleVsCode)
  - Configured three library components: BullBotsLib (native C++), BullBotsLibDriver (JNI), and Java
  - Updated to WPILib 2025.3.2
  - Updated Gradle wrapper to 8.14.3
  - Kept all existing dependencies (CTRE Phoenix 6, REVLib, NavX, Jackson, etc.)

- **config.gradle**: Created configuration for native builds
  - Cross-compilation support for RoboRIO, Linux ARM32/64
  - Platform-specific build configurations

- **publish.gradle**: Created Maven publishing configuration
  - Group ID: `frc.team1891`
  - Artifact ID: `BullBotsLib`
  - Version: `2025.0.0`
  - Publishes Java, C++, and driver artifacts
  - Generates vendordep JSON file

- **settings.gradle**: Updated with plugin management configuration

### 2. Directory Structure
Created the following directories:
```
src/
├── main/
│   ├── java/frc/team1891/  (existing Java code preserved)
│   ├── driver/
│   │   ├── cpp/            (JNI driver implementation)
│   │   │   ├── BullBotsLibJNI.cpp
│   │   │   └── driversource.cpp
│   │   ├── include/        (driver headers)
│   │   │   └── driverheader.h
│   │   └── symbols.txt     (exported JNI symbols)
│   └── native/
│       ├── cpp/            (C++ library implementation)
│       │   └── source.cpp
│       └── include/        (C++ headers)
│           └── header.h
└── test/
    ├── driver/cpp/         (driver tests)
    │   └── main.cpp
    ├── native/cpp/         (native tests)
    │   └── main.cpp
    └── java/               (existing Java tests preserved)
```

### 3. Native Code Placeholders
Since BullBotsLib is currently Java-only, minimal C++ placeholders were created:
- **Driver library**: Provides JNI interface for Java code to load native libraries
- **Native library**: Provides C++ interface that links against the driver library
- These can be expanded in the future if native functionality is needed

### 4. JNI Integration
Created `frc.team1891.common.jni.BullBotsLibJNI` class:
- Loads the BullBotsLibDriver native library
- Provides template for JNI method calls
- Includes test class

### 5. Vendor Dependency File
Created `BullBotsLib.json`:
- FRC Year: 2025
- UUID: `7a8c0651-a0cd-4c12-b17a-c64f58a89d55`
- Maven URL: `https://bullbots.github.io/BullBotsLib/maven`
- JSON URL: `https://bullbots.github.io/BullBotsLib/BullBotsLib.json`
- Includes Java, JNI, and C++ dependencies
- Supports all platforms: Windows, Linux (x86-64, ARM32, ARM64, Athena), macOS

### 6. Configuration Files
- `.wpilib/wpilib_preferences.json`: WPILib VS Code extension configuration
- `.vscode/settings.json`: VS Code workspace settings
- `LICENSE.txt`: MIT License

## How to Build

### Prerequisites
1. **Java 17**: Required for WPILib 2025
2. **WPILib 2025**: Install from https://github.com/wpilibsuite/allwpilib/releases
3. **C++ Toolchain**: Install RoboRIO toolchain via `./gradlew installRoboRIOToolchain`

### Building the Project
```bash
# Build everything (Java, C++, and driver libraries)
./gradlew build

# Build for release mode (against WPILib tagged release)
./gradlew build -PreleaseMode

# Publish to local Maven repository
./gradlew publish

# Build artifacts will be in:
# - build/outputs/ - all packaged artifacts
# - build/libs/ - Java JARs
# - build/repos/releases/ - Maven repository
```

### Build Outputs
The build creates:
1. **Java artifacts**:
   - `BullBotsLib-java-2025.0.0.jar` (main JAR)
   - `BullBotsLib-java-2025.0.0-sources.jar` (source code)
   - `BullBotsLib-java-2025.0.0-javadoc.jar` (documentation)

2. **C++ artifacts**:
   - Platform-specific native library binaries
   - Headers and sources

3. **Driver artifacts**:
   - Platform-specific JNI driver binaries
   - Headers

4. **Vendordep JSON**:
   - `BullBotsLib.json` (processed with version/groupId/artifactId)

## Next Steps

### 1. Test the Build
Once Java is properly configured in your environment:
```bash
./gradlew build
```

### 2. Verify All Artifacts
Check that all artifacts are created in `build/outputs/`:
- Java JARs
- C++ libraries for all platforms
- Driver libraries for all platforms
- Vendordep JSON

### 3. Publish to Maven Repository
You'll need to configure a Maven repository for distribution. Options:
- **GitHub Pages**: Host Maven repo and vendordep JSON
- **GitHub Releases**: Attach artifacts to releases
- **Custom Maven server**: S3, Artifactory, Nexus, etc.

Update `BullBotsLib.json` with your actual Maven and JSON URLs.

### 4. Update README
Update the main README.md with:
- Instructions for adding BullBotsLib via vendordep JSON
- Build instructions for contributors
- Reference to this VENDOR_SETUP.md

### 5. Testing in Robot Project
After publishing:
1. Add vendordep JSON URL to a robot project
2. Test that Java classes are accessible
3. Test that native libraries load correctly (if using JNI)
4. Verify all platforms build correctly

## Customization

### Changing Library Names
If you need to rename components, update these locations:

**build.gradle**:
- Line 113-114: `BullBotsLib` (exportsConfigs)
- Line 118-119: `BullBotsLibDriver` (privateExportsConfigs)
- Line 132: `BullBotsLib(NativeLibrarySpec)` (component name)
- Line 145: `lib library: 'BullBotsLibDriver'` (link dependency)
- Line 150: `BullBotsLibDriver(JniNativeLibrarySpec)` (component name)

**publish.gradle**:
- Line 51: `baseArtifactId = 'BullBotsLib'`
- Line 201: `['BullBotsLib']` (taskList)
- Line 203: `['BullBotsLibDriver']` (driverTaskList)

**C++ code**:
- Update include guards and class names as needed
- Update JNI method signatures in BullBotsLibJNI.cpp and .java

### Changing Maven Coordinates
**publish.gradle**:
- Line 50: `artifactGroupId = 'frc.team1891'`
- Line 51: `baseArtifactId = 'BullBotsLib'`
- Line 10: `pubVersion = '2025.0.0'`

**build.gradle**:
- Line 13: `group 'frc.team1891'`
- Line 14: `version '2025.0.0'`

### Changing WPILib Version
**build.gradle**:
- Line 17: `ext.wpilibVersion = "2025.3.2"`

## Additional Resources
- [WPILib Vendor Template](https://github.com/wpilibsuite/vendor-template)
- [WPILib Vendor JSON Repository](https://github.com/wpilibsuite/vendor-json-repo)
- [Vendordep Checker](https://github.com/wpilibsuite/vendor-json-repo/blob/main/check.py)
- [WPILib 3rd Party Libraries Documentation](https://docs.wpilib.org/en/stable/docs/software/vscode-overview/3rd-party-libraries.html)

## Notes
- The vendor structure maintains backward compatibility with existing Java code
- All existing Java classes remain unchanged in `src/main/java/frc/team1891/`
- Native libraries are optional - if you don't need C++/JNI, they're just placeholders
- The build system supports cross-compilation for all FRC platforms
