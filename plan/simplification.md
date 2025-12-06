# Codebase Simplification Plan

## Overview
Simplify the ledrx codebase before updating to ESP-IDF 5.5.1 by removing OTA functionality and consolidating submodules directly into the project.

## Goals
1. Remove OTA (Over-The-Air update) functionality - not needed for this project
2. Remove OTA-related submodules
3. Integrate remaining submodules directly into the codebase (flatten the dependency structure)

## Tasks

### 1. Remove OTA Functionality
- [ ] Remove `components/iotp-ota` submodule
- [ ] Remove `components/iotp-crc32` submodule (only used by OTA)
- [ ] Remove OTA-related code from main application
- [ ] Remove OTA build targets from [Makefile](file:///Users/matthew.everett/code/ledrx/Makefile) (`ota`, `increment` recipes)
- [ ] Remove OTA scripts directory:
  - `ota/inc-version.py`
  - `ota/publishbin.sh`
- [ ] Search for and remove any OTA-related includes/references in the codebase
- [ ] Remove submodule references from [.gitmodules](file:///Users/matthew.everett/code/ledrx/.gitmodules)

### 2. Integrate iotp-wifi Submodule
- [ ] Copy `components/iotp-wifi` source files directly into the project
- [ ] Update component CMakeLists.txt if needed
- [ ] Remove submodule reference from [.gitmodules](file:///Users/matthew.everett/code/ledrx/.gitmodules)
- [ ] Remove submodule tracking: `git rm components/iotp-wifi`
- [ ] Add the source files to git: `git add components/iotp-wifi`

### 4. Clean Up Build Configuration
- [ ] Update [Makefile](file:///Users/matthew.everett/code/ledrx/Makefile) - remove OTA targets
- [ ] Verify [CMakeLists.txt](file:///Users/matthew.everett/code/ledrx/CMakeLists.txt) doesn't reference removed components
- [ ] Remove `.gitmodules` file entirely (if no submodules remain)

### 5. Verification
- [ ] Ensure the project builds successfully: `idf.py build`
- [ ] Verify no references to OTA functionality remain
- [ ] Confirm all necessary functionality is preserved

## Benefits
- **Simpler dependency management**: No external submodule dependencies
- **Easier maintenance**: All code is in one repository
- **Reduced complexity**: Fewer moving parts before the ESP-IDF 5.5.1 upgrade
- **Cleaner codebase**: Remove unused OTA functionality

## Notes
- Create a backup branch before starting: `git checkout -b backup-before-simplification`
- This should be completed before attempting the ESP-IDF 5.5.1 upgrade
- OTA functionality removal rationale: Not needed for the current use case and adds unnecessary complexity
- CRC32 module removal: Only used by OTA component for firmware verification, no other uses in the codebase
