# Motion Master Lib Examples

This repository provides usage examples for the **Motion Master** static C++ libraries. Included are:

- The **SPoE** library, which implements a custom protocol for communicating with Synapticon SOMANET Integro devices over Ethernet.
- The **SOEM** library for communicating with Synapticon SOMANET devices via EtherCAT.

For full documentation on the Motion Master libraries, please visit our [official documentation](https://synapticon.github.io/motion_master/).

## Building on Windows 11

**Required Software:**

- [Git](https://git-scm.com/)
- [Visual Studio 2022](https://visualstudio.microsoft.com/vs/)
- [CMake](https://cmake.org/)

To temporarily enable `cl.exe` in your PowerShell 7 (x64) session, follow these steps:

1. Open **PowerShell 7 (x64)**.
2. Run the following command to initialize the Visual Studio environment:

   ```pwsh
   cmd /c '"C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvarsall.bat" x64 && powershell'
   ```

3. Once the new PowerShell session starts, verify that cl.exe is available by running:

   ```pwsh
   cl
   ```

> ℹ️ The previous steps are necessary because the Developer Command Prompt for VS 2022 may default to the 32-bit toolset, causing linker failures in 64-bit builds. You can verify this by running `cl` and checking if the output includes **for x86**.

### Build Instructions

1. **Clone the repository with all submodules:**

```powershell
git clone --recursive https://github.com/synapticon/motion_master_lib_examples.git
cd motion_master_lib_examples
```

2. **Build the Debug version:**

```powershell
cmake --preset x64-windows-debug
cmake --build --preset x64-windows-debug --parallel
```

The compiled executables will be located in: `build/x64-windows-debug/`
