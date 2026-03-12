# Custom Drone Modules

This page explains how to add custom code to *PX4* for your own drone without modifying any of the core *PX4* source files.

## Understanding the Approach

*PX4* is a complete, self-contained autopilot firmware.
You do not need to remove or replace parts of it when building firmware for your custom drone.
Instead, you keep the full *PX4* codebase and add your own code as a separate module.

A module is a self-contained component that *PX4* can start, stop, and manage independently.
Modules can read sensor data, update parameters, and communicate with other firmware components via the [uORB messaging system](../middleware/uorb.md).

::: info
The *PX4* CMake build system automatically sets up all required header paths and library links for your module.
You do not need to configure these yourself.
:::

## Why External (Out-of-Tree) Modules?

*PX4* supports writing custom modules in a directory outside the `PX4-Autopilot` repository.
This is called an _external_ or _out-of-tree_ module.
Using this approach:

- You never need to edit any core *PX4* source files.
- Upgrading *PX4* to a newer version does not overwrite your code.
- All *PX4* include paths are set up automatically by the build system.

## Creating Your First External Module

The following steps create a minimal working external module.

### Step 1: Create the Directory Structure

Create a new folder anywhere outside the `PX4-Autopilot` directory.
It must contain a `src` subdirectory:

```sh
mkdir -p ~/my_drone/src/modules/my_module
```

### Step 2: Add Your Module Files

Create the two files below inside `~/my_drone/src/modules/my_module/`.

The `CMakeLists.txt` file tells the build system about your module:

```cmake
px4_add_module(
    MODULE modules__my_module
    MAIN my_module
    SRCS
        my_module.cpp
    EXTERNAL
)
```

The `EXTERNAL` keyword marks this module as out-of-tree.
The `px4_add_module()` function automatically adds all *PX4* header directories to your module's include path.

The `my_module.cpp` source file contains your module's code:

```cpp
#include <px4_platform_common/log.h>

extern "C" __EXPORT int my_module_main(int argc, char *argv[]);

int my_module_main(int argc, char *argv[])
{
    PX4_INFO("Hello from my custom module!");
    return 0;
}
```

### Step 3: Add the Top-Level CMakeLists.txt

Create `~/my_drone/CMakeLists.txt` to register your module with the build system:

```cmake
set(config_module_list_external
    modules/my_module
    PARENT_SCOPE
)
```

### Step 4: Build with Your Module

Run the standard *PX4* build command with the path to your external directory:

```sh
cd ~/PX4-Autopilot
make px4_sitl EXTERNAL_MODULES_LOCATION=~/my_drone
```

Replace `px4_sitl` with your target board name, for example `px4_fmu-v5_default`.
For subsequent builds, `EXTERNAL_MODULES_LOCATION` does not need to be specified again provided the build directory already exists.

## How Include Paths Work

When you use `px4_add_module()`, the *PX4* CMake build system automatically adds:

- Platform-common headers from `platforms/common/include/`
- uORB message headers generated into the build directory
- All standard *PX4* library headers

This means any `#include` used by built-in *PX4* modules will also work in your custom module without any additional configuration.

## Further Reading

- [External Modules (Out-of-Tree)](../advanced/out_of_tree_modules.md) — Full reference, including out-of-tree uORB message definitions
- [Writing your First Application (Hello Sky!)](hello_sky.md) — Detailed step-by-step first-application tutorial
- [Application/Module Template](module_template.md) — A full-featured module example with parameters and uORB subscriptions
- [uORB Messaging](../middleware/uorb.md) — How to subscribe to sensor data and publish your own messages
