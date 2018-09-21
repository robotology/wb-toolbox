# Tutorial: How to create a new library

This guide describes how to use the core infrastructure of the Whole Body Toolbox for creating a new toolbox **Toolbox Example**. It will contain a single block _Signal math_ with the following specifications:

- Accepts two input signals
- Performs element-wise operations: sum, subtraction, multiplication
- Allows to select the operation with a user friendly GUI (mask)
- Produces an output signal with the result of the operation

Despite it is a very trivial example, it shows how to structure both the C++ and the Matlab components of a toolbox. From this starting point is then possible to build more complex architectures which might need e.g. to be split in many parts or to link against third-party libraries.

!!! example "Toolbox Example project"
    You can find the files of this example in the [`example`](https://github.com/robotology/wb-toolbox/tree/master/example) folder.

!!! info
    Until `v4` this core machinery is shipped together with our robotics framework. We're currently working on splitting the toolbox components for providing a standalone process. Stay tuned for upcoming news!

## Introduction

Before jumping in the example of this tutorial, in this section you will find helpful information useful to grasp the key ideas about the toolbox and a refreshing of common terms and patterns used in programming.

### Algorithm specifications

The execution of a generic algorithm can be split in the following basic phases:

1. Configuration
2. Initialization
3. Step
4. Termination

In the configuration phase the algorithm can, for instance, read parameters and specify details about its inputs and outputs. During the initialization it might need to allocate resources. When everything is ready, the simulation starts and on every cycle of the loop the algorithm computes a step. Before finishing the simulation, in the termination step the resources that are not anymore needed can be deallocated and final operations can be executed.

### Terminology

There are few key components which are part of the core infrastructure, and it is important they are clear from the very beginning.

!!! note
    This project has strong roots with Simulink. Despite it is not anymore strictly related to it, the structure keeps as inheritance its terminology.

#### Block

The Block is the main component of the toolbox. You may think of it as a wrapper for a generic algorithm. It provides support of all the phases discussed above.

#### Port

Blocks need to interface with the outside for reading input data and writing their results. Ports are attached to the Block and let data flow in and out from it. They have properties like the accepted data type and they are characterized by a size.

#### Signal

A Signal is the generalization of a buffer. It can be plugged to multiple Ports and allows sharing data between them. Similarly to Ports, a Signal has a data type and a width. When a Signal is connected to a Port, their dimension must match.

#### Engine

The engine is the framework that calls and executes the code implementing the Blocks. We currently provide support for Simulink and Simulink Coder. Alternative engines might be Scilab or even standalone C++ code.

#### BlockInformation

BlockInformation is the interface between a Block and the engine that executes its code. Despite blocks are not aware of what engine is executing them, blocks and engine need to exchange information such as user parameters and Port data. BlockInformation provides methods for setting and getting this piece of information.

#### Simulink Block

A Simulink Block is the wrapper of a (C++) Block. It provides a visual representation of it, with input and output ports and gives the possibility to connect it with other Simulink blocks through signals. The terms above come from this representation and map it to C++.

#### Simulink Model

A Simulink Model is a project composed of a set of Simulink Blocks interconnected by Signals.

#### Simulink Library

A Simulink Library is a container of a set of Blocks that populates the _Simulink Library Browser_. Every toolbox must be shipped with an associated Simulink Library file in order to expose its Blocks.

#### S-Function

There are many types of functions for implementing an algorithm wrapped by a Simulink Block. In Matlab terminology, these functions are referred as [S-Functions](https://it.mathworks.com/help/simulink/sfg/what-is-an-s-function.html). You can read more about the supported types in [What type of S-Function should you use](https://it.mathworks.com/help/simulink/sfg/what-type-of-s-function-should-you-use.html).

In short S-Functions provide a sort of interface (through C callbacks) where a given number of functions need to be implemented. Simulink knows the name of these functions and calls them on demand.

#### Block Mask

A Simulink Block is just a square with input and output ports. It is possible to "mask" a Simulink Block in order to provide a user-friendly GUI that can be useful e.g. for setting Block parameters. The mask may contain buttons, sliders, check boxes, dropdown menus, etc.

#### Software library

A library is a file containing compiled code (functions, classes, etc.) which cannot be executed standalone. It can be either _static_ or _dynamic_. Static libraries are meant to be embedded inside the executable that calls their symbols, instead the code of dynamic libraries (also called shared libraries) is only referenced inside the executable and called when needed.

For grasping better this difference, if multiple executables link against the same static library, the same code is embedded inside all of them, resulting in bigger executables and code duplication. A dynamic library object instead can be shared by multiple executables that need only to know their location in the filesystem and which symbols they provide.

!!! info
    You can find more detailed information about software libraries and linkers at [this link](https://www.lurklurk.org/linkers/linkers.html). 

#### Plugin Library

A plugin library is a particular type of a dynamic library. An executable can load dynamic libraries either at load time, i.e. when the executable is launched, or at run time, i.e. when needed during the execution. The libraries that can be loaded during run time are referred as plugins.

!!! info
    On UNIX systems the load the a plugin is executed by a `dlopen`

### Overview of Simulink execution

The core of the WB-Toolbox provides a framework capable of loading during runtime shared libraries. When the Simulink simulation starts, its engine assigns a deterministic order to the blocks of the Simulink Model. If one of these blocks is not contained in the system toolboxes, it needs to be associated to a _S-Function_ that implements its logic. The toolbox provides a streamlined approach to implement these functions without the need of being an expert of the complex Simulink APIs.

The blocks of our library are implemented in Simulink using the following _S-Function_ block:

![EmptySFunctionBlock](images/EmptySFunctionBlock.png)

What you need to know for the time being is that the name of the plugin library generated from the C++ code will be stored in the _S-function parameters_ field. We provide a generic **Level-2 MEX S-Function** that reads this parameter and uses the library name name to dynamically load the dynamic shared library that contains the block's code.

After the library is found in the filesystem and successfully loaded, Simulink allocates an object of the C++ class associated to the block functionality. Again, this information (the class name) is passed in the _S-function parameters_ field.

## Develop the C++ plugin

You already learned that Blocks are no more than regular C++ classes. They are not an exception, in fact all the components discussed until now are mapped to C++ classes or interfaces ([abstract classes](https://en.cppreference.com/w/cpp/language/abstract_class)).

The following resources provide further information about them:

- Blocks are implementations of the [`wbt::Block`](https://robotology.github.io/wb-toolbox/doxygen/classwbt_1_1_block.html) interface
- BlockInformation is defined in the  [`wbt::BlockInformation`](https://robotology.github.io/wb-toolbox/doxygen/classwbt_1_1_block_information.html) interface
- Signals are mapped to the [`wbt::Signal`](https://robotology.github.io/wb-toolbox/doxygen/classwbt_1_1_signal.html) class
- Blocks parameters are mapped to the [`wbt::Parameter`](https://robotology.github.io/wb-toolbox/doxygen/classwbt_1_1_parameter.html) class

We need the following folder structure for the C++ project. Create already the empty files so the project can compile from the very first attempt.

```
.
├── CMakeLists.txt
├── include
│   └── SignalMath.h
└── src
    ├── Factory.cpp
    └── SignalMath.cpp
```

!!! info
    All the path of this tutorial will be relative to the root folder of the project. This means that if the directory tree above is stored in `/home/foo/tutorial`, when you read to go the `./build` directory it means `/home/foo/tutorial/build`.  

!!! tip
     Bear in mind that this C++ class is independent from Simulink. Potentially, it can be called by a standalone C++ executable.

!!! tip
    It is not mandatory to implement a new class for every Simulink Block. If they share a lot of code, a single class can be referenced by multiple Simulink Blocks and its behavior can be selected using parameters.

### CMake project for compiling the library

You are free to use your favorite tool for compiling the project. We recommend CMake. If you are not an expert user of this tool, just follow the steps. The comments in the file should be enough to understand what it is happening.

Fill the file `CMakeLists.txt` with the following content:

```cmake
cmake_minimum_required(VERSION 3.5)
project(ExampleToolbox LANGUAGES CXX VERSION 0.1)

# C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Export all symbols in Windows
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# Tweak linker flags in Linux.
# Matlab is very strict on missing symbols and by default ld do not warn if
# something is missing.
if(UNIX AND NOT APPLE)
    get_filename_component(LINKER_BIN ${CMAKE_LINKER} NAME)
    if(${LINKER_BIN} STREQUAL "ld")
        set(CMAKE_SHARED_LINKER_FLAGS "-Wl,--unresolved-symbols=report-all")
    endif()
endif()

# ===========
# C++ LIBRARY
# ===========

# Find the needed WBToolbox components:
# - ToolboxCore contains the core classes such as Block and Signal
# - ToolboxMex is required at runtime for loading the library from Simulink
find_package(WBToolbox 4 REQUIRED COMPONENTS ToolboxCore ToolboxMex)

# Find the project for the multiplatform support of plugin libraries.
# It is shipped with WBToolbox and it should be already installed in your system.
find_package(shlibpp REQUIRED)

# Create the plugin library. This must be a SHARED library.
add_library(ExampleToolbox SHARED
    include/SignalMath.h
    src/SignalMath.cpp
    src/Factory.cpp)

# Manually set the name of the output library. This is not required and it
# is done only for sake of clarity.
set_target_properties(ExampleToolbox PROPERTIES
    OUTPUT_NAME "ExampleToolbox")

# Link the library with the Core component containing the core classes
# and the target that provides the plugin support
target_link_libraries(ExampleToolbox
    WBToolbox::Core shlibpp::shlibpp)

# Setup the include directories
target_include_directories(ExampleToolbox PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/ExampleToolbox>)
```

!!! note
    If your library needs to link against other libraries, use `find_package` to load their targets and then add them to the `target_link_libraries` directive.

From the root folder of the project, execute:

```bash
mkdir build
cd build
cmake ..
cmake --build .
```

You should now find in the `./build` directories a new library file, which depending on your OS is:

- `libExampleToolbox.so` on Linux
- `libExampleToolbox.dylib` on macOS
- `ExampleToolbox.dll` on Windows

This is the toolbox's plugin library which is loaded during runtime by the Engine. 

### Implement the block logic

The only _Signal math_ block of our new toolbox will be implemented in a `example::SignalMath` C++ class. 

#### Header

Here below the `./include/SignalMath.h` header.

Given the simple logic it should be straightforward to understand. The class inherits from the `wbt::Block` interface and implements some of its methods.

!!! info
    The only mandatory method to implement is the `wbt::Block::output`. By default the other methods are dummy and they always return `true`.

```cpp
#ifndef EXAMPLE_SIGNALMATH_H
#define EXAMPLE_SIGNALMATH_H

#include <Core/Block.h>
#include <Core/BlockInformation.h>

#include <memory>
#include <string>

namespace example {
    class SignalMath;
} // namespace example

class example::SignalMath : public wbt::Block
{
private:
    enum class Operation
    {
        ADDITION,
        SUBTRACTION,
        MULTIPLICATION,
    };

    Operation m_operation;

public:
    static const std::string ClassName;

    SignalMath() = default;
    ~SignalMath() override = default;

    unsigned numberOfParameters() override;
    bool parseParameters(wbt::BlockInformation* blockInfo) override;
    bool configureSizeAndPorts(wbt::BlockInformation* blockInfo) override;
    bool initialize(wbt::BlockInformation* blockInfo) override;
    bool output(const wbt::BlockInformation* blockInfo) override;
    bool terminate(const wbt::BlockInformation* blockInfo) override;
};

#endif // EXAMPLE_SIGNALMATH_H

```

The `./src/SignalMath.cpp` file is more interesting, let's split it in chunks.

#### Parameters

If you recall, BlockInformation is used by the Block for interfacing with the Engine. When this code is executed inside Simulink, the [`wbt::SimulinkBlockInformation`](https://robotology.github.io/wb-toolbox/doxygen/classwbt_1_1_simulink_block_information.html) implementation will perform the required operations calling Simulink APIs.

```cpp
#include "SignalMath.h"

#include <Core/Log.h>
#include <Core/Parameter.h>
#include <Core/Signal.h>

using namespace example;

unsigned SignalMath::numberOfParameters()
{
    // The base wbt::Block class needs parameters (e.g. the ClassName).
    // You must specify here how many more parameters this class needs.
    // Our example needs just one more: the operation to perform.
    return Block::numberOfParameters() + 1;
}

// This method should let BlockInformation know the parameters metadata
bool SignalMath::parseParameters(wbt::BlockInformation* blockInfo)
{
    // Initialize information for our only parameter
    int rows = 1;
    int cols = 1;
    std::string name = "Operation"; // This label is used to access the param later
    unsigned index = Block::numberOfParameters(); // Indices start from 0
    wbt::ParameterType type = wbt::ParameterType::STRING;

    // Create the parameter
    wbt::ParameterMetadata parameterMetadata(type, index, rows, cols, name);

    // Add the parameter metadata into the BlockInformation
    if (!blockInfo->addParameterMetadata(parameterMetadata)) {
        wbtError << "Failed to store parameter metadata";
        return false;
    }

    // Ask to the BlockInformation interface to parse the parameters and store them into
    // the m_parameters variable. This variable is contained in the wbt::Block class.
    bool paramParsedOk = blockInfo->parseParameters(m_parameters);

    // Return the outcome of the parameter parsing.
    // If the parsing fails, the execution stops.
    return paramParsedOk;
}
```

#### Configuration

The configuration of the Block is performed in the following steps:

1. The base class needs to be configured. It needs some parameters (e.g. the class name and the library name) and this call asks the Engine to parse them.
2. The ports of the Block need to be defined. In this example the size is set as dynamic so that it accepts signals with any width.
3. The [`wbt::BlockInformation::IOData`](https://robotology.github.io/wb-toolbox/doxygen/structwbt_1_1_block_information_1_1_i_o_data.html) class is used to store the data of all the ports. It is a `struct` containing two `std::vectors`.
4. The data is then sent to the Engine through the BlockInformation interface.

!!! info
    If needed, parameters can be accessed from this step. Refer to the initialization phase to understand how to gather them.

!!! warning "Signal size"
    Simulink has the support of inheriting the port size from the signal size, though use this feature only when strictly needed. In complex Simulink Models it might be difficult executing this size propagation, and fixing the Port size provides helpful constraints for the Engine.

!!! danger "Important"
    Be careful on memory allocations during this step. A temporary object is created only for configuration means, and then destroyed. All the allocated memory will be hereby deleted.

```cpp
bool SignalMath::configureSizeAndPorts(wbt::BlockInformation* blockInfo)
{
    // The base wbt::Block class need to be configured
    if (!wbt::Block::configureSizeAndPorts(blockInfo)) {
        return false;
    }

    // Create data about input and output ports.
    wbt::BlockInformation::PortData input1;
    wbt::BlockInformation::PortData input2;
    wbt::BlockInformation::PortData output;
    input1 = {/*portIndex=*/0, std::vector<int>{wbt::Signal::DynamicSize}, wbt::DataType::DOUBLE};
    input2 = {/*portIndex=*/1, std::vector<int>{wbt::Signal::DynamicSize}, wbt::DataType::DOUBLE};
    output = {/*portIndex=*/0, std::vector<int>{wbt::Signal::DynamicSize}, wbt::DataType::DOUBLE};

    // Populate a structure with the overall input / output data
    wbt::BlockInformation::IOData ioData;
    ioData.input.push_back(input1);
    ioData.input.push_back(input2);
    ioData.output.push_back(output);

    // Store this data into the BlockInformation
    if (!blockInfo->setIOPortsData(ioData)) {
        wbtError << "Failed to configure input / output ports";
        return false;
    }

    return true;
}
```

#### Initialization

!!! info
    Starting from this step, memory persistence is guaranteed.

In the initialization step, the input parameter that defines the operation is parsed. In this example the parameter is passed as a string. In the header a new `enum class Operation` was defined and here the related private member is properly initialized. Additional checks can be added, i.e. testing that both ports have the same width given that a dynamic size was previously assigned.

!!! info
    In this case there's no need to allocate memory. If your class has buffers to initialize, this is the right time to do it.

```cpp
bool SignalMath::initialize(wbt::BlockInformation* blockInfo)
{
    // The base wbt::Block class need to be initialized
    if (!Block::initialize(blockInfo)) {
        return false;
    }

    // Parse the parameters
    if (!SignalMath::parseParameters(blockInfo)) {
        wbtError << "Failed to parse parameters.";
        return false;
    }

    // Read the Operation parameter and store it as a private member
    std::string operation;
    if (!m_parameters.getParameter("Operation", operation)) {
        wbtError << "Failed to parse Operation parameter";
        return false;
    }

    // Check the content of the parameter
    if (operation == "Addition") {
        m_operation = Operation::ADDITION;
    }
    else if (operation == "Subtraction") {
        m_operation = Operation::SUBTRACTION;
    }
    else if (operation == "Multiplication") {
        m_operation = Operation::MULTIPLICATION;
    }
    else {
        wbtError << "Operation " << operation << " not supported";
        return false;
    }

    // Check that the size of the input signals match
    if (blockInfo->getInputPortWidth(/*index=*/0) != blockInfo->getInputPortWidth(/*index=*/1)) {
        wbtError << "Input signals widths do not match";
        return false;
    }

    return true;
}
```

#### Output

The `output` method is where the real algorithm is implemented. The Signals are firstly gathered from the Engine using their index. The classes `wbt::InputSignalPtr` and `wbt::OutputSignalPtr` are particular `typedef` of the [`wbt::Signal`](https://robotology.github.io/wb-toolbox/doxygen/classwbt_1_1_signal.html) class and they have the same methods. In the `for` loop the configured operation is performed and the result stored in the output signal.

!!! note
    Note that input signals are read-only. You can write data only to the output signals objects.

```cpp
bool SignalMath::output(const wbt::BlockInformation* blockInfo)
{
    // Get the input signals
    wbt::InputSignalPtr input1 = blockInfo->getInputPortSignal(/*index=*/0);
    wbt::InputSignalPtr input2 = blockInfo->getInputPortSignal(/*index=*/1);

    // Get the output signal
    wbt::OutputSignalPtr output = blockInfo->getOutputPortSignal(/*index=*/0);

    // Check the signal validity
    if (!input1 || !input2 || !output) {
        wbtError << "Signals not valid";
        return false;
    }

    // Check the width of the output signal.
    // This is recommended for dynamically sized signals.
    if (output->getWidth() != input1->getWidth()) {
        wbtError << "Output signal has a width of " << output->getWidth()
                 << " while input signals have a width of " << input1->getWidth();
        return false;
    }

    // Perform the given operation
    for (unsigned i = 0; i < output->getWidth(); ++i) {
        switch (m_operation) {
            case Operation::ADDITION:
                output->set(i, input1->get<double>(i) + input2->get<double>(i));
                break;
            case Operation::SUBTRACTION:
                output->set(i, input1->get<double>(i) - input2->get<double>(i));
                break;
            case Operation::MULTIPLICATION:
                output->set(i, input1->get<double>(i) * input2->get<double>(i));
                break;
        }
    }

    return true;
}
```

#### Terminate

Given the simplicity of our Block, the `terminate` step is a dummy implementation. This method is reported just for the sake of clarity. It can be omitted since `wbt::Block::terminate` already provides the same dummy implementation.

```cpp
bool SignalMath::terminate(const wbt::BlockInformation* /*blockInfo*/)
{
    return true;
}
```

### Implement the plugin factory

A plugin library usually contains multiple classes used for multiple Blocks. The `sharedlibpp` tool for plugins requires declaring what classes are part of the plugin. This operation is done in the `./include/Factory.cpp` file:

```cpp
#include "SignalMath.h"

// Class factory API
#include <shlibpp/SharedLibraryClassApi.h>

// Add the example::SignalMath class to the plugin factory
SHLIBPP_DEFINE_SHARED_SUBCLASS(SignalMath, example::SignalMath, wbt::Block);
```

The `SHLIBPP_DEFINE_SHARED_SUBCLASS` macro needs the following three arguments:

- A label used to extract the class from the plugin library
- The class of the block
- The base class of the block

The only interesting part here is the label. Keep this name in mind because we need to know it later in the Simulink section.

If everything works as expected, the library is now ready and can be compiled again:

```bash
cd build
cmake --build .
```

!!! tip
    Simulink will open this library at the beginning of the simulation loop and it needs to find it from the filesystem. Be sure that the `./build` folder is in the searching path of your dynamic linker. In the supported OSs you should add it to:
    - **Linux** `LD_LIBRARY_PATH`
    - **macOS** `DYLIB_LIBRARY_PATH`
    - **Windows** `Path`

## Matlab and Simulink

Once the C++ library is ready, the classes can be wrapped by a Simulink Block. If, as in this case, there's no existing Simulink Library to which the new block can be added, some extra step to create a new one is necessary.

We're going to store the files discussed in this section in the `./matlab` folder, obtaining at the end the following project structure:

```
.
├── CMakeLists.txt
├── include
│   └── SignalMath.h
├── matlab
│   ├── ExampleToolbox.slx
│   ├── Model.mdl
│   └── slblocks.m
└── src
    ├── Factory.cpp
    └── SignalMath.cpp
```

- `ExampleToolbox.slx` is the Simulink Library
- `slblocks.m` is a m script necessary for loading external Simulink Libraries into the Simulink Library Browser 
- `Model.mdl` is a Simulink Model for testing the Block

### Create an new Simulink Library

The first step is creating a new Simulink Library. Open the Simulink Start Page and create a Blank Library. Save it in `./matlab/ExampleToolbox.slx`.

![NewBlankLibrary](images/NewBlankLibrary.png)

In order to [populate the Simulink Library Browser](https://it.mathworks.com/help/simulink/ug/adding-libraries-to-the-library-browser.html), you need to create a `slblocks.m` file with the following content:

```matlab
function blkStruct = slblocks

Browser.Library = 'ExampleToolbox';
Browser.Name    = 'Example Toolbox';
Browser.IsFlat  =  0;

blkStruct.Browser =  Browser;
```

As explained in the [official documentation](https://it.mathworks.com/help/simulink/ug/adding-libraries-to-the-library-browser.html), we also need to modify a property of the library file:

```matlab
>> set_param('ExampleToolbox','EnableLBRepository','on');
```

If you followed these steps, the new library should be ready. Be sure that the `./matlab` folder is in the Matlab `path`, then open the Simulink Library Browser and press ++f5++. You should now see the empty _Example Toolbox_ entry.

!!! note
    In order to add a folder to the `path`, you can either browse it from the Matlab tree view making it the current folder, or executing `addpath('/path/of/the/folder')`.  Find more details at [What Is the MATLAB Search Path?](https://it.mathworks.com/help/matlab/matlab_env/what-is-the-matlab-search-path.html).

!!! info
    Every time you close the library, Matlab locks it. Press the lock icon in the bottom left corner to unlock the library.

### Add a block

Open the library and add a new _S-Function_ block with the following parameters:

![SFunctionBlock](images/SFunctionBlock.png)

- The _S-function name_ is the name of the generic S-Function that Whole Body Toolbox provides
- The _S-function parameters_ are the parameters passed to the S-Function. The first two are required by `wbt::Block`, and the third one is the parameter of our `example::SignalMath` class:
    - `'SignalMath'` is the label we used in `Factory.cpp`
    - `'ExampleToolbox'` is the name of the plugin library.  It must match the `OUTPUT_NAME` we assigned in `CMakeLists.txt`
    - `'Addition'` is the kind of operation we want this block to perform

Press OK and save the library. If you insert wrong information, like a wrong number of parameters or parameters with a wrong type or content, the `wbtError`s we added in the C++ class should provide more details.

### Create a test Simulink Model

Now it's time for finally testing all our work. Create a new _Blank Model_ and populate it with the following blocks:

![TestSimulinkModel](images/TestSimulinkModel.png)

Then press the Play icon, and the _Display_ connected to the block output should show the result of the addition.

### Create a block mask

The type of the operation is defined as a parameter of the _S-Function_ block. It is not very intuitive changing it in this way. Beyond the effort of changing the string, we should remember exactly what parameters the block accepts.

This limitation can be overcome masking the block, that means providing a GUI to the block. Right-click the block from the _Example Toolbox_ library and press `Mask > Create Mask`.

In the `Icon & Ports` tab, fill the `Icon drawing commands` with:

```matlab
disp('Signal Math')

port_label('input',1,'Input 1')
port_label('input',2,'Input 2')

port_label('output',1,'Output')
```

Then, in the `Parameters & Dialog` tab, add a `Popup` and fill it obtaining the following status:

![BlockMask](images/BlockMask.png)

!!! note
    Note that the evaluate attribute has been unchecked. When a Popup is not evaluated, its variable (`operation` in this case) will contain the string of the selected option. Instead, if it is evaluated, it contains a 0-based index.

Save the library and substitute the new Block in the Simulink Model. You can now select the operation double-clicking the block and changing the popup menu entry.

![TestSimulinkModelWithMask](images/TestSimulinkModelWithMask.png)

## Final comments

Whole Body Toolbox provides great abstraction capabilities for wrapping C and C++ algorithms to Simulink Blocks. You don't need to be a Simulink expert for this kind of operation, all the machinery is hidden under the hood.

Possibilities are endless. Despite Whole Body Toolbox was originally designed for robotic applications, it is interesting discovering how many new toolboxes can be created on top of it. The `v4` release provides the first experimental support to external usage. The toolbox APIs are already mature and reached an acceptable stability after being developed, tested, and refined for many years.

We are eager to hear your feedback in order to have a great `v1.0` release of the standalone toolbox core! If you find bugs or want to propose enhancements, please fill a [new Issue](https://github.com/robotology/wb-toolbox/issues?q=is%3Aissue+is%3Aopen+sort%3Aupdated-desc).
