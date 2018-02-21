The following steps are necessary in order to add additional blocks to the Library.

#### C++

##### Generic Block
- Inherit from `Block` class
- Implement the `numberOfParameters()` function returning the number of parameters your block takes
- If the parameter is tunable (i.e. it can be changed during the simulation) implement `parameterAtIndexIsTunable`: by default parameters are not tunable
- Implement the `configureSizeAndPorts` method to properly set the number and type of input and output ports
- Implement the `initialize` method to perform any initialization your block requires
- Implement the `finilize` method to cleanup any resources your block requested in the `initialize` method
- Implement the `output`method to perform the actual operations

You can access the block parameters by using the usual mex C functions. **NOTE** your parameters are 1-based numbered, **NOT** 0-based. So the first parameter is at index 1, etc...

##### WBI-based Block
If you need to implement a WBI-based block it is highly advisable that you inherit from `WBIBlock` class.
This base class already implements a lot of functionalities and it is highly probable you need to just implement the `output` function.

The `WBIBlock` base class already provide you the following features:

- The number of parameters is already set to `4` (`robot name`, `model name`, `wbi filename`, `wbi joint list`) and they are correctly parsed.
- An instance of the `Yarp WholeBody interface` is configured, initialized, and properly released in the `finilize` method.
- The Yarp network properly initialized and terminated.
- You can obtain a reference to the singleton WB Interface wrapper by calling the static method `WBInterface::sharedInstance`.

**Note** Additional parameters you specify starts from the index `5`.

##### Notes on implementation

- During `configureSizeAndPorts` you should not allocate any memory or save any data because the object will not persist after the method call. The correct place is the `initialize` method.
- Every function takes as last parameter an `Error` object. It can be `NULL`, so check before dereferencing the pointer.

#### Final steps
Independently of the type of block you implemented some more steps are required to properly add the block:

- `CMake`: of course you should add the files to the CMake project. You can you the macro provided by this project:
```cmake
configure_block(BLOCK_NAME ${HUMAN_READABLE_DESCRIPTION}
    LIST_PREFIX WBT
    SOURCES ${CPP_FILES}
    HEADERS ${HEADER_FILES})
```
where
  - `${HUMAN_READABLE_DESCRIPTION}` is a string used in the group folder (for projects which support it),
  - `${CPP_FILES}` is a list of `.cpp` files needed by your block
  - `${HEADER_FILES}` is a list of `.h` files needed by your block

- Add you main header to the `toolbox.h` file
- Add the code needed for the creation of your class in `factory.cpp`, `Block::instantiateBlockWithClassName` method. The string passed as argument is the one you specify in the S-Function block in Simulink (see next section)

#### Simulink

- Add an S-Function block
- Specify as s-function name `WB-Toolbox`
- Add the parameters:
  - The first parameter is the name of the class, e.g. `YarpRead`
  - If you are creating a WBI-based block you have to specify 4 additional parameters: `robot name`, `model name`, `YarpWBI configuration file`, `YarpWBI Joint list`
  - Add any additional parameter required by your block
- Create a Subsystem and add a Mask to it

**Note** you should use variables for the S-Function parameters and you should specify them in the mask parameters.
**Note** you can also supply default values for the variables by adding code in the `Block->Properties->Callbacks->Init`

You can skip most of the aforementioned steps by duplicating an already existing block. Just be sure you are duplicating it and not creating an alias.
