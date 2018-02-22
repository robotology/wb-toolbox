## Environment variables

The following environment variables must be specified:

- `YARP_ROBOT_NAME`
- `YARP_DATA_DIRS`

!!! tip "Tip: launch Matlab from the command line"
    The environment variables stored in the `.bashrc` or `.bash_profile` files are automatically loaded. Store here additional variables if needed.

!!! tip "Tip: launch Matlab from the desktop launcher"
    You can store environment variables from the Matlab command line using the `setenv` function.

## Creating a model

Before using or creating a new model keep in mind that `WB-Toolbox` is discrete in principle and your simulation should be discrete as well. By going to `Simulation > Configuration Parameters > Solver` you should change the solver options to `Fixed Step` and use a `discrete (no continuous states)` solver.

In order to start dragging and dropping blocks from the `WB-Toolbox`, open the _Simulink Library Browser_ and search for `Whole Body Toolbox` in the tree view.
