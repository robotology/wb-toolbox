## Environment variables

If you launch Matlab from command line, it inherits the configuration from the `.bashrc` or `.bash_profile` file. If you launch Matlab directly from the GUI you should define this variables with the Matlab function `setenv`:

- `YARP_ROBOT_NAME`
- `YARP_DATA_DIRS`

## Creating a model

Before using or creating a new model keep in mind that `WB-Toolbox` is discrete in principle and your simulation should be discrete as well. By going to `Simulation > Configuration Parameters > Solver` you should change the solver options to `Fixed Step` and use a `discrete (no continuous states)` solver.

In order to start dragging and dropping blocks from the `WB-Toolbox`, open the Simulink Library Browser and search for `Whole Body Toolbox` in the tree view.
