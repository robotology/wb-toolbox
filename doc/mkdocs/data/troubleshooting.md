## Problems finding libraries and `libstdc++`

In case Matlab has trouble finding a specific library, a workaround is to launch it preloading the variable `LD_PRELOAD` (or `DYLD_INSERT_LIBRARIES` on macOS) with the full path of the missing library.

On Linux you might have trouble with `libstdc++.so` since Matlab comes with its own. To use your system's `libstdc++` you would need to launch Matlab with:

`LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6 matlab`

The current version on Ubuntu 16:04 is `libstdc++.so.6`, make sure that this is the case also on your OS.

You could additionally create an alias to launch Matlab this way:

`alias matlab_wbt="cd ~/Documents/MATLAB && LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6 matlab"`

Another solution involving the `.matlab7rc.sh` file can be found in [`#141`](https://github.com/robotology/codyco-superbuild/issues/141#issuecomment-257892256).

## `YARP` not installed in the system default directory

In case you compiled `YARP` in a directory different from the system default one and you are not using RPATH, you need to tell to MATLAB the location in which to find the shared libraries for `YARP`. If you launch MATLAB from command line, this task is already done for you by `bash` (if you edited `.bashrc`). If you launch MATLAB from the UI (e.g. on macOS by double clicking the application icon) you need to further add the variables in `${MATLAB_ROOT}/bin/.matlab7rc.sh` by first doing

```bash
    chmod +w .matlab7rc.sh
```

Then looking for the variable `LDPATH_SUFFIX` and assign to every instance the contents of your `DYLD_LIBRARY_PATH`. Finally do:

```bash
    chmod -w .matlab7rc.sh
```

The error message you get in this case might look something like:

```bash
Library not loaded: libyarpwholeBodyinterface.0.0.1.dylib
Referenced from:
${CODYCO_SUPERBUILD_DIR}/install/mex/robotState.mexmaci64
```
