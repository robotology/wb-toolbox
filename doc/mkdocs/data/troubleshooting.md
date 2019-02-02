## Problems finding libraries and `libstdc++`

In case Matlab has trouble finding a specific library, a workaround is to launch it preloading the variable `LD_PRELOAD` (or `DYLD_INSERT_LIBRARIES` on macOS) with the full path of the missing library.

On Linux you might have trouble with `libstdc++.so` since Matlab comes with its own. To use your system's `libstdc++` you would need to launch Matlab with:

```bash
LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6 matlab
```

The current version on Ubuntu 16:04 is `libstdc++.so.6`, make sure this is the case also on your OS.

!!! tip
    You could additionally create an alias to launch Matlab this way:
    `alias matlab_wbt="cd ~/Documents/MATLAB && LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6 matlab"`

!!! info
    Another solution involving the `.matlab7rc.sh` file can be found in https://github.com/robotology/codyco-superbuild/issues/141#issuecomment-257892256.
