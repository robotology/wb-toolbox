This page contains some tricks (or not so obvious features) needed for creating Blocks in simulink

### Use of popup mask parameters
If you need to use a popup mask parameter (i.e. a dropdown list) in you block you should notice the following thing:

- If **EVALUATE** is set to **TRUE** then the variable associated with the parameter will contain the **INDEX** (1-based) of the selection.
- If **EVALUATE** is set to **FALSE** then the variable associated with the parameter will contain the **ACTUAL VALUE** of the selection.

[Original source](http://www.mathworks.com/matlabcentral/answers/95199-how-can-i-assign-the-actual-value-of-a-selected-popup-to-my-mask-variables-in-simulink-6-6-r2007a)

### Self-modifiable blocks
If you block needs to modify its options (via matlab callbacks associated with a parameter), e.g. when changing the parameters mask depending on another parameter, or when changing the number of input/outputs, be sure to check (on) the "Mask Editor -> Initialization -> Allow library block to modify its contents"

![](https://cloud.githubusercontent.com/assets/2189180/11606884/19fd2f5a-9b32-11e5-9f1e-89347a6df895.png)

### Create a new block
This is more heuristic than the "correct method", but it has been proved to work.

Create a new Simulink Model and start creating the block. You can also duplicate an existing block from the Library. In this second case be sure to do the following steps:

- Right click on the block -> Library Link -> Disable Link
- Right click on the block -> Library Link -> Break Link

At this point finish your work, compile the model and only at this point add the block to the library, so that its aspect will reflect the real one.

#### Note:
The step of creating the model separately is needed until we find out how to update the aspect of the block directly in the Library.
Further more, this process can hinder the already existing links to the block.
