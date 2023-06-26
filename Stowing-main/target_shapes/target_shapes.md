# Documentation: State Format

## File Format: `state.npz` and `rgb_agentview.png`

The state data are stored in NumPy's binary format (`*.npz`) which allows for the storage of array-like data structures in a disk-efficient and load-efficient manner.

Each state file follows the naming convention `XXX_state.npz` where `XXX` is a three-digit identifier starting from `000`. For example: `000_state.npz`, `001_state.npz`, etc.

The image file follows the same naming convention `XXX_rgb_agentview.png` where `XXX` is a three-digit identifier starting from `000`. For example: `000_rgb_agentview.png`, `001_rgb_agentview.png`, etc.

## Loading the Data

The state files can be loaded using the `np.load` function from the NumPy library in Python. This function reads the binary `.npz` file and returns a dictionary-like object of numpy arrays. 

The data is loaded with the following command:

```python
dic = np.load(state_file, allow_pickle=True)
```

## State Data Structure
Once loaded, the `dic` object is a dictionary-like structure where each key corresponds to a specific state data.

The `dic["obs"]` key provides access to the main state information. This data is itself a dictionary with the following keys:

- cubeA_pos: The position of Cube A.
- cubeA_quat: The orientation of Cube A, represented as a quaternion.
- cubeB_0_pos: The position of the first instance of Cube B.
- cubeB_0_quat: The orientation of the first instance of Cube B, represented as a quaternion.
- cubeB_1_pos: The position of the second instance of Cube B.
- cubeB_1_quat: The orientation of the second instance of Cube B, represented as a quaternion.
- And so on, for as many instances of Cube B as exist in the data.
- Each of these keys accesses a numpy array that contains the actual state data for the corresponding object.

For example, you can access the position of Cube A with `dic["obs"]["cubeA_pos"]`.

Note: The position is typically a 3D vector (x, y, z) representing the coordinates in space, and the quaternion is a 4D vector (x, y, z, w) representing the orientation in 3D space.


