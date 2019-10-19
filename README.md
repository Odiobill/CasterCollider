# CasterCollider
A lightweight alternative to the standard Unity's WheelCollider, with no spring, and that can be rotated in any direction.

## Usage
Add the CasterCollider script to any wheel. It doesn't add any spring force, so you need to add another actual collider and modify the radius or sizes accordingly.
You can check the Gizmos to see how to set up the ground-detection check and all the required parameters.
Then you can just change the public properties to apply steer, torque, and braking like with the original WheelCollider.

## Credits
Based on the WheelColliderSource project by Nic Tolentino.

