# CasterCollider
A lightweight alternative to the standard Unity's WheelCollider, with no spring, and that can be rotated in any direction.

## Usage
Add the CasterCollider script to any wheel. It doesn't add any spring force, so you need to add another actual collider and modify the radius or sizes accordingly.
You can check the Gizmos to see how to set up the ground-detection check and all the required parameters.
Then you can just change the public properties to apply steer, torque, and braking.

## Issues
Formulas implemented here are not really precise while being adequate for most of the basic needs. You may notice it in particular when trying to steer with wheels that don't apply any torque at the same time. Hopefully you, dear user, can help us out improving this. :)

## Credits
Forces are applied using the same formulas implemented here:
http://wiki.unity3d.com/index.php/VWheelCollider

