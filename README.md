# Kinematics Hub

Create a modern, interactive 3D Kinematics Dashboard for visualizing and analyzing Forward Kinematics (FK) and Inverse Kinematics (IK) of a robotic arm.

Main Goal

The dashboard should display a realistic 3D robotic arm and dynamically visualize how the robot's joint angles and end-effector position change during Forward and Inverse Kinematics calculations.

Dashboard Layout

Create a clean professional dashboard with:

3D Robot Visualization — Main Panel

Display the robotic arm in a 3D coordinate system.

Show X, Y, and Z axes.

Show each robot link and joint clearly.

Display the end-effector position.

Allow the user to rotate, zoom, and pan the 3D view.

The robot should update in real time when parameters change.

Forward Kinematics Panel

Provide input controls for joint angles:

θ1

θ2

θ3

Additional joints if required.

When the user changes joint angles, calculate the end-effector position using Forward Kinematics.

Update the 3D robot immediately.

Display:

X position

Y position

Z position

Joint angles

Transformation matrix.

Inverse Kinematics Panel

Allow the user to enter a target end-effector position:

Target X

Target Y

Target Z

Calculate the required joint angles using Inverse Kinematics.

Move the 3D robot to the calculated position.

Display:

Calculated θ1

Calculated θ2

Calculated θ3

Target position

Actual end-effector position.

Clearly indicate whether the target position is reachable.

3D Graph / Coordinate System

Display a proper 3D Cartesian coordinate system.

Add grid lines and axis labels.

Show the robot's trajectory/path when the end-effector moves.

Use different visual indicators for:

Robot links

Joints

End-effector

Target position

End-effector trajectory.

Real-Time Status
Display a status section showing:

FK / IK mode

Target position

Current end-effector position

Position error

Joint angles

Reachability status.

Interaction

The user should be able to:

Switch between Forward Kinematics and Inverse Kinematics.

Change joint angles using sliders or numeric inputs.

Enter target X/Y/Z coordinates.

See the robotic arm move interactively.

Reset the robot to its default position.

Rotate and zoom the 3D visualization.

Toggle trajectory/path visualization.

Visual Design

Use a professional engineering/robotics dashboard design:

Dark modern interface.

Large 3D visualization area.

Cards/panels for FK and IK controls.

Clear numerical values.

Smooth animations when the robot moves.

Responsive layout.

Use suitable colors to distinguish the robot, target point, axes, and trajectory.

Make the interface look like a real robotics simulation/control dashboard rather than a basic form.

Technical Requirements

Use a web-based 3D visualization framework such as:

React

Three.js / React Three Fiber

JavaScript or TypeScript

Tailwind CSS

Implement the actual mathematical calculations for:

Forward Kinematics:
Use transformation matrices / DH parameters to calculate the end-effector pose from the joint angles.

Inverse Kinematics:
Calculate joint angles required to reach the requested X/Y/Z target.

Keep the FK and IK calculations modular so the robot configuration and DH parameters can easily be changed later.

Important

The 3D graph must be the central component of the dashboard. Do not create only a 2D chart. The user must be able to visually see the robotic arm, coordinate axes, target point, end-effector, and movement in a true interactive 3D environment.

Build the dashboard as a functional prototype with working FK/IK calculations and real-time 3D visualization.

This project was built with [Lovable](https://lovable.dev).

**Live app**: https://robot-viz-studio.lovable.app

## Build with Lovable

Continue developing this project in the [Lovable editor](https://lovable.dev/projects/8a37fffc-44ef-4107-b6ad-8b013eed61ed).

- **Ship faster**: describe what you want to build and Lovable handles the code.
- **Stay in sync**: every change made in Lovable is committed straight to this repository.
- **Full ownership**: this code is yours. Push to `main` on GitHub and your changes sync back into Lovable, ready for your next prompt.

## Development

Prefer working locally? You need Node.js and npm — [install with nvm](https://github.com/nvm-sh/nvm#installing-and-updating).

```sh
git clone <this-repository-url>
cd <repository-name>
npm i
npm run dev
```
