# Sources and attribution

- `src/limo_car`: subset of the existing KUDOS_scalecar workspace's LIMO model, authored by Zhui Li (TU Braunschweig); original comments and upstream README retained. Original package declares Apache-2.0. This export retains only the simulator assets needed by the new launch, corrects ROS2 package dependencies, and removes the legacy ROS1 control plugin and invalid optional steering-wheel joint entry.
- Workspace source: https://github.com/hotsixthekingrush/KUDOS_scalecar
- `src/kudos_arena`: KUDOS Autorace 2026 practice arena, generators and traffic-light controller.
- Track drawing and sign images were supplied for the team's competition preparation. No additional redistribution license is asserted for those images.
- This is a practice environment, not an official competition simulator or final track specification.
