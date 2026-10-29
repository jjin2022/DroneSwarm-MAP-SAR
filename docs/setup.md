cat > docs/setup.md << 'EOF'
# Working setup

- Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic
- PX4-Autopilot v1.17.0
- px4_msgs branch release/1.17
- Micro XRCE-DDS Agent v2.4.3
- Versioned topic: /fmu/out/vehicle_status_v1
- Do not run `make distclean`; clear builds with `rm -rf build`
EOF

