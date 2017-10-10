# Cruiser-OnboardROS
## Introduction
Onboard programs for Cruiser project.

**For more details, please refer to [Cruiser-Documentation](https://github.com/CruiserProject/Cruiser-Documentation)**
## Collaborators
- [@Cuijie12358](https://github.com/Cuijie12358)
- [@hanzheteng](https://github.com/hanzheteng)
- [@XiangqianMa](https://github.com/XiangqianMa)
- [@ShoupingShan](https://github.com/ShoupingShan)
- [@finaldong](https://github.com/finaldong)
## Functions
- Auto takeoff and go home (optional)
- Movement control
- Executing command from mobile terminal
- Object detection and tracking
- Autonomous landing by Hough circle detection
## Platform
- [DJI Matrice100](http://www.dji.com/matrice100)
- [Manifold](http://www.dji.com/manifold) (Ubuntu 14.04)
- [Robot Operating System](http://wiki.ros.org/) (indigo)
- [Onboard-SDK-ROS](https://github.com/dji-sdk/Onboard-SDK-ROS)(v3.2.2)
## Notice
`.config` directory should not have been placed in this metapackage. It is just a demo to show that how to autostart when system boots. There are three files in this directory. The right usage is as follows:
1. modify `launch.desktop` and place it in `~/.config/autostart/`;
2. modity `root.sh` (especially replace the passward) and put it on the Desktop (or other path); (if you do not need root privilege, you can skip this step and set the path in `launch.desktop` directly towards `launch.sh`)
3. modity `launch.sh` and place it on the Desktop (or other path);
4. reboot your system and see what happens.

