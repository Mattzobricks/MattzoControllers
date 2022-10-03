How to configure your MattzoController
**************************************
There are two files that you need to configure the MattzoController.

1. network_config.h
This file contains network configuration settings like the SSID of your WIFI network etc.
The information in this file is usually IDENTICAL for all your MattzoControllers.

2. controller_config.h
This file contains the configuration for a specific MattzoController.
This file is usually DIFFERENT for each of your MattzoControllers.


Setup
*****
Follow the following steps to setup the configuration:

1.
After you have checked out the repository, you must create a subfolder "my" in the "conf" directory.
This directory is ignored by git. Whatever you do here will not count as a changed file in git.

2.
There are default configuration files in the subdirectory "default". Copy the two files into the new "my" directory.

3.
Adapt the two configuration files to your requirements.


Example files
*************
In addition to the default files, there is a bunch of example files that cover different wirings and applications.
Check them out if you have problems to setup more complicated stuff all by yourself.


Best practice for handling multiple configurations
**************************************************
While network_config.h is usually the same for all of your MattzoControllers, controller_config.h often 
needs to be exchanged when you upload the firmware for a specific controller.

It is a good practice to have a folder somewhere with the configuration files that you frequently need.
As the "my" directory is ignored by git, it is a good practice to place them in this directory.
Before compiling, simply copy the specific configuration file over the my/controller_config.h file.
