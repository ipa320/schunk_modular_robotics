^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package schunk_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.5.6 (2014-08-27)
------------------

0.5.5 (2014-08-26)
------------------
* merge with hydro_dev
* spaces
* obsolete dependency
* Gazebo only detect links with inertia
* fix pg70 property name
* consistency changes due to new transmission format
* consistency changes due to latest gazebo tag format
* remove unused meshes
* fix arm_1_link mesh + use collision stl
* merge with latest 320 updates
* pg70 description and fixed origins for lwa4d
* pg70 setup
* mesh file
* Coloured mesh files
* use meshes with reduced vertices and reduce joint_limits for moveit_config
* use meshes with reduced vertices and reduce joint_limits for moveit_config
* cleaning up after testing
* lwa4d: fixed offsets error
* inertias for arm_1_link
* new meshes
* temporary modifications for easier controller tuning
* use inertias and physic properties from controller_tuning tests
* modifications in tranmissions, removing of bumpers and small cleanups
* Renamed links and added shoulder model
* mesh for shoulder added
* origin for collision model is in the center of the box
* pg70 collada model
* wrong lenght
* materials should not be loaded in the components urdf
* beautify mesh files
* Merge pull request `#81 <https://github.com/ipa320/schunk_modular_robotics/issues/81>`_ from ipa320/hydro_release_candidate
  bring back changes from Hydro release candidate
* New maintainer
* Redefined color LightGrey
* Contributors: Alexander Bubeck, Felix Messmer, Nadia Hammoudeh García, Tim Fröhlich, ipa-cob3-8, ipa-fxm, ipa-nhg

0.5.4 (2014-03-28)
------------------

0.5.3 (2014-03-27)
------------------
* Merge branch 'hydro_dev' into hydro_release_candidate
* install tags
* Merge branch 'hydro_dev' of github.com:ipa320/schunk_modular_robotics into hydro_dev
* some catkin_lint
* Contributors: Florian Weisshardt, ipa-fxm

0.5.2 (2014-03-27)
------------------

0.5.1 (2014-03-20)
------------------
* update xacro file format
* merge with groovy
* meshes files for lwa4p_extended
* added meshes files for lwa4p_extended
* tested on real arm
* 27.02. current status
* new meshes
* Tested on real arm
* Fixed arm_7_joint position
* bring in groovy updates
* Fix mesh files for lwa4d
* Added calibration arm_1_calibrationg_rising
* description for the a new lwa4p version
* Adjust lwa limits
* adapt limits for lwa and lwa_extended
* update lwa4d description
* fix arm_6_joint
* update transmission for schunk components
* update xmlns + beautifying
* transmission for new simulation controllers
* 2DOF Hack for finger
* fix fingers
* update pg70
* add pg70 gripper
* Corrected xacro files for hydro.
* Removed instalation of gazebo folder which doesn't exist.
* Updated lwa4d description
* Created lwa4d urdf model
* remove install command for gazebo subdirectory
* merge
* More changes from powerball to lwa4d
* Changed from powerball to lwa4p
* remove mesh file generation
* installation stuff
* remove generation of mesh files
* Initial catkinization. Still a linking error in sdh lib.
* some more fixes and cleaning up for gazebo simulation
* fix sdh description according to new gazebo format
* fix blue color
* Groovy migration
* adjust color settings
* change to light grey
* Reorganized list of colors
* Redefined colors
* Merge branch 'master' of github.com:ipa320/schunk_modular_robotics
* update limits for lwa
* Renamed the colors
* Redefined Schunk component colors for gazebo and rviz
* merge
* Fixed arm_0_link origin
* modified mesh files
* powerball stl changes
* Revised powerball  urdf and mesh files
* New meshes files for powerball
* fixes for powerball arm urdf
* New colors for powerball in simulation
* changed stl files not using solid
* changed stlb links to stl
* New model schunk powerball
* fix lwa
* renamed to schunk names
* renamed arm to lwa
* rename from arm to lwa
* renamed arm to lwa
* moved schunk desc
* Contributors: Alexander Bubeck, Denis Štogl, Frederik Hegger, IPR-SR2, Thiago de Freitas, abubeck, fmw, ipa-cob3-5, ipa-cob3-6, ipa-fmw, ipa-fxm, ipa-nhg, ipa-tys, rmb-om
