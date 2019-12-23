# AMapper

AMapper is a library that provides an abstraction to OccupancyGrids and Costmaps. The library includes the following:

* A `Grid` class which easily converts between ros's `nav_msgs::OccupancyGrid`.
* Commonly used algorithms such as blurs, ray tracers etc
* Environmental Filters to remove things such as noise from rain and movement (WIP)