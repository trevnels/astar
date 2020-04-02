# astar
A Python implementation of the A* pathing algorithm to find the best path from the right side to the left side of the mountain range based on a heightmap.
"Best" means many things in this context. The algorithm takes into account raw distance, elevation change/steepness, and tries to avoid crossing large rivers if possible.

### Sample Output
![Sample Output](https://raw.githubusercontent.com/trevnels/astar/master/sample_output.jpg)

### Data Sources
- Height data from [terrain.party](https://terrain.party/)
- Water data from [USGS Streamer](https://txpub.usgs.gov/DSS/streamer/web/)