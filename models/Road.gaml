/**
* Name: Species
* Based on the internal empty template. 
* Author: PC
* Tags: 
*/


model Road
import "Intersection.gaml"
import "Roi.gaml"
import "Vehicles.gaml"
import"Main.gaml"

species road skills: [road_skill] {
	int lanes;
	int num_lanes;
	float width;

	
	init {
		if (lanes = nil or lanes <= 0) {
			lanes <- 2; 
		}
		num_lanes <- lanes; 
		if (width = nil or width <= 0.0) {
			width <- lanes * 3.5; 
		}
	}

	
	aspect default {
		draw shape + (width / 2) color: #gray;
	}

	
	aspect heatmap_base {
		draw shape + (width / 2) color: rgb(35, 55, 90);
	}

	
	aspect heatmap_heat {
		float h <- (road_heat contains_key self) ? road_heat[self] : 0.0;
		if (h >= 0.5) {
			draw shape + (width / 2) color: rgb(220, 30, 30);
		} else if (h >= 0.2) {
			draw shape + (width / 2) color: rgb(255, 190, 0);
		} else if (h >= 0.06) {
			draw shape + (width / 2) color: rgb(40, 200, 80);
		}
	}
}

species building {

	aspect default {
		draw shape color: #grey;
	}

}



