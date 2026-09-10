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
//import"Main.gaml"
global{
	file roof_texture <- file('../includes/building_texture/roof_top.jpg');		
	list textures <- [file('../includes/building_texture/texture1.jpg'), file('../includes/building_texture/texture2.jpg'), file('../includes/building_texture/texture3.jpg'), file('../includes/building_texture/texture4.jpg'), file('../includes/building_texture/texture5.jpg'), file('../includes/building_texture/texture6.jpg'), file('../includes/building_texture/texture7.jpg'), file('../includes/building_texture/texture8.jpg'), file('../includes/building_texture/texture9.jpg'), file('../includes/building_texture/texture10.jpg')];
}
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
	float depth;
	
	file texture;
	init{ 
		depth<-	(rnd(100) / 100) * (rnd(100) / 100) * (rnd(100) / 50 * shape.perimeter/100) * 10 + 10;
	} 
	aspect default {
		draw shape color: #grey;
	}

	aspect textured {
		draw shape texture:[roof_texture.path, texture.path] depth: depth color: rnd_color(255);
	}
}



