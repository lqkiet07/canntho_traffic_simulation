/**
* Name: Vehicles
* Based on the internal empty template. 
* Author: PC
* Tags: 
*/


model Vehicles
import "Road.gaml"
import "Intersection.gaml"
import "Roi.gaml"
import "Main.gaml"

/* Insert your model definition here */
species vehicle skills: [driving] {

	//obj for vehicle width - chieu ngang phuong tien, duoc ghi de boi tung loai con
	float vehicle_width <- 1.0;
	float total_delay <- 0.0;

	init {
		right_side_driving <- true;
		safety_distance_coeff <- 3.0;
	}
//	reflex debug_roi {
//        // Tìm xem xe có chạm bất kỳ roi_lane nào không
//        list<roi_lane> touched_rois <- roi_lane where (each.shape intersects self.shape);
//        if (!empty(touched_rois)) {
//            write "XE " + name + " ĐÃ CHẠM ĐƯỢC VÀO LÀN: " + (touched_rois[0]).name;
//        }
//    }

	road previous_road <- nil;
	//find road
	reflex move {
		if (final_target = nil or (location distance_to final_target.location < 5.0)) {
			do die;
		}
		else{
			// Accumulate stopped delay if speed is very low (< 1 km/h = 0.28 m/s)
			if (speed < 0.28) {
				total_delay <- total_delay + step;
			}

			if (current_path = nil) {
				do compute_path graph: road_network target: final_target;
				if (current_path = nil) {
					// remove if no path available
					//write "vehicle killed due to no path";
					do die;
					return;
				}
			}	
			
			// obj for throughput tracking - dem so xe roi khoi nga tu
			if (current_road != previous_road) {
				if (previous_road != nil) {
					intersection crossed_node <- intersection(road_network target_of road(previous_road));
					if (crossed_node != nil and crossed_node.is_traffic_signal) {
						// Only count throughput if the previous road was NOT an internal link within the cluster
						intersection source_node <- intersection(road_network source_of road(previous_road));
						bool is_internal_link <- (source_node != nil and source_node.is_traffic_signal);
						
						ask crossed_node { 
							if (!is_internal_link) {
								throughput_count <- throughput_count + 1; 
							}
							total_delay_in_cycle <- total_delay_in_cycle + myself.total_delay;
						}
						total_delay <- 0.0; // reset vehicle delay after crossing
					}
				}
				previous_road <- road(current_road);
			}
			// =========================================================================
			// TRAFFIC LIGHT LOGIC (UNIFIED FOR ALL ALGORITHMS)
			// =========================================================================
			bool should_stop <- false;
			bool should_slow <- false;
			float dist_to_light <- #infinity;

			traffic_light_visual light_ahead <- traffic_light_visual closest_to self;
			dist_to_light <- (light_ahead != nil) ? self distance_to light_ahead : #infinity;

			if (light_ahead != nil and light_ahead.state = "red") {
				float angle_to_light <- float(self towards light_ahead);
				float diff_ang <- abs(angle_to_light - heading) mod 360.0;
				if (diff_ang > 180.0) { diff_ang <- 360.0 - diff_ang; }
				if (diff_ang < 90.0) {
					if (dist_to_light < 7.5)  { should_stop <- true; }  // hard stop zone (increased to 7.5m for turning lanes)
					else if (dist_to_light < 20.0) { should_slow <- true; } // braking zone (increased to 20.0m)
				}
			}

			if (should_stop) {
				speed <- 0.0;
			} else if (should_slow) {
				float brake_ratio <- (dist_to_light - 7.5) / 12.5; // 1.0 far, 0.0 at stop line
				speed <- max_speed * brake_ratio * 0.4;
				do drive;
			} else {
				if (speed = 0.0) { speed <- max_speed * 0.5; }
				do drive;
			}
		}
		
	}
// ROI detection is now handled by roi_lane self-scan (removed vehicle-push approach)
	point compute_position {
		if (current_road != nil) {
			float road_width <- road(current_road).width;
			int n_lanes <- road(current_road).num_lanes;
			float lane_w <- road_width / n_lanes;

			
			float dist_from_left_edge <- (n_lanes - lowest_lane - 0.5) * lane_w;
			float center_offset <- dist_from_left_edge - (road_width / 2);
			float final_dist <- -center_offset;
			point shift_pt <- {cos(heading + 90) * final_dist, sin(heading + 90) * final_dist};
			return location + shift_pt;
		} else {
			return location;
		}

	}

}

species motobike parent: vehicle {

	init {
		vehicle_length <- 1.9;
		max_speed <- rnd(40.0, 60.0) #km / #h;
		speed <- max_speed;
	}

	aspect default {
		point pos <- compute_position();
		draw box(1.9, 0.7, 1.2) color: #brown rotate: heading at: {pos.x, pos.y, 0.5};
	}

	// heatmap point display
	aspect heat_dot {
		draw circle(10) color: rgb(255, 60, 0, 80);
	}

}

species car parent: vehicle {

	init {
		vehicle_length <- 4.5;
		max_speed <- rnd(30.0, 50.0) #km / #h;
		speed <- max_speed;
	}

	aspect default {
		point pos <- compute_position();
		draw box(4.5, 1.8, 1.5) color: #purple rotate: heading at: {pos.x, pos.y, 1};
	}

	// larger heatmap point for car
	aspect heat_dot {
		draw circle(13) color: rgb(255, 60, 0, 90);
	}

}

species truck parent: vehicle {

	init {
		vehicle_length <- 8.0;
		max_speed <- rnd(20.0, 40.0) #km / #h;
		speed <- max_speed;
	}

	aspect default {
		point pos <- compute_position();
		draw box(8.0, 2.4, 2.8) color: #pink rotate: heading at: {pos.x, pos.y, 1.5};
	}

	// largest heatmap point for truck
	aspect heat_dot {
		draw circle(16) color: rgb(255, 60, 0, 100);
	}
}

//species ambulance parent: vehicle {
//	init {
//		vehicle_length <- 5.0;
//		//obj for vehicle width
//		vehicle_width <- 2.0; // chieu ngang xe cuu thuong
//		max_speed <- rnd(50.0, 70.0) #km / #h;
//		speed <- max_speed;
//	}
//
//	aspect default {
//		point pos <- compute_position();
//		draw box(5, 2, 2.5) color: #white rotate: heading at: {pos.x, pos.y, 1.25};
//		// den chop do
//		draw box(1, 2.1, 0.5) color: #red rotate: heading at: {pos.x, pos.y, 2.5};
//	}
//
//	aspect heat_dot {
//		draw circle(15) color: rgb(255, 0, 0, 150);
//	}
//}
