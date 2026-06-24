/**
* Name: NewModel
* Based on the internal empty template. 
* Author: smth
* Tags: 
*/

model Traffic
import "Road.gaml"
import "Intersection.gaml"
import "Roi.gaml"
import "Vehicles.gaml"


/* Insert your model definition here */
global {
	file road_shp <- shape_file("../includes/road 4.shp");
	file building_shp <- shape_file("../includes/building.shp");
	// Dynamic shapefile assignment based on algorithm mode (from folder zone when use_cbmp is true)
	file signal_shp <- use_cbmp ? shape_file("../includes/traffic_signals 8.shp") : shape_file("../includes/traffic_signals 6.shp");
	file roi_lane_shp <- use_cbmp ? shape_file("../includes/ROI_zones 2.shp") : shape_file("../includes/ROI_zones 7.shp");
	
	geometry shape <- envelope(road_shp);
	graph road_network;
	float step <- 0.5 #s;
	int target_motobike <- 1000;
	int target_car <- 300;
	int target_truck <- 50;
	int target_ambulance <- 5;
	float spawn_rate <- 1.0;
	
	// Traffic demand scenario from paper
	string traffic_demand <- "Medium (900 vph)" among: ["Low (400 vph)", "Medium (900 vph)", "High (1400 vph)", "Very High (2000 vph)", "Extreme (2400 vph)"];
	float spawn_timer <- 0.0;
	
	//obj for controller mode
	// false/false = Fixed-time | true/false = CBMP v1 (phi) | false/true = CBMP v2 (Paper)
	bool use_cbmp <- false;
	bool use_paper_cbmp <- true;  // Paper-faithful: formula (10)(11)(16), vehicle count + c_{l,m}
	string algorithm_mode <- "CBMP_Paper" among: ["CBMP_Paper", "CBMP_Area", "FixedTime"];
	
	//obj for KPIs (Đo lường hiệu năng)
	string csv_filename;
	
	// --- KPI SUMMARY VARIABLES (for area-based simulation summary) ---
	float total_queue_sum <- 0.0;      
	int total_throughput <- 0;         
	float total_delay_sum <- 0.0;      
	int total_samples <- 0;            
	bool stop_simulation <- false;
	float sim_end_time <- 7200.0;      // 1 hour in seconds
	bool is_batch_mode <- false;       
	string base_output_dir <- "../outputs/";
	int replicate_id <- 0;
	
	list<intersection> spawn_nodes; // spawn points at the edge of the map
	map<road, float> road_heat;  // smoothed heat value for road density
	int heat_tick <- 0;          // step counter to update heatmap periodically

	// update road_heat using ema for smooth color transitions
	reflex update_road_counts when: !is_batch_mode {
		heat_tick <- heat_tick + 1;
		if (heat_tick mod 5 = 0) {
			// instant vehicle count per road
			map<road, int> cur <- map<road,int>([]);
			loop v over: (motobike as list) + (car as list) + (truck as list)  {
				if (v.current_road != nil) {
					road rd <- road(v.current_road);
					if (rd != nil) {
						cur[rd] <- (cur contains_key rd) ? cur[rd] + 1 : 1;
					}
				}
			}
			// apply ema smoothing for heat values
			loop r over: road {
				float new_val <- (cur contains_key r) ? min(float(cur[r]) / 40.0, 1.0) : 0.0;
				float old_val <- (road_heat contains_key r) ? road_heat[r] : 0.0;
				road_heat[r] <- 0.7 * old_val + 0.3 * new_val;
			}
		}
	}
	// Write total summary row dynamically (Rewrite:true ensures the last line is always updated)
	action write_summary {
		if (total_samples > 0) {
			float avg_queue <- total_queue_sum / total_samples;
			float avg_delay <- (total_throughput > 0) ? total_delay_sum / total_throughput : 0.0;
			string summary_line <- "TOTAL,0,0,"
				+ string(avg_queue with_precision 2) + ","
				+ string(total_throughput) + ","
				+ string(avg_delay with_precision 2);
			// save "Junction_Name, Cycle, Time_Seconds, Avg_Queue (Veh), Total_Throughput (Veh), Avg_Delay (s)"
			// 	to: base_output_dir + "KPI_Summary_" + csv_filename format: "csv" rewrite: true;
			// save summary_line to: base_output_dir + "KPI_Summary_" + csv_filename format: "csv" rewrite: false;
		}
	}

	// Simulation stop control check
	reflex check_simulation_time when: time >= sim_end_time and !stop_simulation {
		stop_simulation <- true;
		write "Sim Time Stop: " + time;
		do write_summary;
		if (!is_batch_mode) { 
			do pause; 
		}else{
			do die;
		}
	}

	// ROI density sensor system (100% faithful to zone folder)
	reflex roi_sensor_system when: use_cbmp {
		ask roi_lane {
			float accumulated_area <- 0.0;
			list<vehicle> candidate_vehicles <- (motobike at_distance 25.0) + (car at_distance 25.0) + (truck at_distance 25.0);
			
			loop v over: candidate_vehicles {
				point pos <- v.compute_position();
				if (pos distance_to self.shape < 7.0) {
					float my_area <- 1.5; 
					if (species(v) = car) { my_area <- 7.5; }
					else if (species(v) = truck) { my_area <- 18.0; }
					accumulated_area <- accumulated_area + my_area;
				}
			}
			
			self.phi <- self.area_m2 > 0 ? accumulated_area / self.area_m2 : 0.0;
			if (self.phi > 1.0) { self.phi <- 1.0; }
		}
	}
	
	// Network-wide pressure monitor (100% faithful to zone folder)
	reflex global_network_monitor when: use_cbmp and !stop_simulation {
		float r_left     <- 0.15; 
		float r_straight <- 0.70;
		float r_right    <- 0.15;

		ask roi_lane {
			self.phi_current <- self.phi;
			self.w_max_pressure <- 0.0; 
		}

		ask roi_lane {
			bool is_upstream <- (self.In_roi != nil and self.In_roi != "" and self.In_roi contains "_" and self.phase_id != nil and self.phase_id != "" and upper_case(self.phase_id) != "NONE");
			
			if (is_upstream) {
				list<string> src_parts <- self.In_roi split_with "_"; 
				
				if (length(src_parts) >= 3) {
					string current_junction <- src_parts[0]; 
					string lane_index       <- src_parts[1]; 
					string lane_axis        <- upper_case(src_parts[2]);
					
					string src_prefix <- lower_case(current_junction + "_" + lane_index);
					float downstream_pressure_sum <- 0.0;
					
					list<roi_lane> downstream_links <- roi_lane where (
						each.Out_roi != nil and each.Out_roi != "" and
						lower_case(each.Out_roi) = src_prefix and
						each.In_roi != nil and each.In_roi contains "_" and
						length(each.In_roi split_with "_") >= 3 and
						upper_case((each.In_roi split_with "_")[2]) = lane_axis
					);
					
					loop link over: downstream_links {
						string target_in_roi <- lower_case(link.In_roi);
						
						roi_lane dest <- first(roi_lane where (
							each.In_roi != nil and 
							lower_case(each.In_roi) = target_in_roi and
							each.phase_id != nil and each.phase_id != "" and upper_case(each.phase_id) != "NONE"
						));
						
						if (dest != nil) {
							float weight <- r_straight; 
							downstream_pressure_sum <- downstream_pressure_sum + (weight * dest.phi_current);
						}
					}
					
					self.w_max_pressure <- self.phi_current - downstream_pressure_sum;
				}
			}
		}

		if (cycle mod 10 = 0) {
			list<roi_lane> valid_lanes <- roi_lane where (
				each.In_roi != nil and each.In_roi != "" and each.In_roi contains "_"
				and each.phi_current > 0.001   
			);
			
			loop lane over: valid_lanes {
				string current_p_id <- (lane.phase_id = nil or lane.phase_id = "") ? "INITIALIZING" : lane.phase_id;
				string row_lane_data <- "" + string(round(time)) + "," 
					+ lane.In_roi + "," 
					+ current_p_id + "," 
					+ (lane.phi_current with_precision 3) + "," 
					+ (lane.w_max_pressure with_precision 3);
				// save row_lane_data to: base_output_dir + "ROI_Density_Log_" + csv_filename format: "csv" rewrite: false;
			}
		}
	}
	init {
		if (algorithm_mode = "CBMP_Paper") {
			use_paper_cbmp <- true;
			use_cbmp <- false;
		} else if (algorithm_mode = "CBMP_Area") {
			use_paper_cbmp <- false;
			use_cbmp <- true;
		} else if (algorithm_mode = "FixedTime") {
			use_paper_cbmp <- false;
			use_cbmp <- false;
		}
		signal_shp <- use_cbmp ? shape_file("../includes/traffic_signals 8.shp") : shape_file("../includes/traffic_signals 6.shp");
		roi_lane_shp <- use_cbmp ? shape_file("../includes/ROI_zones 2.shp") : shape_file("../includes/ROI_zones 7.shp");

		
		write "read data";
		list<geometry> fixed_road <- clean_network(list<geometry>(road_shp.contents), 15.0, true, true);
		create road from: road_shp with: [
		    lanes :: int(read("lanes")), 
		    width :: float(read("road_width")) 
		];

		create building from: building_shp;
		
		// Load roi_lane dynamically based on algorithm mode (from folder zone when use_cbmp is true)
		if (use_cbmp) {
			create roi_lane from: roi_lane_shp with: [
				phase_id  :: read("phase_id"),
				area_m2   :: float(read("area_m2")),
				In_roi    :: read("In_roi"),
				Out_roi   :: read("Out_roi")
			];
		} else {
			create roi_lane from: roi_lane_shp with: [
				u_node    :: read("u_node"),
				d_node    :: read("d_node"),
				phase_id  :: read("phase_id"),
				area_m2   :: float(read("area_m2")) 
			]; 
		}
        
		graph temp_graph <- as_edge_graph(road);
		loop v over: temp_graph.vertices {
			create intersection with: [shape::point(v)] {
				is_traffic_signal <- false;
			}
		}
		road_network <- as_driving_graph(road, intersection);
		
		// Initialize signal points dynamically based on algorithm mode
		if (!use_cbmp) {
			create gis_signal_point from: signal_shp with: [osm_id :: string(read("osm_id"))];

			// =========================================================================
			// GIAI ĐOẠN 1: GOM CỤM CHO NGÃ TƯ ĐẶC BIỆT (GÁN CỨNG QUA ID 1, 2, 3, 4)
			// =========================================================================
			list<gis_signal_point> special_signals <- list<gis_signal_point>(gis_signal_point) where (
				each.osm_id = "1" or each.osm_id = "2" or each.osm_id = "3" or each.osm_id = "4"
			);
			
			if (!empty(special_signals)) {
				// Xác định tâm thực tế và tìm nút giao gần nhất cho cụm đặc biệt này
				point real_center <- mean(special_signals collect each.location);
				intersection target_node <- (intersection) closest_to(real_center);
				
				if (target_node != nil) {
					ask target_node {
						is_traffic_signal <- true;
						// Đồng bộ các tuyến đường cho ngã tư đặc biệt
						do compute_crossing(sig_pts: special_signals collect each.location, center_pt: real_center);
					}
					
					// Tạo các thực thể đèn hiển thị và gán cứng Trục theo cặp đối diện 1-3 và 2-4
					loop sg_agent over: special_signals {
						create traffic_light_visual {
							location <- sg_agent.location;
							my_parent <- target_node;
							self.osm_id <- sg_agent.osm_id;
							
							if (self.osm_id = "1" or self.osm_id = "3") {
								axis <- "axis_1";
							} else if (self.osm_id = "2" or self.osm_id = "4") {
								axis <- "axis_2";
							}
						}
					}
				}
			}

			// =========================================================================
			// GIAI ĐOẠN 2: GOM CỤM CHO CÁC NGÃ TƯ TỰ ĐỘNG CÒN LẠI (TÍNH TOÁN THEO GÓC)
			// =========================================================================
			list<gis_signal_point> free_signals <- list<gis_signal_point>(gis_signal_point) where (
				each.osm_id != "1" and each.osm_id != "2" and each.osm_id != "3" and each.osm_id != "4"
			);
			
			loop while: !empty(free_signals) {
				gis_signal_point head_sg <- free_signals[0];
				
				list<gis_signal_point> cluster_sg <- free_signals where (each distance_to head_sg < 70.0);
				
				if (length(cluster_sg) >= 2) {
					point real_center <- mean(cluster_sg collect each.location);
					intersection target_node <- (intersection) closest_to(real_center);
					
					if (target_node != nil) {
						ask target_node {
							is_traffic_signal <- true;
							do compute_crossing(sig_pts: cluster_sg collect each.location, center_pt: real_center);
						}
						
						loop sg_agent over: cluster_sg {
							create traffic_light_visual {
								location <- sg_agent.location;
								my_parent <- target_node;
								
								// Sử dụng đúng logic góc nguyên bản đã chạy đúng của bạn
								float ang <- self.location towards real_center;
								float norm_ang <- ang mod 180;
								if (norm_ang > 45 and norm_ang < 135) {
									axis <- "axis_1";
								} else {
									axis <- "axis_2";
								}
							}
						}
					}
				}
				free_signals <- free_signals - cluster_sg;
			}
		} else {
			// CBMP Area (ROI) mode: Load visual lights directly from signal_shp (from folder zone)
			create traffic_light_visual from: signal_shp with: [
				osm_id   :: string(read("osm_id")),
				my_phase :: upper_case(string(read("sig_phase"))) 
			];
			
			// Filter: only keep lights ending in _STRAIGHT (from folder zone)
			ask traffic_light_visual {
				if (self.my_phase = nil or self.my_phase = "") {
					do die;
				} else {
					string phase <- self.my_phase;
					list<string> parts <- phase split_with "_";
					if (!empty(parts) and upper_case(last(parts)) != "STRAIGHT") {
						do die;
					}
				}
			}
			
			ask traffic_light_visual {
				point my_loc <- self.location;
				list<intersection> candidate_nodes <- intersection where (each distance_to my_loc <= 80.0);
				if (!empty(candidate_nodes)) {
					self.my_parent <- candidate_nodes closest_to my_loc;
					self.my_parent.is_traffic_signal <- true;
				}
			}
		}

		// identify spawn nodes based on graph degree
		spawn_nodes <- intersection where (
			!each.is_traffic_signal and
			!empty(each.roads_out) and
			(length(each.roads_out) + length(each.roads_in) <= 2)
		);
		// fallback if too few spawn nodes found
		if (length(spawn_nodes) < 3) {
			spawn_nodes <- intersection where (!each.is_traffic_signal and !empty(each.roads_out));
		}
		
		int nb_moto <- 0;
		int nb_car <- 0;
		int nb_truck <- 0;
		write "done";

		// CSV filename reflects active algorithm mode and traffic demand
		string demand_str <- "Custom";
		if (traffic_demand = "Low (400 vph)") { demand_str <- "Low_400"; }
		else if (traffic_demand = "Medium (900 vph)") { demand_str <- "Medium_900"; }
		else if (traffic_demand = "High (1400 vph)") { demand_str <- "High_1400"; }
		else if (traffic_demand = "Very High (2000 vph)") { demand_str <- "VeryHigh_2000"; }
		else if (traffic_demand = "Extreme (2400 vph)") { demand_str <- "Extreme_2400"; }
		
		string mode_str <- use_paper_cbmp ? "CBMP_Paper" : (use_cbmp ? "CBMP_Area" : "FixedTime");
		replicate_id <- int(self) mod 10 + 1;
		string rep_str <- is_batch_mode ? "_rep" + replicate_id : "";
		csv_filename <- mode_str + "_" + demand_str + rep_str + ".csv";
		save "Intersection_Name,Cycle,Time_Seconds,Queue_Length,Throughput_per_Cycle,Average_Delay" to: base_output_dir + "KPI_Result_" + csv_filename format: "csv" rewrite: true;
		
		if (use_cbmp) {
			// save "Intersection_Name,Cycle,Phase_NS,Phase_EW" to: base_output_dir + "Phase_GreenTime_Log_" + csv_filename format: "csv" rewrite: true;
			// save "Time_Seconds,Lane_ID,Phase_ID,Density_Phi,Pressure_W" to: base_output_dir + "ROI_Density_Log_" + csv_filename format: "csv" rewrite: true;
		}
		
		list<intersection> signal_nodes <- intersection where (each.is_traffic_signal);
		loop while: not empty(signal_nodes){
			intersection seed <- signal_nodes[0];
			list<intersection> cluster <- signal_nodes where (each distance_to seed <= 50.0);
			
			create traffic_controller{
				my_nodes <- cluster;
				location <- cluster[0].location;
				if (!use_cbmp) {
					ask my_nodes {
						do to_green;
					}
				}
			}
			signal_nodes <- signal_nodes - cluster;
		}

		// Assign phase IDs to ROI lanes based on their axis (100% faithful to zone folder)
		if (use_cbmp) {
			ask intersection where (each.is_traffic_signal) {
				intersection current_intersection <- self;
				list<roi_lane> local_lanes <- roi_lane where (
					each.In_roi != nil and each.In_roi != "" and each.In_roi contains "_" 
					and each.phase_id != nil and each.phase_id != "" and upper_case(each.phase_id) != "NONE"
					and (current_intersection.location distance_to each.location < 120.0)
				);
				self.my_lanes <- local_lanes;
				
				if (!empty(local_lanes)) {
					string junction_name <- upper_case((local_lanes[0].In_roi split_with "_")[0]);
					
					string phase_ns  <- junction_name + "_NS"; 
					string phase_ew  <- junction_name + "_EW"; 
					
					if !(self.signal_phases contains phase_ns)  { add phase_ns to: self.signal_phases; }
					if !(self.signal_phases contains phase_ew)  { add phase_ew to: self.signal_phases; }
					
					loop lane over: local_lanes {
						list<string> tokens <- lane.In_roi split_with "_";
						if (length(tokens) >= 3) {
							string lane_axis <- tokens[2]; 
							
							if (upper_case(lane_axis) = "NS" or upper_case(lane_axis) = "SN") {
								lane.phase_id <- phase_ns;
							} else if (upper_case(lane_axis) = "EW" or upper_case(lane_axis) = "WE") {
								lane.phase_id <- phase_ew;
							}
						}
					}
					
					list<traffic_light_visual> local_lights <- traffic_light_visual where (each.my_parent = current_intersection);
					loop tl over: local_lights {
						string old_p <- upper_case(tl.my_phase);
						
						if (old_p contains "_SN" or old_p contains "_NS") {
							tl.my_phase <- phase_ns;
						} else if (old_p contains "_EW") {
							tl.my_phase <- phase_ew;
						}
					}
				}
			}
		}
	}

	reflex maintain_population {
		if (traffic_demand != "Custom") {
			// --- POISSON-BASED FREE SPAWNING SCENARIOS ---
			spawn_timer <- spawn_timer + step;
			
			float spawn_interval <- 4.0; // Medium (900 vph) default
			if (traffic_demand = "Low (400 vph)") { spawn_interval <- 9.0; } //vehicle/hour =  3600/400 = 9s 1 vehicle
			else if (traffic_demand = "High (1400 vph)") { spawn_interval <- 2.57; }
			else if (traffic_demand = "Very High (2000 vph)") { spawn_interval <- 1.8; }
			else if (traffic_demand = "Extreme (2400 vph)") { spawn_interval <- 1.5; }
			
			// Adjust spawn interval based on the 8 spawn branches
			int n_branches <- 8;
			if (n_branches > 0) {
				spawn_interval <- spawn_interval / n_branches;
			}

			if (spawn_timer >= spawn_interval) {
				spawn_timer <- spawn_timer - spawn_interval;
				
				intersection start_node <- one_of(spawn_nodes);
				intersection end_node <- one_of(spawn_nodes);
				
				if (start_node != nil and end_node != nil and start_node != end_node) {
					// Safe spawning check: less than 2 vehicles within 8m of start point
					if (length(vehicle overlapping circle(8.0, start_node.location)) < 2) {
						// Vehicle composition: 85% motobike, 12% car, 3% truck
						float rnd_val <- rnd(1.0);
						if (rnd_val < 0.85) {
							create motobike number: 1 { location <- start_node.location; final_target <- end_node; }
						} else if (rnd_val < 0.97) {
							create car number: 1 { location <- start_node.location; final_target <- end_node; }
						} else {
							create truck number: 1 { location <- start_node.location; final_target <- end_node; }
						}
					}
				}
			}
		} else {
			// --- ORIGINAL POPULATION MAINTENANCE SCENARIOS ---
			int diff_moto <- target_motobike - length(motobike);
			int diff_car <- target_car - length(car);
			int diff_truck <- target_truck - length(truck);
			int total_diff <- max(0, diff_moto) + max(0, diff_car) + max(0, diff_truck);
	
			if (total_diff > 0) {
				int spawn_count <- min(total_diff, 3);
				loop times: spawn_count {
					intersection end_node <- one_of(spawn_nodes);
					if (end_node != nil) {
						intersection start_node <- one_of(spawn_nodes);
						if (start_node != nil and start_node != end_node) {
							if (length(vehicle overlapping circle(8.0, start_node.location)) < 2) {
								int rand_val <- rnd(total_diff - 1);
								if (rand_val < max(0, diff_moto)) {
									create motobike number: 1 { location <- start_node.location; final_target <- end_node; }
								} else if (rand_val < max(0, diff_moto) + max(0, diff_car)) {
									create car number: 1 { location <- start_node.location; final_target <- end_node; }
								} else if (rand_val < max(0, diff_moto) + max(0, diff_car) + max(0, diff_truck)) {
									create truck number: 1 { location <- start_node.location; final_target <- end_node; }
								}
							}
						}
					}
				}
			}
			// Clean redundant population if custom limit decreases
			if (diff_moto < 0) { ask abs(diff_moto) among (motobike as list) { do die; } }
			if (diff_car < 0) { ask abs(diff_car) among (car as list) { do die; } }
			if (diff_truck < 0) { ask abs(diff_truck) among (truck as list) { do die; } }
		}
	}}


experiment test type: gui {
	parameter "Kịch bản lưu lượng:" var: traffic_demand;
//	parameter "Lưu lượng xe máy (Tùy chỉnh):" var: target_motobike min: 0 max: 3000;
//	parameter "Lưu lượng ô tô (Tùy chỉnh):" var: target_car min: 0 max: 2000;
//	parameter "Lưu lượng xe tải (Tùy chỉnh):" var: target_truck min: 0 max: 1000;
	parameter "Thuật toán điều khiển:" var: algorithm_mode;
	//parameter "Kịch bản di chuyển:" var: routing_scenario among: ["Bình thường", "Trục dọc kẹt cứng", "Đổ dồn về phía Đông"];
	output {
		display main type: 3d background: #lightskyblue axes: false {
			species road refresh: false;
			species roi_lane refresh: true;
			species motobike;
			species car;
			species truck;
			species intersection;
			species traffic_light_visual;
			
		}
		
		display heatmap type: 3d background: rgb(8, 12, 25) axes: false {
			// base road layer 
			species road aspect: heatmap_base refresh: false;
			// overlay heat dots based on actual vehicle positions
			species motobike aspect: heat_dot;
			species car aspect: heat_dot;
			species truck aspect: heat_dot;
			//species ambulance aspect: heat_dot;
		}
		
//		display KPI_Charts type: java2D {
//			chart "Lưu lượng thông hành toàn mạng (Throughput / Chu kỳ)" type: series size: {1, 0.5} position: {0, 0} {
//				data "Số xe thoát (xe/chu kỳ)" value: sum(traffic_controller collect each.my_throughput) color: #green marker: false;
//			}
//			chart "Chiều dài hàng chờ toàn mạng (Queue Length)" type: series size: {1, 0.5} position: {0, 0.5} {
//				data "Số xe đang kẹt tại các ngã tư" value: sum(traffic_controller collect each.my_queue) color: #red marker: false;
//			}
//		}
	}
}

experiment batch_run type: batch keep_seed: false repeat: 10 until: stop_simulation parallel: true {
//	"Low (400 vph)", "Medium (900 vph)", "High (1400 vph)"
	parameter "Kịch bản lưu lượng:" var: traffic_demand among: ["Medium (900 vph)"];
	parameter "Thuật toán điều khiển:" var: algorithm_mode among: ["CBMP_Paper"];
	parameter "Batch mode:" var: is_batch_mode init: true;
	
	reflex write_final_summary {
		ask simulations { do write_summary; }
	}
}