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
	file signal_shp <- shape_file("../includes/traffic_signals 6.shp");
	file roi_lane_shp <- shape_file("../includes/ROI_zones 7.shp");
	
	geometry shape <- envelope(road_shp);
	graph road_network;
	float step <- 0.5 #s;
	int target_motobike <- 1000;
	int target_car <- 300;
	int target_truck <- 50;
	int target_ambulance <- 5;
	float spawn_rate <- 1.0;
	
	// Traffic demand scenario from paper
	string traffic_demand <- "Medium (900 vph)" among: ["Low (400 vph)", "Medium (900 vph)", "High (1400 vph)"];
	float spawn_timer <- 0.0;
	
	//obj for controller mode
	// false/false = Fixed-time | true/false = CBMP v1 (phi) | false/true = CBMP v2 (Paper)
	bool use_cbmp <- false;
	bool use_paper_cbmp <- true;  // Paper-faithful: formula (10)(11)(16), vehicle count + c_{l,m}
	
	//obj for KPIs (Đo lường hiệu năng)
	string csv_filename;
	
	list<intersection> spawn_nodes; // spawn points at the edge of the map
	map<road, float> road_heat;  // smoothed heat value for road density
	int heat_tick <- 0;          // step counter to update heatmap periodically

	// update road_heat using ema for smooth color transitions
	reflex update_road_counts {
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
// ROI phi calculation is now done inside roi_lane reflex (self-managed)
	init {
		write "read data";
		list<geometry> fixed_road <- clean_network(list<geometry>(road_shp.contents), 15.0, true, true);
		create road from: road_shp with: [
		    lanes :: int(read("lanes")), 
		    width :: float(read("road_width")) 
		];

		create building from: building_shp;
create roi_lane from: roi_lane_shp with: [
            u_node    :: read("u_node"),
            d_node    :: read("d_node"),
            phase_id  :: read("phase_id"),
            area_m2   :: float(read("area_m2")) 
        ]; 
        
   
        
		graph temp_graph <- as_edge_graph(road);
		loop v over: temp_graph.vertices {
			create intersection with: [shape::point(v)] {
				is_traffic_signal <- false;
			}
		}
		road_network <- as_driving_graph(road, intersection);
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
		
		string mode_str <- use_paper_cbmp ? "CBMP_Paper" : (use_cbmp ? "CBMP_v1" : "FixedTime");
		csv_filename <- "KPI_Result_" + mode_str + "_" + demand_str + ".csv";
		save "Intersection_Name,Cycle,Time_Seconds,Queue_Length,Throughput_per_Cycle,Average_Delay" to: csv_filename format: "csv" rewrite: true;
		

		
		list<intersection> signal_nodes <- intersection where (each.is_traffic_signal);
		loop while: not empty(signal_nodes){
			intersection seed <- signal_nodes[0];
			list<intersection> cluster <- signal_nodes where (each distance_to seed <= 50.0);
			
			create traffic_controller{
				my_nodes <- cluster;
				location <- cluster[0].location;
				ask my_nodes {
					do to_green;
				}
			}
			signal_nodes <- signal_nodes - cluster;
		}

	}

	reflex maintain_population {
		if (traffic_demand != "Custom") {
			// --- POISSON-BASED FREE SPAWNING SCENARIOS ---
			spawn_timer <- spawn_timer + step;
			
			float spawn_interval <- 4.0; // Medium (900 vph) default
			if (traffic_demand = "Low (400 vph)") { spawn_interval <- 9.0; }
			else if (traffic_demand = "High (1400 vph)") { spawn_interval <- 2.57; }
			
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
	parameter "CBMP v1 (phi diện tích):" var: use_cbmp;
	parameter "CBMP v2 Paper (đếm xe):" var: use_paper_cbmp;
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

experiment batch_run type: batch keep_seed: true until: (cycle >= 7200) {
	parameter "Kịch bản lưu lượng:" var: traffic_demand among: ["Low (400 vph)", "Medium (900 vph)", "High (1400 vph)"];
	parameter "CBMP v2 Paper (đếm xe):" var: use_paper_cbmp among: [false, true];
}