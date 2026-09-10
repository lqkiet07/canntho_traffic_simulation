
/**
* Name: Intersection
* Based on the internal empty template. 
* Author: PC
* Tags: 
*/

model Intersection

import "Road.gaml"
import "Roi.gaml"
import "Vehicles.gaml"
import "Main.gaml"

// =========================================================================
// SPECIES: GIS_SIGNAL_POINT
// =========================================================================
species gis_signal_point {
	string osm_id;
}
species friendly_roi_name_provider {
	string In_roi;
}

// =========================================================================
// SPECIES: INTERSECTION
// =========================================================================
species intersection skills: [intersection_skill] {
	
	// --- CACHE & ACCUMULATED VARIABLES FOR CBMP AREA ---
	list<roi_lane> my_lanes <- [];       // Cache list of lanes for this intersection
	float accumulated_queue <- 0.0;     // Accumulated queue length over steps in cycle
	int queue_sample_count <- 0;         // Number of queue samples in cycle
	list<string> signal_phases <- [];                
	map<string, float> gamma_phase_pressures <- [];  
	map<string, float> accumulated_phase_pressures <- [];
	int pressure_sample_count <- 0;
	float C_saturation <- 2.5;          // Saturation flow constant                     
	int total_node_queue <- 0;           // Waiting vehicles

	// -------------------------------------------------------------------------
	// 1. Biến trạng thái đèn & Đồ họa
	// -------------------------------------------------------------------------
	bool is_green;
	bool is_traffic_signal;
	int throughput_count <- 0;
	float total_delay_in_cycle <- 0.0;
	rgb color_fire;
	int start_phase <- 1;
	
	list<road> ways1 <- [];
	list<road> ways2 <- [];
	
	// -------------------------------------------------------------------------
	// 2. Biến đếm số xe kẹt (Hàng chờ thực tế)
	// -------------------------------------------------------------------------
	map<road,int> queue_per_road;
	int queue_ways1 <- 0;
	int queue_ways2 <- 0;
	
	// 4 bien dem hang cho cho 4 vung nhanh (tinh tu den giao thong lui ve sau)
	int queue_N <- 0; // Nhánh từ Bắc tiến vào ngã tư (đi về hướng Nam)
	int queue_S <- 0; // Nhánh từ Nam tiến vào ngã tư (đi về hướng Bắc)
	int queue_E <- 0; // Nhánh từ Đông tiến vào ngã tư (đi về hướng Tây)
	int queue_W <- 0; // Nhánh từ Tây tiến vào ngã tư (đi về hướng Đông)
	
	// -------------------------------------------------------------------------
	// 3. Biến áp suất diện tích (Tỷ lệ chiếm dụng Φ) — CBMP v1
	// -------------------------------------------------------------------------
	//obj for area occupancy ratio (0.0 -> 1.0) cho tung nhanh - dung cho CBMP phase 1
	float phi_N <- 0.0; // φ nhánh Bắc
	float phi_S <- 0.0; // φ nhánh Nam
	float phi_E <- 0.0; // φ nhánh Đông
	float phi_W <- 0.0; // φ nhánh Tây
	float phi_out_N <- 0.0; // φ hạ lưu Bắc
	float phi_out_S <- 0.0; // φ hạ lưu Nam
	float phi_out_E <- 0.0; // φ hạ lưu Đông
	float phi_out_W <- 0.0; // φ hạ lưu Tây
	float phi_axis1 <- 0.0; // φ tổng hợp của axis_1 (ways1) - dùng cho bộ điều khiển
	float phi_axis2 <- 0.0; // φ tổng hợp của axis_2 (ways2) - dùng cho bộ điều khiển

	// -------------------------------------------------------------------------
	// 3b. Biến đếm xe — CBMP v2 (Paper: Anderson et al. 2018)
	// -------------------------------------------------------------------------
	// x_{l,m}: incoming vehicle count behind stop line — formula (10)
	int x_N <- 0;  // vehicles approaching from North
	int x_S <- 0;  // vehicles approaching from South
	int x_E <- 0;  // vehicles approaching from East
	int x_W <- 0;  // vehicles approaching from West
	// x_{m,p}: outgoing vehicle count (downstream) — formula (10)
	int x_out_N <- 0;
	int x_out_S <- 0;
	int x_out_E <- 0;
	int x_out_W <- 0;
	// Queue weight w_{l,m} = x_{l,m} - sum(r * x_out) — formula (10)
	// NOTE: turn ratios r fixed at 0.7/0.15/0.15; dynamic version is future work
	float w_p1_paper <- 0.0;
	float w_p2_paper <- 0.0;
	float w_p1_sum <- 0.0;
	float w_p2_sum <- 0.0;
	int last_cnt_p1 <- 0;
	int last_cnt_p2 <- 0;
	
	// Accumulators for average cycle pressure (smooth feedback)
	float w_N_sum <- 0.0;
	float w_S_sum <- 0.0;
	float w_E_sum <- 0.0;
	float w_W_sum <- 0.0;
	int paper_step_count <- 0;

	// -------------------------------------------------------------------------
	// 3c. Relative Compass static axes (for adaptive directional counting)
	// -------------------------------------------------------------------------
	float ang_in_N <- -1.0;
	float ang_in_S <- -1.0;
	float ang_in_E <- -1.0;
	float ang_in_W <- -1.0;
	
	float ang_out_N <- -1.0;
	float ang_out_S <- -1.0;
	float ang_out_E <- -1.0;
	float ang_out_W <- -1.0;

	// -------------------------------------------------------------------------
	// 4. Khối Hành động (Actions)
	// -------------------------------------------------------------------------
	
	//caculate lane for intersection
	action compute_crossing(list<point> sig_pts, point center_pt) {
		ways1 <- [];
		ways2 <- [];
		if (empty(roads_in) or empty(sig_pts)) { return; }

		list<road> all_roads <- roads_in collect road(each);

		// Kiểm tra nhanh xem đây có phải cuộc gọi từ cụm đặc biệt được gán tay không
		bool is_hand_assigned <- false;
		list<traffic_light_visual> test_lights <- traffic_light_visual where (each.my_parent = self);
		
		// Nếu đã tạo đèn visual trước đó hoặc kiểm tra gián tiếp qua trạng thái ID
		loop lg over: test_lights {
			if (lg.osm_id = "1" or lg.osm_id = "2" or lg.osm_id = "3" or lg.osm_id = "4") {
				is_hand_assigned <- true;
				break;
			}
		}

		if (is_hand_assigned) {
			// LOGIC CHO NGÃ TƯ LỆCH: Phân trục đường dựa chính xác theo cặp đèn  1-3 và 2-4
			loop lg over: test_lights {
				road best_rd <- all_roads closest_to lg;
				if (best_rd != nil) {
					if (lg.osm_id = "2" or lg.osm_id = "4") {
						if (!(ways1 contains best_rd)) { ways1 <- ways1 + [best_rd]; }
						lg.axis <- "axis_2"; // Ensure correct axis mapping for control
					} else if (lg.osm_id = "1" or lg.osm_id = "3") {
						if (!(ways2 contains best_rd)) { ways2 <- ways2 + [best_rd]; }
						lg.axis <- "axis_1"; // Ensure correct axis mapping for control
					}
				}
			}
		} else {
			// LOGIC Dành cho tất cả ngã tư tự động còn lại trong bản đồ
			loop sg_pt over: sig_pts {
				float ang <- sg_pt towards center_pt;
				float normalized_ang <- ang mod 180;

				road best_rd <- nil;
				float min_d <- #infinity;
				loop rd over: all_roads {
					float d <- sg_pt distance_to road(rd).shape;
					if (d < min_d) { min_d <- d; best_rd <- road(rd); }
				}
				if (best_rd != nil) {
					if (normalized_ang > 45 and normalized_ang < 135) {
						if (!(ways1 contains best_rd)) { ways1 <- ways1 + [best_rd]; }
					} else {
						if (!(ways2 contains best_rd)) { ways2 <- ways2 + [best_rd]; }
					}
				}
			}
		}

		// Tự động bù trừ và phân bổ nốt những đoạn đường lẻ chưa được map vào nhóm nào
		loop rd over: all_roads {
			if (!(ways1 contains rd) and !(ways2 contains rd)) {
				if (length(ways1) <= length(ways2)) {
					ways1 <- ways1 + [rd];
				} else {
					ways2 <- ways2 + [rd];
				}
			}
		}

		// Adaptive Relative Compass axes calculation
		list<road> my_roads <- (roads_in collect road(each)) + (roads_out collect road(each));
		my_roads <- remove_duplicates(my_roads);
		
		map<road, float> road_angles;
		loop rd over: my_roads {
			if (rd != nil and !empty(rd.shape.points)) {
				point pt_start <- rd.shape.points[0];
				point pt_end <- rd.shape.points[length(rd.shape.points) - 1];
				// Select far_pt based on maximum distance to intersection center
				point far_pt <- (pt_start distance_to self.location > pt_end distance_to self.location) ? pt_start : pt_end;
				float ang_in <- float(far_pt towards self.location);
				road_angles[rd] <- ang_in;
			}
		}
		
		list<road> remaining_roads <- keys(road_angles);
		
		// 1. Assign to West axis (closest to 0.0 or 360.0)
		road best_W <- nil;
		float min_diff_W <- 360.0;
		loop rd over: remaining_roads {
			float ang <- road_angles[rd];
			float diff <- min(abs(ang - 0.0), abs(ang - 360.0));
			if (diff < min_diff_W) {
				min_diff_W <- diff;
				best_W <- rd;
			}
		}
		if (best_W != nil) {
			ang_in_W <- road_angles[best_W];
			point pt_start <- best_W.shape.points[0];
			point pt_end <- best_W.shape.points[length(best_W.shape.points) - 1];
			point far_pt <- (pt_start distance_to self.location > pt_end distance_to self.location) ? pt_start : pt_end;
			ang_out_W <- float(self.location towards far_pt);
			remaining_roads <- remaining_roads - best_W;
		}
		
		// 2. Assign to North axis (closest to 90.0)
		road best_N <- nil;
		float min_diff_N <- 360.0;
		loop rd over: remaining_roads {
			float ang <- road_angles[rd];
			float diff <- abs(ang - 90.0);
			if (diff < min_diff_N) {
				min_diff_N <- diff;
				best_N <- rd;
			}
		}
		if (best_N != nil) {
			ang_in_N <- road_angles[best_N];
			point pt_start <- best_N.shape.points[0];
			point pt_end <- best_N.shape.points[length(best_N.shape.points) - 1];
			point far_pt <- (pt_start distance_to self.location > pt_end distance_to self.location) ? pt_start : pt_end;
			ang_out_N <- float(self.location towards far_pt);
			remaining_roads <- remaining_roads - best_N;
		}
		
		// 3. Assign to East axis (closest to 180.0)
		road best_E <- nil;
		float min_diff_E <- 360.0;
		loop rd over: remaining_roads {
			float ang <- road_angles[rd];
			float diff <- abs(ang - 180.0);
			if (diff < min_diff_E) {
				min_diff_E <- diff;
				best_E <- rd;
			}
		}
		if (best_E != nil) {
			ang_in_E <- road_angles[best_E];
			point pt_start <- best_E.shape.points[0];
			point pt_end <- best_E.shape.points[length(best_E.shape.points) - 1];
			point far_pt <- (pt_start distance_to self.location > pt_end distance_to self.location) ? pt_start : pt_end;
			ang_out_E <- float(self.location towards far_pt);
			remaining_roads <- remaining_roads - best_E;
		}
		
		// 4. Assign remaining to South axis (closest to 270.0)
		if (!empty(remaining_roads)) {
			road best_S <- remaining_roads[0];
			ang_in_S <- road_angles[best_S];
			point pt_start <- best_S.shape.points[0];
			point pt_end <- best_S.shape.points[length(best_S.shape.points) - 1];
			point far_pt <- (pt_start distance_to self.location > pt_end distance_to self.location) ? pt_start : pt_end;
			ang_out_S <- float(self.location towards far_pt);
		}
		
//		if (name = "intersection33" or name = "intersection35") {
//			write "=== Relative Compass Init for " + name + " ===";
//			write "  ang_in  | N:" + ang_in_N + " S:" + ang_in_S + " E:" + ang_in_E + " W:" + ang_in_W;
//			write "  ang_out | N:" + ang_out_N + " S:" + ang_out_S + " E:" + ang_out_E + " W:" + ang_out_W;
//		}
	}

	action to_green {
		// update visual state for green light
		color_fire <- #green;
		is_green <- true;
		// switch lights axis_1 to green and axis_2 to red
		ask traffic_light_visual where (each.my_parent = self) {
			if (my_parent.name = "intersection35") {
				// Map Phase 1 (g1) to lights 2 and 4 (diagonal axis)
				state <- (osm_id = "2" or osm_id = "4") ? "green" : "red";
			} else {
				state <- (axis = "axis_1") ? "green" : "red";
			}
		}
	}

	action to_red {
		// update visual state for red light
		color_fire <- #red;
		is_green <- false;
		// switch lights axis_2 to green and axis_1 to red
		ask traffic_light_visual where (each.my_parent = self) {
			if (my_parent.name = "intersection35") {
				// Map Phase 2 (g2) to lights 1 and 3 (vertical axis)
				state <- (osm_id = "1" or osm_id = "3") ? "green" : "red";
			} else {
				state <- (axis = "axis_2") ? "green" : "red";
			}
		}
	}

	// -------------------------------------------------------------------------
	// 5. Khối Phản xạ (Reflexes)
	// -------------------------------------------------------------------------
	reflex calculate_queue when: is_traffic_signal and !use_cbmp and !use_paper_cbmp {
		// Reset queue map
		loop rd over: ways1 + ways2 { queue_per_road[rd] <- 0; }
		
		list<vehicle> all_vehicles <- (motobike as list) + (car as list) + (truck as list);
		list<vehicle> near_vehicles <- all_vehicles where (each distance_to self < 200.0);// detect vehicles
		
		//obj for stop line boundary - lay danh sach cot den cua chinh ngo tu nay de xac dinh ranh gioi
		// Moi cot den la stop line cua mot nhanh duong di vao tuong ung
		list<traffic_light_visual> my_lights <- traffic_light_visual where (each.my_parent = self);
		
		int debug_total_near <- 0;// dem tong so luong xe nam trong ban kinh 200m
		int debug_on_road_in <- 0;// dem tong so luong xe nam tren truc duong vao hoac ra cua nga 
		int debug_is_slow <- 0;
		int debug_behind_line <- 0;
		
		int count_w1 <- 0;
		int count_w2 <- 0;
		int c_N <- 0; int c_S <- 0; int c_E <- 0; int c_W <- 0;
		
		//obj for area-based occupancy - tong dien tich chiem dung theo tung nhanh (m2)
		float area_N <- 0.0; float area_S <- 0.0;
		float area_E <- 0.0; float area_W <- 0.0;
		
		float area_out_N <- 0.0; float area_out_S <- 0.0;
		float area_out_E <- 0.0; float area_out_W <- 0.0;
		
		//obj for queue count - chi xe DUNG/CHAM (speed < 5km/h)
		int queue_c_N <- 0; int queue_c_S <- 0;
		int queue_c_E <- 0; int queue_c_W <- 0;
		
		//obj for detection zone area - dien tich Z_n se duoc cap nhat theo chieu rong duong thuc te
		float detect_length <- 150.0;
		float zone_N <- detect_length * 6.0;
		float zone_S <- detect_length * 6.0;
		float zone_E <- detect_length * 6.0;
		float zone_W <- detect_length * 6.0;
		
		loop v over: near_vehicles {
			debug_total_near <- debug_total_near + 1;
			if (v.current_road != nil) {
				road current_rd <- road(v.current_road);
				intersection dest_node <- intersection(road_network target_of current_rd);
				intersection src_node <- intersection(road_network source_of current_rd);
				
				bool is_incoming <- (dest_node != nil and (dest_node distance_to self < 50.0));
				bool is_outgoing <- (src_node != nil and (src_node distance_to self < 50.0));
				
				if (is_incoming or is_outgoing) {
					debug_on_road_in <- debug_on_road_in + 1;
					
					//obj for vehicle footprint area = length x width (m2) - cong thuc (1)
					float veh_area <- v.vehicle_length * v.vehicle_width;
					
					if (is_incoming) {
						//obj for stop line check - tim cot den gan nhat cung huong voi xe
						float ang_center_to_veh <- float(self.location towards v.location);
						traffic_light_visual stop_light <- nil;
						if (!empty(my_lights)) {
							stop_light <- my_lights with_min_of (
								abs(((float(self.location towards each.location) - ang_center_to_veh) + 360.0) mod 360.0)
							);
						}
						
						bool behind_stop_line <- true;
						if (stop_light != nil) {
							float dist_vehicle <- v distance_to self;
							float dist_light   <- stop_light distance_to self;
							behind_stop_line <- (dist_vehicle > dist_light);// xe dung sau vach dừng
						}
						
						if (behind_stop_line) {
							debug_behind_line <- debug_behind_line + 1;
							
							// Xac dinh xe uu tien (Cong thuc 5 & 6)
							float alpha_j <- 0.0;
							//if (v is ambulance) { alpha_j <- 2.0; } // obj for ambulance amplification
							
							// obj for priority amplification - nhan (1 + alpha_j) vao dien tich tuong duong
							float effective_area <- veh_area * (1.0 + alpha_j);
							
							float ang_to_center <- float(v.location towards self.location);
							bool is_slow <- (v.speed < 5 #km/#h or v.real_speed < 5 #km/#h);
							if (is_slow) { debug_is_slow <- debug_is_slow + 1; }
							
							// Determine direction using Relative Compass
							string direction <- "";
							float min_diff <- 360.0;
							
							if (ang_in_W >= 0.0) {
								float d <- abs(ang_to_center - ang_in_W) mod 360.0;
								if (d > 180.0) { d <- 360.0 - d; }
								if (d < min_diff) { min_diff <- d; direction <- "W"; }
							}
							if (ang_in_N >= 0.0) {
								float d <- abs(ang_to_center - ang_in_N) mod 360.0;
								if (d > 180.0) { d <- 360.0 - d; }
								if (d < min_diff) { min_diff <- d; direction <- "N"; }
							}
							if (ang_in_E >= 0.0) {
								float d <- abs(ang_to_center - ang_in_E) mod 360.0;
								if (d > 180.0) { d <- 360.0 - d; }
								if (d < min_diff) { min_diff <- d; direction <- "E"; }
							}
							if (ang_in_S >= 0.0) {
								float d <- abs(ang_to_center - ang_in_S) mod 360.0;
								if (d > 180.0) { d <- 360.0 - d; }
								if (d < min_diff) { min_diff <- d; direction <- "S"; }
							}
							
							// Fallback to absolute compass if no valid relative direction is set
							if (direction = "") {
								if (ang_to_center >= 315 or ang_to_center < 45) {
									direction <- "W";
								} else if (ang_to_center >= 45 and ang_to_center < 135) {
									direction <- "N";
								} else if (ang_to_center >= 135 and ang_to_center < 225) {
									direction <- "E";
								} else {
									direction <- "S";
								}
							}
							
							if (direction = "W") {
								count_w2 <- count_w2 + 1;
								area_W <- area_W + effective_area;
								if (is_slow) { c_W <- c_W + 1; queue_c_W <- queue_c_W + 1; }
							} else if (direction = "N") {
								count_w1 <- count_w1 + 1;
								area_N <- area_N + effective_area;
								if (is_slow) { c_N <- c_N + 1; queue_c_N <- queue_c_N + 1; }
							} else if (direction = "E") {
								count_w2 <- count_w2 + 1;
								area_E <- area_E + effective_area;
								zone_E <- detect_length * current_rd.width;
								if (is_slow) { c_E <- c_E + 1; queue_c_E <- queue_c_E + 1; }
							} else if (direction = "S") {
								count_w1 <- count_w1 + 1;
								area_S <- area_S + effective_area;
								if (is_slow) { c_S <- c_S + 1; queue_c_S <- queue_c_S + 1; }
							}
						}
					}
					
					if (is_outgoing) {
						// Vehicle leaving the intersection (downstream)
						float ang_from_center <- float(self.location towards v.location);
						
						string direction_out <- "";
						float min_diff_out <- 360.0;
						
						if (ang_out_E >= 0.0) {
							float d <- abs(ang_from_center - ang_out_E) mod 360.0;
							if (d > 180.0) { d <- 360.0 - d; }
							if (d < min_diff_out) { min_diff_out <- d; direction_out <- "E"; }
						}
						if (ang_out_S >= 0.0) {
							float d <- abs(ang_from_center - ang_out_S) mod 360.0;
							if (d > 180.0) { d <- 360.0 - d; }
							if (d < min_diff_out) { min_diff_out <- d; direction_out <- "S"; }
						}
						if (ang_out_W >= 0.0) {
							float d <- abs(ang_from_center - ang_out_W) mod 360.0;
							if (d > 180.0) { d <- 360.0 - d; }
							if (d < min_diff_out) { min_diff_out <- d; direction_out <- "W"; }
						}
						if (ang_out_N >= 0.0) {
							float d <- abs(ang_from_center - ang_out_N) mod 360.0;
							if (d > 180.0) { d <- 360.0 - d; }
							if (d < min_diff_out) { min_diff_out <- d; direction_out <- "N"; }
						}
						
						// Fallback to absolute compass for outgoing
						if (direction_out = "") {
							if (ang_from_center >= 315 or ang_from_center < 45) {
								direction_out <- "E";
							} else if (ang_from_center >= 45 and ang_from_center < 135) {
								direction_out <- "S";
							} else if (ang_from_center >= 135 and ang_from_center < 225) {
								direction_out <- "W";
							} else {
								direction_out <- "N";
							}
						}
						
						if (direction_out = "E") {
							area_out_E <- area_out_E + veh_area;
						} else if (direction_out = "S") {
							area_out_S <- area_out_S + veh_area;
						} else if (direction_out = "W") {
							area_out_W <- area_out_W + veh_area;
						} else if (direction_out = "N") {
							area_out_N <- area_out_N + veh_area;
						}
					}
				}
			}
		}
		
		// Cap nhat bien dem hang cho
		queue_ways1 <- count_w1;
		queue_ways2 <- count_w2;
		queue_N <- queue_c_N; queue_S <- queue_c_S;
		queue_E <- queue_c_E; queue_W <- queue_c_W;
		if (!empty(ways1)) { 
		    zone_N <- detect_length * ways1[0].width; 
		    zone_S <- detect_length * ways1[0].width; 
		}
		if (!empty(ways2)) { 
		    zone_E <- detect_length * ways2[0].width; 
		    zone_W <- detect_length * ways2[0].width; 
		}
		//obj for phi - cong thuc (1): phi = tong_dien_tich_xe / dien_tich_Z_n, rang buoc 0 <= phi <= 1
		phi_N <- (zone_N > 0) ? min(area_N / zone_N, 1.0) : 0.0;
		phi_S <- (zone_S > 0) ? min(area_S / zone_S, 1.0) : 0.0;
		phi_E <- (zone_E > 0) ? min(area_E / zone_E, 1.0) : 0.0;
		phi_W <- (zone_W > 0) ? min(area_W / zone_W, 1.0) : 0.0;
		
		// Tinh luon phi cho cac nhanh ha luu (xem nhu zone bang 150m * 6m de uoc luong)
		float zone_default <- 150.0 * 6.0;
		phi_out_N <- min(area_out_N / zone_default, 1.0);
		phi_out_S <- min(area_out_S / zone_default, 1.0);
		phi_out_E <- min(area_out_E / zone_default, 1.0);
		phi_out_W <- min(area_out_W / zone_default, 1.0);
		
		//obj for axis pressure - tong phi cac nhanh cung pha
		phi_axis1 <- phi_N + phi_S;
		phi_axis2 <- phi_E + phi_W;
		
		// Debug cho intersection33
		if (name = "intersection35" and cycle mod 10 = 0) {
//			write "--- Cycle " + cycle + " | " + name + " ---";
//			write "near: " + debug_total_near + " | on_road: " + debug_on_road_in
//			    + " | behind_stop_line: " + debug_behind_line + " | slow: " + debug_is_slow;
//			if (debug_behind_line > 0) {
////				write "  Queue  | N:" + queue_N + " S:" + queue_S + " E:" + queue_E + " W:" + queue_W;
//				float pN <- round(phi_N * 1000) / 10.0;
//				float pS <- round(phi_S * 1000) / 10.0;
//				float pE <- round(phi_E * 1000) / 10.0;
//				float pW <- round(phi_W * 1000) / 10.0;
//				float pa1 <- round(phi_axis1 * 1000) / 10.0;
//				float pa2 <- round(phi_axis2 * 1000) / 10.0;
//				write "  φ(%)   | N:" + pN + "% S:" + pS + "% E:" + pE + "% W:" + pW + "%";
//				write "  φ_axis | axis1(N+S):" + pa1 + "% | axis2(E+W):" + pa2 + "%";
//			}
		}
	}

	// =========================================================================
	// CBMP v2 — Paper-faithful: Anderson et al. 2018
	// Implements formula (10): w_{l,m} = x_{l,m} - Σ r_{m,p} * x_{m,p}
	// =========================================================================
	reflex calculate_queue_paper when: is_traffic_signal and use_paper_cbmp {
		list<vehicle> all_v <- (motobike as list) + (car as list) + (truck as list);
		list<vehicle> near_v <- all_v where (each distance_to self < paper_detection_radius);
		list<traffic_light_visual> my_lights <- traffic_light_visual where (each.my_parent = self);

		int cnt_p1 <- 0; int cnt_p2 <- 0;
		int out_p1 <- 0; int out_p2 <- 0;

		loop v over: near_v {
			if (v.current_road != nil) {
				road rd <- road(v.current_road);
				intersection dest <- intersection(road_network target_of rd);
				intersection src  <- intersection(road_network source_of rd);
				bool incoming <- (dest != nil and dest distance_to self < 50.0);
				bool outgoing <- (src  != nil and src  distance_to self < 50.0);

				if (incoming) {
					// Stop line check
					float ang_cv <- float(self.location towards v.location);
					traffic_light_visual sl <- nil;
					if (!empty(my_lights)) {
						sl <- my_lights with_min_of (
							abs(((float(self.location towards each.location) - ang_cv) + 360.0) mod 360.0)
						);
					}
					bool behind <- true;
					if (sl != nil) { behind <- (v distance_to self) > (sl distance_to self); }

					if (behind and sl != nil) {
						if (!empty(signal_phases)) {
							if (sl.my_phase = signal_phases[0]) { cnt_p1 <- cnt_p1 + 1; }
							else if (length(signal_phases) > 1 and sl.my_phase = signal_phases[1]) { cnt_p2 <- cnt_p2 + 1; }
						}
					}
				}
				if (outgoing) {
					float ang_out <- float(self.location towards v.location);
					traffic_light_visual sl_out <- nil;
					if (!empty(my_lights)) {
						sl_out <- my_lights with_min_of (
							abs(((float(self.location towards each.location) - ang_out) + 360.0) mod 360.0)
						);
					}
					if (sl_out != nil) {
						if (!empty(signal_phases)) {
							if (sl_out.my_phase = signal_phases[0]) { out_p1 <- out_p1 + 1; }
							else if (length(signal_phases) > 1 and sl_out.my_phase = signal_phases[1]) { out_p2 <- out_p2 + 1; }
						}
					}
				}
			}
		}

		// Formula (10) adaptation: sum of w_l,m over the phase
		float rs <- 0.7;  float rl <- 0.15;  float rr <- 0.15;
		w_p1_paper <- max(0.0, float(cnt_p1) - (rs * float(out_p1) + (rl + rr) * float(out_p2)));
		w_p2_paper <- max(0.0, float(cnt_p2) - (rs * float(out_p2) + (rl + rr) * float(out_p1)));
		
		w_p1_sum <- w_p1_sum + w_p1_paper;
		w_p2_sum <- w_p2_sum + w_p2_paper;
		paper_step_count <- paper_step_count + 1;

		if (debug_mode and paper_step_count mod 10 = 0) {
			write "[PAPER-x] " + name + " @t=" + round(time) 
				+ " | in: P1=" + cnt_p1 + " P2=" + cnt_p2
				+ " | out: P1=" + out_p1 + " P2=" + out_p2
				+ " | w: P1=" + (round(w_p1_paper*10)/10.0) + " P2=" + (round(w_p2_paper*10)/10.0);
		}
	}

	// Calculate queue length for CBMP Area (100% faithful to zone folder)
	reflex calculate_queue_roi when: is_traffic_signal and use_cbmp {
		int queue_count <- 0;
		loop roi over: my_lanes {
			list<motobike> q_moto  <- motobike overlapping roi.shape where (each.speed < each.max_speed * 0.4);
			list<car>      q_car   <- car overlapping roi.shape where (each.speed < each.max_speed * 0.4);
			list<truck>    q_truck <- truck overlapping roi.shape where (each.speed < each.max_speed * 0.4);
			queue_count <- queue_count + length(q_moto) + length(q_car) + length(q_truck);
		}
		total_node_queue <- queue_count;
		accumulated_queue <- accumulated_queue + queue_count;
		queue_sample_count <- queue_sample_count + 1;
	}

	// Calculate composite phase pressures for CBMP Area (100% faithful to zone folder)
	reflex calculate_composite_phase_pressures when: is_traffic_signal and use_cbmp {
		loop p over: signal_phases { 
			if !(gamma_phase_pressures contains_key p) { gamma_phase_pressures[p] <- 0.0; }
			if !(accumulated_phase_pressures contains_key p) { accumulated_phase_pressures[p] <- 0.0; }
		}

		loop p over: signal_phases {
			float inst_pressure <- 0.0;
			loop lane over: my_lanes {
				if (lane.phase_id = p) {
					inst_pressure <- inst_pressure + (C_saturation * lane.w_max_pressure);
				}
			}
			accumulated_phase_pressures[p] <- accumulated_phase_pressures[p] + inst_pressure;
		}
		pressure_sample_count <- pressure_sample_count + 1;
	}

	// -------------------------------------------------------------------------
	// 6. Khối Đồ họa hiển thị (Aspects)
	// -------------------------------------------------------------------------
	aspect default {
		if (is_traffic_signal) {
			// Hiển thị tên các ngã tư có đèn giao thông trực tiếp lên map 3D
//			draw name at: {location.x, location.y, 10} color: #yellow font: font("Arial", 18, #bold);
			
//			rgb light_color <- is_green ? #green : #red;
//			draw cylinder(0.3, 5) at: location color: #black;
//	        draw sphere(1.5) at: {location.x, location.y, 5} color: light_color;
		}else{
//			draw circle(1) color: color;
		}
	}
}

// =========================================================================
// SPECIES: TRAFFIC_CONTROLLER
// =========================================================================
species traffic_controller {
	
	// -------------------------------------------------------------------------
	// 1. Biến liên kết mô hình & KPIs
	// -------------------------------------------------------------------------
	list<intersection> my_nodes;
	
	//obj for KPIs
	int my_queue <- 0;
	int my_throughput <- 0;
	
	// -------------------------------------------------------------------------
	// 2. Biến điều khiển chế độ Thời gian cố định (Fixed-time Mode)
	// -------------------------------------------------------------------------
	//obj for fixed-time mode - thoi gian chuyen pha co dinh
	float time_to_change <- 60 #s;
	float counter <- 0.0;
	bool is_green <- true;
	
	int queue_ways1 <- 0;
	int queue_ways2 <- 0;
	
	// -------------------------------------------------------------------------
	// 3. Biến điều khiển chế độ CBMP (CBMP Mode)
	// -------------------------------------------------------------------------
	//obj for CBMP mode parameters
	// tau = 120s (chu ky), L = 4s (thoi gian mat mat), kappa = 10s (xanh toi thieu)
	float cycle_duration <- 120 #s;
	float lost_time <- 4 #s;
	float min_green <- 10 #s;
	float cbmp_counter <- 0.0;
	
	//obj for cycle-based green time - g1/g2 chi duoc tinh 1 LAN moi chu ky
	// (dung thiet ke CBMP: "Cycle-Based" = phan bo cho chu ky KE TIEP)
	float g1 <- 56 #s; // thoi gian xanh pha 1 trong chu ky hien tai
	float g2 <- 56 #s; // thoi gian xanh pha 2 trong chu ky hien tai

	int completed_cycles <- 0;
	
	// --- CBMP AREA CONTROLLER VARIABLES ---
	map<string, float> green_times_per_phase <- []; // Calculated green times
	int current_phase_index <- 0;        // Currently active phase index
	float phase_counter <- 0.0;          // Counter for current phase (seconds)
	bool is_initialized <- false;        // First cycle flag
	string current_paper_phase <- "";    // Active phase name for Paper mode light control
	
	// Custom variables for non-overlapping cluster-level queue calculation
	float accumulated_stopped_queue <- 0.0;
	int stopped_queue_samples <- 0;

	reflex calculate_stopped_queue_kpi {
		int stopped_count <- 0;
		list<road> all_roads_in <- [];
		loop node over: my_nodes {
			loop rd over: node.roads_in {
				if !(all_roads_in contains road(rd)) {
					all_roads_in <- all_roads_in + [road(rd)];
				}
			}
		}
		
		list<vehicle> all_stopped_vehicles <- [];
		loop rd over: all_roads_in {
			list<vehicle> veh_on_road <- ((motobike as list) + (car as list) + (truck as list)) where (road(each.current_road) = rd);
			list<vehicle> stopped_veh <- veh_on_road where (
				each distance_to self.location < 120.0 and 
				(each.speed < 5 #km/#h or each.real_speed < 5 #km/#h)
			);
			all_stopped_vehicles <- all_stopped_vehicles + stopped_veh;
		}
		all_stopped_vehicles <- remove_duplicates(all_stopped_vehicles);
		
		accumulated_stopped_queue <- accumulated_stopped_queue + length(all_stopped_vehicles);
		stopped_queue_samples <- stopped_queue_samples + 1;
	}
	
	// -------------------------------------------------------------------------
	// 4. Khối Hành động (Actions)
	// -------------------------------------------------------------------------
	action log_kpi {
		if (stop_simulation) { return; }
		if (use_cbmp) {
			completed_cycles <- completed_cycles + 1;
			if (!empty(my_nodes)) {
				intersection first_node <- my_nodes[0];
				list<string> phs <- first_node.signal_phases;
				if (length(phs) >= 2) {
					string jnc_name <- "JNC_" + int(first_node);
					list<roi_lane> nearby_lanes <- roi_lane where (
						each.In_roi != nil and each.In_roi contains "_" and 
						(each distance_to first_node.location < 60.0)
					);
					if (!empty(nearby_lanes)) {
						roi_lane representative_lane <- nearby_lanes[0];
						if (representative_lane.In_roi != nil and representative_lane.In_roi != "") {
							list<string> tokens <- string(representative_lane.In_roi) split_with "_";
							if (!empty(tokens) and length(tokens) >= 1) {
								jnc_name <- upper_case(tokens[0]);
							}
						}
					}
					string row_light <- jnc_name + "," 
						+ completed_cycles + "," 
						+ round(green_times_per_phase[phs[0]]) + "," 
						+ round(green_times_per_phase[phs[1]]);
					// save row_light to: base_output_dir + "Phase_GreenTime_Log_" + csv_filename format: "csv" rewrite: false;
				}
			}
			
			// Compute cluster-level KPI aggregates
			string jnc_friendly_name <- "JNC_" + int(my_nodes[0]);
			if (!empty(my_nodes)) {
				list<roi_lane> nearby_lanes <- roi_lane where (
					each.In_roi != nil and each.In_roi contains "_" and 
					(each distance_to my_nodes[0].location < 60.0)
				);
				if (!empty(nearby_lanes)) {
					roi_lane representative_lane <- nearby_lanes[0];
					if (representative_lane.In_roi != nil and representative_lane.In_roi != "") {
						list<string> name_tokens <- string(representative_lane.In_roi) split_with "_";
						if (!empty(name_tokens) and length(name_tokens) >= 1) {
							string raw_junction_name <- name_tokens[0];
							jnc_friendly_name <- upper_case(raw_junction_name);
						}
					}
				}
			}
			
			int cluster_throughput <- sum(my_nodes collect each.throughput_count);
			float cluster_delay <- sum(my_nodes collect each.total_delay_in_cycle);
			float avg_delay_this_cluster <- (cluster_throughput > 0) ? cluster_delay / cluster_throughput : 0.0;
			
			// Non-overlapping queue calculation at traffic_controller level
			float avg_queue_this_cluster <- stopped_queue_samples > 0 ? accumulated_stopped_queue / stopped_queue_samples : 0.0;
			
			total_queue_sum <- total_queue_sum + avg_queue_this_cluster;
			total_throughput <- total_throughput + cluster_throughput;
			total_delay_sum <- total_delay_sum + cluster_delay;
			total_samples <- total_samples + 1;
			
			string row_kpi <- jnc_friendly_name + "," 
				+ completed_cycles + "," 
				+ round(time) + "," 
				+ (avg_queue_this_cluster with_precision 2) + "," 
				+ cluster_throughput + "," 
				+ (avg_delay_this_cluster with_precision 2);
			if (is_batch_mode) {
				save row_kpi to: base_output_dir + "KPI_Result_" + csv_filename format: "csv" rewrite: false;
			}
			
			loop node over: my_nodes {
				node.total_delay_in_cycle <- 0.0;
				node.throughput_count <- 0;
				node.accumulated_queue <- 0.0;
				node.queue_sample_count <- 0;
			}
			accumulated_stopped_queue <- 0.0;
			stopped_queue_samples <- 0;
		} else {
			completed_cycles <- completed_cycles + 1;
			
			string jnc_friendly_name <- (!empty(my_nodes)) ? my_nodes[0].name : "unknown";
			if (!empty(my_nodes)) {
				list<friendly_roi_name_provider> nearby_helpers <- friendly_roi_name_provider where (
					each.In_roi != nil and each.In_roi contains "_" and 
					(each distance_to my_nodes[0].location < 60.0)
				);
				if (!empty(nearby_helpers)) {
					friendly_roi_name_provider representative <- nearby_helpers[0];
					if (representative.In_roi != nil and representative.In_roi != "") {
						list<string> name_tokens <- string(representative.In_roi) split_with "_";
						if (!empty(name_tokens) and length(name_tokens) >= 1) {
							jnc_friendly_name <- upper_case(name_tokens[0]);
						}
					}
				}
			}
			
			float total_delay_sum <- sum(my_nodes collect each.total_delay_in_cycle);
			float avg_delay <- my_throughput > 0 ? (total_delay_sum / my_throughput) : 0.0;
			
			// Non-overlapping queue calculation at traffic_controller level
			float avg_queue_this_cluster <- stopped_queue_samples > 0 ? accumulated_stopped_queue / stopped_queue_samples : 0.0;
			
			string row <- jnc_friendly_name + "," + completed_cycles + "," + round(time) + "," + (avg_queue_this_cluster with_precision 2) + "," + my_throughput + "," + (round(avg_delay * 100) / 100.0);
			if (is_batch_mode) {
				save row to: base_output_dir + "KPI_Result_" + csv_filename format: "csv" rewrite: false;
			}
			loop node over: my_nodes {
				node.throughput_count <- 0; 
				node.total_delay_in_cycle <- 0.0;
			}
			accumulated_stopped_queue <- 0.0;
			stopped_queue_samples <- 0;
			my_throughput <- 0;
		}
	}
	
	//obj for compute_green_time - tinh g1/g2 cho CHU KY KE TIEP dua vao phi hien tai
	action compute_green_time {
		if (use_cbmp) {
			if (empty(my_nodes)) { return; }
			intersection node <- my_nodes[0]; 
			list<string> phases <- node.signal_phases; 
			if (empty(phases) or length(phases) < 2) { return; }
			
			loop p over: phases {
				float avg_p <- 0.0;
				if (node.accumulated_phase_pressures contains_key p) {
					avg_p <- (node.pressure_sample_count > 0) ? (node.accumulated_phase_pressures[p] / node.pressure_sample_count) : 0.0;
				}
				node.gamma_phase_pressures[p] <- max(0.0, avg_p);
			}
			
			loop p over: phases {
				node.accumulated_phase_pressures[p] <- 0.0;
			}
			node.pressure_sample_count <- 0;

			float gamma_total <- 0.0;
			loop p over: phases {
				float gamma_p <- 0.0;
				if (node.gamma_phase_pressures contains_key p) {
					gamma_p <- node.gamma_phase_pressures[p];
				}
				gamma_total <- gamma_total + gamma_p;
			}
			
			float total_lost_time <- length(phases) * lost_time;
			float total_min_green <- length(phases) * min_green;
			float available_remainder <- cycle_duration - total_lost_time - total_min_green;

			loop p over: phases {
				float gamma_p <- 0.0;
				if (node.gamma_phase_pressures contains_key p) {
					gamma_p <- node.gamma_phase_pressures[p];
				}
				if (gamma_total < 0.05) {
					green_times_per_phase[p] <- min_green + (available_remainder / length(phases));
				} else {
					green_times_per_phase[p] <- min_green + available_remainder * (gamma_p / gamma_total);
				}
			}
		}
	}

	// =========================================================================
	// CBMP v2 — Paper-faithful green time computation
	// Implements formula (11): γ_S = Σ c_{l,m} * w_{l,m} * S_{l,m}
	// Implements formula (16): λ* = arg max Σ λ_S * γ_S  (LP, 2-phase closed form)
	// c_{l,m} = road.num_lanes (capacity proxy)
	// =========================================================================
	action compute_green_time_paper {
		// Formula (11): γ_S = Σ c_{l,m} * w_{l,m} * S_{l,m}
		// Phase 1 (axis_1 green): N and S directions → S_{l,m}=1
		// Phase 2 (axis_2 green): E and W directions → S_{l,m}=1
		// c_{l,m} = road.num_lanes as capacity weight
		float gam1 <- 0.0;
		float gam2 <- 0.0;
		loop node over: my_nodes {	
			// Compute average cycle pressure (smooth feedback - Method 2)
			node.w_p1_paper <- node.paper_step_count > 0 ? (node.w_p1_sum / node.paper_step_count) : 0.0;
			node.w_p2_paper <- node.paper_step_count > 0 ? (node.w_p2_sum / node.paper_step_count) : 0.0;
			
			// Reset accumulators for next cycle
			node.w_p1_sum <- 0.0;
			node.w_p2_sum <- 0.0;
			node.paper_step_count <- 0;
			
			// Constant c=2.5 as specified in original logic
			float c <- 2.5;
			gam1 <- gam1 + c * node.w_p1_paper;
			gam2 <- gam2 + c * node.w_p2_paper;
		}
		float gam_total <- gam1 + gam2;

		// Formula (16): λ* = arg max Σ λ_S*γ_S
		// s.t. λ_S >= κ/τ,  Σλ <= 1 - L/τ
		// For 2 phases: closed-form LP solution = proportional allocation
		float avail <- 1.0 - ((2*lost_time) / cycle_duration);  // 1 - L/τ
		float kappa <- min_green / cycle_duration;           // κ/τ
		float lam1  <- 0.0;
		float lam2  <- 0.0;
		if (gam_total <= 0.0) {
			lam1 <- avail / 2.0;
			lam2 <- avail / 2.0;
		} else {
			float rem <- avail - 2.0 * kappa;
			if (rem <= 0.0) {
				lam1 <- avail / 2.0;
				lam2 <- avail / 2.0;
			} else {
				lam1 <- kappa + rem * (gam1 / gam_total);
				lam2 <- kappa + rem * (gam2 / gam_total);
			}
		}

		// g_S = λ*_S × τ
		g1 <- lam1 * cycle_duration;
		g2 <- lam2 * cycle_duration;

		// Debug: print green time allocation at every phase transition
		if (debug_mode) {
			intersection dbg_node <- my_nodes[0];
			string jname <- (!empty(dbg_node.signal_phases) ? dbg_node.signal_phases[0] : dbg_node.name);
			write "[PAPER-g] " + jname + " @t=" + round(time) + "s"
				+ " | gam1(NS)=" + (round(gam1*10)/10.0) + " gam2(EW)=" + (round(gam2*10)/10.0)
				+ " | g1(NS)=" + (round(g1*10)/10.0) + "s g2(EW)=" + (round(g2*10)/10.0) + "s"
				+ " | g1+g2=" + (round((g1+g2)*10)/10.0) + "s (expected ~" + (cycle_duration - 2*lost_time) + "s)"
				+ ((gam_total <= 0.0) ? " [!] No pressure - equal split" : "");
		}
	}
	
	// -------------------------------------------------------------------------
	// 5. Khối Phản xạ (Reflexes)
	// -------------------------------------------------------------------------
	// Cap nhat lien tuc de bieu do nhay tung giay
	reflex update_live_metrics {
		int temp_q <- 0;
		int temp_t <- 0;
		loop node over: my_nodes {
			temp_q <- temp_q + node.queue_N + node.queue_S + node.queue_E + node.queue_W;
			temp_t <- temp_t + node.throughput_count;
		}
		my_queue <- temp_q;
		my_throughput <- temp_t;
	}
	
	reflex run_cycle {
		if (use_paper_cbmp) {
			// --- CBMP v2 (Paper) mode ---
			cbmp_counter <- cbmp_counter + step;
			// Get phase names from root node (same as Area mode)
			if (!empty(my_nodes)) {
				list<string> phases <- my_nodes[0].signal_phases;
				if (!empty(phases) and length(phases) >= 2) {
					// Initialize current_paper_phase on first step
					if (current_paper_phase = "") {
						current_paper_phase <- phases[0]; // start with NS phase
					}
					if (is_green) {
						if (cbmp_counter >= g1) {
							cbmp_counter <- 0.0;
							is_green <- false;
							current_paper_phase <- phases[1]; // Phase 2: EW active
							traffic_controller ctrl <- self;
							// Use my_phase matching - same mechanism as Area mode (no axis dependency)
							ask traffic_light_visual where (each.my_parent in my_nodes) {
								state <- (my_phase = ctrl.current_paper_phase) ? "green" : "red";
							}
							ask my_nodes { color_fire <- #red; is_green <- false; }
							// Debug: log phase transition
							if (debug_mode) {
								list<traffic_light_visual> green_lights <- traffic_light_visual where (each.my_parent in my_nodes and each.state = "green");
								list<traffic_light_visual> red_lights <- traffic_light_visual where (each.my_parent in my_nodes and each.state = "red");
								write "[PAPER-PHASE] @t=" + round(time) + "s: SWITCH -> EW GREEN (phase2=" + phases[1] + ")"
									+ " | green=" + (green_lights collect each.my_phase) 
									+ " | red=" + (red_lights collect each.my_phase);
							}
						}
					} else {
						if (cbmp_counter >= g2) {
							cbmp_counter <- 0.0;
							is_green <- true;
							current_paper_phase <- phases[0]; // Phase 1: NS active
							traffic_controller ctrl <- self;
							// Use my_phase matching - same mechanism as Area mode (no axis dependency)
							ask traffic_light_visual where (each.my_parent in my_nodes) {
								state <- (my_phase = ctrl.current_paper_phase) ? "green" : "red";
							}
							ask my_nodes { color_fire <- #green; is_green <- true; }
							// Debug: log phase transition
							if (debug_mode) {
								list<traffic_light_visual> green_lights <- traffic_light_visual where (each.my_parent in my_nodes and each.state = "green");
								list<traffic_light_visual> red_lights <- traffic_light_visual where (each.my_parent in my_nodes and each.state = "red");
								write "[PAPER-PHASE] @t=" + round(time) + "s: SWITCH -> NS GREEN (phase1=" + phases[0] + ")"
									+ " | green=" + (green_lights collect each.my_phase)
									+ " | red=" + (red_lights collect each.my_phase);
							}
							do log_kpi;
							do compute_green_time_paper;  // recalculate for next cycle
						}
					}
				}
			}
		} else if (use_cbmp) {
			// --- CBMP Area (ROI) mode from zone ---
			if (empty(my_nodes)) { return; }
			
			intersection root_node <- my_nodes[0];
			list<string> phases <- root_node.signal_phases;
			if (empty(phases)) { return; }
			
			// Initialize green times on first run
			if (!is_initialized) {
				do compute_green_time;
				is_initialized <- true;
			}
			
			// Get current active phase and its green time
			string active_phase <- phases[current_phase_index];
			float allocated_green_time <- min_green;
			if (green_times_per_phase contains_key active_phase) {
			    allocated_green_time <- green_times_per_phase[active_phase];
			}			
			
			phase_counter <- phase_counter + step;
			
			// Update traffic light visuals
			ask traffic_light_visual where (each.my_parent in my_nodes) {
			    if (self.my_phase = active_phase) { 
				state <- "green"; 
			    } else {
				state <- "red";   
			    }
			}

			// Switch to next phase when green time expires
			if (phase_counter >= allocated_green_time) {
				phase_counter <- 0.0;
				current_phase_index <- current_phase_index + 1;
				
				if (current_phase_index >= length(phases)) {
					current_phase_index <- 0;           // Back to first phase
					do compute_green_time;              // Recalculate green times
					do log_kpi;                         // Log KPI for completed cycle
					ask world { do write_summary; }     // Update summary to have latest data
				}
			}
		} else {
			// --- Fixed-time mode ---
			counter <- counter + step;
			if (counter >= time_to_change) {
				counter <- 0.0;
				is_green <- !is_green;
				// Use my_phase matching (same as Area mode) - avoids nil axis issue
				if (!empty(my_nodes)) {
					list<string> phases <- my_nodes[0].signal_phases;
					if (!empty(phases) and length(phases) >= 2) {
						// is_green=true: NS phase (phases[0]), is_green=false: EW phase (phases[1])
						current_paper_phase <- is_green ? phases[0] : phases[1];
						traffic_controller ctrl <- self;
						ask traffic_light_visual where (each.my_parent in my_nodes) {
							state <- (my_phase = ctrl.current_paper_phase) ? "green" : "red";
						}
						ask my_nodes {
							color_fire <- ctrl.is_green ? #green : #red;
							is_green <- ctrl.is_green;
						}
					}
				}
				if (is_green) { do log_kpi; }
			}
		}
	}
}

// =========================================================================
// SPECIES: TRAFFIC_LIGHT_VISUAL
// =========================================================================
species traffic_light_visual {
    intersection my_parent;
    road my_road; // optional variable for compatibility
    string axis;  // axis identifier
    string state <- "red"; // visual state of traffic light
	string osm_id;
	string my_phase;
	
    aspect default {
    	rgb light_color <- (state = "green") ? #green : #red;
        draw cylinder(0.3, 5) color: #black;
        draw sphere(1.2) at: {location.x, location.y, 5} color: light_color;
    }
}

