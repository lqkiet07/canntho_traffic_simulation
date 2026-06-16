
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

// =========================================================================
// SPECIES: INTERSECTION
// =========================================================================
species intersection skills: [intersection_skill] {
	
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
	float w_N_paper <- 0.0;
	float w_S_paper <- 0.0;
	float w_E_paper <- 0.0;
	float w_W_paper <- 0.0;

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
	reflex calculate_queue when: is_traffic_signal {
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
	reflex calculate_queue_paper when: is_traffic_signal {
		list<vehicle> all_v <- (motobike as list) + (car as list) + (truck as list);
		list<vehicle> near_v <- all_v where (each distance_to self < 200.0);
		list<traffic_light_visual> my_lights <- traffic_light_visual where (each.my_parent = self);

		int cnt_N <- 0; int cnt_S <- 0; int cnt_E <- 0; int cnt_W <- 0;
		int out_N <- 0; int out_S <- 0; int out_E <- 0; int out_W <- 0;

		loop v over: near_v {
			if (v.current_road != nil) {
				road rd <- road(v.current_road);
				intersection dest <- intersection(road_network target_of rd);
				intersection src  <- intersection(road_network source_of rd);
				bool incoming <- (dest != nil and dest distance_to self < 50.0);
				bool outgoing <- (src  != nil and src  distance_to self < 50.0);

				if (incoming) {
					// Stop line check — same logic as calculate_queue
					float ang_cv <- float(self.location towards v.location);
					traffic_light_visual sl <- nil;
					if (!empty(my_lights)) {
						sl <- my_lights with_min_of (
							abs(((float(self.location towards each.location) - ang_cv) + 360.0) mod 360.0)
						);
					}
					bool behind <- true;
					if (sl != nil) { behind <- (v distance_to self) > (sl distance_to self); }

					if (behind) {
						if (name = "intersection35") {
							// Special logic for intersection35 using closest stop light's osm_id
							traffic_light_visual closest_lg <- my_lights closest_to v;
							if (closest_lg != nil) {
								if (closest_lg.osm_id = "2") {
									cnt_N <- cnt_N + 1; // North incoming
								} else if (closest_lg.osm_id = "4") {
									cnt_S <- cnt_S + 1; // South incoming
								} else if (closest_lg.osm_id = "1") {
									cnt_W <- cnt_W + 1; // West incoming
								} else if (closest_lg.osm_id = "3") {
									cnt_E <- cnt_E + 1; // East incoming
								}
							}
						} else {
							// Adaptive Relative Compass logic for other intersections
							float ang <- float(v.location towards self.location);
							string direction <- "";
							float min_diff <- 360.0;
							
							if (ang_in_W >= 0.0) {
								float d <- abs(ang - ang_in_W) mod 360.0;
								if (d > 180.0) { d <- 360.0 - d; }
								if (d < min_diff) { min_diff <- d; direction <- "W"; }
							}
							if (ang_in_N >= 0.0) {
								float d <- abs(ang - ang_in_N) mod 360.0;
								if (d > 180.0) { d <- 360.0 - d; }
								if (d < min_diff) { min_diff <- d; direction <- "N"; }
							}
							if (ang_in_E >= 0.0) {
								float d <- abs(ang - ang_in_E) mod 360.0;
								if (d > 180.0) { d <- 360.0 - d; }
								if (d < min_diff) { min_diff <- d; direction <- "E"; }
							}
							if (ang_in_S >= 0.0) {
								float d <- abs(ang - ang_in_S) mod 360.0;
								if (d > 180.0) { d <- 360.0 - d; }
								if (d < min_diff) { min_diff <- d; direction <- "S"; }
							}
							
							// Fallback to absolute compass
							if (direction = "") {
								if      (ang >= 315 or ang <  45)  { direction <- "W"; }
								else if (ang >= 45  and ang < 135)  { direction <- "N"; }
								else if (ang >= 135 and ang < 225)  { direction <- "E"; }
								else                                { direction <- "S"; }
							}
							
							if      (direction = "W") { cnt_W <- cnt_W + 1; }
							else if (direction = "N") { cnt_N <- cnt_N + 1; }
							else if (direction = "E") { cnt_E <- cnt_E + 1; }
							else if (direction = "S") { cnt_S <- cnt_S + 1; }
						}
					}
				}
				if (outgoing) {
					if (name = "intersection35") {
						// Special logic for outgoing vehicles at intersection35 based on closest exit direction light
						// Compute opposite direction angle to find corresponding exit lane light
						float ang_from_center <- float(self.location towards v.location);
						traffic_light_visual closest_lg <- nil;
						if (!empty(my_lights)) {
							closest_lg <- my_lights with_min_of (
								abs(((float(self.location towards each.location) - ((ang_from_center + 180.0) mod 360.0)) + 360.0) mod 360.0)
							);
						}
						if (closest_lg != nil) {
							if (closest_lg.osm_id = "2") {
								out_S <- out_S + 1; // Exit towards South (opposite of North input)
							} else if (closest_lg.osm_id = "4") {
								out_N <- out_N + 1; // Exit towards North (opposite of South input)
							} else if (closest_lg.osm_id = "1") {
								out_E <- out_E + 1; // Exit towards East (opposite of West input)
							} else if (closest_lg.osm_id = "3") {
								out_W <- out_W + 1; // Exit towards West (opposite of East input)
							}
						}
					} else {
						// Adaptive Relative Compass logic for outgoing vehicles
						float ang <- float(self.location towards v.location);
						string direction_out <- "";
						float min_diff_out <- 360.0;
						
						if (ang_out_E >= 0.0) {
							float d <- abs(ang - ang_out_E) mod 360.0;
							if (d > 180.0) { d <- 360.0 - d; }
							if (d < min_diff_out) { min_diff_out <- d; direction_out <- "E"; }
						}
						if (ang_out_S >= 0.0) {
							float d <- abs(ang - ang_out_S) mod 360.0;
							if (d > 180.0) { d <- 360.0 - d; }
							if (d < min_diff_out) { min_diff_out <- d; direction_out <- "S"; }
						}
						if (ang_out_W >= 0.0) {
							float d <- abs(ang - ang_out_W) mod 360.0;
							if (d > 180.0) { d <- 360.0 - d; }
							if (d < min_diff_out) { min_diff_out <- d; direction_out <- "W"; }
						}
						if (ang_out_N >= 0.0) {
							float d <- abs(ang - ang_out_N) mod 360.0;
							if (d > 180.0) { d <- 360.0 - d; }
							if (d < min_diff_out) { min_diff_out <- d; direction_out <- "N"; }
						}
						
						// Fallback to absolute compass for outgoing
						if (direction_out = "") {
							if      (ang >= 315 or ang <  45)  { direction_out <- "E"; }
							else if (ang >= 45  and ang < 135)  { direction_out <- "S"; }
							else if (ang >= 135 and ang < 225)  { direction_out <- "W"; }
							else                                { direction_out <- "N"; }
						}
						
						if      (direction_out = "E") { out_E <- out_E + 1; }
						else if (direction_out = "S") { out_S <- out_S + 1; }
						else if (direction_out = "W") { out_W <- out_W + 1; }
						else if (direction_out = "N") { out_N <- out_N + 1; }
					}
				}
			}
		}

		x_N <- cnt_N; x_S <- cnt_S; x_E <- cnt_E; x_W <- cnt_W;
		x_out_N <- out_N; x_out_S <- out_S; x_out_E <- out_E; x_out_W <- out_W;

		// Formula (10): w_{l,m} = x_{l,m} - Σ r_{m,p} * x_{m,p}
		// Turn ratios: straight=0.7, left=0.15, right=0.15 (fixed; see NOTE above)
		float rs <- 0.7;  float rl <- 0.15;  float rr <- 0.15;
		w_N_paper <- max(0.0, float(x_N) - (rs * x_out_S + rl * x_out_E + rr * x_out_W));
		w_S_paper <- max(0.0, float(x_S) - (rs * x_out_N + rl * x_out_W + rr * x_out_E));
		w_E_paper <- max(0.0, float(x_E) - (rs * x_out_W + rl * x_out_S + rr * x_out_N));
		w_W_paper <- max(0.0, float(x_W) - (rs * x_out_E + rl * x_out_N + rr * x_out_S));

		if ((name = "intersection35") and cycle mod 10 = 0) {
			write "[Debug " + name + "] x_N=" + x_N + ", x_S=" + x_S + ", x_E=" + x_E + ", x_W=" + x_W + " | w_N=" + round(w_N_paper*10)/10.0 + ", w_S=" + round(w_S_paper*10)/10.0 + ", w_E=" + round(w_E_paper*10)/10.0 + ", w_W=" + round(w_W_paper*10)/10.0;
		}
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
	float g1 <- 60 #s; // thoi gian xanh pha 1 trong chu ky hien tai
	float g2 <- 60 #s; // thoi gian xanh pha 2 trong chu ky hien tai

	int completed_cycles <- 0;
	
	// -------------------------------------------------------------------------
	// 4. Khối Hành động (Actions)
	// -------------------------------------------------------------------------
	action log_kpi {
		completed_cycles <- completed_cycles + 1;
		
		// Calculate average delay for all vehicles that passed the intersection during this cycle
		float total_delay_sum <- sum(my_nodes collect each.total_delay_in_cycle);
		float avg_delay <- my_throughput > 0 ? (total_delay_sum / my_throughput) : 0.0;
		
		// Luu ra file CSV
		string node_name <- (!empty(my_nodes)) ? my_nodes[0].name : "unknown";
		string row <- node_name + "," + completed_cycles + "," + round(time) + "," + my_queue + "," + my_throughput + "," + (round(avg_delay * 100) / 100.0);
		save row to: csv_filename format: "csv" rewrite: false;
		
		// Reset throughput and delay for next cycle
		loop node over: my_nodes {
			node.throughput_count <- 0; 
			node.total_delay_in_cycle <- 0.0;
		}
		my_throughput <- 0;
	}
	
	//obj for compute_green_time - tinh g1/g2 cho CHU KY KE TIEP dua vao phi hien tai
	action compute_green_time {
		// Tinh toan ap suat (Pressure) theo Cong thuc 4: w = phi_in - sum(R * phi_out)
		// Gia su ty le re co dinh: 70% di thang, 15% re trai, 15% re phai
		float p_straight <- 0.7;
		float p_left <- 0.15;
		float p_right <- 0.15;
		
		float w_N <- 0.0; float w_S <- 0.0; float w_E <- 0.0; float w_W <- 0.0;
		float gamma1 <- 0.0;
		float gamma2 <- 0.0;
		loop node over: my_nodes {
			w_N <- w_N + max(0.0, node.phi_N - (p_straight * node.phi_out_S + p_left * node.phi_out_E + p_right * node.phi_out_W));
			w_S <- w_S + max(0.0, node.phi_S - (p_straight * node.phi_out_N + p_left * node.phi_out_W + p_right * node.phi_out_E));
			w_E <- w_E + max(0.0, node.phi_E - (p_straight * node.phi_out_W + p_left * node.phi_out_S + p_right * node.phi_out_N));
			w_W <- w_W + max(0.0, node.phi_W - (p_straight * node.phi_out_E + p_left * node.phi_out_N + p_right * node.phi_out_S));
		}
		// Ap suat tong hop cua pha (Cong thuc 7)
		// Gia dinh: axis_1 la pha Bac-Nam, axis_2 la pha Dong-Tay
		// Hệ số năng lực thông hành Clm = 1.0 cho tất cả
		float gamma1 <- w_N + w_S;
		float gamma2 <- w_E + w_W;
		float gamma_total <- gamma1 + gamma2;
		
		//obj for available time ratio: 1 - L/tau (cong thuc 9)
		float available_ratio <- 1.0 - (lost_time / cycle_duration);
		float min_ratio <- min_green / cycle_duration; // kappa/tau

		float lam1 <- 0.0;
		float lam2 <- 0.0;
		
		if (gamma_total <= 0) {
			// Khong co ap suat: chia deu, van dam bao min
			lam1 <- available_ratio / 2.0;
			lam2 <- available_ratio / 2.0;
		} else {
			//obj for min green constraint - rang buoc kappa PHAI DUOC AP TRUOC
			float remainder <- available_ratio - 2.0 * min_ratio;
			
			if (remainder <= 0.0) {
				lam1 <- available_ratio / 2.0;
				lam2 <- available_ratio / 2.0;
			} else {
				// Buoc 2: phan bo phan con lai ty le theo ap suat gamma
				lam1 <- min_ratio + remainder * (gamma1 / gamma_total);
				lam2 <- min_ratio + remainder * (gamma2 / gamma_total);
			}
		}
		
		//obj for g_S calculation - cong thuc (11): g_S = lambda*_S x tau
		g1 <- lam1 * cycle_duration;
		g2 <- lam2 * cycle_duration;
		
		//obj for CBMP verification debug - in ra moi lan tinh chu ky moi
		// Kiem tra: g1+g2 phai xap xi cycle_duration - lost_time = 116s
		// Kiem tra: g1 va g2 phai >= min_green = 10s
		// Kiem tra: neu gamma1 > gamma2 thi g1 > g2 (pha dong xe duoc xanh nhieu hon)
		intersection target_node <- my_nodes first_with (each.name = "intersection33");
		if (target_node != nil) {
			float g_total <- round((g1 + g2) * 10) / 10.0;
			write "=== [CBMP] Cycle " + cycle + " | controller cho " + target_node.name + " ===";
			write "  γ1(axis1): " + (round(gamma1 * 1000) / 10.0) + "% | γ2(axis2): " + (round(gamma2 * 1000) / 10.0) + "%";
			if (gamma_total <= 0) {
				write "  [!] Canh bao: phi = 0, chia deu thoi gian (CBMP chua hoat dong, kiem tra detection zone)";
			}
			write "  g1(N+S xanh): " + (round(g1 * 10) / 10.0) + "s | g2(E+W xanh): " + (round(g2 * 10) / 10.0) + "s | tong: " + g_total + "s";
			bool g1_ok <- g1 >= min_green;
			bool g2_ok <- g2 >= min_green;
			bool total_ok <- abs(g1 + g2 - (cycle_duration - lost_time)) < 0.5;
			write "  Kiem tra: g1>=" + min_green + "s? " + (g1_ok ? "OK" : "FAIL") 
			    + " | g2>=" + min_green + "s? " + (g2_ok ? "OK" : "FAIL")
			    + " | tong hop le? " + (total_ok ? "OK" : "FAIL");
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
			float c1 <- (!empty(node.ways1)) ? float(node.ways1[0].num_lanes) : 1.0;
			float c2 <- (!empty(node.ways2)) ? float(node.ways2[0].num_lanes) : 1.0;
			gam1 <- gam1 + c1 * node.w_N_paper + c1 * node.w_S_paper;
			gam2 <- gam2 + c2 * node.w_E_paper + c2 * node.w_W_paper;
		}
		float gam_total <- gam1 + gam2;

		// Formula (16): λ* = arg max Σ λ_S*γ_S
		// s.t. λ_S >= κ/τ,  Σλ <= 1 - L/τ
		// For 2 phases: closed-form LP solution = proportional allocation
		float avail <- 1.0 - (lost_time / cycle_duration);  // 1 - L/τ
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

		// Debug log — fires every phase transition (same style as CBMP v1)
		intersection target_node <- my_nodes first_with (each.name = "intersection35");
		if (target_node != nil){
			float g_total <- round((g1 + g2) * 10) / 10.0;
			write "=== [PAPER-v2] Cycle " + cycle + " | " + target_node.name + " ===";
			write "  x: N=" + target_node.x_N + " S=" + target_node.x_S
				+ " E=" + target_node.x_E + " W=" + target_node.x_W;
			write "  w: N=" + round(target_node.w_N_paper*10)/10.0
				+ " S=" + round(target_node.w_S_paper*10)/10.0
				+ " E=" + round(target_node.w_E_paper*10)/10.0
				+ " W=" + round(target_node.w_W_paper*10)/10.0;
			write "  γ1(N+S)=" + round(gam1*10)/10.0
				+ " | γ2(E+W)=" + round(gam2*10)/10.0;
			if (gam_total <= 0.0) {
				write "  [!] Canh bao: x=0, chia deu thoi gian (kiem tra vung detect)";
			}
			write "  g1(N+S)=" + round(g1*10)/10.0 + "s"
				+ " | g2(E+W)=" + round(g2*10)/10.0 + "s"
				+ " | tong=" + g_total + "s";
			bool g1_ok <- g1 >= min_green;
			bool g2_ok <- g2 >= min_green;
			bool total_ok <- abs(g1 + g2 - (cycle_duration - lost_time)) < 0.5;
			write "  Check: g1>=" + min_green + "s? " + (g1_ok ? "OK" : "FAIL")
				+ " | g2>=" + min_green + "s? " + (g2_ok ? "OK" : "FAIL")
				+ " | tong hop le? " + (total_ok ? "OK" : "FAIL");
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
			if (is_green) {
				if (cbmp_counter >= g1) {
					cbmp_counter <- 0.0;
					ask my_nodes { do to_red; }
					is_green <- false;
				}
			} else {
				if (cbmp_counter >= g2) {
					cbmp_counter <- 0.0;
					ask my_nodes { do to_green; }
					is_green <- true;
					do log_kpi;
					do compute_green_time_paper;  // recalculate for next cycle
				}
			}
		} else if (use_cbmp) {
			// --- CBMP v1 (phi area-based) mode ---
			cbmp_counter <- cbmp_counter + step;
			if (is_green) {
				if (cbmp_counter >= g1) {
					cbmp_counter <- 0.0;
					ask my_nodes { do to_red; }
					is_green <- false;
					do compute_green_time;
				}
			} else {
				if (cbmp_counter >= g2) {
					cbmp_counter <- 0.0;
					ask my_nodes { do to_green; }
					is_green <- true;
					do log_kpi;
					do compute_green_time;
				}
			}
		} else {
			// --- Fixed-time mode ---
			counter <- counter + step;
			if (counter >= time_to_change) {
				counter <- 0.0;
				ask my_nodes {
					if (is_green) { do to_red; }
					else { do to_green; }
				}
				is_green <- !is_green;
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
	
    aspect default {
    	rgb light_color <- (state = "green") ? #green : #red;
        draw cylinder(0.3, 5) color: #black;
        draw sphere(1.2) at: {location.x, location.y, 5} color: light_color;
    }
}

