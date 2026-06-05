/**
 * Name: Roi
 * Based on the internal empty template. 
 * Author: PC
 * Tags: 
 */

model Roi

import "Road.gaml"
import "Intersection.gaml"
import "Vehicles.gaml"
import "Main.gaml"

species roi_lane { // removed schedules: [] — reflex now runs automatically each step
    // Attributes loaded from SHP file
    string u_node;
    string d_node;
    string phase_id;
    float area_m2;
    
    // Vehicle footprint area constants (m2) — length x width per type
    float motobike_area <- 1.5;  // ~1.9m x 0.7m
    float car_area      <- 7.5;  // ~4.5m x 1.8m
    float truck_area    <- 18.0; // ~8.0m x 2.4m
    
    // Measurement results
    float phi <- 0.0;                      // occupancy ratio 0.0 -> 1.0
    float occupancy_rate <- 0.0;           // occupancy percentage 0 -> 100
    list<vehicle> vehicles_inside <- [];   // vehicles currently detected inside this ROI
    int current_vehicle_count <- 0;        // snapshot count per step

    // ROI self-scans every step: detect vehicles, identify type, accumulate area, compute phi
    reflex monitor_lane_occupancy {
        list<vehicle> all_v <- (motobike as list) + (car as list) + (truck as list);
        
        // Detect all vehicles whose shape overlaps this ROI polygon
        vehicles_inside <- all_v where (self.shape intersects each.shape);
        current_vehicle_count <- length(vehicles_inside);
        
        // Accumulate total occupied area by identifying vehicle type
        float total_vehicle_area <- 0.0;
        loop v over: vehicles_inside {
            if (species(v) = motobike) {
                total_vehicle_area <- total_vehicle_area + motobike_area;
            } else if (species(v) = car) {
                total_vehicle_area <- total_vehicle_area + car_area;
            } else if (species(v) = truck) {
                total_vehicle_area <- total_vehicle_area + truck_area;
            }
        }
        
        // Compute phi = total occupied area / zone area, capped at 1.0
        float valid_area <- (area_m2 > 0) ? area_m2 : shape.area;
        phi <- (valid_area > 0) ? min(1.0, total_vehicle_area / valid_area) : 0.0;
        occupancy_rate <- phi * 100.0;
        
        // DEBUG: print every 5 cycles for ALL roi zones (even when empty) — remove after validation
//        if (cycle mod 5 = 0) {
//            write "[ROI-DEBUG] " + name
//                + " | Xe detect: " + current_vehicle_count
//                + " | φ: " + (round(phi * 1000) / 10.0) + "%"
//                + (current_vehicle_count = 0 ? " <- TRONG" : " <- CO XE");
//        }
    }

    aspect default {
        // Dynamic color: green (empty) -> red (fully occupied)
        int r_val <- int(phi * 255);
        int g_val <- int((1.0 - phi) * 255);
        rgb dynamic_color <- rgb(r_val, g_val, 0, 100);
        draw shape color: dynamic_color depth: 0.1;
        
        // Draw occupancy percentage text above lane (only when phi > 0)
        if (phi > 0.0) {
            int percent_val <- int(phi * 100);
            point vi_tri_chu <- location;
            
            // Offset text per lane index to avoid overlap
            if (name contains "1") { vi_tri_chu <- vi_tri_chu + {8.0, 0, 0.0}; }
            else if (name contains "2") { vi_tri_chu <- vi_tri_chu + {-8.0, 0, 0.0}; }
            else if (name contains "3") { vi_tri_chu <- vi_tri_chu + {10.0, 0, 0.0}; }
            else if (name contains "4") { vi_tri_chu <- vi_tri_chu + {-10.0, 0, 0.0}; }
            
            draw string(percent_val) at: vi_tri_chu + {0, 0, 5.0} color: #black font: font("Arial", 1);
        }
        
        // DEBUG visual: highlight ROI border in cyan when vehicles are detected
        if (current_vehicle_count > 0) {
            draw shape color: rgb(0, 200, 255, 60) border: #cyan width: 2;
            // Draw a dot for each detected vehicle linked to this ROI
            loop v over: vehicles_inside {
                draw line([location, v.location]) color: #cyan width: 1;
            }
        }
    }
}