/**
 * Name: Roi
 * Mode: ROI Sensor Model supporting Area-based Max Pressure
 * Author: PC (Updated)
 */

model Roi

import "Road.gaml"
import "Intersection.gaml"
import "Vehicles.gaml"
import "Main.gaml"

// schedules: [] keeps roi_lane idle unless updated by global system (same as zone)
species roi_lane schedules: [] { 
    // Shapefile attributes
    string u_node;
    string d_node;
    string phase_id;
    float area_m2;
    string In_roi; 
    string Out_roi;
    
    // Area-based density and pressure variables
    float phi <- 0.0;
    float temp_vehicle_area <- 0.0;
    float phi_current <- 0.0; 
    float w_max_pressure <- 0.0; 
    
    aspect default {
        // Dynamic color representation: green (empty) -> red (congested)
        int r_val <- int(phi * 255);
        int g_val <- int((1.0 - phi) * 255);
        rgb dynamic_color <- rgb(r_val, g_val, 0, 100);
        draw shape color: dynamic_color depth: 0.1;
    }
}