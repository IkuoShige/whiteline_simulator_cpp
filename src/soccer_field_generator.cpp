#include "whiteline_simulator_cpp/preprocessor.hpp"
#include <iostream>
#include <string>

int main(int argc, char* argv[]) {
    std::string output_file = "soccer_field_points.txt";
    std::string landmarks_file = "soccer_field_landmarks.txt";
    double point_spacing = 0.05;  // 5cm spacing between points
    
    // Parse command line arguments
    if (argc > 1) {
        output_file = argv[1];
    }
    if (argc > 2) {
        point_spacing = std::atof(argv[2]);
    }
    if (argc > 3) {
        landmarks_file = argv[3];
    }
    
    std::cout << "Generating soccer field white line points and landmarks..." << std::endl;
    std::cout << "White line points file: " << output_file << std::endl;
    std::cout << "Landmarks file: " << landmarks_file << std::endl;
    std::cout << "Point spacing: " << point_spacing << " meters" << std::endl;
    
    // Create preprocessor and generate field lines and landmarks
    Preprocessor preprocessor;
    SoccerFieldDimensions dimensions;  // Uses default AdultSize dimensions
    
    // Generate white line points
    std::vector<Point2f> field_lines = preprocessor.generate_soccer_field_lines(dimensions, point_spacing);
    
    // Generate landmarks
    std::vector<Landmark> landmarks = preprocessor.generate_soccer_field_landmarks(dimensions);
    
    // Save the generated points and landmarks to files
    preprocessor.save_white_line_points(output_file, field_lines);
    preprocessor.save_landmarks(landmarks_file, landmarks);
    
    std::cout << "Soccer field generation complete!" << std::endl;
    std::cout << "Field dimensions:" << std::endl;
    std::cout << "  Length: " << dimensions.field_length << "m" << std::endl;
    std::cout << "  Width: " << dimensions.field_width << "m" << std::endl;
    std::cout << "  Goal depth: " << dimensions.goal_depth << "m" << std::endl;
    std::cout << "  Goal width: " << dimensions.goal_width << "m" << std::endl;
    std::cout << "  Center circle diameter: " << dimensions.centre_circle_diameter << "m" << std::endl;
    
    std::cout << "Landmark types:" << std::endl;
    std::cout << "  T_JUNCTION (Red): T-shaped intersections" << std::endl;
    std::cout << "  L_CORNER (Blue): L-shaped corners" << std::endl;
    std::cout << "  X_CROSS (Yellow): X-shaped intersections" << std::endl;
    std::cout << "  PENALTY_MARK (Magenta): Penalty marks" << std::endl;
    std::cout << "  GOAL_POST (Cyan): Goal posts" << std::endl;
    
    return 0;
} 