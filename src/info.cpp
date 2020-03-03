/**
 * Robomaster Vision program of Autocar
 * Copyright (c) 2018, Xidian University Robotics Vision group.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
 * documentation files(the "Software"), to deal in the Software without restriction.
 */

#include "info.h"

armor_param::armor_param()
{
	read_params();
}

void armor_param::read_params()
{
    cv::FileStorage fs("../armor_params.xml", cv::FileStorage::READ);
	cv::FileStorage fs_cam("../mercure_params.xml", cv::FileStorage::READ);
    if(!fs.isOpened())
        std::cout << "In armor_param 函数: Cannot open armor_params.xml, please check if the file is exist." << std::endl;
	if(!fs_cam.isOpened())
        std::cout << "In armor_param 函数: Cannot open mercure_params.xml, please check if the file is exist." << std::endl;

	fs["light_threshold_val"] >> light_threshold_val;

	fs["light_min_aspect_ratio"] >> light_min_aspect_ratio;
	fs["light_max_aspect_ratio"] >> light_max_aspect_ratio;

    fs["light_min_area"] >> light_min_area;
	fs["light_max_area"] >> light_max_area;
	
	fs["light_max_angle"] >> light_max_angle;

	fs["light_max_angle_diff"] >> light_max_angle_diff;
	fs["light_max_height_diff"] >> light_max_height_diff;
	fs["light_max_width_diff"] >> light_max_width_diff;

	fs["armor_min_ratio"] >> armor_min_ratio;
    fs["armor_max_ratio"] >> armor_max_ratio;

	fs["armor_light_angle_diff"] >> armor_light_angle_diff;

	fs["filter_armor_area"] >> filter_armor_area;

	fs["armor_max_angle"] >> armor_max_angle;
	fs["armor_min_area"] >> armor_min_area;
	fs["armor_max_area"] >> armor_max_area;

	fs["armor_max_aspect_ratio"] >> armor_max_aspect_ratio;
	fs["armor_max_pixel_val"] >> armor_max_pixel_val;
	fs["armor_max_stddev"] >> armor_max_stddev;
	fs["width"] >> armor_width;
	fs["height"] >> armor_height;
	fs["enemy_color"] >> enemy_color;

	fs["blue_color_diff"] >> blue_color_diff;
	fs["red_color_diff"] >> red_color_diff;

	fs_cam["camera_matrix"] >> camera_matrix;
	fs_cam["distortion_coefficients"] >> distortion_coefficients;
}
