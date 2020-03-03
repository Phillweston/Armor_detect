/**
 * Robomaster Vision program of Autocar
 * Copyright (c) 2018, Xidian University Robotics Vision group.
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
 * documentation files(the "Software"), to deal in the Software without restriction.
 */

#include "info.h"
#include "detect.hpp"
/*
int main()
{
    //cv::VideoCapture capture_camera_forward(0);
    int index = 0;
    cv::Mat frame_forward;
    //std::string pattern_jpg = "../red1/*.jpg";
    armor_sample armor_detector;
    std::vector<cv::String> image_files;
    cv::glob(pattern_jpg, image_files);
    while (true) {
        //capture_camera_forward >> frame_forward;
		for(unsigned int frame = 0; frame < image_files.size(); frame++)
        {
            frame_forward = cv::imread(image_files[frame]);
            armor_detector.detect(frame_forward, index);
            //try {
		    //    imshow("result", frame_forward);
            //    cv::waitKey(1);
		    //}
		    //catch (cv::Exception e) {
			//    std::cout << "show image error" << std::endl;
		    //}
            imshow("image", frame_forward);
            cv::waitKey(300);
        }
	}
}
*/
int main(int argc, char** argv)
{
    ros::init(argc, argv, "RGB");
    armor_sample obj;
    ros::spin();
}