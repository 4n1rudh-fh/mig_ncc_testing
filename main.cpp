/*
- In this program, we will take the testing data collected from 'img_acq_testing' and paste it inside the directory, in order to calculate Mean Intensity Gradient (MIG) and Displacement using Normalized Cross-Correlation (NCC). 
- We will also save these results inside an excel file.
- Defining the folder structure for testing:
                              images
                            /        \
                      Gain_1          Gain_2
                    /                       \
                Move_1                    Move_2
                /   \                   /         \
            Exp_1   Exp_2             Exp_1       Exp_2
            /            \              /            \   
        frame_0.png    frame_0.png  frame_0.png     frame_0.png
*/

#include <iostream>
#include <filesystem>
#include <vector>
#include <opencv4/opencv2/opencv.hpp>
#include "xlsxwriter.h"

// Creating a new variable type to get NCC results
struct LocAndConf
{
    cv::Point match_loc;
    double confidence;
    int shift_row, shift_col;
};

/* Function prototyping */
int recursive_folders(const std::string &root_path);
double mig_frame(const cv::Mat &frame);
void create_folders(const std::string &path);
cv::Mat get_roi(cv::Mat &frame, const int &width, const int &height, const int &topLeft_x, const int &topLeft_y);
LocAndConf get_results(cv::Mat &frame, cv::Mat &roi, const int &frameWidth, const int &frameHeight, const int &width, const int &height);

/* Main */
int main()
{
    // Give the absolute path of folder that contains all the experiments and the images
    std::string images_dir = "../images";
    recursive_folders(images_dir);

    return EXIT_SUCCESS;
}

int recursive_folders(const std::string &root_path)
{
    /* Transformation Matrix Parameters */
    const double Txx = -270.0;
    const double Txy = 0.0;
    const double Tyx = -10.0;
    const double Tyy = 270.0;

    /* Constants for NCC */
    const int roi_w = 128, roi_h = 128, topLeft_x = 300, topLeft_y = 208, frameWidth = 728, frameHeight = 544;
    
    /* Checking whether 'image' directory is present. */
    if (!std::filesystem::exists(root_path))
    {
        std::cout << "The 'images' directory does not exist inside this project's directory.\n"
                  << "Please copy it from the project: 'img_acq_testing'.\n"
                  << std::endl;
        return EXIT_FAILURE;
    } else
    {
        std::cout << "/// Directory 'images' found." << std::endl;
    }

    /* Iterating through the 'images' folder */
    for (const auto &cam_param_entry: std::filesystem::directory_iterator(root_path))
    {
        if (cam_param_entry.is_directory())
        {
            std::string cam_param_dir = cam_param_entry.path();
            std::cout << "/// Inside Camera Param Directory     :       " << cam_param_dir << std::endl;

            for (const auto &movement_entry: std::filesystem::directory_iterator(cam_param_dir))
            {
                if (movement_entry.is_directory())
                {
                    std::string movement_dir = movement_entry.path();
                    std::cout << "/// Inside Movement Directory         :       " << movement_dir << std::endl;

                    for (const auto &exp_entry: std::filesystem::directory_iterator(movement_dir))
                    {
                        if (exp_entry.is_directory())
                        {
                            std::string exp_dir = exp_entry.path();
                            std::cout << "/// Inside Experiment Directory       :       " << exp_dir << std::endl;

                            /* Creating folders at this path */
                            create_folders("../Results/" + cam_param_entry.path().filename().string() + "/" + movement_entry.path().filename().string() + "/" + exp_entry.path().filename().string());

                            /* Creating an excel file */
                            std::string xlsx_path = "../Results/" + cam_param_entry.path().filename().string() + "/" + movement_entry.path().filename().string() + "/" + exp_entry.path().filename().string() + "/" + "Results.xlsx";
                            int row_count = 1;
                            lxw_workbook *workbook = workbook_new(xlsx_path.c_str());
                            lxw_worksheet *worksheet = workbook_add_worksheet(workbook, "Results");

                            /* Filling the first row of excel sheet with strings */
                            worksheet_write_string(worksheet, 0, 0, "Pixel Shift X (Columns)", NULL);
                            worksheet_write_string(worksheet, 0, 1, "Pixel Shift Y (Rows)", NULL);
                            worksheet_write_string(worksheet, 0, 2, "Confidence (%)", NULL);
                            worksheet_write_string(worksheet, 0, 3, "Distance X (mm)", NULL);
                            worksheet_write_string(worksheet, 0, 4, "Distance Y (mm)", NULL);
                            worksheet_write_string(worksheet, 0, 5, "Error X (mm)", NULL);
                            worksheet_write_string(worksheet, 0, 6, "Error X (%)", NULL);
                            worksheet_write_string(worksheet, 0, 7, "Error Y (mm)", NULL);
                            worksheet_write_string(worksheet, 0, 8, "Error Y (%)", NULL);
                            worksheet_write_string(worksheet, 0, 9, "MIG", NULL);
                            worksheet_write_string(worksheet, 0, 10, "Avg. MIG", NULL);

                            /* Declaring an empty mig vector and avg. MIG variable */
                            std::vector<double> mig_vector;
                            double avg_mig;
                            double bucket = 0.0;

                            /* Declaring an empty string vector to store frame names */
                            std::vector<std::string> file_names;

                            /* Storing filenames inside the vector */
                            for (const auto &img_entry:std::filesystem::directory_iterator(exp_dir))
                            {
                                // std::string img_path = img_entry.path();
                                file_names.push_back(img_entry.path().filename().string());
                            }

                            /* Sorting filenames */
                            std::sort(file_names.begin(), file_names.end(), [](const std::string &a, const std::string &b)
                            {
                                std::string num_a = a.substr(a.find_first_of("0123456789"));
                                std::string num_b = b.substr(b.find_first_of("0123456789"));
                                int int_a = std::stoi(num_a);
                                int int_b = std::stoi(num_b);
                                return int_a < int_b;
                            });


                            /* Getting ROI for the experiment folder */
                            std::string frame_0_path = exp_entry.path().string() + "/frame_0.png";
                            cv::Mat frame_0 = cv::imread(frame_0_path, cv::IMREAD_GRAYSCALE);
                            cv::Mat roi = get_roi(frame_0, roi_w, roi_h, topLeft_x, topLeft_y);
                            
                            

                            /***** MIG and NCC Start *****/

                            for (const auto &file_name: file_names)
                            {
                                std::string img_path = exp_entry.path().string() + "/" + file_name;
                                std::cout << "/// Reading image                     :       " << img_path << std::endl;
                                // Reading the image 
                                cv::Mat img = cv::imread(img_path, cv::IMREAD_GRAYSCALE);

                                LocAndConf ncc_results = get_results(img, roi, frameWidth, frameHeight, roi_w, roi_h);

                                // Comment the following for calibration
                                double shift_col_mm = ((ncc_results.shift_col * Tyy) - (ncc_results.shift_row * Txy)) / ((Txx * Tyy) - (Txy * Tyx));
                                double shift_row_mm = ((ncc_results.shift_row * Txx) - (ncc_results.shift_col * Tyx)) / ((Txx * Tyy) - (Txy * Tyx));

                                worksheet_write_number(worksheet, row_count, 0, ncc_results.shift_col, NULL);
                                worksheet_write_number(worksheet, row_count, 1, ncc_results.shift_row, NULL);
                                worksheet_write_number(worksheet, row_count, 2, ncc_results.confidence, NULL);
                                
                                // Comment the following for calibration
                                worksheet_write_number(worksheet, row_count, 3, shift_col_mm, NULL);
                                worksheet_write_number(worksheet, row_count, 4, shift_row_mm, NULL);

                                worksheet_write_number(worksheet, row_count, 9, mig_frame(img), NULL);
                                mig_vector.push_back(mig_frame(img));
                                row_count++;
                            }

                            /* Summing up the MIG values for all frames and averaging it and adding it to the excel sheet*/
                            for (const auto& mig_element: mig_vector)
                            {
                                bucket += mig_element;
                            }
                            avg_mig = bucket / static_cast<double>(mig_vector.size());
                            worksheet_write_number(worksheet, 1, 10, avg_mig, NULL);
                            
                            /***** MIG and NCC End *****/



                            // Closing the excel sheet
                            workbook_close(workbook);

                            // Clearing the MIG vector for next experiment directory
                            mig_vector.clear();
                        }
                    }
                }
            }
        }
    }
    
    return EXIT_SUCCESS;
}

void create_folders(const std::string &path)
{
    try
    {
        if (std::filesystem::exists(path))
        {
            std::cout << "\n////////////////////////////////////////////////////////////////////////////////////////\n" 
                      << "/// Folder already exists at path     :       " << path 
                      << "\n////////////////////////////////////////////////////////////////////////////////////////\n" 
                      << std::endl;
        } else
        {
            std::filesystem::create_directories(path);
            std::cout << "\n////////////////////////////////////////////////////////////////////////////////////////\n"
                      << "/// Folder created at path            :       " << path 
                      << "\n////////////////////////////////////////////////////////////////////////////////////////\n"
                      << std::endl;
        }
    } catch (const std::filesystem::filesystem_error &e)
    {
        std::cerr << "/// Error creating folder                 :       " << e.what() << std::endl; 
    }
}

double mig_frame(const cv::Mat &frame)
{
    if (frame.empty())
    {
        std::cout << "Image is empty or corrupted. Please check file." << std::endl;
        return EXIT_FAILURE;
    }
    cv::Mat1f dx, dy, mag; // Declare matrices to store float values
    cv::Sobel(frame, dx, CV_32F, 1, 0, 3);
    cv::Sobel(frame, dy, CV_32F ,0, 1, 3);
    cv::magnitude(dx, dy, mag); // Source -> Destination
    cv::Scalar total = cv::sum(mag);
    double mig = total[0] / (frame.rows * frame.cols);
    return mig;
}

cv::Mat get_roi(cv::Mat &frame, const int &width, const int &height, const int &topLeft_x, const int &topLeft_y)
{
    cv::Mat copy_frame;
    frame.copyTo(copy_frame);
    cv::Rect roiRect(topLeft_x, topLeft_y, width, height);
    cv::Mat roi = copy_frame(roiRect);
    return roi;
}

LocAndConf get_results(cv::Mat &frame, cv::Mat &roi, const int &frameWidth, const int &frameHeight, const int &width, const int &height)
{
    LocAndConf a; // variable of the type struct
    cv::Mat copy_frame;
    cv::Mat copy_roi;
    cv::Point minLoc;
    cv::Point maxLoc;
    cv::Mat result;
    double minVal, maxVal;
    frame.copyTo(copy_frame);
    roi.copyTo(copy_roi);
    cv::matchTemplate(copy_frame, copy_roi, result, cv::TM_CCORR_NORMED, cv::Mat());
    cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat());
    a.match_loc = maxLoc;
    a.confidence = maxVal * 100;

    // -ve value -> template moving up, +ve value -> template moving down
    a.shift_row = (maxLoc.y + ((height)/2)) - ((frameHeight)/2);

    // -ve value -> template moving left, +ve value -> template moving right
    a.shift_col = (maxLoc.x + ((width)/2)) - ((frameWidth)/2);
    return a;
}